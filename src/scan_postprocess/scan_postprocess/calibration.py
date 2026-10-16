"""Camera model and photo-to-scan calibration.

A hand-held phone photo is never exactly at the scanner origin or exactly aligned with its axes,
and its field of view is only known roughly. `calibrate_group` estimates the camera pose of each
photo (rotation and a small offset) and one shared focal length by photometric consistency: when
several registered scans view the same surface, the photos must show the same brightness at the
same 3D point, and that only holds for the right camera poses.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np
from scipy import ndimage, optimize
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

# Scanner axes -> camera axes (right, down, forward) for each supported mounting.
AXES = {
    'frd': np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float),
    'flu': np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float),
}


def rotation_from_degrees(pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """Camera rotation relative to the nominal pose: pitch about the right axis (positive tilts
    the view down), yaw about the down axis (positive turns right), roll about the optical axis."""
    return Rotation.from_euler('xyz', np.radians([pitch, yaw, roll])).as_matrix()


def nominal_focal(width: int, height: int, hfov_deg: float) -> float:
    """Focal length in pixels. `hfov_deg` is the field of view along the photo's long side, which is
    how phone cameras are specified (so it is the same for portrait and landscape photos)."""
    return max(width, height) / 2.0 / np.tan(np.radians(hfov_deg) / 2.0)


@dataclass
class CameraModel:
    focal_px: float
    width: int
    height: int
    rotation: np.ndarray = None  # (3, 3) camera axes relative to the nominal pose
    translation: np.ndarray = None  # (3,) camera position in scanner axes (m)
    frame: str = 'frd'

    def __post_init__(self):
        if self.rotation is None:
            self.rotation = np.eye(3)
        if self.translation is None:
            self.translation = np.zeros(3)

    @property
    def fov_deg(self) -> float:
        return float(np.degrees(2 * np.arctan(max(self.width, self.height) / 2.0 / self.focal_px)))

    @classmethod
    def nominal(cls, width: int, height: int, hfov_deg: float = 65.0, frame: str = 'frd') -> 'CameraModel':
        if frame not in AXES:
            raise ValueError(f'Unknown frame {frame!r}; use one of {sorted(AXES)}.')
        return cls(nominal_focal(width, height, hfov_deg), width, height, frame=frame)

    def project(self, points: np.ndarray):
        """Returns pixel coordinates (u, v) and depth along the optical axis."""
        cam = ((points - self.translation) @ AXES[self.frame].T) @ self.rotation.T
        depth = cam[:, 2]
        safe = np.where(depth > 1e-6, depth, 1.0)
        u = self.width / 2.0 + self.focal_px * cam[:, 0] / safe
        v = self.height / 2.0 + self.focal_px * cam[:, 1] / safe
        return u, v, depth


@dataclass
class GroupCalibration:
    cameras: List[Optional[CameraModel]]
    disagreement_before: float  # robust mean |brightness difference| of matched points, in std units
    disagreement_after: float
    matched_points: int
    improved: bool


SEARCH_ROT = np.radians([15.0, 20.0, 15.0])  # random-search half-range: pitch, yaw, roll
PRIOR_ROT = np.radians([10.0, 12.0, 8.0])  # a hand-held photo is roughly aligned with the scanner
PRIOR_OFFSET = 0.12
PRIOR_LOG_FOCAL = 0.2
PRIOR_WEIGHT = 0.02  # cost of a one-sigma departure from the nominal pose
MIN_COVERAGE = 0.7  # of the nominal camera's matched points that must stay in view
MAX_OFFSET = 0.25  # metres the camera may sit from the scanner origin
PAIR_DISTANCE = 0.04  # metres for two scans' points to count as the same surface spot
LONG_SIDE = 500
MIN_MATCHES = 300


def _rotvec(angles_rad: np.ndarray) -> np.ndarray:
    return Rotation.from_euler('xyz', angles_rad).as_rotvec()


def _small_gray(image: np.ndarray):
    gray = image.astype(np.float32).mean(axis=2)
    scale = LONG_SIDE / max(gray.shape)
    gray = ndimage.zoom(ndimage.gaussian_filter(gray, 0.5 / scale), scale, order=1)
    return ndimage.gaussian_filter(gray, 1.0), scale


def _standardise(values: np.ndarray) -> np.ndarray:
    ok = ~np.isnan(values)
    out = values.copy()
    if ok.sum() > 10:
        out[ok] = (values[ok] - values[ok].mean()) / (values[ok].std() + 1e-6)
    return out


class _Group:
    """Matched points between registered scans plus the low-resolution photos."""

    def __init__(self, points, images, transforms, hfov_deg, frame):
        self.points = points
        self.frame = frame
        self.indices = [i for i, im in enumerate(images) if im is not None]
        self.gray = {i: _small_gray(images[i]) for i in self.indices}
        self.size = {i: images[i].shape[1::-1] for i in self.indices}
        self.base_focal = {i: nominal_focal(*self.size[i], hfov_deg) for i in self.indices}
        world = {i: points[i] @ transforms[i][:3, :3].T + transforms[i][:3, 3] for i in self.indices}
        trees = {i: cKDTree(world[i]) for i in self.indices}
        self.match = {}
        self.floor = {i: MIN_MATCHES for i in self.indices}
        for a in self.indices:
            for b in self.indices:
                if a == b:
                    continue
                dist, j = trees[b].query(world[a], distance_upper_bound=PAIR_DISTANCE)
                ok = np.isfinite(dist)
                if ok.sum():
                    self.match[(a, b)] = (np.where(ok)[0], j[ok])

    def camera(self, i, rotvec, log_focal, offset) -> CameraModel:
        width, height = self.size[i]
        return CameraModel(self.base_focal[i] * np.exp(log_focal), width, height,
                           Rotation.from_rotvec(np.asarray(rotvec)).as_matrix(), np.asarray(offset), self.frame)

    def brightness(self, i, camera: CameraModel) -> np.ndarray:
        gray, scale = self.gray[i]
        u, v, depth = camera.project(self.points[i])
        ok = (depth > 0.2) & (u >= 5) & (u < camera.width - 5) & (v >= 5) & (v < camera.height - 5)
        values = np.full(len(depth), np.nan)
        values[ok] = ndimage.map_coordinates(gray, [v[ok] * scale, u[ok] * scale], order=1)
        return _standardise(values)

    def cost(self, i, camera: CameraModel, others: dict):
        """Huber loss of the brightness difference to the other photos; returns (mean, count)."""
        mine = self.brightness(i, camera)
        total, count = 0.0, 0
        for b, reference in others.items():
            if (i, b) not in self.match:
                continue
            ia, j = self.match[(i, b)]
            diff = np.abs(mine[ia] - reference[j])
            diff = diff[~np.isnan(diff)]
            total += np.where(diff < 1, 0.5 * diff ** 2, diff - 0.5).sum()
            count += len(diff)
        if count < self.floor[i]:
            return 3.0 + (self.floor[i] - count) / self.floor[i], count
        return total / count, count

    def penalty(self, rotvec, log_focal, offset) -> float:
        angles = Rotation.from_rotvec(np.asarray(rotvec)).as_euler('xyz')
        z = (np.sum((angles / PRIOR_ROT) ** 2) + np.sum((np.asarray(offset) / PRIOR_OFFSET) ** 2)
             + (log_focal / PRIOR_LOG_FOCAL) ** 2)
        return 0.5 * PRIOR_WEIGHT * float(z)


def calibrate_group(
    points: Sequence[np.ndarray],
    images: Sequence[Optional[np.ndarray]],
    transforms: Sequence[np.ndarray],
    hfov_deg: float = 65.0,
    frame: str = 'frd',
    seed: int = 0,
    rounds: int = 3,
    min_gain: float = 0.10,
) -> GroupCalibration:
    """Estimates a camera model for every scan that has a photo.

    `points[i]` is scan i in its own frame and `transforms[i]` maps it into the common registered
    frame. The nominal cameras are returned unchanged when the data cannot improve on them (fewer
    than two photographed scans, too little overlap, or less than `min_gain` relative improvement).
    """
    group = _Group(points, images, transforms, hfov_deg, frame)
    nominal = [CameraModel.nominal(*im.shape[1::-1], hfov_deg, frame) if im is not None else None
               for im in images]
    if len(group.indices) < 2:
        return GroupCalibration(nominal, float('nan'), float('nan'), 0, False)

    rng = np.random.default_rng(seed)
    rot = {i: np.zeros(3) for i in group.indices}
    off = {i: np.zeros(3) for i in group.indices}
    focal = 0.0

    def cameras(focal_):
        return {i: group.camera(i, rot[i], focal_, off[i]) for i in group.indices}

    def others_for(i, cams):
        return {b: group.brightness(b, cams[b]) for b in group.indices if b != i}

    def total_cost(focal_):
        cams = cameras(focal_)
        lum = {i: group.brightness(i, cams[i]) for i in group.indices}
        total, count = 0.0, 0
        for i in group.indices:
            c, n = group.cost(i, cams[i], {b: lum[b] for b in group.indices if b != i})
            total += c * n
            count += n
        return (total / count if count else 3.0), count

    before, _ = total_cost(0.0)
    nominal_cams = cameras(0.0)
    nominal_lum = {i: group.brightness(i, nominal_cams[i]) for i in group.indices}
    for i in group.indices:
        counts = 0
        for b in group.indices:
            if (i, b) in group.match:
                ia, j = group.match[(i, b)]
                counts += int((~np.isnan(nominal_lum[i][ia]) & ~np.isnan(nominal_lum[b][j])).sum())
        group.floor[i] = max(MIN_MATCHES, int(MIN_COVERAGE * counts))

    for round_index in range(rounds):
        for i in group.indices:
            others = others_for(i, cameras(focal))
            best = rot[i]
            best_cost = (group.cost(i, group.camera(i, best, focal, off[i]), others)[0]
                         + group.penalty(best, focal, off[i]))
            if round_index == 0:
                for _ in range(600):
                    candidate = _rotvec(rng.uniform(-SEARCH_ROT, SEARCH_ROT))
                    c = (group.cost(i, group.camera(i, candidate, focal, off[i]), others)[0]
                         + group.penalty(candidate, focal, off[i]))
                    if c < best_cost:
                        best_cost, best = c, candidate
            result = optimize.minimize(
                lambda p: group.cost(i, group.camera(i, p, focal, off[i]), others)[0]
                + group.penalty(p, focal, off[i]), best,
                method='Powell', options={'xtol': 1e-3, 'ftol': 1e-5, 'maxiter': 10})
            rot[i] = result.x if result.fun <= best_cost else best
        grid = np.linspace(-0.35, 0.6, 20)
        def focal_objective(f):
            return total_cost(f)[0] + len(group.indices) * 0.5 * PRIOR_WEIGHT * (f / PRIOR_LOG_FOCAL) ** 2

        focal = float(grid[int(np.argmin([focal_objective(f) for f in grid]))])
        refined = optimize.minimize_scalar(focal_objective, bounds=(focal - 0.05, focal + 0.05),
                                           method='bounded', options={'xatol': 1e-3})
        focal = float(refined.x)

    for i in group.indices:
        others = others_for(i, cameras(focal))
        start = group.cost(i, group.camera(i, rot[i], focal, off[i]), others)[0] + group.penalty(rot[i], focal, off[i])
        result = optimize.minimize(
            lambda p: group.cost(i, group.camera(i, rot[i], focal, p), others)[0]
            + group.penalty(rot[i], focal, p), off[i],
            method='Powell', bounds=[(-MAX_OFFSET, MAX_OFFSET)] * 3,
            options={'xtol': 1e-3, 'ftol': 1e-5, 'maxiter': 6})
        if result.fun < start * 0.97:
            off[i] = result.x

    after, matched = total_cost(focal)
    if not after < before * (1.0 - min_gain):
        return GroupCalibration(nominal, before, before, matched, False)
    final = cameras(focal)
    return GroupCalibration([final.get(i) for i in range(len(images))], before, after, matched, True)
