"""Registration for scenes dominated by a ground plane (typically outdoors).

On open ground the geometry is nearly degenerate: the ground is one huge plane, so ICP and overlap scores
accept almost any sliding of one scan over another. This module therefore

1. finds the ground in every scan and levels the scan on it, leaving only yaw and a horizontal shift to solve;
2. matches scans on the structure standing above the ground (stumps, walls, stairs, trees), which is the only
   thing that actually pins the shift;
3. registers every pair of scans, keeps the pairs that agree with each other around loops, and finally
   solves all poses together with a pose graph.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree

reg = o3d.pipelines.registration

GROUND_TOLERANCE = 0.04  # m, plane inlier distance
GROUND_MAX_TILT_DEG = 20.0
MIN_GROUND_FRACTION = 0.05
STRUCTURE_MIN_HEIGHT = 0.15  # m above the ground
STRUCTURE_MAX_HEIGHT = 6.0
MAP_CELL = 0.2
MAP_RADIUS = 12.0
YAW_STEP_DEG = 1.5
STRUCTURE_VOXEL = 0.06
MATCH_DISTANCE = 0.12


class NoGroundError(RuntimeError):
    """The ground plane could not be found in at least one scan."""


@dataclass
class LeveledScan:
    level: np.ndarray  # 4x4, raw scan frame -> levelled frame (ground at z = 0, up = +z)
    structure: np.ndarray  # Nx3 points standing above the ground, levelled frame
    ground: np.ndarray  # Mx3 ground points, levelled frame (thinned)


@dataclass
class Edge:
    i: int
    j: int
    transform: np.ndarray  # levelled scan j -> levelled scan i
    score: float
    matched: int
    rmse: float


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1.0 + c)


def _yaw_matrix(yaw: float, tx: float = 0.0, ty: float = 0.0) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    out = np.eye(4)
    out[:2, :2] = [[c, -s], [s, c]]
    out[0, 3], out[1, 3] = tx, ty
    return out


def _apply(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _thin(points: np.ndarray, voxel: float) -> np.ndarray:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    return np.asarray(pcd.voxel_down_sample(voxel).points)


def level_scan(points: np.ndarray, seed: int = 0) -> LeveledScan:
    """Finds the ground with a RANSAC restricted to near-horizontal planes and levels the scan on it."""
    rng = np.random.default_rng(seed)
    thin = _thin(points, 0.05)
    if len(thin) < 200:
        raise NoGroundError('too few points')
    best_count, best_plane = 0, None
    min_nz = np.cos(np.radians(GROUND_MAX_TILT_DEG))
    for _ in range(3000):
        tri = thin[rng.choice(len(thin), 3, replace=False)]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if abs(n[2]) < min_nz:
            continue
        count = int(np.count_nonzero(np.abs((thin - tri[0]) @ n) < GROUND_TOLERANCE))
        if count > best_count:
            best_count, best_plane = count, (n, tri[0])
    if best_plane is None or best_count < MIN_GROUND_FRACTION * len(thin):
        raise NoGroundError('no horizontal plane')
    n, p0 = best_plane
    for _ in range(3):  # least-squares refinement on the inliers
        inliers = thin[np.abs((thin - p0) @ n) < GROUND_TOLERANCE]
        p0 = inliers.mean(axis=0)
        n = np.linalg.svd(inliers - p0)[2][-1]
    if n[2] < 0:
        n = -n

    rot = _rotation_between(n, np.array([0.0, 0.0, 1.0]))
    level = np.eye(4)
    level[:3, :3] = rot
    level[:3, 3] = -rot @ p0
    flat = _apply(points, level)
    # The scanner sits above the ground, so "up" is the side its origin is on.
    origin_height = _apply(np.zeros((1, 3)), level)[0, 2]
    if origin_height < 0:
        flip = np.diag([1.0, -1.0, -1.0, 1.0])  # 180 deg about x keeps the transform a proper rotation
        level = flip @ level
        flat = _apply(points, level)
    height = flat[:, 2]
    structure = flat[(height > STRUCTURE_MIN_HEIGHT) & (height < STRUCTURE_MAX_HEIGHT)]
    ground = flat[np.abs(height) < GROUND_TOLERANCE * 2]
    return LeveledScan(level, _thin(structure, STRUCTURE_VOXEL), _thin(ground, 0.15))


def _occupancy(points: np.ndarray, half: float, cell: float) -> np.ndarray:
    n = int(round(2 * half / cell))
    inside = (np.abs(points[:, 0]) < MAP_RADIUS) & (np.abs(points[:, 1]) < MAP_RADIUS)
    idx = np.floor((points[inside, :2] + half) / cell).astype(int)
    grid = np.zeros((n, n))
    grid[idx[:, 0], idx[:, 1]] = 1.0
    return ndimage.gaussian_filter(grid, 1.0)


def _candidates(source: np.ndarray, target: np.ndarray, count: int) -> List[Tuple[float, float, float, float]]:
    """(score, yaw, tx, ty) of the best top-down footprint correlations over all yaws."""
    half = MAP_RADIUS * 2
    cell = MAP_CELL
    n = int(round(2 * half / cell))
    target_fft = np.fft.rfft2(_occupancy(target, half, cell))
    limit = int(MAP_RADIUS / cell)
    found: List[Tuple[float, float, float, float]] = []
    for yaw in np.radians(np.arange(0, 360, YAW_STEP_DEG)):
        rotated = _apply(source, _yaw_matrix(yaw))
        corr = np.fft.irfft2(target_fft * np.conj(np.fft.rfft2(_occupancy(rotated, half, cell))), s=(n, n))
        mask = np.zeros_like(corr, dtype=bool)
        mask[:limit + 1, :limit + 1] = mask[:limit + 1, -limit:] = True
        mask[-limit:, :limit + 1] = mask[-limit:, -limit:] = True
        corr = np.where(mask, corr, -1)
        flat = int(np.argmax(corr))
        ix, iy = np.unravel_index(flat, corr.shape)
        sx = (ix if ix <= n // 2 else ix - n) * cell
        sy = (iy if iy <= n // 2 else iy - n) * cell
        found.append((float(corr[ix, iy]), float(yaw), sx, sy))
    found.sort(reverse=True)
    picked: List[Tuple[float, float, float, float]] = []
    for cand in found:
        if all(abs(np.angle(np.exp(1j * (cand[1] - p[1])))) > np.radians(8) or np.hypot(cand[2] - p[2], cand[3] - p[3]) > 1.0
               for p in picked):
            picked.append(cand)
        if len(picked) >= count:
            break
    return picked


def _planar(matrix: np.ndarray) -> np.ndarray:
    """Projects a rigid transform of levelled scans onto yaw + horizontal shift."""
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    return _yaw_matrix(yaw, matrix[0, 3], matrix[1, 3])


def _match(source: np.ndarray, target: np.ndarray, transform: np.ndarray) -> Tuple[float, int, float]:
    """Mutual structure agreement in the region both scans can see: (score, matched points, rms)."""
    moved = _apply(source, transform)
    if len(moved) == 0 or len(target) == 0:
        return 0.0, 0, 1.0
    tt, ts = cKDTree(target), cKDTree(moved)
    d_s, _ = tt.query(moved)
    d_t, _ = ts.query(target)
    # Only points inside the other scan's footprint can be expected to match.
    near_t = d_t < 1.5
    near_s = d_s < 1.5
    in_s = (d_s < MATCH_DISTANCE)
    in_t = (d_t < MATCH_DISTANCE)
    n_s, n_t = int(near_s.sum()), int(near_t.sum())
    if n_s < 30 or n_t < 30:
        return 0.0, 0, 1.0
    fit = 0.5 * (in_s[near_s].mean() + in_t[near_t].mean())
    matched = int(in_s.sum())
    rmse = float(np.sqrt(np.mean(d_s[in_s] ** 2))) if matched else 1.0
    return float(fit * min(1.0, np.sqrt(matched / 400.0))), matched, rmse


def _refine(source: np.ndarray, target: np.ndarray, init: np.ndarray) -> np.ndarray:
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source))
    tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target))
    for p in (src, tgt):
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
    transform = init
    for dist, iters in ((0.6, 30), (0.3, 30), (0.15, 30)):
        result = reg.registration_icp(
            src, tgt, dist, transform, reg.TransformationEstimationPointToPlane(),
            reg.ICPConvergenceCriteria(relative_fitness=1e-7, relative_rmse=1e-7, max_iteration=iters))
        transform = _planar(np.array(result.transformation))
    return transform


def register_pair(source: LeveledScan, target: LeveledScan, i: int, j: int,
                  candidates: int = 12, keep: int = 6, min_score: float = 0.12) -> List[Edge]:
    """Distinct plausible planar transforms taking scan `source` (j) into the frame of `target` (i), best first.

    Several hypotheses are kept because a scene with one long wall lets scans slide along it with no
    change in score; the other scans decide which one is right (see `select_consistent`).
    """
    if len(source.structure) < 100 or len(target.structure) < 100:
        return []
    found: List[Edge] = []
    for _, yaw, tx, ty in _candidates(source.structure, target.structure, candidates):
        transform = _refine(source.structure, target.structure, _yaw_matrix(yaw, tx, ty))
        score, matched, rmse = _match(source.structure, target.structure, transform)
        if score >= min_score:
            found.append(Edge(i, j, transform, score, matched, rmse))
    found.sort(key=lambda e: -e.score)
    distinct: List[Edge] = []
    for e in found:
        if all(_pose_error(e.transform, d.transform)[0] > 0.5 or _pose_error(e.transform, d.transform)[1] > 4.0
               for d in distinct):
            distinct.append(e)
        if len(distinct) >= keep:
            break
    return distinct


def _pose_error(a: np.ndarray, b: np.ndarray) -> Tuple[float, float]:
    delta = np.linalg.inv(a) @ b
    angle = abs(np.degrees(np.arctan2(delta[1, 0], delta[0, 0])))
    return float(np.hypot(delta[0, 3], delta[1, 3])), float(angle)


def _merged(scans: Sequence[LeveledScan], poses: Dict[int, np.ndarray], skip: Optional[int] = None) -> np.ndarray:
    parts = [_apply(scans[k].structure, p) for k, p in poses.items() if k != skip]
    return _thin(np.vstack(parts), STRUCTURE_VOXEL)


def build_poses(scans: Sequence[LeveledScan], hypotheses: Dict[Tuple[int, int], List[Edge]],
                min_score: float = 0.25, rounds: int = 3):
    """Places scans one at a time, each judged against everything placed so far.

    Starts from the best-matching pair. Every unplaced scan tries all pairwise hypotheses against every placed
    scan; the one that agrees best with the whole merged scene wins, so a pair that only fits along a wall is
    rejected once other scans disagree. Finally each pose is refined against all the others.
    Returns (poses, scores, unplaced); poses[k] maps scan k into the frame of the first placed scan.
    """
    def between(a: int, b: int) -> List[np.ndarray]:
        """Transforms taking scan b into the frame of scan a."""
        if (a, b) in hypotheses:
            return [e.transform for e in hypotheses[(a, b)]]
        if (b, a) in hypotheses:
            return [np.linalg.inv(e.transform) for e in hypotheses[(b, a)]]
        return []

    best_edge = max((e for h in hypotheses.values() for e in h), key=lambda e: e.score, default=None)
    if best_edge is None:
        return {}, {}, list(range(len(scans)))
    poses: Dict[int, np.ndarray] = {best_edge.i: np.eye(4), best_edge.j: best_edge.transform}
    scores: Dict[int, float] = {best_edge.i: best_edge.score, best_edge.j: best_edge.score}
    remaining = [k for k in range(len(scans)) if k not in poses]
    while remaining:
        merged = _merged(scans, poses)
        pick: Optional[Tuple[float, int, np.ndarray]] = None
        for k in remaining:
            for i, pose_i in poses.items():
                for transform in between(i, k):
                    pose = pose_i @ transform
                    score = _match(scans[k].structure, merged, pose)[0]
                    if pick is None or score > pick[0]:
                        pick = (score, k, pose)
        if pick is None or pick[0] < min_score:
            break
        score, k, pose = pick
        poses[k] = _refine(scans[k].structure, merged, pose)
        scores[k] = _match(scans[k].structure, merged, poses[k])[0]
        remaining.remove(k)
    for _ in range(rounds):
        for k in list(poses):
            if len(poses) < 3:
                break
            others = _merged(scans, poses, skip=k)
            poses[k] = _refine(scans[k].structure, others, poses[k])
    for k in poses:
        scores[k] = _match(scans[k].structure, _merged(scans, poses, skip=k), poses[k])[0]
    return poses, scores, remaining
