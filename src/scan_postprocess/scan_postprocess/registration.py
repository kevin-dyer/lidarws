"""Two-stage point cloud registration: global (coarse) alignment, then local (fine) alignment.

Coarse: FPFH feature matching with RANSAC; needs no initial guess.
Fine:   multi-scale point-to-plane ICP started from the coarse result.
Each additional cloud is aligned to everything merged so far, so a cloud only has to overlap
part of the scene, not necessarily the first scan.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d
from scipy import ndimage, signal
from scipy.spatial import cKDTree

from . import ground_registration as ground
from .ply_io import Cloud

reg = o3d.pipelines.registration

# observer(stage, items): items is a list of (cloud_index, Nx3 points placed in the output frame)
Observer = Callable[[str, List[Tuple[int, np.ndarray]]], None]


class RegistrationError(RuntimeError):
    """Raised when two point clouds cannot be aligned with sufficient confidence."""


@dataclass
class RegistrationParams:
    voxel: Optional[float] = None  # metres; None = derive from the scene size
    min_coarse_fitness: float = 0.05
    min_fine_fitness: float = 0.20
    max_fine_rmse_voxels: float = 1.5
    ransac_iterations: int = 2_000_000
    ransac_confidence: float = 0.9999
    icp_chunk: int = 2  # ICP iterations between observer updates
    seed: int = 0
    use_ground: bool = True  # level scans on the ground and match structure first (outdoor scenes)
    min_structure_score: float = 0.25  # a scan agreeing less than this with the others is left out


@dataclass
class PairResult:
    index: int
    transformation: np.ndarray
    coarse_fitness: float
    fine_fitness: float
    fine_rmse: float


@dataclass
class RegistrationResult:
    cloud: Cloud
    transformations: List[np.ndarray]
    pairs: List[PairResult] = field(default_factory=list)
    source_index: Optional[np.ndarray] = None  # which input cloud each output point came from
    excluded: List[int] = field(default_factory=list)  # input clouds that could not be aligned and were left out
    included: Optional[List[int]] = None  # input indices of the aligned clouds (None = all, in order)


def _to_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def _transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def auto_voxel(clouds: Sequence[Cloud]) -> float:
    extent = max(float(np.linalg.norm(np.percentile(c.points, 99, axis=0) - np.percentile(c.points, 1, axis=0))) for c in clouds)
    return float(np.clip(extent / 200.0, 0.01, 0.2))


def _prepare(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=30))
    return down


def _ransac_registration(source, target, voxel: float, params: RegistrationParams):
    """Coarse alignment of two downsampled clouds without any initial guess."""
    def features(pcd):
        return reg.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 6, max_nn=100))

    if hasattr(o3d.utility, 'random'):
        o3d.utility.random.seed(params.seed)
    distance = voxel * 1.5
    return reg.registration_ransac_based_on_feature_matching(
        source, target, features(source), features(target), True, distance,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9), reg.CorrespondenceCheckerBasedOnDistance(distance)],
        reg.RANSACConvergenceCriteria(params.ransac_iterations, params.ransac_confidence),
    )


def _footprint(points: np.ndarray, yaw: float, cell: float, radius: float, size: int) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    x = c * points[:, 0] - s * points[:, 1]
    y = s * points[:, 0] + c * points[:, 1]
    ix = np.floor((x + radius) / cell).astype(int)
    iy = np.floor((y + radius) / cell).astype(int)
    ok = (ix >= 0) & (ix < size) & (iy >= 0) & (iy < size)
    grid = np.zeros((size, size))
    grid[ix[ok], iy[ok]] = 1.0
    return ndimage.gaussian_filter(grid, 1.0)


class _Coarse:
    def __init__(self, transformation, fitness, correspondences):
        self.transformation = transformation
        self.fitness = fitness
        self.correspondence_set = correspondences


def global_registration(source, target, voxel: float, params: RegistrationParams):
    """Coarse alignment without an initial guess: best of feature-based RANSAC and level-scan search."""
    guesses = [np.array(_ransac_registration(source, target, voxel, params).transformation)]
    guesses += planar_candidates(np.asarray(source.points), np.asarray(target.points), voxel)
    best = None
    for guess in guesses:
        # a short, wide ICP makes the candidates comparable
        refined = reg.registration_icp(
            source, target, voxel * 4, guess, reg.TransformationEstimationPointToPlane(),
            reg.ICPConvergenceCriteria(max_iteration=30))
        if best is None or refined.fitness > best.fitness:
            best = refined
    return _Coarse(np.array(best.transformation), best.fitness, best.correspondence_set)


def planar_candidates(source: np.ndarray, target: np.ndarray, voxel: float, count: int = 12) -> List[np.ndarray]:
    """Coarse guesses assuming both scans were taken level (rotation about the vertical axis only).

    Correlates top-down occupancy maps over every yaw and picks the best shifts. Feature matching
    is unreliable on sparse, noisy room scans; this is a robust complement that ICP then refines.
    """
    cell = max(voxel, 0.05)
    radius = float(max(np.abs(source[:, :2]).max(), np.abs(target[:, :2]).max())) + cell
    size = int(2 * radius / cell) + 1
    target_grid = _footprint(target, 0.0, cell, radius, size)
    scored = []
    for yaw in np.radians(np.arange(0, 360, 2.0)):
        corr = signal.fftconvolve(target_grid, _footprint(source, yaw, cell, radius, size)[::-1, ::-1])
        # exclude shifts that would require more than the full extent
        n = np.unravel_index(np.argmax(corr), corr.shape)
        scored.append((float(corr[n]), yaw, (n[0] - (size - 1)) * cell, (n[1] - (size - 1)) * cell))
    scored.sort(key=lambda item: -item[0])

    tree = cKDTree(target[:, :2])
    candidates = []
    for _score, yaw, tx, ty in scored[:count]:
        c, s = np.cos(yaw), np.sin(yaw)
        moved = source @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T + [tx, ty, 0]
        dist, idx = tree.query(moved[:, :2])
        close = dist < cell
        dz = float(np.median(target[idx[close], 2] - moved[close, 2])) if close.sum() > 20 else 0.0
        t = np.eye(4)
        t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        t[:3, 3] = [tx, ty, dz]
        candidates.append(t)
    return candidates


def local_registration(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    init: np.ndarray,
    voxel: float,
    params: RegistrationParams,
    on_step: Optional[Callable[[np.ndarray], None]] = None,
) -> np.ndarray:
    """Coarse-to-fine point-to-plane ICP; calls `on_step(transform)` after every small batch of iterations."""
    transform = np.array(init)
    for scale, iterations in ((4.0, 40), (2.0, 30), (1.0, 20)):
        size = voxel * scale
        src = _prepare(source_pcd, size)
        tgt = _prepare(target_pcd, size)
        done = 0
        while done < iterations:
            step = min(params.icp_chunk, iterations - done)
            result = reg.registration_icp(
                src, tgt, size * 1.5, transform,
                reg.TransformationEstimationPointToPlane(),
                reg.ICPConvergenceCriteria(relative_fitness=1e-7, relative_rmse=1e-7, max_iteration=step),
            )
            delta = float(np.abs(result.transformation - transform).max())
            transform = np.array(result.transformation)
            done += step
            if on_step:
                on_step(transform)
            if delta < 1e-6:
                break
    return transform


def _register_once(
    clouds: Sequence[Cloud],
    params: Optional[RegistrationParams] = None,
    observer: Optional[Observer] = None,
) -> RegistrationResult:
    """Aligns all clouds into the frame of the first one and returns the combined cloud."""
    params = params or RegistrationParams()
    if len(clouds) < 2:
        raise ValueError('Registration needs at least two point clouds.')
    voxel = params.voxel or auto_voxel(clouds)

    downs = [_prepare(_to_pcd(c.points), voxel) for c in clouds]
    down_points = [np.asarray(d.points) for d in downs]
    for i, points in enumerate(down_points):
        if len(points) < 50:
            raise RegistrationError(f'Cloud {i + 1} has too few points ({len(points)}) to align.')

    transforms: List[np.ndarray] = [np.eye(4)]
    pairs: List[PairResult] = []
    merged = _to_pcd(down_points[0])

    def placed_items(moving: Optional[Tuple[int, np.ndarray]] = None):
        items = [(i, _transform(down_points[i], transforms[i])) for i in range(len(transforms))]
        if moving:
            items.append(moving)
        return items

    if observer:
        observer('start: scans in their own frames', list(enumerate(down_points)))

    for index in range(1, len(clouds)):
        label = f'cloud {index + 1}/{len(clouds)}'
        source = downs[index]
        target = _prepare(merged, voxel)

        coarse = global_registration(source, target, voxel, params)
        if coarse.fitness < params.min_coarse_fitness or len(coarse.correspondence_set) < 10:
            raise RegistrationError(
                f'Global registration failed for cloud {index + 1}: no consistent match with the '
                f'previous scans (overlap {coarse.fitness:.0%}). The scans may not view the same scene, '
                'or need more overlap.')
        coarse_t = np.array(coarse.transformation)
        if observer:
            observer(f'{label}: global alignment',
                     placed_items((index, _transform(down_points[index], coarse_t))))

        on_step = None
        if observer:
            def on_step(t, _i=index, _label=label):
                observer(f'{_label}: refining', placed_items((_i, _transform(down_points[_i], t))))
        fine_t = local_registration(source, target, coarse_t, voxel, params, on_step)

        check = reg.evaluate_registration(source, target, voxel * 1.5, fine_t)
        if check.fitness < params.min_fine_fitness or check.inlier_rmse > voxel * params.max_fine_rmse_voxels:
            raise RegistrationError(
                f'Alignment of cloud {index + 1} is not reliable (overlap {check.fitness:.0%}, '
                f'rms error {check.inlier_rmse * 100:.1f} cm). Scans need a shared, well-structured view.')
        transforms.append(fine_t)
        pairs.append(PairResult(index, fine_t, float(coarse.fitness), float(check.fitness), float(check.inlier_rmse)))
        merged = (merged + _to_pcd(_transform(down_points[index], fine_t))).voxel_down_sample(voxel)
        if observer:
            observer(f'{label}: aligned', placed_items())

    points = np.vstack([_transform(c.points, t) for c, t in zip(clouds, transforms)])
    colors = None
    if any(c.colors is not None for c in clouds):
        gray = np.full((1, 3), 128, dtype=np.uint8)
        colors = np.vstack([c.colors if c.colors is not None else np.tile(gray, (len(c), 1)) for c in clouds])
    source_index = np.concatenate([np.full(len(c), i) for i, c in enumerate(clouds)])
    return RegistrationResult(Cloud(points, colors), transforms, pairs, source_index)


def voxel_merge(cloud: Cloud, voxel: float) -> Cloud:
    """Optional thinning of the combined cloud, averaging colours inside each voxel."""
    pcd = _to_pcd(cloud.points)
    if cloud.colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(cloud.colors / 255.0)
    out = pcd.voxel_down_sample(voxel)
    colors = None
    if out.has_colors():
        colors = np.clip(np.asarray(out.colors) * 255 + 0.5, 0, 255).astype(np.uint8)
    return Cloud(np.asarray(out.points), colors)


def _register_on_ground(
    clouds: Sequence[Cloud],
    params: RegistrationParams,
    observer: Optional[Observer] = None,
) -> Optional[RegistrationResult]:
    """Ground-levelled registration; None when the scans have no usable ground or too few could be aligned."""
    try:
        scans = [ground.level_scan(c.points, params.seed) for c in clouds]
    except ground.NoGroundError:
        return None
    count = len(clouds)
    hypotheses = {}
    for i in range(count):
        for j in range(i + 1, count):
            found = ground.register_pair(scans[j], scans[i], i, j)
            if found:
                hypotheses[(i, j)] = found
    poses, scores, _ = ground.build_poses(scans, hypotheses, params.min_structure_score)
    included = sorted(poses)
    if len(included) < 2:
        return None
    excluded = [k for k in range(count) if k not in poses]

    first = included[0]
    to_first = np.linalg.inv(poses[first] @ scans[first].level)
    transforms = [to_first @ poses[k] @ scans[k].level for k in included]
    kept = [clouds[k] for k in included]
    pairs = []
    for n, k in enumerate(included):
        if n > 0:
            rms = ground._match(scans[k].structure, ground._merged(scans, poses, skip=k), poses[k])[2]
            pairs.append(PairResult(k, transforms[n], float(scores[k]), float(scores[k]), float(rms)))
    if observer:
        observer('aligned', [(n, _transform(c.points[::20], t)) for n, (c, t) in enumerate(zip(kept, transforms))])
    points = np.vstack([_transform(c.points, t) for c, t in zip(kept, transforms)])
    colors = None
    if any(c.colors is not None for c in kept):
        gray = np.full((1, 3), 128, dtype=np.uint8)
        colors = np.vstack([c.colors if c.colors is not None else np.tile(gray, (len(c), 1)) for c in kept])
    source_index = np.concatenate([np.full(len(c), n) for n, c in enumerate(kept)])
    return RegistrationResult(Cloud(points, colors), transforms, pairs, source_index, excluded, included)


def register_clouds(
    clouds: Sequence[Cloud],
    params: Optional[RegistrationParams] = None,
    observer: Optional[Observer] = None,
) -> RegistrationResult:
    """Aligns all clouds into the frame of the first one and returns the combined cloud.

    With no explicit voxel size, several resolutions are tried because sparse scans align reliably
    only at a suitable scale; the error from the last attempt is raised if none succeeds.
    """
    params = params or RegistrationParams()
    if params.use_ground and len(clouds) >= 2:
        grounded = _register_on_ground(clouds, params, observer)
        if grounded is not None:
            return grounded
    if params.voxel:
        return _register_once(clouds, params, observer)
    base = auto_voxel(clouds)
    error: Optional[RegistrationError] = None
    for factor in (1.0, 1.25, 0.8, 1.6, 2.0, 0.6):
        attempt = RegistrationParams(**{**params.__dict__, 'voxel': base * factor})
        try:
            return _register_once(clouds, attempt, observer)
        except RegistrationError as e:
            error = e
    raise error
