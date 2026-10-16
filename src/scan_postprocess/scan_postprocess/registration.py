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


def _to_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def _transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def auto_voxel(clouds: Sequence[Cloud]) -> float:
    extent = max(float(np.linalg.norm(c.points.max(axis=0) - c.points.min(axis=0))) for c in clouds)
    return float(np.clip(extent / 200.0, 0.01, 0.2))


def _prepare(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=30))
    return down


def global_registration(source, target, voxel: float, params: RegistrationParams):
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
    return RegistrationResult(Cloud(points, colors), transforms, pairs)


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
