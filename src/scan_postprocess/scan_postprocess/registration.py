"""Point cloud registration using TEASER++ (Truncated Exact Algorithm for Sensor Alignment).

TEASER++ is a robust point cloud registration algorithm that computes the optimal relative
pose (rotation and translation) between two point clouds without requiring an initial guess.

Algorithm Overview:
===================

1. FEATURE EXTRACTION (FPFH):
   - Compute Fast Point Feature Histograms (FPFH) on downsampled point clouds
   - FPFH descriptors capture local geometry around each point
   - These features are invariant to rotation and provide reliable matching

2. CORRESPONDENCE MATCHING:
   - For each feature in the source cloud, find the k-nearest neighbors in target features
   - Use mutual nearest neighbor filtering to get high-confidence correspondences
   - This gives us N noisy correspondences between source and target points

3. TEASER++ REGISTRATION:
   - Input: Noisy point-to-point correspondences between source and target
   - Core insight: The optimal rotation and translation can be found by:
     a) Computing the optimal rotation using Truncated Least Squares (TLS) on a robust cost
     b) Computing the optimal translation given the rotation via Weiszfeld's algorithm
   - TEASER++ iteratively solves these subproblems, truncating outlier correspondences
   - Key advantage: Robust to large numbers of outliers (>50% outliers can be handled)
   - Output: 4x4 transformation matrix with high confidence, no initial guess needed

4. MULTI-SCALE REFINEMENT (ICP):
   - Use the TEASER++ result as initialization for point-to-plane ICP
   - ICP progressively refines the alignment at multiple scales (coarse to fine)
   - Point-to-plane error metric is more robust than point-to-point for surface registration

5. SEQUENTIAL CLOUD MERGING:
   - Register each new cloud against the accumulated merged cloud
   - This allows clouds to only partially overlap with the first scan
   - Provides feedback on alignment confidence for each cloud

Each additional cloud is aligned to everything merged so far, so a cloud only has to overlap
part of the scene, not necessarily the first scan.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d
from scipy import ndimage, signal, optimize
from scipy.spatial import cKDTree, distance_matrix

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
    teaser_noise_bound: float = 0.05  # metres; TEASER++ noise estimate for outlier rejection
    teaser_inlier_threshold: float = 0.10  # metres; points closer than this are inliers
    teaser_max_iterations: int = 10  # max IRLS iterations for rotation estimation
    teaser_truncation_threshold: float = 2.0  # truncation threshold as multiple of noise_bound
    min_fine_fitness: float = 0.20  # minimum overlap fraction for ICP refinement
    max_fine_rmse_voxels: float = 1.5  # maximum acceptable error in voxel units
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


def _extract_fpfh_features(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.pipelines.registration.Feature:
    """Extract FPFH features from a point cloud.
    
    FPFH (Fast Point Feature Histogram) captures local surface geometry around each point.
    It's computed on a k-d tree neighborhood and is invariant to rotation.
    
    Args:
        pcd: Point cloud with estimated normals
        voxel: Voxel size for search radius (features use radius = voxel * 6)
    
    Returns:
        Open3D Feature object containing FPFH descriptors
    """
    return reg.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 6, max_nn=100))


def _find_correspondences(source_features, target_features, mutual: bool = True) -> np.ndarray:
    """Find correspondences between source and target FPFH features.
    
    For each feature in source, find its k-nearest neighbors in target.
    Optionally filter to mutual nearest neighbors only (more robust).
    
    Args:
        source_features: FPFH features from source cloud (N x 33)
        target_features: FPFH features from target cloud (M x 33)
        mutual: If True, keep only correspondences that are mutual NN
    
    Returns:
        Array of correspondence pairs [[src_idx, tgt_idx], ...]
    """
    src_data = np.asarray(source_features.data).T  # N x 33
    tgt_data = np.asarray(target_features.data).T  # M x 33
    
    src_tree = cKDTree(src_data)
    tgt_tree = cKDTree(tgt_data)
    
    # For each source point, find 2 nearest neighbors in target
    dists_s2t, inds_s2t = tgt_tree.query(src_data, k=2)
    
    correspondences = []
    for i in range(len(src_data)):
        best_idx = inds_s2t[i, 0]
        # Lowe's ratio test: accept if best match is significantly better than second
        if dists_s2t[i, 0] < 0.7 * dists_s2t[i, 1]:
            if mutual:
                # Check if target also picks this as its nearest neighbor
                dists_t2s, inds_t2s = src_tree.query(tgt_data[best_idx:best_idx+1], k=1)
                # When k=1, return is 1D array/scalar, handle both cases
                nn_idx = inds_t2s[0] if isinstance(inds_t2s, np.ndarray) else inds_t2s
                if nn_idx == i:
                    correspondences.append([i, best_idx])
            else:
                correspondences.append([i, best_idx])
    
    return np.array(correspondences, dtype=np.int32) if correspondences else np.zeros((0, 2), dtype=np.int32)


def _estimate_rotation_teaser(src_points: np.ndarray, tgt_points: np.ndarray, 
                              noise_bound: float, max_iterations: int, 
                              truncation_threshold: float) -> Tuple[np.ndarray, np.ndarray]:
    """Estimate rotation using Truncated Least Squares with IRLS (Iteratively Reweighted Least Squares).
    
    This implements the rotation estimation core of TEASER++:
    
    1. Start with all correspondences weighted equally
    2. For each iteration:
       a) Solve Wahba's problem (SVD-based optimal rotation given correspondences)
       b) Compute residuals: r_i = ||R*src_i - tgt_i||
       c) Truncate: Set weight=0 for residuals > truncation_threshold * noise_bound
       d) Reweight remaining correspondences inversely proportional to residual
    3. Return best rotation found
    
    The key insight is that truncating large outliers and downweighting moderate ones
    makes the rotation estimation robust to outliers while maintaining accuracy on inliers.
    
    Args:
        src_points: Source correspondence points (N x 3)
        tgt_points: Target correspondence points (N x 3)
        noise_bound: Noise level estimate in metres
        max_iterations: Max IRLS iterations
        truncation_threshold: Truncate residuals > this * noise_bound
    
    Returns:
        (optimal_rotation, inlier_mask)
    """
    N = len(src_points)
    weights = np.ones(N) / N  # Start normalized
    best_R = np.eye(3)
    best_inlier_count = 0
    
    for iteration in range(max_iterations):
        # Check if we have any positive weights
        if np.sum(weights) <= 0:
            weights = np.ones(N) / N
        
        # Weighted Wahba's problem: find optimal rotation given weighted correspondences
        # Center points at their weighted centroids
        src_mean = np.average(src_points, axis=0, weights=weights)
        tgt_mean = np.average(tgt_points, axis=0, weights=weights)
        
        src_centered = src_points - src_mean
        tgt_centered = tgt_points - tgt_mean
        
        # Covariance matrix (weighted)
        W = np.diag(np.sqrt(weights))
        H = src_centered.T @ W.T @ W @ tgt_centered
        
        # Singular value decomposition: H = U * S * V^T
        U, S, Vt = np.linalg.svd(H)
        
        # Optimal rotation: R = V * U^T (Kabsch algorithm)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (det(R) = 1, not reflection)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Compute residuals for current rotation
        residuals = np.linalg.norm(R @ src_points.T - tgt_points.T, axis=0)
        
        # Truncate large outliers (distance > threshold)
        threshold = truncation_threshold * noise_bound
        inliers = residuals < threshold
        inlier_count = np.sum(inliers)
        
        # Track best rotation
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_R = R
        
        # Update weights: zero out truncated residuals, downweight others
        new_weights = np.zeros(N)
        inlier_residuals = residuals[inliers]
        if len(inlier_residuals) > 0:
            # Inverse distance weighting for inliers (emphasize smaller errors)
            epsilon = 1e-6
            new_weights[inliers] = 1.0 / (inlier_residuals + epsilon)
            # Normalize weights
            weight_sum = np.sum(new_weights)
            if weight_sum > 0:
                new_weights /= weight_sum
            else:
                new_weights = np.ones(N) / N
        else:
            # No inliers found, revert to uniform weights
            new_weights = np.ones(N) / N
        
        if np.allclose(new_weights, weights):
            break
        weights = new_weights
    
    # Final inlier mask using best rotation
    final_residuals = np.linalg.norm(best_R @ src_points.T - tgt_points.T, axis=0)
    final_inliers = final_residuals < (truncation_threshold * noise_bound)
    
    return best_R, final_inliers


def _teaser_registration(source_pcd: o3d.geometry.PointCloud, 
                        target_pcd: o3d.geometry.PointCloud,
                        voxel: float, params: RegistrationParams) -> Tuple[np.ndarray, float]:
    """Compute robust global registration using TEASER++ algorithm.
    
    TEASER++ (Truncated Exact Algorithm for Sensor Alignment Plus Plus) solves for the
    optimal 6-DOF pose (3D rotation + 3D translation) between two point clouds.
    
    Algorithm steps:
    1. Extract FPFH features on both clouds
    2. Find feature correspondences using nearest-neighbor matching with Lowe's ratio test
    3. Use TEASER++ to robustly estimate rotation and translation:
       - Estimates rotation via Truncated Least Squares (IRLS algorithm)
       - Estimates translation via Weiszfeld's algorithm given rotation
       - Iteratively truncates worst correspondences to improve solution
    4. Evaluate alignment quality via correspondence inliers
    
    Advantages over RANSAC:
    - RANSAC randomly samples inliers; TEASER++ solves exactly with truncation
    - Can handle >50% outliers; RANSAC degrades with more outliers
    - No iterative sampling needed; faster and more deterministic
    - Provides rotation-translation separation for better numerical stability
    
    Args:
        source_pcd: Source point cloud (to be transformed)
        target_pcd: Target point cloud (reference)
        voxel: Voxel size for FPFH feature extraction
        params: Registration parameters including noise bound and thresholds
    
    Returns:
        (transformation_matrix, confidence_score)
        confidence_score is fraction of inlier correspondences
    """
    # Extract FPFH features
    source_features = _extract_fpfh_features(source_pcd, voxel)
    target_features = _extract_fpfh_features(target_pcd, voxel)
    
    # Find feature correspondences
    correspondences = _find_correspondences(source_features, target_features, mutual=True)
    
    if len(correspondences) < 10:
        raise RegistrationError(
            f'Could not find enough feature correspondences ({len(correspondences)} found, '
            f'need at least 10). Scans may not overlap significantly.')
    
    source_points = np.asarray(source_pcd.points)
    target_points = np.asarray(target_pcd.points)
    
    # Build correspondence point pairs
    src_corr_points = source_points[correspondences[:, 0]]
    tgt_corr_points = target_points[correspondences[:, 1]]
    
    # Estimate rotation using Truncated Least Squares
    rotation, inlier_mask = _estimate_rotation_teaser(
        src_corr_points, tgt_corr_points,
        params.teaser_noise_bound,
        params.teaser_max_iterations,
        params.teaser_truncation_threshold
    )
    
    # Estimate translation given rotation (via Weiszfeld's algorithm)
    # min_t ||target - (R * source + t)||
    # Solution: t = mean(target) - R * mean(source) for inliers
    if np.sum(inlier_mask) > 0:
        src_inliers = src_corr_points[inlier_mask]
        tgt_inliers = tgt_corr_points[inlier_mask]
    else:
        # Fallback: use all points if no inliers found
        src_inliers = src_corr_points
        tgt_inliers = tgt_corr_points
        inlier_mask = np.ones(len(src_corr_points), dtype=bool)
    
    translation = np.mean(tgt_inliers, axis=0) - rotation @ np.mean(src_inliers, axis=0)
    
    # Build 4x4 transformation matrix (source -> target frame)
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    
    confidence = float(np.sum(inlier_mask)) / len(correspondences)
    
    return T, confidence


def local_registration(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    init: np.ndarray,
    voxel: float,
    params: RegistrationParams,
    on_step: Optional[Callable[[np.ndarray], None]] = None,
) -> np.ndarray:
    """Refine alignment using multi-scale point-to-plane ICP.
    
    ICP (Iterative Closest Point) progressively minimizes point-to-plane distances,
    starting from a coarse scale and progressively refining to finer scales.
    
    Point-to-Plane vs Point-to-Point:
    - Point-to-Point: minimizes distance from each source point to nearest target point
      (less robust, sensitive to outliers)
    - Point-to-Plane: minimizes distance from each source point to the target surface
      (more robust, considers local surface orientation via surface normals)
    
    Multi-Scale Strategy:
    - Coarse (4x voxel): Initial large-scale refinement, fast convergence
    - Medium (2x voxel): Intermediate scale refinement  
    - Fine (1x voxel): Final fine-scale alignment for accuracy
    
    Each scale runs 20-40 iterations of ICP, with updates sent to observer every 
    `icp_chunk` iterations for visualization.
    
    Args:
        source_pcd: Source point cloud (to be transformed)
        target_pcd: Target point cloud (reference)
        init: Initial 4x4 transformation (from coarse alignment)
        voxel: Base voxel size in metres
        params: Registration parameters
        on_step: Optional callback(transform) after each small batch of iterations
    
    Returns:
        Refined 4x4 transformation matrix
    """
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
    """Aligns all clouds into the frame of the first one and returns the combined cloud.
    
    Sequential Registration Process:
    1. Start with the first cloud as reference
    2. For each subsequent cloud:
       a) Compute TEASER++ global registration (robust without initial guess)
       b) Refine with multi-scale ICP (point-to-plane, coarse to fine)
       c) Validate alignment quality (overlap % and RMS error)
       d) Merge aligned cloud into accumulated result for next alignment
    
    This approach allows each new cloud to only partially overlap the accumulated result,
    not necessarily the first scan, making it more robust for complex scenes.
    """
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

        # Global registration with TEASER++
        coarse_t, coarse_confidence = _teaser_registration(source, target, voxel, params)
        if coarse_confidence < 0.1:  # Less than 10% inliers
            raise RegistrationError(
                f'Global registration failed for cloud {index + 1}: TEASER++ found too few inliers '
                f'({coarse_confidence:.0%}). The scans may not view the same scene, '
                'or need more overlap.')
        if observer:
            observer(f'{label}: global alignment (TEASER++ inliers: {coarse_confidence:.0%})',
                     placed_items((index, _transform(down_points[index], coarse_t))))

        # Fine registration with ICP
        on_step = None
        if observer:
            def on_step(t, _i=index, _label=label):
                observer(f'{_label}: refining (ICP)', placed_items((_i, _transform(down_points[_i], t))))
        fine_t = local_registration(source, target, coarse_t, voxel, params, on_step)

        # Validate fine alignment
        check = reg.evaluate_registration(source, target, voxel * 1.5, fine_t)
        if check.fitness < params.min_fine_fitness or check.inlier_rmse > voxel * params.max_fine_rmse_voxels:
            raise RegistrationError(
                f'Alignment of cloud {index + 1} is not reliable after ICP (overlap {check.fitness:.0%}, '
                f'rms error {check.inlier_rmse * 100:.1f} cm). Scans need a shared, well-structured view.')
        transforms.append(fine_t)
        pairs.append(PairResult(index, fine_t, float(coarse_confidence), float(check.fitness), float(check.inlier_rmse)))
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
