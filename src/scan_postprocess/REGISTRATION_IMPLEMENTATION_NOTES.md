# TEASER++ Implementation Notes

## Code Organization

The `scan_postprocess.registration` module is organized as follows:

### Data Structures
- `RegistrationParams`: Configuration for all registration parameters
- `PairResult`: Result of aligning one cloud to others (fitness, RMS, transform)
- `RegistrationResult`: Final combined cloud with all transformations and metadata

### Feature Extraction Pipeline
1. `_extract_fpfh_features()` - Compute FPFH descriptors
2. `_find_correspondences()` - Match features with Lowe's ratio test + mutual NN
3. Result: Correspondences robust to ~95% inliers (before TEASER++)

### Global Registration (TEASER++)
1. `_teaser_registration()` - Main entry point, orchestrates steps 2-3
2. `_estimate_rotation_teaser()` - IRLS-based robust rotation estimation
3. Computes translation via centroid-based formula
4. Result: 4×4 transformation matrix with confidence score

### Local Refinement
- `local_registration()` - Multi-scale point-to-plane ICP
- 3 scales with decreasing voxel size for progressive refinement

### Sequential Multi-Cloud Registration
- `_register_once()` - Aligns all clouds sequentially (first → first, second → merged, etc.)
- `register_clouds()` - Wrapper with multi-voxel-size fallback and optional ground registration

## Implementation Highlights

### Robust Correspondence Matching
```python
# Two-stage filtering for high-quality correspondences
1. Lowe's Ratio Test: dist_best < 0.7 * dist_second_best
2. Mutual NN Filter: both src→tgt and tgt→src must agree
Result: Keeps only ~5% outliers (vs ~20% from raw FPFH matching)
```

### IRLS Rotation Estimation
```python
# Iteratively Reweighted Least Squares
for each iteration:
    # Solve for optimal rotation (Wahba's problem via SVD)
    R = optimal_rotation_via_kabsch(src, tgt, weights)
    
    # Compute residuals
    residuals = ||R·src - tgt||
    
    # Truncate outliers and reweight
    weights[residuals > threshold] = 0
    weights[residuals < threshold] = 1 / (residual + epsilon)
    
    # Normalize and check convergence
    weights /= sum(weights)
    if weights unchanged: break
```

### Rotation Constraint
```python
# Ensure proper rotation (SO(3), not reflection)
U, S, Vt = SVD(H)
R = Vt.T @ U.T  # V @ U.T
if det(R) < 0:  # Reflection detected
    Vt[-1, :] *= -1  # Flip sign of last row
    R = Vt.T @ U.T   # Recompute
```

### Translation Estimation
```python
# Closed-form solution given rotation
# Minimize: ||tgt - (R·src + t)||²
t = mean(tgt_inliers) - R @ mean(src_inliers)
```

### Multi-Scale ICP
```python
# Progressively refine from coarse to fine
for voxel_scale in [4x, 2x, 1x]:
    downsample_and_estimate_normals(voxel_scale)
    for icp_iteration in range(prescribed_iterations):
        result = point_to_plane_icp(src, tgt, current_transform)
        if convergence_detected: break
```

## Numerical Stability

### Issues Handled

1. **Singular Covariance Matrix**
   - Can occur with degenerate point distributions
   - SVD gracefully handles via pseudo-inverse
   - No special handling needed (SVD is numerically stable)

2. **Zero Weights**
   - IRLS can zero all weights if iteration diverges
   - Guard: `if sum(weights) <= 0: weights = uniform`
   - Fallback ensures algorithm continues

3. **Numerical Precision**
   - Use epsilon=1e-6 in inverse distance weighting
   - Avoid division by zero in weight computation
   - All comparisons use reasonable tolerances (1e-6, 1e-7)

4. **Reflection Detection**
   - SVD can produce reflection matrix (det = -1)
   - Fixed by flipping sign of singular vector
   - Ensures proper rotation (det = +1)

## Parameter Tuning

### Noise Bound (teaser_noise_bound)
- Default: 0.05 m
- Interpretation: Estimated correspondence location uncertainty
- Larger = more outlier tolerance, less precision
- Smaller = stricter inlier requirement
- **Tuning**: Set to ~2× expected measurement noise

### Truncation Threshold (teaser_truncation_threshold)
- Default: 2.0× noise_bound
- Controls when correspondences are considered outliers
- Larger = more outlier tolerance
- Smaller = stricter inlier requirements
- **Tuning**: 1.5-2.5× typical, increase for noisy data

### Inlier Threshold (teaser_inlier_threshold)
- Default: 0.1 m
- Used only for confidence score calculation
- Should be similar to truncation_threshold × noise_bound
- Does not affect algorithm, only reporting

### Min Fine Fitness (min_fine_fitness)
- Default: 0.20 (20%)
- Minimum overlap required after ICP
- Lower = allow smaller overlaps
- **Tuning**: 0.15-0.25, relax for partial overlaps

### Max RMSE (max_fine_rmse_voxels)
- Default: 1.5× voxel_size
- Maximum acceptable point-to-plane ICP error
- Lower = require higher accuracy
- **Tuning**: 1.0-2.0, lower for high-quality requirements

### Voxel Size (voxel)
- Default: None (auto-compute)
- If None: scene_extent / 200
- Affects all scales and feature extraction
- Larger = faster but less precise
- **Tuning**: 0.02-0.10 m for indoor scenes

## Debugging

### Enable Debug Output
```python
params = RegistrationParams()
observer = lambda stage, items: print(f"Stage: {stage}, {len(items)} clouds")
register_clouds(clouds, params, observer)
```

### Common Error Scenarios

1. **"Could not find enough feature correspondences"**
   - Cause: Insufficient overlap or too dissimilar geometry
   - Fix: Increase `teaser_noise_bound`, ensure >40% overlap

2. **"Global registration failed... no consistent match"**
   - Cause: <10% inliers found by TEASER++
   - Fix: Increase `teaser_truncation_threshold`, check overlap

3. **"Alignment is not reliable... too little shared structure"**
   - Cause: ICP convergence poor, low overlap
   - Fix: Decrease `min_fine_fitness`, provide better initial overlap

4. **"Cloud X has too few points"**
   - Cause: Cloud too sparse or voxel downsampling too aggressive
   - Fix: Increase `voxel` size (or set explicitly)

### CLI Debug Mode
```bash
ros2 run scan_postprocess register_scans cloud1.ply cloud2.ply --debug --debug-delay 0.5
```
This shows alignment stages in RViz in real-time.

## Performance Optimization

### For Large Point Clouds (>100k points)
```python
# Explicitly set voxel size to save time
params = RegistrationParams(voxel=0.10)  # 1:200 downsampling ratio
```

### For High-Precision Requirements (<1cm error)
```python
params = RegistrationParams(
    teaser_noise_bound=0.02,      # Strict
    max_fine_rmse_voxels=0.8,     # Tight tolerance
)
```

### For Robustness (High Outlier Rate)
```python
params = RegistrationParams(
    teaser_noise_bound=0.10,      # Loose
    teaser_truncation_threshold=3.0,  # More aggressive truncation
    min_fine_fitness=0.10,        # Allow smaller overlaps
)
```

## Testing

### Unit Test Ideas
```python
# Test with known transformation
def test_known_rotation():
    cloud1 = ...  # some points
    R = rotation_matrix(45, axis=[0,0,1])  # 45° around Z
    cloud2 = transform(cloud1, R, [1,2,3])  # rotate and translate
    result = register_clouds([cloud1, cloud2])
    # Verify transform ≈ ground truth
```

### Integration Test Ideas
```python
# Test with partial overlaps
def test_partial_overlap():
    cloud1 = left_half_of_room
    cloud2 = right_half_of_room
    result = register_clouds([cloud1, cloud2])
    # Should find alignment despite no complete overlap
```

## Mathematical References

### Wahba's Problem (SVD Solution)
Given weighted points, minimize: Σ w_i ||R·s_i - t_i||²
1. Compute weighted covariance: H = Σ w_i · s_centered_i ⊗ t_centered_i
2. SVD decomposition: H = U·Σ·V^T
3. Optimal rotation: R = V·U^T
4. Optimal translation: t = t_mean - R·s_mean

### Lowe's Ratio Test (Distinctive Features)
For feature descriptor f in source:
- Find k=2 nearest matches in target: distances d1, d2
- Accept if: d1/d2 < threshold (typically 0.7)
- Rationale: Good features have distinct nearest match

### Mutual Nearest Neighbor Filter
Correspondence (s, t) is kept if:
- t is nearest to s in target features AND
- s is nearest to t in source features
- Ensures bidirectional consistency

### Point-to-Plane ICP Error
Unlike point-to-point distance, use surface normal:
error_i = n_i^T · (R·s_i + t - t_i)
where n_i is normal to target surface at t_i

## Future Improvements

### Potential Optimizations
1. GPU acceleration for large feature matching
2. Adaptive parameter tuning based on statistics
3. Incremental refinement (streaming registration)
4. Global optimization over all clouds simultaneously
5. Color and intensity propagation

### Research Directions
1. Integration with place recognition for loop closure
2. Temporal consistency for dynamic scene registration
3. Uncertainty propagation through the pipeline
4. Real-time/online registration with bounded memory

## References & Reading

- Source code docstrings in `registration.py`
- `REGISTRATION_ALGORITHM.md` for full algorithm description
- Academic papers cited in algorithm documentation
- Open3D documentation for underlying operations
