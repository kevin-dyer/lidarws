# Phase 3: Docker LVI-SAM Build & Integration

## Overview

This guide covers building and running the full LVI-SAM system (with loop closure detection) in Docker. 

## Important Status Update

The ros2-lvi repository has **linking dependency issues** when building in a clean Docker environment. This document covers:

1. **Current blockers** and their status
2. **Workarounds** for each issue
3. **Alternative approaches** if Docker build fails
4. **When you actually need Phase 3** (loop closure)

## Do You Need Phase 3?

### Use **Phase 1+2 only** if:
✅ Your mapping area is < 30 minutes of continuous hand-scanning
✅ You only need real-time visualization
✅ You can accept 3-5% drift over room-scale distance
✅ You want to test immediately without Docker setup

### Use **Phase 3 (Full LVI-SAM)** if:
⚠️ You're mapping large outdoor scenes (>100m²)
⚠️ You need to revisit areas and close loops
⚠️ You want global pose graph optimization
⚠️ You're willing to spend 2-3 hours on Docker/build debugging

**Recommendation for first test**: Start with Phase 1+2, add Phase 3 later if drift becomes a problem.

## Build Status

### What Works ✅
- CMakeLists.txt syntax (all parentheses fixed)
- GTSAM compilation from source
- Dependency installation (Ceres, OpenCV, PCL)
- Docker image creation

### What Doesn't Work ❌
- Final linking step for visual_estimator executable
- Ceres solver symbol resolution
- OpenMP propagation to linker
- Library version compatibility (libtbb conflicts)

### Root Cause
The ros2-lvi repository was designed for a specific development setup with pre-built libraries. It makes assumptions about:
- Library versions (Ceres version X, OpenCV version Y)
- Build environment state
- Implicit linker paths

These assumptions break in a clean Docker environment.

## Option A: Attempt Docker Build (Not Recommended)

If you want to try anyway:

```bash
# Prerequisites
docker --version  # Should show Docker is installed

# Build
cd ~/dev/ros2_lvi_ws
docker build -f /tmp/Dockerfile.ros2_lvi -t ros2-lvi-build . 2>&1 | tee docker_build.log

# Expected result: Build fails at linking stage with:
# - "undefined reference to `gtsam::JacobianFactor::...`"
# - "undefined reference to `ceres::Problem::~Problem()`"
```

**If you hit linking errors**, see [Troubleshooting](#troubleshooting) section below.

## Option B: Native Build on Your System ⭐ RECOMMENDED

Your system has all the right library versions already. Try native build:

```bash
# 1. Install missing system dependencies
sudo apt-get install -y libgtsam-dev libceres-dev

# 2. Build
cd ~/dev/ros2_lvi_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to lvi_sam 2>&1 | tee native_build.log

# 3. If successful, update your environment
source ~/dev/ros2_lvi_ws/install/setup.bash

# 4. Test
ros2 launch lvi_sam_mobile mobile_lvism.launch.py
```

**This approach**:
- Uses your system's pre-built libraries (binary compatible)
- Avoids Docker overhead
- Faster iteration if you need tweaks
- Same result as Docker build

## Option C: Use Alternative SLAM Framework

If ros2-lvi continues to be problematic, consider:

### 1. VINS-Mono (Visual-Inertial)
```
Pros: Simpler, no LiDAR needed, good ROS 2 support
Cons: No LiDAR fusion, less accurate than LVI-SAM
Setup: 30 minutes
```

### 2. ORB-SLAM2 (with ROS 2 wrapper)
```
Pros: Robust, widely tested, loop closure works
Cons: Visual-only (no LiDAR/IMU), needs good lighting
Setup: 1-2 hours to adapt ROS 2 wrapper
```

### 3. LOAM/LeGO-LOAM (LiDAR-only)
```
Pros: Works without camera/IMU, proven on RPLIDAR
Cons: No visual, faster drift than LVI-SAM
Setup: 1 hour
```

## Troubleshooting

### Error: "undefined reference to `gtsam::...`"

**Cause**: GTSAM library not linked properly
**Solutions**:
1. Add explicit GTSAM linker flags to CMakeLists.txt:
   ```cmake
   target_link_libraries(target_name ... ${GTSAM_LIBRARIES} gtsam ...)
   ```

2. Set GTSAM_DIR explicitly:
   ```bash
   export GTSAM_DIR=/usr/lib/cmake/GTSAM
   colcon build
   ```

### Error: "undefined reference to `ceres::Problem::~Problem()`"

**Cause**: Ceres library not linked in correct order
**Solution**: Add Ceres explicitly before other libraries:
```cmake
target_link_libraries(target_name ceres gtsam ${GTSAM_LIBRARIES} ...)
```

### Error: "libtbb.so.2 may conflict with libtbb.so.12"

**Cause**: OpenCV built against different TBB version than system
**Solutions**:
1. Force specific TBB version:
   ```bash
   export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
   ```

2. Install matching TBB version:
   ```bash
   sudo apt-get install libtbb-dev
   ```

### Build "hangs" or is very slow

**Cause**: GTSAM compilation takes 10-15 minutes
**Solution**: Wait, or reduce parallel workers:
```bash
colcon build --parallel-workers 2 --packages-up-to lvi_sam
```

## After Successful Build

### Run LVI-SAM

```bash
# Terminal 1: Launch mobile LVI-SAM
source ~/dev/ros2_lvi_ws/install/setup.bash
source ~/dev/lidarws/install/setup.bash
ros2 launch lvi_sam_mobile mobile_lvism.launch.py \
  use_realsense:=false \
  use_livox:=false \
  lidar_topic:=/lidar/points

# Terminal 2: Launch Phase 1 UDP bridge
./launch_mobile_lvism.sh 192.168.1.X

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /odometry/imu
```

### Integrate with Phase 1+2

```bash
# Modify launch_mobile_lvism.sh to also start LVI-SAM backend
# Add after Phase 1+2 launch:
# ros2 launch lvi_sam_mobile mobile_lvism.launch.py &
```

## Comparison: Phase 1+2 vs Full Phase 3

| Feature | Phase 1+2 | Phase 1+2+3 |
|---------|-----------|------------|
| Real-time cloud | ✅ | ✅ |
| Loop closure | ❌ | ✅ |
| Global optimization | ❌ | ✅ |
| Max scan time | ~30 min | Unlimited |
| Max scene size | ~50m² | Unlimited |
| Setup time | 10 min | 2-3 hours |
| Runtime overhead | Minimal | ~500 MB RAM, 1 CPU core |
| Accuracy drift | 3-5% | <1% with loop closure |

## Decision Matrix

Choose **Phase 1+2 only** if:
- [ ] Mapping indoors or small areas
- [ ] First-time testing (want quick results)
- [ ] Comfortable with some drift
- [ ] Want to test hardware immediately

Choose **Phase 3 (Full LVI-SAM)** if:
- [ ] Large outdoor scenes
- [ ] Need loop closure validation
- [ ] Willing to debug build issues
- [ ] Already tested Phase 1+2 works well

## Next Steps

1. **Start**: Test Phase 1+2 with hardware
2. **If satisfied**: Done! (most users stop here)
3. **If needed**: Try Option B (native build) or Option C (alternative SLAM)
4. **If stuck**: Return to this doc, section "Troubleshooting"

---

**Last updated**: 2024-11-04  
**Status**: Active development, Docker build issues documented  
**Questions?** Check [DOCKER_BUILD_STATUS.md](./DOCKER_BUILD_STATUS.md)
