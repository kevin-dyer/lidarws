# Docker LVI-SAM Build Status

## Current Situation

The ros2-lvi repository has complex CMake and linking dependencies that make it difficult to build in a clean Docker environment. The original project was designed for a specific development setup and has several hardcoded assumptions about the build environment.

### Issues Encountered

1. **CMakeLists.txt Syntax Errors** ✓ FIXED
   - Missing closing parentheses in `target_link_libraries()` calls
   - All syntax errors have been corrected

2. **Missing Dependencies** ✓ PARTIALLY FIXED
   - GTSAM (built from source successfully)
   - Ceres (installed from Ubuntu repos)
   - OpenCV (installed from Ubuntu repos)
   - PCL (installed from Ubuntu repos)

3. **Linking Errors** ✗ REMAINING
   - Undefined references to Ceres solver symbols
   - OpenCV library linking issues  
   - OpenMP linking not properly propagated
   - Multiple library version conflicts

### Recommended Path Forward

#### Option 1: Phase 1+2 Only (RECOMMENDED) ⭐
- **Status**: ✅ WORKING - Both packages build successfully on your system
- **Time to test**: Immediate
- **Capabilities**: Real-time 3D mapping for room-scale scenes
- **Action**: Use Phase 1+2 implementation from `~/dev/lidarws/src/`

#### Option 2: Pre-built LVI-SAM Docker Image
- **Status**: ⏳ PENDING
- **Time**: 30-45 minutes for image download + container setup
- **Action**: Use official LVI-SAM docker image instead of building from ros2-lvi:
  ```bash
  docker pull ros2-lvi-official:latest  # or similar pre-built image
  ```

#### Option 3: Manual Compilation with Fix Script
- **Status**: ⏳ NOT YET IMPLEMENTED
- **Time**: 2-3 hours with active debugging
- **Action**: If you need full LVI-SAM Phase 3 with loop closure, we can create a targeted build script that:
  1. Patches CMakeLists.txt comprehensively
  2. Installs all dependencies in correct order
  3. Adds explicit linker flags for Ceres/OpenCV/OpenMP
  4. Validates build against test targets

#### Option 4: Alternative SLAM Framework
- **Status**: ⏳ RESEARCH NEEDED
- **Examples**: vins-fusion, orb-slam2 (with ROS 2 wrapper)
- **Pros**: Better ROS 2 integration, more actively maintained
- **Cons**: Different message formats, may need app changes

## Recommendation

**For immediate testing and development**: Use Phase 1+2 as-is. These provide:
- ✅ Real-time LiDAR point cloud streaming (10 Hz)
- ✅ IMU integration for motion tracking
- ✅ Camera feed visualization
- ✅ Dual desktop + mobile display
- ✅ Tilt stage control for sweep pattern

**Sufficient for**: Room-scale hand-held scanning, real-time feedback, offline refinement

**Limitations**: No loop closure (only for scenes <30min continuous scanning)

## Next Steps

1. **This week**: Test Phase 1+2 with actual hardware
2. **If needed**: Implement Option 3 (build script) after hardware validation
3. **Later**: Consider alternative SLAM if Phase 1+2 drift is unacceptable

---

**Built from**: ros2-lvi (ryinin/ros2-lvi)  
**Docker base**: ubuntu:22.04 + ROS Humble  
**Last updated**: 2024-11-04
