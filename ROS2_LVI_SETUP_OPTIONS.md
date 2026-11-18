# ROS 2 LVI-SAM Setup Options

## Problem Summary

Building full LVI-SAM (Phase 3) from source has dependency issues on standard Ubuntu systems.

**Your current system**: Cannot build ryinin/ros2-lvi due to library conflicts
**Alternative tested**: valentinomario/LiDAR-Visual-Inertial-SLAM requires Livox driver (incompatible with RPLIDAR)

## Solutions Ranked by Complexity

### ✅ Option 1: Use Phase 1+2 Only (RECOMMENDED)

**Status**: Ready now, no additional setup needed
**Effort**: 0 minutes
**Pros**:
- Fully built and tested
- Works immediately for hardware testing
- Excellent for small-medium rooms
- Real-time point cloud + trajectory
- No dependencies to manage

**Cons**:
- No loop closure (drift over very long sessions)
- Limited to hand-held scanning (not outdoor rover mapping)

**When to use**: 95% of cases (room-scale 3D scanning)

**Start testing**:
```bash
cd ~/dev/lidarws
colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile
./launch_mobile_lvism.sh 192.168.1.X
```

---

### 🐳 Option 2: Docker Build (RECOMMENDED for Phase 3)

**Status**: Ready to setup
**Effort**: 20-30 minutes
**Pros**:
- Clean build environment
- All dependencies guaranteed compatible
- Reproducible across machines
- Can keep host system clean

**Cons**:
- Requires Docker installed
- Slower than native build (first time)
- Need to mount workspace as volume

**Step-by-step**:

1. **Install Docker** (if not already):
   ```bash
   sudo apt-get install docker.io
   sudo usermod -aG docker $USER
   newgrp docker
   ```

2. **Ensure your ros2-lvi is in place**:
   ```bash
   ls ~/dev/ros2_lvi_ws/src/ros2-lvi/
   # Should show: CMakeLists.txt, package.xml, msg/, etc.
   ```

3. **Build in Docker**:
   ```bash
   docker build -f /tmp/Dockerfile.ros2_lvi -t ros2-lvi-build .
   docker run -it \
     -v ~/dev/ros2_lvi_ws:/ros2_lvi_ws \
     -w /ros2_lvi_ws \
     ros2-lvi-build
   ```

4. **After build completes**:
   ```bash
   # Back on host: source the install
   source ~/dev/ros2_lvi_ws/install/setup.bash
   # Launch with Phase 3 enabled
   ./launch_mobile_lvism.sh 192.168.1.X true true
   ```

**Status after Docker build**: Full LVI-SAM with loop closure

---

### 🔧 Option 3: Fix System Dependencies (NOT RECOMMENDED)

**Status**: Complex, may not work
**Effort**: 1-2 hours
**Pros**:
- Keeps ros2-lvi on host system
- No Docker overhead
- Faster compilation after first build

**Cons**:
- Requires sudo access
- Complex library debugging
- May break other packages
- System-specific (solution won't port)
- Root cause still unclear

**Attempt if desperate**:
```bash
# Note: These may not fully resolve the issue
sudo apt-get install -y \
  libpcap-dev \
  libcurl4-openssl-dev \
  libgdal-dev \
  libgdal-dev \
  libopencv-dev

# Clean rebuild
cd ~/dev/ros2_lvi_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### 🚫 Option 4: Use Different SLAM (NOT RECOMMENDED)

Alternative SLAM systems that might work:
- **Cartographer** (Google's 2D/3D SLAM)
- **LOAM** (Lidar Odometry and Mapping)
- **ORB-SLAM3** (Visual-only, not LVI)

**Problem**: These would require complete re-architecture of Phase 1+2
**Recommendation**: Not worth the effort, use Phase 1+2 or Docker instead

---

## Decision Matrix

| Need | Recommendation | Next Step |
|------|---|---|
| Test hand-held scanning | **Phase 1+2 only** | Run `./launch_mobile_lvism.sh` now |
| Want full LVI-SAM loop closure | **Docker build** | Follow Option 2 steps above |
| Large outdoor/rover mapping | **Docker + tune config** | Setup Docker, adjust parameters |
| Don't have Docker/can't use it | **Accept Phase 1+2 limitations** | Use as-is, may drift on long sessions |

---

## What Phase 1+2 Can Do

✅ Real-time LiDAR streaming (UDP 5000)
✅ IMU integration (UDP 5001, 20 Hz)
✅ Camera capture (RTSP 8554)
✅ Point cloud visualization (RViz2 + Open3D)
✅ Trajectory tracking
✅ Live mobile app overlay
✅ Export mapped point clouds

⚠️ No loop closure (drift if > 20 min continuous)
⚠️ No relocalization (revisiting areas won't correct drift)
⚠️ Best for < 30m² areas

## Phase 3 Additions (Full LVI-SAM)

✅ Loop closure detection
✅ Pose graph optimization
✅ Long-term drift correction
✅ Relocalization
✅ Large-scene support

---

## Recommended Timeline

**Week 1**: Test Phase 1+2 on hardware
- Rebuild mobile app
- Reflash firmware
- Run live mapping test
- Verify point cloud quality

**Week 2**: Debug any hardware issues
- Sensor calibration (if needed)
- Network setup optimization
- Performance tuning

**Week 3+**: Setup Docker + Phase 3 (if needed)
- Only after confirming Phase 1+2 works well
- Can integrate gradually

---

## Quick Reference

**All systems working**:
```bash
./launch_mobile_lvism.sh 192.168.1.X
```

**With full LVI-SAM (after Docker build)**:
```bash
docker run -it -v ~/dev/ros2_lvi_ws:/ros2_lvi_ws ros2-lvi-build
# Then launch with full features
```

**Troubleshoot**:
- See COMMAND_REFERENCE.md
- See SETUP_CHECKLIST.md Phase 7
- See MOBILE_LVISM_GUIDE.md

---

## Still Stuck?

**Option 1+2 won't build?**
- Check `ros2 pkg list | grep lidar_ros2_bridge`
- Should show both packages installed
- Run `colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile`

**Docker won't run?**
- Ensure Docker installed: `docker --version`
- Add user to docker group: `sudo usermod -aG docker $USER`
- Restart shell or run: `newgrp docker`

**Still need help?**
- Review ROS2_LVI_BUILD_WORKAROUND.md
- Check if ryinin repo has recent updates
- Ask in ROS forums with error details

---

**Recommendation**: Start with Phase 1+2 today, add Phase 3 via Docker later if needed. ✅
