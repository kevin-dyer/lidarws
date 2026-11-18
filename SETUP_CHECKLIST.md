# LVI-SAM Mobile Mapping Setup Checklist

## Hardware Requirements
- [ ] ESP32 with tilt stage and stepper motor
- [ ] RPLIDAR A1 (or compatible)
- [ ] Phone with camera, gyroscope, accelerometer (Android 10+ or iOS 13+)
- [ ] Phone mount/holder for scanner front
- [ ] Home Wi-Fi network (or fallback AP on scanner)
- [ ] Desktop running Ubuntu 22.04 LTS

## Phase 0: Firmware & Environment Setup

### Tilt Stage Firmware
- [ ] Flash `firmware/tilt_stage/tilt_stage.ino` to ESP32 Nano
  - [ ] Create `firmware/tilt_stage/wifi_config.h` with home WiFi credentials
  - [ ] Verify oscillation command `W<min>,<max>,<period>` compiles
  - [ ] Test: send `W-20,20,5` (±20° over 5 s) and verify sweep in Arduino Serial Monitor

### Desktop (Ubuntu 22.04)
- [ ] Install ROS 2 Humble from https://docs.ros.org/en/humble/Installation.html
- [ ] Source ROS 2: `source /opt/ros/humble/setup.bash`
- [ ] Install build tools: `sudo apt install python3-colcon-common-extensions`
- [ ] Install dependencies:
  ```bash
  sudo apt install python3-opencv python3-numpy gstreamer1.0-tools libgstreamer1.0-dev
  pip install open3d
  ```
- [ ] Clone ros2-lvi:
  ```bash
  mkdir -p ~/dev/ros2_lvi_ws/src
  cd ~/dev/ros2_lvi_ws/src
  git clone https://github.com/ryinin/ros2-lvi.git
  cd ~/dev/ros2_lvi_ws
  colcon build --packages-up-to lvi_sam
  source install/setup.bash
  ```

### Mobile Phone
- [ ] Install Expo development client (via App Store / Play Store)
- [ ] Connect to home Wi-Fi

## Phase 1: Build ROS 2 Bridge

### lidar_ros2_bridge Package
- [ ] Build the bridge:
  ```bash
  cd ~/dev/lidarws
  colcon build --packages-up-to lidar_ros2_bridge
  source install/setup.bash
  ```
- [ ] Verify nodes are found:
  ```bash
  ros2 run lidar_ros2_bridge udp_lidar_receiver --help
  ros2 run lidar_ros2_bridge udp_imu_receiver --help
  ros2 run lidar_ros2_bridge live_cloud_publisher --help
  ros2 run lidar_ros2_bridge desktop_viewer --help
  ```

### lvi_sam_mobile Package
- [ ] Build LVI-SAM wrapper:
  ```bash
  colcon build --packages-up-to lvi_sam_mobile
  source install/setup.bash
  ```
- [ ] Verify nodes:
  ```bash
  ros2 run lvi_sam_mobile camera_bridge --help
  ros2 run lvi_sam_mobile tilt_sync --help
  ```

## Phase 2: Network Configuration

### Find IP Addresses
- [ ] **Desktop IP**: `hostname -I` → note as `<DESKTOP_IP>` (e.g., `192.168.1.50`)
- [ ] **Phone IP**: Settings → Wi-Fi → connected network → IP address → note as `<PHONE_IP>` (e.g., `192.168.1.100`)
- [ ] **Test connectivity**:
  ```bash
  ping <PHONE_IP>
  # From phone: ping desktop by IP (check in WiFi settings if available)
  ```

### Configure Launch Parameters
- [ ] Update `src/lidar_ros2_bridge/lidar_ros2_bridge/launch/mobile_lidar_bridge.launch.py`:
  - [ ] Set default `mobile_ip` to your phone's IP
  - [ ] Set default `desktop_ip` to your desktop IP

## Phase 3: Mobile App Setup

### Build Development Client
- [ ] In `mobile/` directory:
  ```bash
  npm install
  npx expo prebuild --clean
  ```
- [ ] For iOS (macOS + Xcode required):
  ```bash
  npx expo run:ios
  ```
- [ ] For Android (Android Studio + SDK):
  ```bash
  npx expo run:android
  ```
- [ ] Test on device:
  - [ ] Verify camera permission prompt
  - [ ] Verify motion sensor permission prompt
  - [ ] Test home button → "Mobile mapping" is visible

## Phase 4: Desktop Visualization Setup

### RViz2 Configuration
- [ ] The launch file includes RViz2 with pre-configured visualization
- [ ] Manual config (if needed):
  ```bash
  rviz2 -d src/lidar_ros2_bridge/lidar_ros2_bridge/config/rviz_mobile_mapping.rviz
  ```
- [ ] Verify topics appear:
  - [ ] `/lidar/points` (LiDAR stream)
  - [ ] `/lidar/mapping/cloud` (LVI-SAM output)
  - [ ] `/lidar/mapping/trajectory` (pose trajectory)

### Open3D Desktop Viewer
- [ ] Test startup (will be auto-launched):
  ```bash
  ros2 run lidar_ros2_bridge desktop_viewer
  ```
- [ ] Verify window opens with grid

## Phase 5: Integration Test (Dry Run)

### Simulation on Desktop (no phone yet)
- [ ] Launch the full pipeline:
  ```bash
  ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
    show_rviz:=true \
    show_viewer:=true
  ```
- [ ] In another terminal, verify topics:
  ```bash
  ros2 topic list
  # Should see: /lidar/points, /imu/data, /lvi_sam/cloud_registered, /lidar/mapping/cloud, /lidar/mapping/trajectory
  ```
- [ ] Verify nodes running:
  ```bash
  ros2 node list
  # Should see: udp_lidar_receiver, udp_imu_receiver, lvi_sam, live_cloud_publisher, desktop_viewer, rviz2
  ```

## Phase 6: Live Mapping Test

### Before Starting
- [ ] Phone securely mounted on scanner (camera facing forward)
- [ ] Scanner connected to WiFi (check in Nano Serial Monitor)
- [ ] Phone connected to same WiFi
- [ ] Desktop terminal ready (sourced all workspaces)

### Start Desktop Pipeline
```bash
cd ~/dev/lidarws
source install/setup.bash
source ~/dev/ros2_lvi_ws/install/setup.bash
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=<PHONE_IP> \
  show_rviz:=true \
  show_viewer:=true
```

### Start Mapping on Phone
1. [ ] Open app on phone
2. [ ] Navigate to **Home**
3. [ ] Tap **Mobile mapping** button
4. [ ] Set parameters:
   - [ ] Sweep period: 5 s (default)
   - [ ] Min angle: -20°
   - [ ] Max angle: 20°
   - [ ] Max range: 12 m
   - [ ] Phone orientation: upright (or as mounted)
5. [ ] Press **Start mapping**
6. [ ] **Wait 5 seconds** (hold still; "Hold still" banner should appear)
7. [ ] Slowly walk scanner through space, point camera forward

### Monitor Desktop
- [ ] **Terminal output**: Should show LVI-SAM tracking updates
- [ ] **RViz2**: Point cloud and trajectory appearing in real-time
- [ ] **Open3D viewer**: 3D cloud and red trajectory line visible

### Stop & Save
- [ ] Press red **Stop mapping** button on phone
- [ ] Wait for "Finishing..." message
- [ ] Choose **Save point cloud**
- [ ] Check saved files:
  - [ ] `Gallery` tab shows new scan with thumbnail
  - [ ] Verify files in app storage (share to confirm)

### Verify Output Files
On phone file system (accessible via file sharing):
- [ ] `{scan_id}_raw_cloud.ply` exists
- [ ] `{scan_id}_video.mp4` exists
- [ ] `{scan_id}_trajectory.json` exists

On desktop ROS:
```bash
ros2 topic echo /lvi_sam/cloud_registered --once | head -20
ros2 topic echo /lvi_sam/path --once | head -5
```

## Phase 7: Troubleshooting Checklist

If mapping doesn't start:
- [ ] Phone IP matches `mobile_ip` in launch command
- [ ] Desktop and phone can ping each other
- [ ] Camera and motion permissions granted on phone
- [ ] Tilt stage responding (test with Arduino Serial Monitor)

If no LiDAR data:
- [ ] UDP port 5000 not blocked: `sudo ufw allow 5000/udp`
- [ ] Check firewall logs: `sudo tail -f /var/log/ufw.log`
- [ ] Verify RPLIDAR A1 connected to Nano ESP32

If IMU data missing:
- [ ] Phone has gyroscope/accelerometer (check Sensor Box app)
- [ ] Motion permission granted (app → Settings → Permissions)

If camera frames not appearing:
- [ ] RTSP server port 8554 open (might be behind NAT)
- [ ] Test RTSP: `ffplay rtsp://<PHONE_IP>:8554/stream` (from desktop)

If desktop viewer frozen:
- [ ] Open3D installed: `pip show open3d`
- [ ] Restart viewer: `C-c` in terminal, rerun `ros2 launch ...`

If LVI-SAM not converging:
- [ ] Move slower (< 0.5 m/s)
- [ ] Ensure textured environment (not blank walls)
- [ ] Increase `featureMaxCount` in `lvism_mobile.yaml`

## Phase 8: Performance Tuning (Optional)

- [ ] Adjust cloud downsampling rate for phone FPS:
  ```bash
  ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
    ... \
    downsample_rate:=5  # More frequent updates
  ```

- [ ] Reduce LVI-SAM processing time:
  - Edit `src/lvi_sam_mobile/lvi_sam_mobile/config/lvism_mobile.yaml`
  - Lower `featureMaxCount` (e.g., 150 instead of 300)
  - Lower `baMaxIterations` (e.g., 5 instead of 10)

- [ ] Monitor CPU/memory on desktop:
  ```bash
  watch -n 1 'ps aux | grep ros2'
  ```

## Completion

- [ ] All phases complete ✓
- [ ] Live mapping tested ✓
- [ ] Point cloud and trajectory saved ✓
- [ ] Desktop and mobile displays working ✓

**Next**: Refer to [MOBILE_LVISM_GUIDE.md](MOBILE_LVISM_GUIDE.md) for advanced tuning and post-processing.

---

**Support**: Check app terminal output, ROS logs, and tilt_coordinator documentation in `src/tilt_coordinator/doc/`.
