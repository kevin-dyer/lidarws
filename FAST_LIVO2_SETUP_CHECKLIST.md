re43555555555555555555555555555555555555555fdc# FAST-LIVO2 Setup Checklist

## Pre-Launch Verification

### System Requirements
- [ ] ROS 2 Humble installed and working
- [ ] Both workspaces built:
  - [ ] `~/dev/lidarws` with lidar_ros2_bridge + lvi_sam_mobile
  - [ ] `~/dev/ros2_fast_livo_ws` with FAST-LIVO2
- [ ] RPLIDAR 3D scanner connected to phone
- [ ] Phone can run WiFi hotspot or on local network

### Network Setup
- [ ] Mobile device IP address known (e.g., 192.168.1.100)
- [ ] Mobile device is on same WiFi network as desktop
- [ ] Desktop can ping mobile device: `ping 192.168.1.100`
- [ ] Firewall allows UDP ports 5000, 5001 and TCP 8554
- [ ] Mobile app is running LiDAR scanner broadcast

### ROS 2 Setup
- [ ] `source ~/.bashrc` to load workspace paths
- [ ] Both workspaces in `~/.bashrc` OR manually source:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/dev/lidarws/install/setup.bash
  source ~/dev/ros2_fast_livo_ws/install/setup.bash
  ```

## Build Verification

### lidarws Build
```bash
cd ~/dev/lidarws
colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile
```
- [ ] Build completed successfully (0 errors)
- [ ] `install/lib/python*/site-packages/lidar_ros2_bridge/fast_livo_adapter.py` exists
- [ ] `install/share/lidar_ros2_bridge/launch/fast_livo_mobile.launch.py` exists

### FAST-LIVO2 Build
```bash
cd ~/dev/ros2_fast_livo_ws
colcon build --packages-select fast_livo
```
- [ ] Build completed successfully (0 errors)
- [ ] `install/lib/fast_livo/fastlivo_mapping` executable exists

## Pre-Launch Configuration

### Environment Variables
```bash
# Verify workspace paths
echo $ROS_PACKAGE_PATH | grep -q "lidarws" && echo "✓ lidarws in path" || echo "✗ Missing"
echo $ROS_PACKAGE_PATH | grep -q "ros2_fast_livo_ws" && echo "✓ FAST-LIVO2 in path" || echo "✗ Missing"
```
- [ ] Both workspaces appear in `ROS_PACKAGE_PATH`

### File Permissions
```bash
# Check adapter is executable
ls -la ~/dev/lidarws/install/lib/python*/site-packages/lidar_ros2_bridge/fast_livo_adapter.py
```
- [ ] File is readable and in Python site-packages

## Launch Command Preparation

### Option 1: All-in-One Launch
```bash
# Replace IP with your actual mobile device IP
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py \
  mobile_ip:=192.168.1.100 \
  use_rviz:=True
```
- [ ] Command is ready to copy/paste
- [ ] Mobile IP is correct (not placeholder)

### Option 2: Manual Launch (3 terminals)
**Terminal 1:**
```bash
cd ~/dev/lidarws
source install/setup.bash
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \
  show_rviz:=false \
  show_viewer:=false
```
- [ ] Command ready
- [ ] Mobile IP is correct

**Terminal 2:**
```bash
cd ~/dev/lidarws
source install/setup.bash
ros2 run lidar_ros2_bridge fast_livo_adapter
```
- [ ] Command ready to paste

**Terminal 3:**
```bash
cd ~/dev/ros2_fast_livo_ws
source install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```
- [ ] Command ready to paste

## Launch Execution

### Starting Services
- [ ] Mobile app is running and broadcasting LiDAR data
- [ ] Mobile app is streaming camera video (RTSP)
- [ ] Mobile device is reachable on network

### Initial Startup
1. [ ] Start terminal 1 (or all-in-one launch)
   - Watch for: "UDP LiDAR receiver listening on port 5000"
   - Watch for: "UDP IMU receiver listening on port 5001"
   - Watch for: "Camera bridge listening on rtsp://..."

2. [ ] If using manual launch, start terminal 2
   - Watch for: "FAST-LIVO2 adapter started"

3. [ ] If using manual launch, start terminal 3
   - Watch for: "laserMapping" node starting

### RViz2 Initialization
- [ ] RViz2 window opens
- [ ] Default visualization appears (empty world)
- [ ] No critical errors in RViz console

## Data Flow Verification

### Topic Checking (in new terminal)
```bash
# List all topics
ros2 topic list
```
- [ ] `/livox/lidar` appears in list
- [ ] `/livox/imu` appears in list
- [ ] `/left_camera/image` appears in list
- [ ] `/odom` appears in list (from FAST-LIVO2)

### Topic Activity (in new terminal)
```bash
# Check LiDAR rate
ros2 topic hz /livox/lidar
```
Expected: "rate: 30-40 Hz" or similar
- [ ] Rate is non-zero
- [ ] Rate is reasonable (>10 Hz)

```bash
# Check IMU rate
ros2 topic hz /livox/imu
```
Expected: "rate: 20.00 Hz"
- [ ] Rate is approximately 20 Hz
- [ ] Rate is consistent

```bash
# Check camera rate
ros2 topic hz /left_camera/image
```
Expected: "rate: 20-30 Hz"
- [ ] Rate is non-zero
- [ ] Rate is reasonable (>10 Hz)

### Topic Content Inspection
```bash
# Sample one LiDAR message
ros2 topic echo /livox/lidar | head -20
```
- [ ] Shows PointCloud2 message with `width`, `height`, `fields`
- [ ] Data section contains binary point data

```bash
# Sample one IMU message
ros2 topic echo /livox/imu | head -20
```
- [ ] Shows Imu message with accelerometer and gyroscope values
- [ ] Timestamp is recent (within last second)

## Visual Confirmation

### RViz2 Visualization
- [ ] Point cloud appears in viewport (white/colored points)
- [ ] Point cloud is moving/updating (not static)
- [ ] Trajectory line appears and extends (camera path)
- [ ] Coordinate frame axes visible (red=X, green=Y, blue=Z)

### Terminal Output
- [ ] No error messages in launch terminal
- [ ] No warnings about missing topics (or minimal warnings)
- [ ] FAST-LIVO2 printing status updates (frame numbers, processing time)

## Performance Monitoring

### CPU/Memory (in new terminal)
```bash
# Watch resource usage
ros2 run ros2_simple_monitor ros2_simple_monitor
# OR
top
```
- [ ] FAST-LIVO2 CPU usage: 20-60%
- [ ] Memory usage: stable, not growing unbounded

### Network (if needed)
```bash
# Check WiFi signal
iwconfig  # Linux
netstat -s  # Check packet loss
```
- [ ] No excessive packet loss (>1%)
- [ ] WiFi signal strength good

## Data Recording (Optional)

```bash
# Record all topics to ROS bag
ros2 bag record -a -o my_run_$(date +%Y%m%d_%H%M%S)
```
- [ ] Rosbag file created and growing
- [ ] Stop with Ctrl+C when done

## Troubleshooting If Issues Occur

### No topics appearing
- [ ] Check mobile device IP in launch command
- [ ] Verify mobile app is running
- [ ] Check network connectivity: `ping 192.168.1.100`

### Topics appear but no data flow
- [ ] Verify adapter is running: `ros2 node list | grep adapter`
- [ ] Check FAST-LIVO2 console for errors
- [ ] Verify FAST-LIVO2 config points to correct topics (`avia.yaml`)

### RViz2 shows no map
- [ ] Ensure `/cloud_pcd` topic is added to RViz displays
- [ ] Check FAST-LIVO2 output in console (should show frame numbers)
- [ ] Verify `dense_map_en: true` in `avia.yaml`

### Poor map quality or no tracking
- [ ] Check camera-LiDAR calibration (extrinsic parameters in `avia.yaml`)
- [ ] Verify sensor moved during scan (not staying still)
- [ ] Check lighting (well-lit environment preferred for visual tracking)
- [ ] Consider adjusting `scan_line` parameter if using non-standard LiDAR

## Success Indicators

✅ **You'll know it's working when:**

1. **Topics are active** (`ros2 topic hz` returns non-zero rates)
2. **Point cloud updates** in RViz2 shows growing map
3. **Trajectory extends** as you move the scanner
4. **Console shows progress** like "Frame 123: tracking successful"
5. **No errors** in any terminal windows
6. **Responsive** (not frozen or stuttering)

## Post-Launch Activities

### If successful, you can:
- [ ] Record data for offline analysis
- [ ] Export point cloud map
- [ ] Adjust parameters in `avia.yaml` for better results
- [ ] Integrate with other ROS 2 nodes

### If issues occur, review:
- [ ] Terminal output for specific error messages
- [ ] Topic connectivity: `ros2 topic info /livox/lidar`
- [ ] Node status: `ros2 node info lvi_sam` (or node name)
- [ ] See FAST_LIVO2_INTEGRATION.md Troubleshooting section

## Files Referenced

📂 Location of key files:
```
~/dev/lidarws/
├── src/lidar_ros2_bridge/
│   ├── fast_livo_adapter.py          ← Topic adapter
│   ├── launch/fast_livo_mobile.launch.py  ← Main launch file
│   └── setup.py                      ← Updated entry points
├── FAST_LIVO2_INTEGRATION.md         ← Full setup guide
├── FAST_LIVO2_TOPIC_MAP.md           ← Topic specs
├── FAST_LIVO2_DATA_FLOW.txt          ← Data flow diagram
├── FAST_LIVO2_QUICK_COMMANDS.sh      ← Quick commands
└── FAST_LIVO2_SETUP_CHECKLIST.md     ← This file

~/dev/ros2_fast_livo_ws/
└── src/FAST-LIVO2/
    └── config/avia.yaml              ← FAST-LIVO2 config
```

## Quick Command Reference

```bash
# Full build
cd ~/dev/lidarws && colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile

# Source workspaces
source ~/.bashrc

# One-command launch
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100 use_rviz:=True

# Verify topics
ros2 topic list | grep -E "livox|left_camera"
ros2 topic hz /livox/lidar

# Debug adapter
ros2 node list | grep adapter

# Check FAST-LIVO2 config
cat ~/dev/ros2_fast_livo_ws/src/FAST-LIVO2/config/avia.yaml
```

---

**Next Step:** Once all checkboxes are complete, you're ready to launch!
