# FAST-LIVO2 Topic Mapping Reference

## Complete Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Mobile Device (Phone)                            │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────────────┐   │
│  │ LiDAR Scanner│  │ Accelerometer│  │  Camera (RTSP Server)       │   │
│  │  (RPLIDAR)   │  │ + Gyroscope  │  │  :8554/stream               │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┬──────────────┘   │
│         │                 │                        │                    │
│         │ UDP:5000        │ UDP:5001               │ RTSP TCP:8554     │
│         │ 120° scanner    │ 20 Hz                  │ H.264 encoded     │
└─────────┼─────────────────┼────────────────────────┼────────────────────┘
          │                 │                        │
          │ Network         │ Network                │ Network
          │                 │                        │
┌─────────▼─────────────────▼────────────────────────▼────────────────────┐
│                      Desktop / ROS System                                │
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Mobile Bridge (UDP Receivers)                                  │    │
│  │  └──────────────────────────────────────────────────────────┐   │    │
│  │    ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐ │    │
│  │    │udp_lidar     │  │udp_imu       │  │camera_bridge     │ │    │
│  │    │_receiver     │  │_receiver     │  │(RTSP → ROS)      │ │    │
│  │    └────────┬─────┘  └────────┬─────┘  └────────┬─────────┘ │    │
│  │             │                 │                 │            │    │
│  │    /lidar/points       /imu/data         /camera/image_raw   │    │
│  │    (PointCloud2)         (Imu)              (Image)          │    │
│  │             │                 │                 │            │    │
│  │  └──────────┴─────────────────┴─────────────────┴────────┘   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                              ▼                                           │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Topic Adapter (fast_livo_adapter)                              │    │
│  │  ┌──────────────────────────────────────────────────────────┐   │    │
│  │  │ Subscribe from mobile bridge:                            │   │    │
│  │  │   /lidar/points → /livox/lidar                           │   │    │
│  │  │   /imu/data → /livox/imu                                 │   │    │
│  │  │   /camera/image_raw → /left_camera/image                 │   │    │
│  │  └──────────────────────────────────────────────────────────┘   │    │
│  │                              ▼                                    │    │
│  │  ┌──────────────────────────────────────────────────────────┐   │    │
│  │  │ Publish to FAST-LIVO2:                                   │   │    │
│  │  │   /livox/lidar (PointCloud2)                             │   │    │
│  │  │   /livox/imu (Imu)                                       │   │    │
│  │  │   /left_camera/image (Image)                             │   │    │
│  │  └──────────────────────────────────────────────────────────┘   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                              ▼                                           │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  FAST-LIVO2 (fastlivo_mapping)                                  │    │
│  │  LiDAR-Inertial-Visual SLAM                                     │    │
│  │  ┌──────────────────────────────────────────────────────────┐   │    │
│  │  │ Inputs:                                                  │   │    │
│  │  │   /livox/lidar ← Point clouds                            │   │    │
│  │  │   /livox/imu ← Inertial measurements                     │   │    │
│  │  │   /left_camera/image ← Visual features                   │   │    │
│  │  │                                                          │   │    │
│  │  │ Outputs:                                                │   │    │
│  │  │   /tf (camera ↔ world transforms)                        │   │    │
│  │  │   /odom (visual-inertial odometry)                       │   │    │
│  │  │   /cloud_pcd (dense map)                                 │   │    │
│  │  │   /key_points (feature tracking)                         │   │    │
│  │  └──────────────────────────────────────────────────────────┘   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                              ▼                                           │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  RViz2 Visualization                                            │    │
│  │  Display: 3D map, trajectory, camera pose, point clouds        │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

## Topic Specification Table

| Purpose | Mobile Bridge | Adapter (In) | Adapter (Out) | FAST-LIVO2 | Message Type | Rate |
|---------|---|---|---|---|---|---|
| **LiDAR Points** | `/lidar/points` | ✓ | ✓ | `/livox/lidar` | `sensor_msgs/PointCloud2` | Variable |
| **Inertial Data** | `/imu/data` | ✓ | ✓ | `/livox/imu` | `sensor_msgs/Imu` | ~20 Hz |
| **Camera Image** | `/camera/image_raw` | ✓ | ✓ | `/left_camera/image` | `sensor_msgs/Image` | ~20-30 Hz |
| **Tilt Angle** | `/tilt/angle` | ✗ | ✗ | ✗ | `std_msgs/Float32` | Variable |

## PointCloud2 Details

```yaml
Topic: /livox/lidar
Message Type: sensor_msgs/PointCloud2

Header:
  frame_id: "lidar"
  stamp: <ROS time>

Data Layout:
  width: <number of points>
  height: 1
  point_step: 12 bytes (x, y, z as float32)
  row_step: width * 12
  fields:
    - name: "x", offset: 0, datatype: FLOAT32
    - name: "y", offset: 4, datatype: FLOAT32
    - name: "z", offset: 8, datatype: FLOAT32

Coordinate System:
  Origin: LiDAR scanner head
  X-axis: Forward
  Y-axis: Left
  Z-axis: Up
```

## IMU Details

```yaml
Topic: /livox/imu
Message Type: sensor_msgs/Imu

Header:
  frame_id: "phone_imu"
  stamp: <ROS time>

Orientation: NOT USED (covariance = -1)
Angular Velocity: (rad/s)
  x, y, z: Gyroscope output
Linear Acceleration: (m/s²)
  x, y, z: Accelerometer output

Covariances:
  angular_velocity_covariance: 0.001 (diagonal)
  linear_acceleration_covariance: 0.01 (diagonal)
  orientation_covariance: -1 (not provided)
```

## Image Details

```yaml
Topic: /left_camera/image
Message Type: sensor_msgs/Image

Header:
  frame_id: "camera"
  stamp: <ROS time>

Data:
  height: <image height in pixels>
  width: <image width in pixels>
  encoding: "rgb8"
  is_bigendian: false
  step: width * 3  (3 channels: R, G, B)
  data: <raw pixel bytes>
```

## Launch File Remapping

**In fast_livo_mobile.launch.py:**
```python
# No explicit remapping needed - adapter handles it
# But you can verify the names in each node config:

# Mobile bridge publishes:
self.lidar_pub = self.create_publisher(PointCloud2, "lidar/points", 10)
self.imu_pub = self.create_publisher(Imu, "imu/data", 10)

# Camera bridge publishes:
self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)

# Adapter subscribes:
self.lidar_sub = self.create_subscription(PointCloud2, "/lidar/points", ...)
self.imu_sub = self.create_subscription(Imu, "/imu/data", ...)
self.camera_sub = self.create_subscription(Image, "/camera/image_raw", ...)

# Adapter publishes:
self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)
self.imu_pub = self.create_publisher(Imu, "/livox/imu", 10)
self.camera_pub = self.create_publisher(Image, "/left_camera/image", 10)
```

## Configuration Tuning

FAST-LIVO2 expects specific sensor specifications. Check your `avia.yaml`:

```yaml
common:
  img_topic: "/left_camera/image"      # ← Must match adapter output
  lid_topic: "/livox/lidar"             # ← Must match adapter output
  imu_topic: "/livox/imu"               # ← Must match adapter output
  img_en: 1                             # Enable visual processing
  lidar_en: 1                           # Enable LiDAR processing

extrin_calib:
  extrinsic_T: [0.04165, 0.02326, -0.0284]  # Camera → LiDAR offset (meters)
  extrinsic_R: [...]                         # Camera → LiDAR rotation matrix
  Rcl: [...]                                 # Optional camera distortion
  Pcl: [...]                                 # Optional camera principal point

preprocess:
  lidar_type: 1                    # Type: 1=Livox Avia, adjust if needed
  scan_line: 6                     # Number of laser lines (RPLIDAR: 8-16)
  point_filter_num: 1              # Skip every Nth point (1=no skip)
```

## Debugging Checklist

- [ ] Both workspaces source in shell
- [ ] `ros2 topic list` shows all 6 topics active
- [ ] `ros2 topic hz /livox/lidar` > 0 Hz
- [ ] `ros2 topic hz /livox/imu` ≈ 20 Hz
- [ ] `ros2 topic hz /left_camera/image` > 0 Hz
- [ ] FAST-LIVO2 config has correct topic names
- [ ] `lidar_type` matches your scanner model
- [ ] Camera-LiDAR calibration (`extrinsic_T`, `extrinsic_R`) is reasonable

