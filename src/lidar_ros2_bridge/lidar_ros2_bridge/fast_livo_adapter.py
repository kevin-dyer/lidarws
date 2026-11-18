"""
Topic adapter: republishes mobile bridge data to FAST-LIVO2 expected topics.

Subscribes to:
  - /lidar/points (PointCloud2 from mobile RPLIDAR)
  - /imu/data (Imu from mobile phone)
  - /camera/image_raw (Image from phone camera)

Publishes to:
  - /livox/lidar (PointCloud2 for FAST-LIVO2)
  - /livox/imu (Imu for FAST-LIVO2)
  - /left_camera/image (Image for FAST-LIVO2)

This allows running FAST-LIVO2 with your mobile scanner setup without
modifying the underlying mobile bridge or FAST-LIVO2 config.

Usage:
  ros2 run lidar_ros2_bridge fast_livo_adapter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image


class FastLivoAdapter(Node):
    def __init__(self):
        super().__init__("fast_livo_adapter")

        # Subscribe to mobile bridge topics
        self.lidar_sub = self.create_subscription(
            PointCloud2, "/lidar/points", self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10
        )

        # Publish to FAST-LIVO2 topics
        self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", 10)
        self.camera_pub = self.create_publisher(Image, "/left_camera/image", 10)

        self.get_logger().info(
            "FAST-LIVO2 adapter started. Republishing mobile bridge data to Livox topics."
        )

    def lidar_callback(self, msg: PointCloud2):
        """Forward LiDAR points to FAST-LIVO2 topic."""
        self.lidar_pub.publish(msg)

    def imu_callback(self, msg: Imu):
        """Forward IMU data to FAST-LIVO2 topic."""
        self.imu_pub.publish(msg)

    def camera_callback(self, msg: Image):
        """Forward camera image to FAST-LIVO2 topic."""
        self.camera_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FastLivoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
