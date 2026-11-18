"""
Live cloud publisher: subscribes to LVI-SAM output and republishes as UDP for phone.
Also publishes to ROS 2 topics for RViz2 visualization.

Subscribes to:
  - /lvi_sam/cloud_registered (PointCloud2)
  - /lvi_sam/path (nav_msgs/Path)

Publishes:
  - /lidar/mapping/cloud (downsampled PointCloud2 for phone)
  - /lidar/mapping/trajectory (nav_msgs/Path)

Sends UDP to phone at <desktop_ip>:5002 with downsampled cloud.
"""

import socket
import struct
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

CLOUD_UDP_PORT = 5002
CLOUD_DOWNSAMPLE_RATE = 10  # Keep every Nth point


class LiveCloudPublisher(Node):
    def __init__(self):
        super().__init__("live_cloud_publisher")
        self.declare_parameter("desktop_broadcast_ip", "255.255.255.255")
        self.declare_parameter("mobile_ip", "192.168.1.100")  # Will be overridden by RViz or config
        self.declare_parameter("udp_port", CLOUD_UDP_PORT)
        self.declare_parameter("downsample_rate", CLOUD_DOWNSAMPLE_RATE)

        self.mobile_ip = self.get_parameter("mobile_ip").value
        self.udp_port = self.get_parameter("udp_port").value
        self.downsample_rate = self.get_parameter("downsample_rate").value

        # Subscribe to LVI-SAM outputs
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/lvi_sam/cloud_registered", self.cloud_callback, 10
        )
        self.path_sub = self.create_subscription(Path, "/lvi_sam/path", self.path_callback, 10)

        # Republish for ROS ecosystem
        self.cloud_pub = self.create_publisher(PointCloud2, "/lidar/mapping/cloud", 10)
        self.path_pub = self.create_publisher(Path, "/lidar/mapping/trajectory", 10)

        # UDP socket for phone
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.get_logger().info(
            f"Live cloud publisher: LVI-SAM → ROS topics + UDP to {self.mobile_ip}:{self.udp_port}"
        )

    def cloud_callback(self, msg: PointCloud2):
        """Receive LVI-SAM cloud, downsample, republish locally and via UDP."""
        # Parse incoming cloud
        points = self.pointcloud2_to_xyz(msg)
        if points is None or len(points) == 0:
            return

        # Downsample
        downsampled = points[:: self.downsample_rate]

        # Republish as ROS message (for RViz2)
        output_msg = self.xyz_to_pointcloud2(downsampled, msg.header.frame_id)
        output_msg.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(output_msg)

        # Send via UDP to phone
        self.send_cloud_udp(downsampled)

        self.get_logger().debug(
            f"Cloud: {len(points)} → {len(downsampled)} points, sent to {self.mobile_ip}:{self.udp_port}"
        )

    def path_callback(self, msg: Path):
        """Republish trajectory."""
        self.path_pub.publish(msg)
        self.get_logger().debug(f"Path: {len(msg.poses)} poses")

    def pointcloud2_to_xyz(self, cloud: PointCloud2) -> Optional[np.ndarray]:
        """Convert PointCloud2 to numpy Nx3 array."""
        points = np.frombuffer(cloud.data, dtype=np.float32).reshape(-1, 3)
        return points

    def xyz_to_pointcloud2(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        """Convert Nx3 numpy array to PointCloud2."""
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.astype(np.float32).tobytes()

        return msg

    def send_cloud_udp(self, points: np.ndarray):
        """Send downsampled cloud as UDP binary."""
        # Format: N (u32) followed by N*3 float32 values
        payload = struct.pack("<I", len(points))
        payload += points.astype(np.float32).tobytes()

        try:
            self.udp_socket.sendto(payload, (self.mobile_ip, self.udp_port))
        except OSError as e:
            self.get_logger().warn(f"Failed to send UDP: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LiveCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.udp_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
