"""
UDP receiver for LiDAR frames.
Converts RPLIDAR UDP packets to ROS 2 PointCloud2.

Phone sends raw LiDAR frames (from scanner) + tilt angle on UDP port 5000.
"""

import socket
import struct
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
import io

RPLIDAR_FRAME_PORT = 5000
RPLIDAR_FRAME_FORMAT = "IHHHHH"  # time_us (u32), quality, angle, distance x4


class UDPLidarReceiver(Node):
    def __init__(self):
        super().__init__("udp_lidar_receiver")
        self.declare_parameter("lidar_port", RPLIDAR_FRAME_PORT)
        self.declare_parameter("frame_id", "lidar")

        self.lidar_port = self.get_parameter("lidar_port").value
        self.frame_id = self.get_parameter("frame_id").value

        self.lidar_pub = self.create_publisher(PointCloud2, "lidar/points", 10)
        self.tilt_pub = self.create_publisher(Float32, "tilt/angle", 10)

        # Open UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.lidar_port))
        self.socket.setblocking(False)

        self.create_timer(0.01, self.recv_callback)
        self.get_logger().info(f"UDP LiDAR receiver listening on port {self.lidar_port}")

    def recv_callback(self):
        try:
            data, addr = self.socket.recvfrom(4096)
        except BlockingIOError:
            return

        # Parse payload: first 4 bytes = tilt angle (float32), rest = RPLIDAR frames
        if len(data) < 4:
            return

        tilt_rad = struct.unpack("<f", data[0:4])[0]
        self.publish_tilt(tilt_rad)

        # Parse remaining data as RPLIDAR frames
        offset = 4
        points = []

        while offset + 12 <= len(data):
            try:
                frame = struct.unpack("<" + RPLIDAR_FRAME_FORMAT, data[offset : offset + 12])
                time_us, quality, angle, d0, d1, d2 = frame
                offset += 12

                # Unpack distance: each is 15 bits value + 1 bit quality flag
                for d_packed in [d0, d1, d2]:
                    dist_mm = (d_packed >> 2) & 0x3FFF
                    if dist_mm > 0:
                        angle_rad = np.radians(angle / 64.0)
                        x = dist_mm * np.cos(angle_rad) / 1000.0
                        y = dist_mm * np.sin(angle_rad) / 1000.0
                        z = 0.0
                        points.append([x, y, z])

            except struct.error:
                break

        if points:
            self.publish_cloud(np.array(points, dtype=np.float32))

    def publish_tilt(self, tilt_rad: float):
        msg = Float32()
        msg.data = tilt_rad
        self.tilt_pub.publish(msg)

    def publish_cloud(self, points: np.ndarray):
        """Convert numpy points array to PointCloud2 message."""
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = self.frame_id
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True

        # Define fields
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

        # Flatten to bytes
        cloud_msg.data = points.astype(np.float32).tobytes()
        self.lidar_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UDPLidarReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
