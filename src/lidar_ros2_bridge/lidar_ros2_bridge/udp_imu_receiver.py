"""
UDP receiver for phone IMU (accelerometer + gyroscope).
Converts UDP packets to ROS 2 sensor_msgs/Imu.

Phone sends IMU samples @20 Hz on UDP port 5001.
Packet format: timestamp (f64), accel_x/y/z (f32 x3), gyro_x/y/z (f32 x3)
"""

import socket
import struct
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

IMU_FRAME_PORT = 5001
IMU_SAMPLE_SIZE = 8 + 12 + 12  # timestamp (f64) + accel (3x f32) + gyro (3x f32)


class UDPIMUReceiver(Node):
    def __init__(self):
        super().__init__("udp_imu_receiver")
        self.declare_parameter("imu_port", IMU_FRAME_PORT)
        self.declare_parameter("frame_id", "phone_imu")

        self.imu_port = self.get_parameter("imu_port").value
        self.frame_id = self.get_parameter("frame_id").value

        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)

        # Open UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.imu_port))
        self.socket.setblocking(False)

        self.seq = 0
        self.create_timer(0.01, self.recv_callback)
        self.get_logger().info(f"UDP IMU receiver listening on port {self.imu_port}")

    def recv_callback(self):
        try:
            data, addr = self.socket.recvfrom(4096)
        except BlockingIOError:
            return

        # Parse IMU samples
        offset = 0
        while offset + IMU_SAMPLE_SIZE <= len(data):
            try:
                ts, ax, ay, az, gx, gy, gz = struct.unpack(
                    "<dfffffff", data[offset : offset + IMU_SAMPLE_SIZE]
                )
                offset += IMU_SAMPLE_SIZE

                # Publish IMU message
                msg = Imu()
                msg.header.seq = self.seq
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                # Accelerometer (m/s²)
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az

                # Gyroscope (rad/s)
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz

                # Covariance (diagonal)
                msg.linear_acceleration_covariance = [0.01] * 9
                msg.angular_velocity_covariance = [0.001] * 9
                msg.orientation_covariance = [-1.0] * 9  # Orientation not provided

                self.imu_pub.publish(msg)
                self.seq += 1

            except struct.error:
                break


def main(args=None):
    rclpy.init(args=args)
    node = UDPIMUReceiver()
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
