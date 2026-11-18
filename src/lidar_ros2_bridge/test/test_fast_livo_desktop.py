import socket
import struct
import time
import unittest

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from lidar_ros2_bridge.fast_livo_desktop import (
    FastLivoDesktopBridge,
    FragmentReassembler,
    decode_timed_points,
)


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TransportTests(unittest.TestCase):
    def test_fragment_reassembly_rejects_conflicting_parts(self):
        message = b'x' * 1201
        prefix = struct.pack('<IQIII', 4, 17, 0, 2, len(message))
        reassembler = FragmentReassembler()
        address = ('127.0.0.1', 42)
        self.assertIsNone(reassembler.accept(prefix + message[:1200], address))
        with self.assertRaises(ValueError):
            reassembler.accept(prefix + b'y' * 1200, address)

    def test_invalid_point_times_are_rejected(self):
        payload = struct.pack('<IfffIfffI', 2, 1, 2, 3, 10, 4, 5, 6, 9)
        with self.assertRaises(ValueError):
            decode_timed_points(payload)
        with self.assertRaises(ValueError):
            decode_timed_points(payload[:-1])

    def test_udp_stream_publishes_actual_ros_message_types(self):
        rclpy.init(args=['--ros-args', '-p', 'udp_port:=0'])
        node = FastLivoDesktopBridge()
        node.imu, node.image, node.lidar = Collector(), Collector(), Collector()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.settimeout(2)
        destination = ('127.0.0.1', node.socket.getsockname()[1])
        try:
            sender.sendto(b'LIVO_HELLO', destination)
            self.assertEqual(sender.recv(64), b'LIVO_OK')
            sender.sendto(struct.pack('<Idffffff', 3, 100.25, 1, 2, 3, 4, 5, 6), destination)
            image = np.random.default_rng(42).integers(0, 256, (64, 64, 3), dtype=np.uint8)
            success, jpeg = cv2.imencode('.jpg', image)
            self.assertTrue(success)
            image_packet = struct.pack('<Id', 5, 100.5) + jpeg.tobytes()
            count = (len(image_packet) + 1199) // 1200
            self.assertGreater(count, 1)
            for index in reversed(range(count)):
                fragment = struct.pack('<IQIII', 4, 17, index, count, len(image_packet))
                sender.sendto(fragment + image_packet[index * 1200:(index + 1) * 1200], destination)
            lidar = struct.pack('<IdIfffIfffI', 6, 100.0, 2, 1, -2, -3, 0, 4, -5, -6, 20_000_000)
            sender.sendto(lidar, destination)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and not all((node.imu.messages, node.image.messages, node.lidar.messages)):
                time.sleep(0.01)
            self.assertEqual(len(node.imu.messages), 1)
            self.assertEqual(len(node.image.messages), 1)
            self.assertEqual(len(node.lidar.messages), 1)
            self.assertEqual(node.imu.messages[0].angular_velocity.z, 6)
            self.assertEqual((node.image.messages[0].width, node.image.messages[0].height), (64, 64))
            scan = node.lidar.messages[0]
            self.assertEqual(scan.width, 2)
            self.assertEqual(scan.point_step, 32)
            self.assertEqual(struct.unpack_from('<I', scan.data, 32 + 16)[0], 20_000_000)
            self.assertEqual(struct.unpack_from('<f', scan.data, 4)[0], -2)
            self.assertEqual([(field.name, field.offset, field.datatype) for field in scan.fields], [
                ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                ('z', 8, PointField.FLOAT32), ('intensity', 12, PointField.FLOAT32),
                ('t', 16, PointField.UINT32), ('reflectivity', 20, PointField.UINT16),
                ('ring', 22, PointField.UINT8), ('ambient', 24, PointField.UINT16),
                ('range', 28, PointField.UINT32),
            ])
        finally:
            sender.close()
            node.destroy_node()
            rclpy.shutdown()

    def test_timed_scan_reaches_a_pointcloud_ros_subscriber(self):
        rclpy.init(args=['--ros-args', '-p', 'udp_port:=0'])
        node = FastLivoDesktopBridge()
        received = []
        subscriber = node.create_subscription(PointCloud2, '/livox/lidar', received.append, 10)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        destination = ('127.0.0.1', node.socket.getsockname()[1])
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and node.count_subscribers('/livox/lidar') == 0:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(node.count_subscribers('/livox/lidar'), 0)
            packet = struct.pack('<IdIfffIfffI', 6, 100.0, 2, 1, 2, 3, 0, 4, 5, 6, 20_000_000)
            sender.sendto(packet, destination)
            while time.monotonic() < deadline and not received:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual(len(received), 1)
            self.assertEqual(struct.unpack_from('<I', received[0].data, 32 + 16)[0], 20_000_000)
        finally:
            sender.close()
            node.destroy_subscription(subscriber)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
