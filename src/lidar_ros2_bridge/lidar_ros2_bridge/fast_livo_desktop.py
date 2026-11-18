"""Receive the Android mapping stream and publish FAST-LIVO2 sensor messages."""

import math
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header


FRAGMENT_HEADER = struct.Struct('<IQIII')
FRAGMENT_SIZE = 1200
MAX_MESSAGE_SIZE = 10 * 1024 * 1024
OUSTER_POINT = struct.Struct('<ffffIHBBHxxI')


class FragmentReassembler:
    def __init__(self):
        self.pending = {}

    def expire(self):
        now = time.monotonic()
        expired = [key for key, entry in self.pending.items() if now - entry['started'] >= 5]
        for key in expired:
            del self.pending[key]
        return len(expired)

    def accept(self, data, address):
        now = time.monotonic()
        if len(data) < 4 or struct.unpack_from('<I', data)[0] != 4:
            return data
        if len(data) < FRAGMENT_HEADER.size:
            raise ValueError('Truncated fragment header')
        _, message_id, index, count, total = FRAGMENT_HEADER.unpack_from(data)
        expected_count = (total + FRAGMENT_SIZE - 1) // FRAGMENT_SIZE
        if not (FRAGMENT_SIZE < total <= MAX_MESSAGE_SIZE and count == expected_count and index < count):
            raise ValueError('Invalid fragment metadata')
        part = data[FRAGMENT_HEADER.size:]
        if len(part) != min(FRAGMENT_SIZE, total - index * FRAGMENT_SIZE):
            raise ValueError('Invalid fragment length')
        key = (address, message_id)
        entry = self.pending.get(key)
        if entry is None:
            if len(self.pending) >= 8:
                raise ValueError('Too many incomplete messages')
            entry = {'started': now, 'total': total, 'count': count, 'parts': {}}
            self.pending[key] = entry
        if entry['total'] != total or entry['count'] != count:
            del self.pending[key]
            raise ValueError('Conflicting fragment metadata')
        previous = entry['parts'].get(index)
        if previous is not None and previous != part:
            del self.pending[key]
            raise ValueError('Conflicting duplicate fragment')
        entry['parts'][index] = part
        if len(entry['parts']) != count:
            return None
        result = b''.join(entry['parts'][i] for i in range(count))
        del self.pending[key]
        return result


def decode_timed_points(payload):
    if len(payload) < 4:
        raise ValueError('Missing LiDAR point count')
    count = struct.unpack_from('<I', payload)[0]
    if count == 0 or len(payload) != 4 + count * 16:
        raise ValueError('Invalid LiDAR point count or payload length')
    points = []
    previous = -1
    for x, y, z, offset in struct.iter_unpack('<fffI', payload[4:]):
        if not all(math.isfinite(value) for value in (x, y, z)) or offset < previous:
            raise ValueError('Invalid LiDAR coordinates or timestamps')
        points.append((x, y, z, offset))
        previous = offset
    return points


class FastLivoDesktopBridge(Node):
    def __init__(self):
        super().__init__('fast_livo_desktop_bridge')
        self.declare_parameter('udp_port', 5005)
        port = self.get_parameter('udp_port').value
        self.lidar = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.imu = self.create_publisher(Imu, '/livox/imu', 100)
        self.image = self.create_publisher(Image, '/left_camera/image', 10)
        self.cv_bridge = CvBridge()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.socket.bind(('0.0.0.0', port))
        self.socket.settimeout(1.0)
        self.counts = [0, 0, 0]
        self.running = True
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self.statistics)
        self.get_logger().info(f'Listening on UDP {port}; use only on a trusted network')

    def receive(self):
        reassembler = FragmentReassembler()
        while self.running:
            try:
                data, address = self.socket.recvfrom(65535)
                expired = reassembler.expire()
                if expired:
                    self.get_logger().warning(f'Discarded {expired} incomplete sensor message(s)')
                if data == b'LIVO_HELLO':
                    self.socket.sendto(b'LIVO_OK', address)
                    continue
                message = reassembler.accept(data, address)
                if message is not None:
                    self.publish_message(message)
            except socket.timeout:
                expired = reassembler.expire()
                if expired:
                    self.get_logger().warning(f'Discarded {expired} incomplete sensor message(s)')
                continue
            except (ValueError, OverflowError, struct.error, cv2.error, CvBridgeError) as error:
                self.get_logger().warning(f'Invalid sensor message: {error}')
            except OSError as error:
                if self.running:
                    self.get_logger().error(f'UDP receive failed: {error}')
                    break

    @staticmethod
    def header(timestamp, frame_id):
        if not math.isfinite(timestamp) or timestamp < 0:
            raise ValueError('Invalid timestamp')
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1_000_000_000)
        header = Header()
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        header.frame_id = frame_id
        return header

    def publish_message(self, message):
        if len(message) < 12:
            raise ValueError('Truncated sensor message')
        kind, timestamp = struct.unpack_from('<Id', message)
        payload = message[12:]
        if kind == 3:
            if len(payload) != 24:
                raise ValueError('Invalid IMU payload length')
            values = struct.unpack('<ffffff', payload)
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Invalid IMU values')
            msg = Imu()
            msg.header = self.header(timestamp, 'imu')
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values[:3]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values[3:]
            self.imu.publish(msg)
            self.counts[0] += 1
        elif kind == 5:
            image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Invalid JPEG frame')
            msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header = self.header(timestamp, 'camera')
            self.image.publish(msg)
            if self.counts[1] == 0:
                self.get_logger().info(f'Camera frames: {msg.width}x{msg.height}; calibrate at this size and orientation')
            self.counts[1] += 1
        elif kind == 6:
            points = decode_timed_points(payload)
            msg = PointCloud2()
            msg.header = self.header(timestamp, 'lidar')
            msg.height = 1
            msg.width = len(points)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='t', offset=16, datatype=PointField.UINT32, count=1),
                PointField(name='reflectivity', offset=20, datatype=PointField.UINT16, count=1),
                PointField(name='ring', offset=22, datatype=PointField.UINT8, count=1),
                PointField(name='ambient', offset=24, datatype=PointField.UINT16, count=1),
                PointField(name='range', offset=28, datatype=PointField.UINT32, count=1),
            ]
            msg.point_step = OUSTER_POINT.size
            msg.row_step = msg.point_step * msg.width
            msg.is_dense = True
            msg.data = b''.join(OUSTER_POINT.pack(x, y, z, 0, offset, 0, 0, 0, 0, 0)
                                for x, y, z, offset in points)
            self.lidar.publish(msg)
            self.counts[2] += 1
        else:
            raise ValueError(f'Unsupported sensor message type {kind}')

    def statistics(self):
        self.get_logger().info(
            f'IMU: {self.counts[0]}, images: {self.counts[1]}, LiDAR scans: {self.counts[2]}'
        )

    def destroy_node(self):
        self.running = False
        self.socket.close()
        self.thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FastLivoDesktopBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
