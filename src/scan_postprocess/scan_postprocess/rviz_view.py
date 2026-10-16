"""Publishes point clouds to RViz (PointCloud2 on a latched topic) and optionally launches RViz."""
from __future__ import annotations

import os
import subprocess
import time
from typing import Optional, Sequence, Tuple

import numpy as np

TOPIC = '/registration/cloud'
MESH_TOPIC = '/registration/mesh'
FRAME_ID = 'map'
MAX_VIEW_POINTS = 400_000
PALETTE = np.array([
    (230, 57, 70), (69, 123, 157), (42, 157, 143), (244, 162, 97),
    (131, 56, 236), (255, 190, 11), (6, 214, 160), (239, 71, 111),
], dtype=np.uint8)


def display_points(points: np.ndarray, frame: str) -> np.ndarray:
    """RViz expects z up; scans in the 'frd' frame (z down, y right) are turned upright."""
    return points * np.array([1.0, -1.0, -1.0]) if frame == 'frd' else points


def height_colors(points: np.ndarray) -> np.ndarray:
    z = points[:, 2]
    t = (z - z.min()) / max(float(np.ptp(z)), 1e-9)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def _find_rviz_config() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('scan_postprocess'), 'config', 'registration.rviz')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    local = os.path.join(os.path.dirname(__file__), '..', 'config', 'registration.rviz')
    return os.path.abspath(local) if os.path.exists(local) else None


def pack_cloud(points: np.ndarray, colors: np.ndarray) -> np.ndarray:
    packed = (colors[:, 0].astype(np.uint32) << 16) | (colors[:, 1].astype(np.uint32) << 8) | colors[:, 2]
    data = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    data['x'], data['y'], data['z'], data['rgb'] = points[:, 0], points[:, 1], points[:, 2], packed
    return data


class RvizViewer:
    def __init__(self, frame: str = 'frd', launch_rviz: bool = True):
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2, PointField
        except ImportError as error:
            raise RuntimeError('RViz output needs ROS 2: source /opt/ros/humble/setup.bash first.') from error
        self._rclpy = rclpy
        self._PointCloud2 = PointCloud2
        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.frame = frame
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node('scan_registration_viewer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self._node.create_publisher(PointCloud2, TOPIC, qos)
        from visualization_msgs.msg import Marker
        self._Marker = Marker
        self._mesh_pub = self._node.create_publisher(Marker, MESH_TOPIC, qos)
        self._last = None
        self._last_mesh = None
        self._rviz: Optional[subprocess.Popen] = None
        if launch_rviz:
            config = _find_rviz_config()
            command = ['rviz2'] + (['-d', config] if config else [])
            try:
                self._rviz = subprocess.Popen(command)
            except FileNotFoundError:
                print('rviz2 not found; the cloud is still published on ' + TOPIC)

    def publish(self, points: np.ndarray, colors: Optional[np.ndarray] = None) -> None:
        points = display_points(points, self.frame)
        if colors is None:
            colors = height_colors(points)
        if len(points) > MAX_VIEW_POINTS:
            keep = np.linspace(0, len(points) - 1, MAX_VIEW_POINTS).astype(np.int64)
            points, colors = points[keep], colors[keep]
        msg = self._PointCloud2()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(points)
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True
        msg.data = pack_cloud(points, colors).tobytes()
        self._last = msg
        self._pub.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0)

    def publish_mesh(self, vertices: np.ndarray, triangles: np.ndarray, colors: Optional[np.ndarray] = None) -> None:
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        vertices = display_points(vertices, self.frame)
        if colors is None:
            colors = height_colors(vertices)
        marker = self._Marker()
        marker.header.frame_id = FRAME_ID
        marker.ns, marker.id = 'registration_mesh', 0
        marker.type, marker.action = self._Marker.TRIANGLE_LIST, self._Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        corners = triangles.reshape(-1)
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices[corners]]
        marker.colors = [ColorRGBA(r=float(r) / 255, g=float(g) / 255, b=float(b) / 255, a=1.0)
                         for r, g, b in colors[corners]]
        self._last_mesh = marker
        self._mesh_pub.publish(marker)
        self._rclpy.spin_once(self._node, timeout_sec=0)

    def show_alignment(self, stage: str, items: Sequence[Tuple[int, np.ndarray]], delay: float = 0.0) -> None:
        """Debug view: every cloud in its own flat colour, so misalignment is visible."""
        points = np.vstack([p for _, p in items])
        colors = np.vstack([np.tile(PALETTE[i % len(PALETTE)], (len(p), 1)) for i, p in items])
        print(stage)
        self.publish(points, colors)
        if delay > 0:
            time.sleep(delay)

    def hold(self) -> None:
        print('Showing result in RViz. Press Ctrl-C to exit.')
        try:
            while self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=1.0)
                if self._last is not None:
                    self._pub.publish(self._last)
                if self._last_mesh is not None:
                    self._mesh_pub.publish(self._last_mesh)
                if self._rviz is not None and self._rviz.poll() is not None:
                    break
        except KeyboardInterrupt:
            pass

    def close(self) -> None:
        if self._rviz is not None and self._rviz.poll() is None:
            self._rviz.terminate()
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
