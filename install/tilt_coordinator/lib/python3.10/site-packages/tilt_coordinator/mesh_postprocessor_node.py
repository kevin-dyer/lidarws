"""Save completed tilt scans and reconstruct a surface mesh."""
from datetime import datetime
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .topic_names import CAPTURE_COMPLETE_TOPIC, POINT_CLOUD_TOPIC


def pointcloud2_to_numpy(message):
    """Extract finite XYZ points from a packed float32 PointCloud2."""
    fields = {field.name: field for field in message.fields}
    required = ('x', 'y', 'z')
    if any(name not in fields for name in required):
        raise ValueError('point cloud must contain x, y and z fields')
    if any(fields[name].datatype != 7 or fields[name].count != 1
           for name in required):
        raise ValueError('point cloud x, y and z fields must be float32 scalars')
    if message.width == 0 or message.height == 0:
        return np.empty((0, 3), dtype=np.float64)

    dtype = np.dtype({
        'names': list(required),
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [fields[name].offset for name in required],
        'itemsize': message.point_step,
    })
    rows = np.ndarray(
        shape=(message.height, message.width),
        dtype=dtype,
        buffer=message.data,
        strides=(message.row_step, message.point_step),
    )
    points = np.column_stack([
        rows[name].reshape(-1) for name in required
    ]).astype(np.float64, copy=False)
    return points[np.isfinite(points).all(axis=1)]


def reconstruct_ball_pivoting(points):
    """Create an Open3D ball-pivoting mesh from the captured XYZ points."""
    import open3d as o3d

    if len(points) < 6:
        raise ValueError('at least 6 points are required to reconstruct a mesh')

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    distances = np.asarray(cloud.compute_nearest_neighbor_distance())
    distances = distances[np.isfinite(distances) & (distances > 0.0)]
    if len(distances) == 0:
        raise ValueError('point cloud has no distinct neighboring points')

    spacing = float(np.mean(distances))
    cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=spacing * 4.0, max_nn=30))
    cloud.orient_normals_consistent_tangent_plane(
        k=min(30, len(points) - 1))
    radii = o3d.utility.DoubleVector([
        spacing * 1.5, spacing * 3.0, spacing * 5.0,
    ])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, radii)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) == 0:
        raise ValueError(
            'ball-pivoting produced no triangles; scan density may be too low')
    mesh.triangles = o3d.utility.Vector3iVector(
        reverse_triangle_winding(np.asarray(mesh.triangles)))
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    return mesh


def reverse_triangle_winding(triangles):
    """Flip every triangle face so saved meshes and RViz share normals."""
    triangles = np.asarray(triangles, dtype=np.int32)
    if triangles.ndim != 2 or triangles.shape[1] != 3:
        raise ValueError('triangles must be an Nx3 array')
    return triangles[:, [0, 2, 1]]


class MeshPostprocessorNode(Node):
    """Wait for successful capture, save PLY files, then publish a mesh marker."""

    def __init__(self):
        super().__init__('tilt_mesh_postprocessor')
        self.declare_parameter('output_directory', '~/dev/lidarws/lidar_scans')
        self._output_directory = os.path.abspath(os.path.expanduser(
            self.get_parameter('output_directory').value))
        self._latest_cloud = None
        self._capture_complete = False
        self._processing = False
        self._capture_started_at = None
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, POINT_CLOUD_TOPIC,
            self._on_cloud, latched_qos)
        self._complete_sub = self.create_subscription(
            Bool, CAPTURE_COMPLETE_TOPIC,
            self._on_capture_complete, latched_qos)
        self._mesh_pub = self.create_publisher(
            MarkerArray, '/surface_mesh', latched_qos)
        self._status_pub = self.create_publisher(
            String, '/tilt_mesh_postprocessor/status', 10)
        self._timer = self.create_timer(0.25, self._process_when_ready)

    def _publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _on_cloud(self, message):
        self._latest_cloud = message

    def _on_capture_complete(self, message):
        if not message.data:
            self._capture_complete = False
            self._processing = False
            self._capture_started_at = None
            return
        # Log message in the console and publish a status message for RViz.
        self.get_logger().info('capture completed, processing point cloud...')
        self._publish_status('capture completed, processing point cloud...')

        self._capture_complete = True
        self._capture_started_at = self.get_clock().now()
        self._process_when_ready()

    def _process_when_ready(self):
        self.get_logger().debug('STARTING _process_when_ready')
        if not self._capture_complete or self._processing:
            return
        if self._latest_cloud is None:
            elapsed = (self.get_clock().now() - self._capture_started_at)
            if elapsed.nanoseconds > 10_000_000_000:
                self._capture_complete = False
                self.get_logger().error(
                    'capture completed but no point cloud was received')
            return

        self._processing = True
        try:
            self._save_and_publish_mesh(self._latest_cloud)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            self.get_logger().error('surface reconstruction failed: %s' % exc)
            self._publish_status('surface reconstruction failed: %s' % exc)
        finally:
            self._capture_complete = False

    def _save_and_publish_mesh(self, message):
        import open3d as o3d

        points = pointcloud2_to_numpy(message)
        if len(points) < 6:
            raise ValueError('not enough finite points to build a surface mesh')
        os.makedirs(self._output_directory, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = os.path.join(self._output_directory, 'tilt_scan_' + stamp)

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud_path = prefix + '_cloud.ply'
        if not o3d.io.write_point_cloud(cloud_path, cloud):
            raise OSError('failed to save point cloud to %s' % cloud_path)

        mesh = reconstruct_ball_pivoting(points)
        mesh_path = prefix + '_mesh.ply'
        if not o3d.io.write_triangle_mesh(mesh_path, mesh):
            raise OSError('failed to save surface mesh to %s' % mesh_path)

        self._mesh_pub.publish(MarkerArray(
            markers=[self._to_marker(mesh, message.header)]))
        self._publish_status(
            'saved point cloud (%d points): %s; mesh (%d triangles): %s'
            % (len(points), cloud_path, len(mesh.triangles), mesh_path))

    @staticmethod
    def _to_marker(mesh, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'tilt_scan_surface'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.15
        marker.color.g = 0.75
        marker.color.b = 0.95
        marker.color.a = 0.85
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        for triangle in triangles:
            marker.points.extend([
                Point(x=float(vertices[index, 0]),
                      y=float(vertices[index, 1]),
                      z=float(vertices[index, 2]))
                for index in triangle
            ])
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MeshPostprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
