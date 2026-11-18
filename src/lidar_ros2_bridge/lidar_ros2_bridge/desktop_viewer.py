"""
Desktop viewer for LVI-SAM outputs during and after mapping.
Displays point cloud and trajectory using matplotlib/open3d in a separate window.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from typing import Optional

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


class DesktopViewer(Node):
    def __init__(self):
        super().__init__("desktop_viewer")
        self.declare_parameter("fps", 10)

        self.fps = self.get_parameter("fps").value
        self.current_cloud = None
        self.current_poses = []

        # Subscribe to outputs
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/lidar/mapping/cloud", self.cloud_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, "/lidar/mapping/trajectory", self.path_callback, 10
        )

        # Visualization timer
        self.create_timer(1.0 / self.fps, self.render_callback)

        # Open3D visualizer
        if HAS_OPEN3D:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name="LVI-SAM Mobile Mapping", width=1200, height=800)
            self.cloud_geom = None
            self.path_geom = None
            self.get_logger().info("Desktop viewer initialized with Open3D")
        else:
            self.get_logger().warn(
                "Open3D not installed. Install with: pip install open3d"
            )
            self.vis = None

    def cloud_callback(self, msg: PointCloud2):
        """Receive downsampled cloud from live_cloud_publisher."""
        points = self.pointcloud2_to_xyz(msg)
        if points is not None and len(points) > 0:
            self.current_cloud = points
            self.get_logger().debug(f"Updated cloud: {len(points)} points")

    def path_callback(self, msg: Path):
        """Receive trajectory."""
        self.current_poses = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.get_logger().debug(f"Updated trajectory: {len(self.current_poses)} poses")

    def pointcloud2_to_xyz(self, cloud: PointCloud2) -> Optional[np.ndarray]:
        """Convert PointCloud2 to numpy Nx3 array."""
        points = np.frombuffer(cloud.data, dtype=np.float32).reshape(-1, 3)
        return points

    def render_callback(self):
        """Render current cloud and trajectory."""
        if not HAS_OPEN3D or self.vis is None:
            return

        # Update cloud geometry
        if self.current_cloud is not None:
            if self.cloud_geom is None:
                self.cloud_geom = o3d.geometry.PointCloud()
                self.vis.add_geometry(self.cloud_geom)

            self.cloud_geom.points = o3d.utility.Vector3dVector(self.current_cloud)
            self.cloud_geom.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray
            self.vis.update_geometry(self.cloud_geom)

        # Update trajectory
        if len(self.current_poses) > 0:
            if self.path_geom is not None:
                self.vis.remove_geometry(self.path_geom)

            # Create line set for trajectory
            lines = [[i, i + 1] for i in range(len(self.current_poses) - 1)]
            self.path_geom = o3d.geometry.LineSet()
            self.path_geom.points = o3d.utility.Vector3dVector(np.array(self.current_poses))
            self.path_geom.lines = o3d.utility.Vector2iVector(np.array(lines))
            self.path_geom.paint_uniform_color([1.0, 0.0, 0.0])  # Red

            if self.path_geom is not None:
                self.vis.add_geometry(self.path_geom)

        self.vis.poll_events()
        self.vis.update_renderer()


def main(args=None):
    rclpy.init(args=args)
    node = DesktopViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.vis is not None:
            node.vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
