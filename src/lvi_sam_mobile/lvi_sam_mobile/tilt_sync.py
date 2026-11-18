"""
Tilt synchronizer: receives tilt angle and publishes as a static transform or joint state
for deskewing and frame alignment in LVI-SAM.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


class TiltSync(Node):
    def __init__(self):
        super().__init__("tilt_sync")

        self.tilt_sub = self.create_subscription(Float32, "/tilt/angle", self.tilt_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.current_tilt = 0.0
        self.seq = 0

        self.get_logger().info("Tilt synchronizer initialized")

    def tilt_callback(self, msg: Float32):
        """Receive tilt angle and publish as joint state."""
        self.current_tilt = msg.data

        # Publish joint state for robot_state_publisher
        joint_msg = JointState()
        joint_msg.header.seq = self.seq
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = "scanner"
        joint_msg.name = ["tilt_stage"]
        joint_msg.position = [self.current_tilt]
        joint_msg.velocity = [0.0]
        joint_msg.effort = [0.0]

        self.joint_pub.publish(joint_msg)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = TiltSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
