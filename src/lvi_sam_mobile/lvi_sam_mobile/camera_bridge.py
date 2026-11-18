"""
Camera bridge: receives video frames from phone RTSP and publishes as ROS 2 Image messages.
Uses GStreamer or OpenCV to decode RTSP and timestamp frames.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.declare_parameter("rtsp_url", "rtsp://192.168.1.100:8554/stream")
        self.declare_parameter("frame_rate", 30)

        rtsp_url = self.get_parameter("rtsp_url").value
        frame_rate = self.get_parameter("frame_rate").value

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()
        self.seq = 0

        # Open RTSP stream
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream: {rtsp_url}")
            return

        self.create_timer(1.0 / frame_rate, self.capture_frame)
        self.get_logger().info(f"Camera bridge listening on {rtsp_url}")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from RTSP")
            return

        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Publish as ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
        msg.header.seq = self.seq
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        self.image_pub.publish(msg)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
