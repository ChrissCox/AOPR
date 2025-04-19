#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

class KinectImagePublisher(Node):
    def __init__(self):
        super().__init__('kinect_image_publisher')
        # Publish on the topic that detect_block subscribes to
        self.publisher = self.create_publisher(Image, '/image_in', 10)
        self.bridge = CvBridge()

        # Try to open the Kinect camera device.
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Kinect camera not accessible - check device and drivers!")
        else:
            self.get_logger().info("Kinect camera opened successfully.")

        # Timer to continuously grab and publish frames at 10 Hz
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame from Kinect")
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KinectImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Kinect image publisher")
    finally:
        # Release the video capture
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
