#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.bridge = CvBridge()
        self.cv_image = cv2.imread('/home/james/ros2_ws/src/AOPR/sample.jpg')  
        if self.cv_image is None:
            self.get_logger().error('Failed to load sample image.')
        else:
            self.get_logger().info('Sample image loaded successfully.')

    def timer_callback(self):
        if self.cv_image is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
                self.publisher.publish(img_msg)
                self.get_logger().info('Published sample image.')
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
