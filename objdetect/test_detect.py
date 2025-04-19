#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import block_tracker.process_image as proc
import serial
import time

class DetectBlock(Node):
    def __init__(self):
        super().__init__('detect_block')
        self.get_logger().info('Looking for the block...')

        # Declare parameters for image topics and serial port
        self.declare_parameter('image_topic', '/image_in')
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 9600)
        
        image_topic = self.get_parameter('image_topic').value
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.image_sub = self.create_subscription(
            Image, image_topic,
            self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.block_pub = self.create_publisher(Point, "/detected_block", 1)

        # Declare parameters for tuning
        self.declare_parameter('tuning_mode', False)
        self.declare_parameter("x_min", 0)
        self.declare_parameter("x_max", 100)
        self.declare_parameter("y_min", 0)
        self.declare_parameter("y_max", 100)
        self.declare_parameter("h_min", 0)
        self.declare_parameter("h_max", 180)
        self.declare_parameter("s_min", 0)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_min", 0)
        self.declare_parameter("v_max", 255)
        self.declare_parameter("sz_min", 0)
        self.declare_parameter("sz_max", 100)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        # Open the serial port for sending block info
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Opened serial port {serial_port} at {baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.bridge = CvBridge()
        if self.tuning_mode:
            proc.create_tuning_window(self.tuning_params)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        try:
            if self.tuning_mode:
                self.tuning_params = proc.get_tuning_params()

            # Use block detection instead of circle detection
            keypoints_norm, out_image, tuning_image = proc.find_blocks(cv_image, self.tuning_params)

            # Publish the output and tuning images for debugging
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()
            # Keep the largest detected block (using its size as the measure)
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size
                self.get_logger().info(f"Block {i}: ({x:.1f}, {y:.1f}, {s:.1f})")
                if s > point_out.z:
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if point_out.z > 0:
                self.block_pub.publish(point_out)
                # Send block info over serial
                # Get the image dimensions to scale the coordinates into a byte (0-255)
                height, width, _ = cv_image.shape
                x_scaled = int((point_out.x / width) * 255) & 0xFF
                y_scaled = int((point_out.y / height) * 255) & 0xFF
                s_scaled = int(point_out.z) & 0xFF  # simple scaling for size
                # Build packet: start byte 0xFF, then x, y, s, then end byte 0xFE
                packet = bytes([0xFF, x_scaled, y_scaled, s_scaled, 0xFE])
                self.ser.write(packet)
                self.get_logger().info(f"Sent serial packet: {[hex(b) for b in packet]}")

        except Exception as e:
            self.get_logger().error(f"Error during block processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    detect_block = DetectBlock()
    while rclpy.ok():
        rclpy.spin_once(detect_block)
        proc.wait_on_gui()
    detect_block.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
