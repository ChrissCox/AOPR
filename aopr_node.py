#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class AOPRNode(Node):
    def __init__(self):
        super().__init__('aopr_node')
        # Declare parameters for the serial port and baud rate
        self.declare_parameter('serial_port', '/dev/ttyV0')
        self.declare_parameter('baud_rate', 9600)
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Open serial port
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Opened serial port {serial_port} at {baud_rate} baud.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('AOPR node started and listening to /cmd_vel.')

    def cmd_vel_callback(self, msg):
        # For a differential drive: left_speed & right_speed
        linear = msg.linear.x
        angular = msg.angular.z

        # Basic mixing: adjust as needed for your robot’s geometry
        left_speed = linear - angular
        right_speed = linear + angular

        # Convert speeds from -1..1 to Sabertooth's 1..127 range
        #  -1 (full reverse) -> ~1
        #   0 (stop)         -> 64
        #  +1 (full forward) -> 127
        left_cmd  = 64 + int(63 * left_speed)
        right_cmd = 64 + int(63 * right_speed)

        # Clip the values between 1 and 127
        left_cmd = max(1, min(127, left_cmd))
        right_cmd = max(1, min(127, right_cmd))

        # Build & send the commands for each channel (simplified serial)
        # Channel 1: command byte is 0 (0x00) or 1 depending on your Sabertooth version
        # Channel 2: command byte is 4 (0x04) or 5
        # Example using command bytes 0x00 for M1, 0x04 for M2:
        try:
            m1_cmd = bytes([0x00, left_cmd])    # Motor 1 (Channel 1)
            m2_cmd = bytes([0x04, right_cmd])   # Motor 2 (Channel 2)

            self.ser.write(m1_cmd)
            self.ser.write(m2_cmd)
            self.get_logger().info(f'M1: {left_cmd}, M2: {right_cmd}')
        except Exception as e:
            self.get_logger().error(f'Error writing to serial port: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AOPRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AOPR node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
