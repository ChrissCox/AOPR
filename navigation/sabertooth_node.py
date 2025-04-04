# color_box_nav/sabertooth_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SabertoothNode(Node):
    def __init__(self):
        super().__init__('sabertooth_node')
        self.serial_port = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)  # adjust if using USB
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - angular
        right_speed = linear + angular

        left_cmd = self.map_to_sabertooth(left_speed, motor=1)
        right_cmd = self.map_to_sabertooth(right_speed, motor=2)

        self.send_command(left_cmd)
        self.send_command(right_cmd)

    def map_to_sabertooth(self, value, motor):
        # value: -1.0 (full reverse) to 1.0 (full forward)
        value = max(-1.0, min(1.0, value))  # clamp
        if motor == 1:
            return int(64 + (63 * value)) if value >= 0 else int(64 + (63 * value))
        elif motor == 2:
            return int(192 + (63 * value)) if value >= 0 else int(192 + (63 * value))

    def send_command(self, command):
        self.serial_port.write(bytes([command]))

def main(args=None):
    rclpy.init(args=args)
    node = SabertoothNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
