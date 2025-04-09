from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='AOPR',
            executable='aopr_node.py',
            name='aopr_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyAMA0', 'baud_rate': 9600}]
        )
    ])
