from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='AOPR', 
            executable='detect_block.py',
            name='detect_block',
            output='screen',
            parameters=[
                {'image_topic': '/image_in'},
                {'serial_port': '/dev/ttyAMA0'},
                {'baud_rate': 9600},
                {'tuning_mode': False},
                {'x_min': 0},
                {'x_max': 100},
                {'y_min': 0},
                {'y_max': 100},
                {'h_min': 0},
                {'h_max': 180},
                {'s_min': 0},
                {'s_max': 255},
                {'v_min': 0},
                {'v_max': 255},
                {'sz_min': 0},
                {'sz_max': 100}
            ]
        )
    ])
