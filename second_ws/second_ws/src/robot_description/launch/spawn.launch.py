#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

packagepath = get_package_share_directory('robot_description')  
xacro_file = os.path.join(packagepath, 'urdf', 'arm.xacro')

# process it into URDF
doc = xacro.process_file(xacro_file)
robot_desc = doc.toxml()

def generate_launch_description():
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        )

    robot_to_gazebo = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[ '-string', robot_desc, '-x', '-2', '-y', '0', '-z','0.05' ,'-name', 'lewan_arm']
        )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(packagepath, 'config', 'controller.yaml'),
                    {'use_sim_time': True},
                    {'robot_description':robot_desc}],
        output='screen',
        )
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        )
    spawn_arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        )
    
    delay_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[spawn_jsb,spawn_arm_ctrl]
        )
    )
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters =[{'use_sim_time': True}, {'robot_description':robot_desc}]
    )
    bridge_node=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[ {'config_file': os.path.join(packagepath, 'config', 'world.yaml')}],
            output = 'screen'
        )

    image_bridge_node=Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera']
    )


    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(packagepath, 'config', 'rviz.rviz')],
        output='screen'
    )

    ##slam_node = IncludeLaunchDescription(
     ##   PythonLaunchDescriptionSource([os.path.join(
       ##     get_package_share_directory('slam_toolbox'), 'launch'),
        ##    '/online_async_launch.py'])
        ##)
    
    ##nav_node = IncludeLaunchDescription(
      ##  PythonLaunchDescriptionSource([os.path.join(
        ##    get_package_share_directory('nav2_bringup'), 'launch'),
          ##  '/bringup_launch.py']),
           ## launch_arguments=[('use_sim_time','true')]
    ##    )

    return LaunchDescription([
        gazebo_node,
        robot_to_gazebo,        
        ros2_control_node,
        delay_spawners,
        robot_state_pub,
        bridge_node,
        image_bridge_node,
        rviz_node,
        



      ##  slam_node,
       ## nav_node,
    ])