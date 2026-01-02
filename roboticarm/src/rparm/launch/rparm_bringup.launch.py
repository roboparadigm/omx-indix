#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('rparm')
    use_ready = LaunchConfiguration('use_ready')
    port_name = LaunchConfiguration('port_name')
    servo_speed = LaunchConfiguration('servo_speed_steps_per_s')
    servo_acc = LaunchConfiguration('servo_acc_steps_per_s2')

    # Robot description
    xacro_file = os.path.join(pkg_share, 'urdf', 'rparm.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'false'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    bridge = Node(
        package='st3215_bridge',
        executable='st3215_bridge_node',
        output='screen',
        parameters=[
            {'port_name': port_name},
            {'servo_speed_steps_per_s': servo_speed},
            {'servo_acc_steps_per_s2': servo_acc},
        ],
    )

    ready_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
            'trajectory_msgs/JointTrajectory',
            '{joint_names:["joint1","joint2","joint3","joint4","joint5","joint6"],'
            ' points:[{positions:[3.139,3.317,3.567,2.097,3.141,5.809], time_from_start:{sec:8,nanosec:0}}]}'
        ],
        output='screen',
        condition=None,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ready', default_value='true', description='Send ready pose once'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0', description='Servo port'),
        DeclareLaunchArgument('servo_speed_steps_per_s', default_value='400', description='Servo speed steps/s'),
        DeclareLaunchArgument('servo_acc_steps_per_s2', default_value='20', description='Servo accel steps/s^2'),
        rsp,
        bridge,
        # Only send ready if requested
        ready_pub if use_ready else None,
    ])
