#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():
    urdf_arg = DeclareLaunchArgument('urdf',
                                    default_value='ar4.urdf',
                                    description='URDF file to load.')
    rviz_arg = DeclareLaunchArgument('rviz',
                                     default_value='true',
                                     description='Start RViz.')
    rsp_arg = DeclareLaunchArgument(
        'rsp',
        default_value='true',
        description='Run robot state publisher node.')
    jsp_arg = DeclareLaunchArgument(
        'jsp',
        default_value='true',
        description='Run joint state publisher node.')

    # Get the path of the necessary packages
    pkg_share_path = get_package_share_directory('urdf_and_tf')
    urdf_file_path = PathJoinSubstitution([pkg_share_path, 'urdf',  LaunchConfiguration('urdf')])

    # Read the URDF file
    urdf_description = Command(['cat', ' ', urdf_file_path])
    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               namespace='ar4',
               output='both',
               parameters=[{
                   'robot_description': ParameterValue(urdf_description)
               }],
               condition=IfCondition(LaunchConfiguration('rsp')))
    # Joint state publisher
    jsp = Node(package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               namespace='ar4',
               name='joint_state_publisher_gui',
               condition=IfCondition(LaunchConfiguration('jsp')))

    # RViz
    rviz = Node(package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d',
                    os.path.join(pkg_share_path, 'config', 'config.rviz')
                ],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        urdf_arg,
        rviz_arg,
        rsp_arg,
        jsp_arg,
        rsp,
        jsp,
        rviz,
    ])
