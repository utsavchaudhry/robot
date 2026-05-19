#!/usr/bin/env python3
"""
Launch RViz + robot_state_publisher + joint_state_publisher_gui for URDF testing.
Use the joint sliders to move the robot and verify the model in RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    pkg_desc = get_package_share_directory('robot_description')
    humanoid_xacro = os.path.join(pkg_desc, 'urdf', 'humanoid.urdf.xacro')
    robot_description = process_file(humanoid_xacro).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
    )

    publish_robot_description = Node(
        package='robot_bringup',
        executable='publish_robot_description.py',
        name='publish_robot_description',
        parameters=[{'robot_description': robot_description}],
    )

    pkg_bringup = get_package_share_directory('robot_bringup')
    rviz_config = os.path.join(pkg_bringup, 'config', 'urdf_viz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        publish_robot_description,
        rviz,
    ])
