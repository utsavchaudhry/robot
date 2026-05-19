#!/usr/bin/env python3
"""
Launch RViz + humanoid kinematics + joint_state relay + interactive marker for IK visualization.
Drag the green control in RViz to move the IK target; the arm will follow.
Uses unified humanoid URDF for whole-body IK.
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

    publish_robot_description = Node(
        package='robot_bringup',
        executable='publish_robot_description.py',
        name='publish_robot_description',
        parameters=[{'robot_description': robot_description}],
    )

    arm_kinematics = Node(
        package='robot_kinematics',
        executable='humanoid_kinematics_node',
        name='humanoid_kinematics_node',
        output='screen',
        parameters=[{
            'left_end_effector': 'left_end_effector',
            'right_end_effector': 'right_end_effector',
            'update_rate': 50.0,
            'debug_mode': False,
        }]
    )

    joint_state_relay = Node(
        package='robot_bringup',
        executable='joint_state_relay.py',
        name='joint_state_relay',
    )

    ik_interactive_marker = Node(
        package='robot_bringup',
        executable='ik_interactive_marker_node.py',
        name='ik_interactive_marker_node',
    )

    pkg_bringup = get_package_share_directory('robot_bringup')
    rviz_config = os.path.join(pkg_bringup, 'config', 'ik_viz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    return LaunchDescription([
        robot_state_publisher,
        publish_robot_description,
        arm_kinematics,
        joint_state_relay,
        ik_interactive_marker,
        rviz,
    ])
