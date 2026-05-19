#!/usr/bin/env python3
"""
Launch file for kinematics-only testing (e.g. on x86 without cameras/WebRTC).
Brings up only the humanoid_kinematics_node for whole-body IK using unified humanoid URDF.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    arm_kinematics_node = Node(
        package='robot_kinematics',
        executable='humanoid_kinematics_node',
        name='humanoid_kinematics_node',
        output='screen',
        parameters=[{
            'left_end_effector': 'left_end_effector',
            'right_end_effector': 'right_end_effector',
            'update_rate': 50.0,
            'debug_mode': LaunchConfiguration('debug'),
        }]
    )

    return LaunchDescription([
        declare_debug_arg,
        arm_kinematics_node
    ])
