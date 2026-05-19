#!/usr/bin/env python3
"""
Visualize the live teleop in RViz, without touching the production stack.

Run this in a second terminal alongside `robot_teleop.launch.py`. It starts:
  - publish_robot_description : latches /robot_description from humanoid URDF
  - robot_state_publisher     : URDF + /joint_states → /tf
  - joint_state_relay         : combines head/left_arm/right_arm topics into
                                /joint_states. `source` arg selects which:
                                  commands (default) = what IK is asking for
                                  states             = what motors report
  - rviz2                     : ik_viz.rviz preconfigured with RobotModel

Examples:
    # What the IK is solving for (default — best for debugging IK):
    ros2 launch robot_bringup teleop_rviz.launch.py

    # What the motors are actually reporting back:
    ros2 launch robot_bringup teleop_rviz.launch.py source:=states
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from xacro import process_file


def generate_launch_description():
    pkg_desc = get_package_share_directory('robot_description')
    humanoid_xacro = os.path.join(pkg_desc, 'urdf', 'humanoid.urdf.xacro')
    robot_description = process_file(humanoid_xacro).toxml()

    declare_source = DeclareLaunchArgument(
        'source',
        default_value='commands',
        description='joint_state_relay source: commands (IK output) or states (motor readback)',
    )

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

    joint_state_relay = Node(
        package='robot_bringup',
        executable='joint_state_relay.py',
        name='joint_state_relay',
        parameters=[{'source': LaunchConfiguration('source')}],
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
        declare_source,
        publish_robot_description,
        robot_state_publisher,
        joint_state_relay,
        rviz,
    ])
