#!/usr/bin/env python3
"""
Motor calibration mode — no teleop, no IK, no cameras.

Launches:
  - motor_handler        (talks to ESP32s, sends default pose on startup)
  - joint_state_publisher_gui  (slider UI → /joint_states)
  - slider_command_relay (forwards /joint_states → motor command topics)
  - robot_state_publisher + RViz  (URDF visualisation)

Sliders start at the default pose from default_pose.yaml so motors don't
jerk on launch.

Usage:
    ros2 launch robot_bringup motor_calibration.launch.py enabled_joints:=head_yaw
    ros2 launch robot_bringup motor_calibration.launch.py enabled_joints:=all

Requires: sudo apt install ros-humble-joint-state-publisher-gui
"""

import os
import yaml as pyyaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from xacro import process_file


def _parse_joints(raw):
    raw = raw.strip()
    if raw.startswith('['):
        return pyyaml.safe_load(raw)
    return [s.strip() for s in raw.split(',') if s.strip()]


def _motor_handler_setup(context):
    joints = _parse_joints(
        LaunchConfiguration('enabled_joints').perform(context))
    return [Node(
        package='motor_handler', executable='motor_handler_node',
        name='motor_handler', output='screen',
        parameters=[{'enabled_joints': joints}],
    )]


def _load_default_zeros(pkg_bringup):
    """Build {joint_name: angle} dict from default_pose.yaml."""
    path = os.path.join(pkg_bringup, 'config', 'default_pose.yaml')
    with open(path) as f:
        pose = pyyaml.safe_load(f)
    zeros = {}
    for name, val in pose.get('head', {}).items():
        zeros[f'head_{name}'] = float(val)
    for name, val in pose.get('left_arm', {}).items():
        zeros[f'left_{name}'] = float(val)
    for name, val in pose.get('right_arm', {}).items():
        zeros[f'right_{name}'] = float(val)
    return zeros


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_desc = get_package_share_directory('robot_description')
    rviz_config = os.path.join(pkg_bringup, 'config', 'ik_viz.rviz')
    humanoid_xacro = os.path.join(pkg_desc, 'urdf', 'humanoid.urdf.xacro')
    robot_description = process_file(humanoid_xacro).toxml()
    default_zeros = _load_default_zeros(pkg_bringup)

    declare_joints = DeclareLaunchArgument(
        'enabled_joints',
        default_value='all',
        description='Comma-separated motor names to enable, or "all".')

    # URDF → robot_state_publisher (reads /joint_states for TF)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Publish URDF on /robot_description topic (TRANSIENT_LOCAL for RViz)
    publish_robot_description = Node(
        package='robot_bringup', executable='publish_robot_description.py',
        name='publish_robot_description',
        parameters=[{'robot_description': robot_description}],
    )

    # Slider GUI → publishes to /joint_states
    # 'zeros' sets initial slider positions to the default pose.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description,
                      'zeros': default_zeros}],
    )

    # Relay: /joint_states (sliders) → head/right_arm/left_arm joint_commands
    slider_relay = Node(
        package='robot_bringup', executable='slider_command_relay.py',
        name='slider_command_relay', output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    return LaunchDescription([
        declare_joints,
        robot_state_publisher,
        publish_robot_description,
        joint_state_publisher_gui,
        slider_relay,
        OpaqueFunction(function=_motor_handler_setup),
        rviz_node,
    ])
