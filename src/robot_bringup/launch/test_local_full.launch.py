#!/usr/bin/env python3
"""
Full local test: WebRTC (test sources) + teleop controller + IK + RViz.

Combines signaling + WebRTC with videotestsrc/audiotestsrc, teleop controller,
IK, joint relay, data logger and RViz so you can:
  1. Open the webapp, connect, and use the joysticks
  2. See the robot arm move in RViz driven by the WebRTC → teleop → IK pipeline

All tunable teleop/IK parameters live in:
    robot_bringup/config/teleop_config.yaml

Usage:
    ros2 launch robot_bringup test_local_full.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from xacro import process_file


def _parse_joints(raw):
    """Accept YAML list (\"['a','b']\") or comma-separated string (\"a,b\")."""
    raw = raw.strip()
    if raw.startswith('['):
        return yaml.safe_load(raw)
    return [s.strip() for s in raw.split(',') if s.strip()]


def _motor_handler_setup(context):
    joints = _parse_joints(
        LaunchConfiguration('enabled_joints').perform(context))
    return [Node(
        package='motor_handler', executable='motor_handler_node',
        name='motor_handler', output='screen',
        parameters=[{'enabled_joints': joints}],
    )]


def generate_launch_description():
    # --- Paths ---
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_desc = get_package_share_directory('robot_description')
    teleop_config = os.path.join(pkg_bringup, 'config', 'teleop_config.yaml')
    rviz_config = os.path.join(pkg_bringup, 'config', 'ik_viz.rviz')
    humanoid_xacro = os.path.join(pkg_desc, 'urdf', 'humanoid.urdf.xacro')
    robot_description = process_file(humanoid_xacro).toxml()

    # --- Launch arguments ---
    declare_signaling_port = DeclareLaunchArgument(
        'signaling_port', default_value='8443',
        description='WebSocket signaling server port'
    )
    declare_joints = DeclareLaunchArgument(
        'enabled_joints',
        default_value='head_yaw,head_pitch,right_shoulder_pitch,left_shoulder_pitch',
        description='Comma-separated motor names to enable, or "all".')
    signaling_port = LaunchConfiguration('signaling_port')

    # =====================================================================
    # WebRTC + Signaling
    # =====================================================================
    signaling_server = ExecuteProcess(
        cmd=['ros2', 'run', 'robot_webrtc', 'signaling_server',
             '--port', signaling_port],
        output='screen'
    )

    signaling_bridge = Node(
        package='robot_webrtc', executable='signaling_bridge',
        name='signaling_bridge', output='screen',
        parameters=[{'signaling_port': signaling_port}]
    )

    webrtc_node = Node(
        package='robot_webrtc', executable='webrtc_node',
        name='webrtc_node', output='screen',
        parameters=[{
            'signaling_server_port': signaling_port,
            'video_source': 'v4l2',
            'video_device': '',
            'video_width': 640, 'video_height': 480, 'video_framerate': 30,
            'video_codec': 'vp8',
            'enable_audio': True, 'audio_device': 'test',
            'audio_source_type': 'pulse',
            'enable_stereo': False, 'camera_mode': 'mono',
        }]
    )

    # =====================================================================
    # Teleop Controller — params from teleop_config.yaml
    # =====================================================================
    teleop_controller_node = Node(
        package='robot_webrtc', executable='teleop_controller_node',
        name='teleop_controller_node', output='screen',
        parameters=[teleop_config],
    )

    # =====================================================================
    # IK Solver — params from teleop_config.yaml
    # =====================================================================
    humanoid_kinematics_node = Node(
        package='robot_kinematics', executable='humanoid_kinematics_node',
        name='humanoid_kinematics_node', output='screen',
        parameters=[teleop_config],
    )

    # =====================================================================
    # Robot model + visualisation
    # =====================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    publish_robot_description = Node(
        package='robot_bringup', executable='publish_robot_description.py',
        name='publish_robot_description',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_relay = Node(
        package='robot_bringup', executable='joint_state_relay.py',
        name='joint_state_relay',
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    # =====================================================================
    # Data Logger — params from teleop_config.yaml
    # =====================================================================
    teleop_data_logger = Node(
        package='robot_bringup', executable='teleop_data_logger.py',
        name='teleop_data_logger', output='screen',
        parameters=[teleop_config],
    )

    return LaunchDescription([
        declare_signaling_port,
        declare_joints,
        signaling_server,
        signaling_bridge,
        webrtc_node,
        teleop_controller_node,
        robot_state_publisher,
        publish_robot_description,
        humanoid_kinematics_node,
        joint_state_relay,
        rviz,
        teleop_data_logger,
        OpaqueFunction(function=_motor_handler_setup),
    ])
