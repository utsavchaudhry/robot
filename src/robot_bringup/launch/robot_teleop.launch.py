#!/usr/bin/env python3
"""
Launch file for robot teleoperation system (production).
Brings up all necessary nodes for WebRTC-based teleoperation.

All tunable teleop/IK parameters live in:
    robot_bringup/config/teleop_config.yaml
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


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
    pkg_bringup = get_package_share_directory('robot_bringup')
    teleop_config = os.path.join(pkg_bringup, 'config', 'teleop_config.yaml')

    # --- Launch arguments ---
    # Note: stream_mode and flat_eye knobs are gone — the robot now always
    # produces a single 2:1 SBS output and the frontend handles cropping
    # (flat = right eye via CSS) and splitting (VR = per-eye planes).
    declare_debug = DeclareLaunchArgument('debug', default_value='false')
    declare_stereo = DeclareLaunchArgument('stereo_mode', default_value='true')
    declare_port = DeclareLaunchArgument('signaling_port', default_value='8443')
    declare_dashboard = DeclareLaunchArgument('operator_dashboard', default_value='false')
    declare_joints = DeclareLaunchArgument(
        'enabled_joints',
        default_value='all',
        description='Comma-separated motor names to enable, or "all".')

    debug = LaunchConfiguration('debug')
    stereo_mode = LaunchConfiguration('stereo_mode')
    signaling_port = LaunchConfiguration('signaling_port')
    operator_dashboard = LaunchConfiguration('operator_dashboard')

    # =====================================================================
    # IK Solver — params from teleop_config.yaml
    # =====================================================================
    arm_kinematics_node = Node(
        package='robot_kinematics', executable='humanoid_kinematics_node',
        name='humanoid_kinematics_node', output='screen',
        parameters=[teleop_config],
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
    # Camera pipeline
    # =====================================================================
    # Single capture node: opens /dev/video0, JPEG-encodes, publishes directly
    # to /camera/output/compressed. The frontend handles all SBS rendering
    # logic (right-eye crop on flat, per-eye sampling in VR) — backend serves
    # the camera's whole frame at its native 2:1 SBS aspect.
    #
    # 2400x1200 is the lowest TRUE 2:1 (advertised) format on VR.Cam at 30fps;
    # smaller listed sizes (e.g. 1280x720) are 16:9, where the camera squishes
    # the two eyes into a non-stereo layout. Set fourcc=MJPG first so the
    # driver allows the higher resolution; v4l2 falls back to YUYV otherwise.
    # Capture native SBS (2400x1200) and resize to 1600x800 (still 2:1) before
    # JPEG encoding. The resize avoids a CPU spike in the downstream webrtc
    # pipeline (jpegdec + x264enc), which was segfaulting webrtcbin during
    # concurrent ICE on full-size frames.
    stereo_camera_node = Node(
        package='robot_webrtc', executable='stereo_camera_node',
        name='stereo_camera_node', output='screen',
        parameters=[{
            'device': '/dev/video0',
            'width': 2400, 'height': 1200, 'fps': 30.0,
            'fourcc': 'MJPG', 'jpeg_quality': 85,
            'publish_width': 1600, 'publish_height': 800,
            'output_topic': '/camera/output/compressed',
        }]
    )

    # =====================================================================
    # WebRTC + Signaling
    # =====================================================================
    # Encode target is 2:1 SBS so the webapp's aspect-detect (>= 1.95) fires.
    # 1600x800 is the lightest 2:1 size that still gives each eye 800x800 —
    # well below the 2400x1200 native that was making x264enc segfault on the
    # LattePanda CPU. We also force audio OFF because the VR.Cam's mic shares
    # USB bandwidth with its camera, and alsasrc-vs-v4l2src contention was
    # dropping the capture rate to ~14fps. Flip enable_audio back to True
    # if/when we have a separate audio source.
    webrtc_node = Node(
        package='robot_webrtc', executable='webrtc_node',
        name='webrtc_node', output='screen',
        parameters=[{
            'signaling_server_port': signaling_port,
            'video_source': 'ros', 'video_device': '/dev/video0',
            'video_width': 1600, 'video_height': 800, 'video_framerate': 30,
            'audio_device': 'auto', 'audio_source_type': 'alsa',
            'enable_audio': False, 'enable_stereo': stereo_mode, 'camera_mode': 'stereo',
        }]
    )

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

    # =====================================================================
    # Data Logger — only when debug:=true
    # =====================================================================
    teleop_data_logger = Node(
        package='robot_bringup', executable='teleop_data_logger.py',
        name='teleop_data_logger', output='screen',
        parameters=[teleop_config],
        condition=IfCondition(debug),
    )

    # =====================================================================
    # Operator Dashboard — only when operator_dashboard:=true
    # =====================================================================
    operator_dashboard_node = Node(
        package='robot_webrtc', executable='operator_dashboard_node',
        name='operator_dashboard', output='screen',
        condition=IfCondition(operator_dashboard),
    )

    # =====================================================================
    # Session Recorder — always on; auto-records per operator session
    # =====================================================================
    session_recorder_node = Node(
        package='robot_webrtc', executable='session_recorder_node',
        name='session_recorder', output='screen',
        parameters=[teleop_config],
    )

    return LaunchDescription([
        declare_debug, declare_stereo, declare_port,
        declare_dashboard, declare_joints,
        arm_kinematics_node,
        teleop_controller_node,
        stereo_camera_node,
        webrtc_node,
        signaling_server,
        signaling_bridge,
        teleop_data_logger,
        OpaqueFunction(function=_motor_handler_setup),
        operator_dashboard_node,
        session_recorder_node,
    ])
