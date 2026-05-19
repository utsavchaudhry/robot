#!/usr/bin/env python3
"""
Minimal launch file for testing WebRTC on a single x86 laptop.

Runs ONLY signaling_server + signaling_bridge + webrtc_node with synthetic
video (videotestsrc) and audio (audiotestsrc), so no camera or mic is needed
on the robot side.

Architecture:
  PC1 (operator_recv): Robot sends VP8 video + Opus audio via GStreamer webrtcbin
  PC2 (operator_send): Browser sends data channels only (control, video, audio)
    - Operator video/audio travel as binary blobs over data channels
    - No WebRTC media tracks on PC2 (avoids GStreamer 1.20 webrtcbin receive bugs)

Usage:
    ros2 launch robot_bringup test_local.launch.py

Then open the webapp (npm run dev) and:
  1. Click Connect (ws://localhost:8443)
  2. Click "Test Media" to start simulated operator media
  3. You should see the GStreamer test pattern in the browser (robot -> operator)
     and the robot node logs receiving operator video frames (operator -> robot)
  4. Data channel commands (joystick) work as usual.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_signaling_port_arg = DeclareLaunchArgument(
        'signaling_port',
        default_value='8443',
        description='WebSocket signaling server port'
    )

    signaling_port = LaunchConfiguration('signaling_port')

    # Signaling Server (WebSocket, as separate process)
    signaling_server = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'robot_webrtc', 'signaling_server',
            '--port', signaling_port
        ],
        output='screen'
    )

    # Signaling Bridge (robot <-> WebSocket)
    signaling_bridge = Node(
        package='robot_webrtc',
        executable='signaling_bridge',
        name='signaling_bridge',
        output='screen',
        parameters=[{'signaling_port': signaling_port}]
    )

    # WebRTC Node with test sources (no real camera/mic needed)
    webrtc_node = Node(
        package='robot_webrtc',
        executable='webrtc_node',
        name='webrtc_node',
        output='screen',
        parameters=[{
            'signaling_server_port': signaling_port,
            'video_source': 'v4l2',           # v4l2 path, but device='' triggers videotestsrc
            'video_device': '',                # empty -> videotestsrc (colour bars)
            'video_width': 640,
            'video_height': 480,
            'video_framerate': 30,
            'video_codec': 'vp8',
            'enable_audio': True,
            'audio_device': 'test',            # -> audiotestsrc (sine wave tone)
            'audio_source_type': 'pulse',      # ignored when audio_device='test'
            'enable_stereo': False,
            'camera_mode': 'mono',
        }]
    )

    return LaunchDescription([
        declare_signaling_port_arg,
        signaling_server,
        signaling_bridge,
        webrtc_node,
    ])
