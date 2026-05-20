#!/usr/bin/env python3
"""
ROS2 Node for WebRTC using GStreamer webrtcbin
Handles bidirectional audio/video streaming and DataChannel control

Architecture:
  PC1 (operator_recv): Robot SENDS video+audio via GStreamer webrtcbin (robot offers)
  PC2 (operator_send): Browser SENDS control+video+audio via data channels (browser offers)
    - "control" data channel: JSON teleop commands (ordered, reliable)
    - "video" data channel: JPEG frames from operator camera (unordered, unreliable)
    - "audio" data channel: PCM audio chunks from operator mic (ordered, reliable)
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
import json
import os
import time
import threading
import subprocess
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

from robot_interfaces.msg import TeleopCommand, ArmCommand
from std_msgs.msg import String, UInt8MultiArray, Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose

import numpy as np
from typing import Optional, Dict


class WebRTCNode(Node):
    """
    ROS2 Node for WebRTC media streaming and control using GStreamer webrtcbin
    """

    def __init__(self):
        super().__init__('webrtc_node')

        # Lock protecting self.peers across ROS executor and GLib threads
        self._peers_lock = threading.Lock()

        # Declare parameters
        self.declare_parameter('signaling_server_port', 8443)
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('video_width', 1280)
        self.declare_parameter('video_height', 720)
        self.declare_parameter('video_framerate', 30)
        self.declare_parameter('video_bitrate', 2000000)  # 2 Mbps default
        self.declare_parameter('video_codec', 'h264')  # 'vp8', 'vp9', 'h264' — h264 has universal mobile decoder support
        self.declare_parameter('video_cpu_usage', 4)  # 0-16, lower = faster encoding (less efficient)
        self.declare_parameter('audio_bitrate', 64000)  # 64 kbps for Opus
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('audio_source_type', 'pulse')  # 'alsa' or 'pulse' (pulsesrc is more robust on Jetson)
        self.declare_parameter('enable_audio', False)  # False = no audio (headless); True = pulsesrc or alsasrc
        self.declare_parameter('enable_stereo', True)
        self.declare_parameter('camera_mode', 'mono')  # 'mono' or 'stereo'
        self.declare_parameter('video_source', 'v4l2')  # 'v4l2' or 'ros' (ros = /camera/output/compressed from camera_manager)
        self.declare_parameter('allow_operator_takeover', False)  # True = new operator kicks old; False = reject new when busy
        self.declare_parameter('operator_max_time_sec', 600.0)  # Max session length (seconds). 0 = unlimited.
        self.declare_parameter('heartbeat_timeout_sec', 15.0)   # Operator considered stale after this many seconds without heartbeat
        self.declare_parameter('turn_url', '')       # e.g. 'turn:relay.example.com:3478'
        self.declare_parameter('turn_username', '')
        self.declare_parameter('turn_credential', '')
        # When False, operators won't send their camera/mic to the robot.
        # Re-read on each control-channel open so changing it via
        # `ros2 param set /webrtc_node enable_operator_media true` takes
        # effect on the NEXT operator session without a service restart.
        self.declare_parameter('enable_operator_media', False)

        # Get parameters
        self.signaling_port = self.get_parameter('signaling_server_port').value
        self.video_device = self.get_parameter('video_device').value
        self.video_width = self.get_parameter('video_width').value
        self.video_height = self.get_parameter('video_height').value
        self.video_framerate = self.get_parameter('video_framerate').value
        self.video_bitrate = self.get_parameter('video_bitrate').value
        self.video_codec = self.get_parameter('video_codec').value
        self.video_cpu_usage = self.get_parameter('video_cpu_usage').value
        self.audio_bitrate = self.get_parameter('audio_bitrate').value
        self.audio_device = self.get_parameter('audio_device').value
        self.audio_source_type = self.get_parameter('audio_source_type').value
        self.enable_audio = self.get_parameter('enable_audio').value
        self.enable_stereo = self.get_parameter('enable_stereo').value
        self.camera_mode = self.get_parameter('camera_mode').value
        self.video_source = self.get_parameter('video_source').value
        self.allow_operator_takeover = self.get_parameter('allow_operator_takeover').value
        self.operator_max_time_sec = float(self.get_parameter('operator_max_time_sec').value)
        self.heartbeat_timeout_sec = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.turn_url = self.get_parameter('turn_url').value
        self.turn_username = self.get_parameter('turn_username').value
        self.turn_credential = self.get_parameter('turn_credential').value

        # Auto-fetch Cloudflare TURN credentials if env vars are set
        if not self.turn_url:
            self._fetch_cf_turn_credentials()

        # Initialize GStreamer
        Gst.init(None)

        # WebRTC peer connections (can have multiple peers)
        self.peers: Dict[str, Dict] = {}

        # Single operator enforcement: track active operator and connection time
        self.active_operator: Optional[str] = None
        self.operator_connected_at: Optional[float] = None
        self.last_heartbeat_time: Optional[float] = None

        # Echo ducking: reduce robot mic volume when operator audio is active
        self._mic_volume_elem = None
        self._op_audio_active_until = 0.0

        # Current active camera
        self.active_camera = 'front'  # 'front', 'back', 'arm', 'feet'
        
        # Operator video frame counter
        self._operator_video_frame_count = 0

        # Send pipeline frame counter (for diagnostic logging)
        self._send_frame_count = 0
        
        # Publisher for teleop commands from DataChannel
        self.teleop_pub = self.create_publisher(
            TeleopCommand,
            'teleop_commands',
            10
        )

        # Publisher for signaling messages
        self.signaling_pub = self.create_publisher(
            String,
            'webrtc/signaling_out',
            10
        )

        # Publisher for operator video (from operator -> robot via data channel)
        self.operator_video_pub = self.create_publisher(
            CompressedImage,
            'operator/video/compressed',
            10
        )

        # Publisher for operator audio PCM data (Int16 LE, mono, from data channel)
        self.operator_audio_pub = self.create_publisher(
            UInt8MultiArray,
            'operator/audio/raw',
            10
        )

        # Publisher for operator audio status
        self.operator_audio_status_pub = self.create_publisher(
            String,
            'operator/audio/status',
            10
        )
        
        self.get_logger().info('Operator media publishers created:')
        self.get_logger().info('  - /operator/video/compressed (sensor_msgs/CompressedImage)')
        self.get_logger().info('  - /operator/audio/raw (std_msgs/UInt8MultiArray) — PCM Int16 LE mono')
        self.get_logger().info('  - /operator/audio/status (std_msgs/String)')

        # Publisher for robot audio (raw PCM from mic, for session recorder)
        self.robot_audio_pub = self.create_publisher(
            UInt8MultiArray,
            'robot/audio/raw',
            10
        )
        self.get_logger().info('  - /robot/audio/raw (std_msgs/UInt8MultiArray) — PCM Int16 LE mono')

        # Subscriber for camera switching commands
        self.camera_switch_sub = self.create_subscription(
            String,
            'camera_switch',
            self.camera_switch_callback,
            10
        )

        # Signaling callbacks in a reentrant group so they're never blocked by camera/timer callbacks
        from rclpy.callback_groups import ReentrantCallbackGroup
        self._signaling_cb_group = ReentrantCallbackGroup()

        self.peer_joined_sub = self.create_subscription(
            String, 'webrtc/peer_joined', self._on_peer_joined, 10,
            callback_group=self._signaling_cb_group
        )
        self.peer_left_sub = self.create_subscription(
            String, 'webrtc/peer_left', self._on_peer_left, 10,
            callback_group=self._signaling_cb_group
        )
        self.signaling_in_sub = self.create_subscription(
            String, 'webrtc/signaling_in', self._on_signaling_in, 10,
            callback_group=self._signaling_cb_group
        )

        # Publisher for operator status (for centralized server integration)
        self.operator_status_pub = self.create_publisher(String, 'robot/operator_status', 10)

        # /vr_teleop carries the flat 25-float atomic payload from the WebXR
        # client (see useVRTeleopSender.ts). humanoid_kinematics_node has a
        # MutuallyExclusive callback that consumes this and does head+arms+
        # grippers+cmd_vel publishing in a single solve, eliminating the
        # multi-subscriber multi-IK-call race the old TeleopCommand path had.
        self.vr_teleop_pub = self.create_publisher(Float32MultiArray, 'vr_teleop', 10)

        if self.video_source == 'ros':
            self.camera_sub = self.create_subscription(
                CompressedImage,
                'camera/output/compressed',
                self._on_camera_output,
                10
            )
            self.get_logger().info('WebRTC video from ROS topic /camera/output/compressed')
        else:
            self.camera_sub = None

        # Run GLib main loop in a background thread so GStreamer signals (on-negotiation-needed, on-ice-candidate) dispatch.
        # rclpy.spin does not run GLib; without a running main loop, no offer is ever sent.
        def _run_glib():
            try:
                GLib.MainLoop().run()
            except Exception:
                pass
        self._glib_thread = threading.Thread(target=_run_glib, daemon=True)
        self._glib_thread.start()

        # Timer for periodic status updates + staleness/timeout checks (every 1 second)
        self.status_timer = self.create_timer(1.0, self._periodic_check)

        self.get_logger().info('WebRTC Node initialized')
        self.get_logger().info('Signaling via webrtc/signaling_out (signaling_bridge connects to server as robot)')
        self.get_logger().info(f'Operator policy: {"takeover allowed" if self.allow_operator_takeover else "single operator enforced"}')
        self.get_logger().info(f'Max session time: {self.operator_max_time_sec}s, heartbeat timeout: {self.heartbeat_timeout_sec}s')

    def _fetch_cf_turn_credentials(self):
        """Fetch TURN credentials from Cloudflare Calls API.

        Reads CF_TURN_KEY_ID and CF_TURN_API_TOKEN from environment.
        Sets self.turn_url/turn_username/turn_credential on success.
        """
        import urllib.request
        key_id = os.environ.get('CF_TURN_KEY_ID', '')
        api_token = os.environ.get('CF_TURN_API_TOKEN', '')
        if not key_id or not api_token:
            return
        url = f'https://rtc.live.cloudflare.com/v1/turn/keys/{key_id}/credentials/generate-ice-servers'
        req = urllib.request.Request(url, method='POST',
            data=json.dumps({'ttl': 86400}).encode(),
            headers={
                'Authorization': f'Bearer {api_token}',
                'Content-Type': 'application/json',
                'User-Agent': 'robot-webrtc/1.0',
            })
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read())
                servers = data.get('iceServers', [])
                ice = next((s for s in servers if 'username' in s), {})
                self.turn_username = ice.get('username', '')
                self.turn_credential = ice.get('credential', '')
                self.turn_url = 'turn:turn.cloudflare.com:3478'
                self.get_logger().info('Cloudflare TURN credentials fetched successfully')
        except Exception as e:
            self.get_logger().warn(f'Failed to fetch Cloudflare TURN credentials: {e}')

    def _auto_detect_audio_device(self) -> Optional[str]:
        """Auto-detect audio capture device from ALSA by scanning /proc/asound/cards.

        Returns an ALSA device string like 'hw:1,0', or None if no suitable device found.
        Searches by card name so the result is correct regardless of USB port.
        """
        # Names to look for, in priority order
        KNOWN_NAMES = ['VR.Cam', 'USB Audio', 'USB audio']
        try:
            result = subprocess.run(
                ['arecord', '-l'], capture_output=True, text=True, timeout=3
            )
            if result.returncode != 0:
                self.get_logger().warn('arecord -l failed; cannot auto-detect audio')
                return None

            for name in KNOWN_NAMES:
                for line in result.stdout.splitlines():
                    if name in line and line.startswith('card '):
                        # e.g. "card 1: V02 [VR.Cam 02], device 0: USB Audio [USB Audio]"
                        card_num = line.split(':')[0].replace('card ', '').strip()
                        # Extract device number after "device "
                        dev_part = line.split('device ')[-1] if 'device ' in line else '0'
                        dev_num = dev_part.split(':')[0].strip()
                        device = f'hw:{card_num},{dev_num}'
                        self.get_logger().info(f'Auto-detected ALSA audio: {device} ({name})')
                        return device

        except Exception as e:
            self.get_logger().warn(f'Audio auto-detect failed: {e}')
        return None

    def get_connection_type(self, peer_id: str) -> str:
        """
        Determine connection type from peer ID
        - peer_id ending with '_send': operator sends media/commands to robot (robot receives)
        - peer_id ending with '_recv': operator receives media from robot (robot sends)
        - Default: 'send' for backward compatibility
        """
        if peer_id.endswith('_send'):
            return 'recv'  # Robot receives from this connection
        elif peer_id.endswith('_recv'):
            return 'send'  # Robot sends to this connection
        return 'send'  # Default for backward compatibility (old single-connection clients)
    
    @staticmethod
    def _extract_base_operator_id(peer_id: str) -> str:
        """Extract base operator ID from a peer_id like 'operator_abc123_recv' -> 'operator_abc123'."""
        if peer_id.endswith('_send'):
            return peer_id[:-5]
        elif peer_id.endswith('_recv'):
            return peer_id[:-5]
        return peer_id

    def _on_peer_joined(self, msg: String):
        pid = (msg.data or '').strip()
        if not pid:
            return
        
        # Skip duplicate peer-joined (signaling server may broadcast twice for initial registration)
        if pid in self.peers:
            return
        
        connection_type = self.get_connection_type(pid)
        self.get_logger().info(f'Peer joined: {pid} (connection type: {connection_type})')
        
        # For two-connection architecture, we track base operator ID
        base_operator_id = self._extract_base_operator_id(pid)
        
        # Single operator enforcement: check if we already have an active operator
        if self.active_operator is not None:
            # Allow same operator to establish both connections (_recv and _send)
            if self.active_operator == base_operator_id:
                self.get_logger().info(f'Operator {base_operator_id} adding connection: {pid}')
                GLib.idle_add(lambda p=pid: self.add_peer(p))
                return

            # Different operator — allow takeover if: flag set, heartbeat stale, or session expired
            if self.allow_operator_takeover or self._is_operator_stale() or self._is_operator_expired():
                reason = 'takeover allowed' if self.allow_operator_takeover else (
                    'heartbeat stale' if self._is_operator_stale() else 'session expired')
                self.get_logger().info(f'Operator takeover ({reason}): {self.active_operator} -> {base_operator_id}')
                GLib.idle_add(lambda old=self.active_operator, new=pid: self._takeover_operator(old, new))
            else:
                self.get_logger().warn(f'Operator {base_operator_id} rejected: robot busy with {self.active_operator}')
                self.send_signaling_message(pid, {
                    'type': 'session_rejected',
                    'reason': 'Robot is currently being operated by another user. Please try again later.'
                })
        else:
            # No active operator, accept this one (active_operator set in add_peer on success)
            GLib.idle_add(lambda p=pid: self.add_peer(p))

    def _on_peer_left(self, msg: String):
        pid = (msg.data or '').strip()
        if pid:
            GLib.idle_add(lambda p=pid: self.remove_peer(p))

    def _on_signaling_in(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('webrtc/signaling_in invalid JSON')
            return
        peer_id = data.get('peer_id') or data.get('from')
        if not peer_id:
            return
        self.get_logger().info(f'signaling_in: type={data.get("type")} peer={peer_id}')
        self.handle_signaling_message(peer_id, data)

    # -------------------------------------------------------------------------
    # Pipeline creation
    # -------------------------------------------------------------------------

    def create_pipeline(self, peer_id: str) -> Gst.Pipeline:
        """Create GStreamer pipeline based on connection type."""
        connection_type = self.get_connection_type(peer_id)
        self.get_logger().info(f'Creating {connection_type} pipeline for peer {peer_id}')
        
        if connection_type == 'recv':
            return self._create_recv_pipeline(peer_id)
        return self._create_send_pipeline(peer_id)
    
    def _create_recv_pipeline(self, peer_id: str) -> Gst.Pipeline:
        """Create data-channel-only pipeline (operator -> robot).
        
        No media lines: operator video/audio arrive as binary blobs on data channels,
        completely bypassing GStreamer 1.20 webrtcbin receive bugs.
        """
        self.get_logger().info(f'Creating data-channel-only pipeline for peer {peer_id}')
        
        pipeline = Gst.Pipeline.new(f'recv-pipeline-{peer_id}')
        if not pipeline:
            self.get_logger().error('Failed to create GStreamer pipeline')
            return None
        
        webrtc = Gst.ElementFactory.make('webrtcbin', 'webrtc')
        if not webrtc:
            self.get_logger().error('Failed to create webrtcbin element')
            return None
        
        webrtc.set_property('stun-server', 'stun://stun.l.google.com:19302')
        if self.turn_url and self.turn_username and self.turn_credential:
            turn_uri = f'turn://{self.turn_username}:{self.turn_credential}@{self.turn_url.replace("turn://", "").replace("turn:", "")}'
            webrtc.set_property('turn-server', turn_uri)
            self.get_logger().info(f'TURN relay configured for recv pipeline')

        pipeline.add(webrtc)

        webrtc.connect('on-negotiation-needed', self.on_negotiation_needed, peer_id)
        webrtc.connect('on-ice-candidate', self.on_ice_candidate, peer_id)
        webrtc.connect('on-data-channel', self.on_data_channel, peer_id)

        # Monitor ICE/DTLS state changes
        webrtc.connect('notify::ice-connection-state', self._on_ice_state_change, peer_id)
        webrtc.connect('notify::connection-state', self._on_connection_state_change, peer_id)

        self.get_logger().info(f'Data-channel-only pipeline created for peer {peer_id}')
        return pipeline
    
    def _create_send_pipeline(self, peer_id: str) -> Gst.Pipeline:
        """Create send-only pipeline (robot -> operator)"""
        use_testsrc = (self.video_device or '').strip().lower() in ('', 'videotestsrc', '1', 'true')
        if self.video_source == 'ros':
            import glob
            has_camera = bool(glob.glob('/dev/video*'))
            if has_camera:
                self.get_logger().info(
                    'Using appsrc (video from /camera/output/compressed) — '
                    'stereo_camera_node already resized to publish size'
                )
                # Note: NO videoscale here. Adding videoscale dynamically into
                # this GStreamer 1.20 pipeline triggers a webrtcbin ICE race
                # that segfaults under combined load. stereo_camera_node does
                # the resize before publishing instead, so this pipeline keeps
                # the known-stable topology from before.
                video_src = (
                    'appsrc name=ros_video do-timestamp=true is-live=true format=time caps="image/jpeg" ! '
                    'jpegdec ! videoconvert ! '
                )
            else:
                self.get_logger().warn('No camera device found — using test pattern as fallback')
                video_src = (
                    f'videotestsrc pattern=ball ! video/x-raw,width={self.video_width},height={self.video_height},'
                    f'framerate={self.video_framerate}/1 ! videoconvert ! '
                )
        elif use_testsrc:
            self.get_logger().info('Using videotestsrc (no camera) for local testing')
            video_src = (
                f'videotestsrc ! video/x-raw,width={self.video_width},height={self.video_height},'
                f'framerate={self.video_framerate}/1 ! videoconvert ! '
            )
        else:
            video_src = (
                f'v4l2src device={self.video_device} ! '
                f'video/x-raw,width={self.video_width},height={self.video_height},'
                f'framerate={self.video_framerate}/1 ! videoconvert ! '
            )

        audio_pipeline = ''
        if self.enable_audio:
            audio_dev = self.audio_device
            audio_src = None
            if audio_dev == 'test':
                self.get_logger().info('Using audiotestsrc (test tone) for local testing')
                audio_src = 'audiotestsrc wave=sine freq=440 volume=0.3'
            elif audio_dev in ('auto', 'default'):
                # Auto-detect via ALSA (works without PulseAudio)
                detected = self._auto_detect_audio_device()
                if detected:
                    audio_src = f'alsasrc device={detected}'
                else:
                    self.get_logger().warn('No audio capture device found — disabling audio')
            elif self.audio_source_type == 'pulse':
                audio_src = f'pulsesrc device={audio_dev}' if audio_dev != 'default' else 'pulsesrc'
            else:
                audio_src = f'alsasrc device={audio_dev}'

            if audio_src is not None:
                # Noise suppression: prefer webrtcdsp, fall back to audiodynamic gate
                noise_chain = ''
                if Gst.ElementFactory.find('webrtcdsp'):
                    noise_chain = 'webrtcdsp noise-suppression-level=3 echo-cancel=false gain-control=false ! '
                    self.get_logger().info('Audio: using webrtcdsp noise suppression')
                elif Gst.ElementFactory.find('audiodynamic'):
                    noise_chain = 'audiodynamic characteristics=soft-knee mode=compressor threshold=0.01 ratio=0.3 ! '
                    self.get_logger().info('Audio: using audiodynamic noise gate')

                audio_pipeline = (
                    f'{audio_src} ! audioconvert ! audioresample ! '
                    f'{noise_chain}'
                    f'volume name=mic_volume ! '
                    f'tee name=audiotee '
                    f'audiotee. ! queue ! opusenc bitrate={self.audio_bitrate} ! rtpopuspay pt=97 ! queue ! webrtc. '
                    f'audiotee. ! queue ! audio/x-raw,format=S16LE,rate=48000,channels=1,layout=interleaved ! '
                    f'appsink name=robot_audio_sink emit-signals=true sync=false'
                )

        # Configure video encoder based on codec choice
        if self.video_codec.lower() == 'vp9':
            video_enc = f'vp9enc target-bitrate={self.video_bitrate} deadline=1 cpu-used={self.video_cpu_usage} keyframe-max-dist=90 name=video_encoder'
            video_pay = 'rtpvp9pay pt=96 mtu=1200'
        elif self.video_codec.lower() == 'h264':
            if Gst.ElementFactory.find('nvv4l2h264enc'):
                # Jetson hardware H.264 (zero CPU cost)
                video_enc = (
                    f'nvvidconv ! video/x-raw(memory:NVMM) ! '
                    f'nvv4l2h264enc bitrate={self.video_bitrate} preset-level=1 '
                    f'iframeinterval=90 name=video_encoder ! h264parse'
                )
                self.get_logger().info('H.264: using NVIDIA hardware encoder')
            else:
                # key-int-max=30 at 30fps → keyframe every 1s, not every 3s.
                # On lossy Quest/Pico wifi, a single dropped P-frame would
                # freeze the picture until the next IDR; with key-int-max=90
                # that wait was 3 s, and if the IDR itself lost a packet it
                # stretched to 5 s+. Intra-refresh smears each "keyframe"
                # across the GOP so there is no big packet burst to drop in
                # the first place. bframes=0 is implied by tune=zerolatency
                # but we set it explicitly for clarity.
                video_enc = (
                    f'x264enc bitrate={self.video_bitrate // 1000} '
                    f'speed-preset=ultrafast tune=zerolatency '
                    f'key-int-max=30 intra-refresh=true bframes=0 '
                    f'name=video_encoder'
                )
                self.get_logger().info('H.264: using x264enc software encoder')
            video_pay = 'rtph264pay pt=96 config-interval=1 mtu=1200'
        else:  # VP8
            video_enc = f'vp8enc deadline=1 target-bitrate={self.video_bitrate} cpu-used={self.video_cpu_usage} keyframe-max-dist=90 name=video_encoder'
            video_pay = 'rtpvp8pay pt=96 mtu=1200'

        # Build TURN property for GStreamer pipeline if configured
        turn_prop = ''
        if self.turn_url and self.turn_username and self.turn_credential:
            turn_host = self.turn_url.replace('turn://', '').replace('turn:', '')
            turn_prop = f'turn-server=turn://{self.turn_username}:{self.turn_credential}@{turn_host}'
            self.get_logger().info('TURN relay configured for send pipeline')

        pipeline_str = f"""
        webrtcbin name=webrtc bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 {turn_prop}

        {video_src}
        queue ! {video_enc} ! {video_pay} ! queue ! webrtc.

        {audio_pipeline}
        """

        self.get_logger().info(f'Creating send-only pipeline for peer {peer_id}')
        self.get_logger().info(f'Video: {self.video_codec.upper()} @ {self.video_bitrate//1000}kbps, {self.video_width}x{self.video_height}@{self.video_framerate}fps')
        if audio_pipeline:
            self.get_logger().info(f'Audio: Opus @ {self.audio_bitrate//1000}kbps')
        else:
            self.get_logger().info('Audio: disabled (no audio device available)')
        pipeline = Gst.parse_launch(pipeline_str)
        
        if not pipeline:
            self.get_logger().error('Failed to create GStreamer pipeline')
            return None

        # Configure appsrc for ROS video
        if self.video_source == 'ros':
            appsrc = pipeline.get_by_name('ros_video')
            if appsrc:
                appsrc.set_property('caps', Gst.Caps.from_string('image/jpeg'))
                appsrc.set_property('block', False)
                appsrc.set_property('stream-type', 0)

        # Connect robot audio appsink → ROS publisher
        robot_audio_sink = pipeline.get_by_name('robot_audio_sink')
        if robot_audio_sink:
            robot_audio_sink.connect('new-sample', self._on_robot_audio_sample)

        # Store mic volume element for echo ducking
        mic_vol = pipeline.get_by_name('mic_volume')
        if mic_vol:
            self._mic_volume_elem = mic_vol

        webrtc = pipeline.get_by_name('webrtc')
        if not webrtc:
            self.get_logger().error('Failed to get webrtcbin element')
            return None

        webrtc.connect('on-negotiation-needed', self.on_negotiation_needed, peer_id)
        webrtc.connect('on-ice-candidate', self.on_ice_candidate, peer_id)
        webrtc.connect('on-data-channel', self.on_data_channel, peer_id)
        
        webrtc.connect('notify::ice-connection-state', self._on_ice_state_change, peer_id)
        webrtc.connect('notify::connection-state', self._on_connection_state_change, peer_id)

        return pipeline

    # -------------------------------------------------------------------------
    # SDP / ICE helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def _normalize_sdp(sdp_text: str) -> str:
        """SDP normalization for browser compatibility.

        Fixes:
        - Removes ``a=rtcp-mux-only`` (not supported by all browsers).
        - Strips payload types > 127 from m= lines and their associated
          a=rtpmap/a=fmtp/a=rtcp-fb attributes.  GStreamer webrtcbin can
          emit pt=255 (e.g. for telephone-event) which browsers reject
          as invalid per RFC 3551.
        """
        if not sdp_text or not isinstance(sdp_text, str):
            return sdp_text or ''

        lines = sdp_text.replace('\r\n', '\n').replace('\r', '\n').split('\n')

        # --- pass 1: collect invalid PTs and fix m= lines ----------------
        invalid_pts: set[str] = set()
        cleaned: list[str] = []
        for ln in lines:
            if ln.strip() == 'a=rtcp-mux-only':
                continue
            if ln.startswith('m='):
                parts = ln.split()
                if len(parts) >= 4:
                    # parts[0:3] = "m=audio 9 UDP/TLS/RTP/SAVPF", rest = PTs
                    header = parts[:3]
                    valid_pts = []
                    for pt in parts[3:]:
                        try:
                            if int(pt) > 127:
                                invalid_pts.add(pt)
                                continue
                        except ValueError:
                            pass
                        valid_pts.append(pt)
                    # If all PTs removed, disable m-section (port 0)
                    if not valid_pts:
                        header[1] = '0'
                        valid_pts = ['0']
                    ln = ' '.join(header + valid_pts)
            cleaned.append(ln)

        # --- pass 2: remove attribute lines for invalid PTs ---------------
        if invalid_pts:
            out: list[str] = []
            for ln in cleaned:
                skip = False
                for pt in invalid_pts:
                    if (ln.startswith(f'a=rtpmap:{pt} ')
                            or ln.startswith(f'a=fmtp:{pt} ')
                            or ln.startswith(f'a=rtcp-fb:{pt} ')):
                        skip = True
                        break
                if not skip:
                    out.append(ln)
        else:
            out = cleaned

        result = '\r\n'.join(out)
        if not result.endswith('\r\n'):
            result += '\r\n'
        return result

    def _on_ice_state_change(self, webrtc, pspec, peer_id):
        try:
            state = webrtc.get_property('ice-connection-state')
            self.get_logger().info(f'[ICE] peer={peer_id} ice-connection-state={state}')
        except Exception as e:
            self.get_logger().warn(f'[ICE] Could not get ice-connection-state for {peer_id}: {e}')

    def _on_connection_state_change(self, webrtc, pspec, peer_id):
        try:
            state = webrtc.get_property('connection-state')
            self.get_logger().info(f'[CONN] peer={peer_id} connection-state={state}')
        except Exception as e:
            self.get_logger().warn(f'[CONN] Could not get connection-state for {peer_id}: {e}')

    # -------------------------------------------------------------------------
    # Signaling (offer / answer / ICE)
    # -------------------------------------------------------------------------

    def on_negotiation_needed(self, webrtc, peer_id):
        """Handle negotiation needed. For recv (operator_send), browser creates the offer so we ignore this signal."""
        peer_data = self.peers.get(peer_id)
        if not peer_data:
            return
        connection_type = peer_data.get('connection_type', 'send')
        if connection_type == 'recv':
            self.get_logger().info(f'Ignoring on-negotiation-needed for {peer_id} (browser will offer)')
            return
        # Guard against duplicate negotiation (GStreamer may fire this signal
        # multiple times as pads are added; only the first one should create an offer)
        if peer_data.get('negotiation_done'):
            self.get_logger().info(f'Ignoring duplicate on-negotiation-needed for {peer_id}')
            return
        peer_data['negotiation_done'] = True
        self.get_logger().info(f'Negotiation needed for peer {peer_id}')
        promise = Gst.Promise.new_with_change_func(
            self.on_offer_created, webrtc, peer_id
        )
        webrtc.emit('create-offer', None, promise)

    def on_offer_created(self, promise, webrtc, peer_id):
        """Handle created offer"""
        promise.wait()
        reply = promise.get_reply()

        if not reply:
            self.get_logger().error('Failed to create offer')
            return

        offer = reply.get_value('offer')
        promise = Gst.Promise.new()
        webrtc.emit('set-local-description', offer, promise)
        promise.interrupt()

        raw_sdp = offer.sdp.as_text()
        sdp_text = self._normalize_sdp(raw_sdp)
        if raw_sdp != sdp_text:
            self.get_logger().info(f'SDP normalized for {peer_id} (stripped invalid payload types)')
        self.send_signaling_message(peer_id, {
            'type': 'offer',
            'sdp': sdp_text
        })

    def on_ice_candidate(self, webrtc, mlineindex, candidate, peer_id):
        """Handle ICE candidate"""
        self.send_signaling_message(peer_id, {
            'type': 'ice',
            'candidate': candidate,
            'sdpMLineIndex': mlineindex
        })

    def handle_signaling_message(self, peer_id: str, message: dict):
        """Handle incoming signaling message."""
        msg_type = message.get('type')
        
        if msg_type == 'answer':
            sdp = message.get('sdp')
            if sdp is not None:
                GLib.idle_add(lambda p=peer_id, s=sdp: self.handle_answer(p, s))
        elif msg_type == 'offer':
            sdp = message.get('sdp')
            if sdp is not None:
                self.get_logger().info(f'Received offer from {peer_id}')
                GLib.idle_add(lambda p=peer_id, s=sdp: self.handle_offer(p, s))
        elif msg_type == 'ice':
            GLib.idle_add(lambda p=peer_id, m=message: self.handle_ice_candidate(p, m))

    def handle_offer(self, peer_id: str, sdp_text: str):
        """Handle offer from browser (data-channel-only for operator_send)."""
        if peer_id not in self.peers:
            # Queue the offer so add_peer can flush it once the peer dict entry exists
            self.get_logger().info(f'Offer from {peer_id} arrived before add_peer — queuing for later')
            # Store on a temporary dict keyed by peer_id; add_peer will check this
            if not hasattr(self, '_pending_offers'):
                self._pending_offers = {}
            self._pending_offers[peer_id] = sdp_text
            return
        
        self.get_logger().info(f'Processing offer from {peer_id}')
        
        ret, sdp = GstSdp.SDPMessage.new_from_text(sdp_text)
        if ret != GstSdp.SDPResult.OK:
            self.get_logger().error('Failed to parse offer')
            return
            
        offer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.OFFER, sdp
        )
        
        pipeline = self.peers[peer_id]['pipeline']
        webrtc = pipeline.get_by_name('webrtc')
        
        self.peers[peer_id]['answer_applied'] = False
        self.peers[peer_id]['ice_queue'] = []
        
        # Set remote description (the offer) — wait for completion before flushing ICE
        promise = Gst.Promise.new()
        webrtc.emit('set-remote-description', offer, promise)
        promise.wait()

        # Remote description applied, allow ICE to be applied
        self.peers[peer_id]['answer_applied'] = True
        for mline, cand in self.peers[peer_id].get('ice_queue', []):
            webrtc.emit('add-ice-candidate', mline, cand)
        self.peers[peer_id]['ice_queue'] = []
        
        # Create and set local description (answer)
        promise = Gst.Promise.new_with_change_func(
            self.on_answer_created, webrtc, peer_id
        )
        webrtc.emit('create-answer', None, promise)
    
    def on_answer_created(self, promise, webrtc, peer_id):
        """Handle created answer for browser-offer flow."""
        promise.wait()
        reply = promise.get_reply()
        if not reply:
            self.get_logger().error('Failed to create answer')
            return
        answer = reply.get_value('answer')
        if answer is None:
            self.get_logger().error('Answer value is None')
            return
        try:
            desc_promise = Gst.Promise.new()
            webrtc.emit('set-local-description', answer, desc_promise)
            desc_promise.interrupt()
        except Exception as e:
            self.get_logger().error(f'set-local-description failed: {e}')
            return
        sdp_text = self._normalize_sdp(answer.sdp.as_text())
        self.send_signaling_message(peer_id, {'type': 'answer', 'sdp': sdp_text})
        self.get_logger().info(f'Answer sent to {peer_id}')

    def handle_answer(self, peer_id: str, sdp_text: str):
        """Handle SDP answer from browser (for send connection where robot offered)"""
        if peer_id not in self.peers:
            self.get_logger().error(f'Unknown peer {peer_id}')
            return
        
        self.get_logger().info(f'Received SDP answer from {peer_id}')
            
        ret, sdp = GstSdp.SDPMessage.new_from_text(sdp_text)
        if ret != GstSdp.SDPResult.OK:
            self.get_logger().error('Failed to parse SDP answer')
            return
            
        answer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.ANSWER, sdp
        )
        
        pipeline = self.peers[peer_id]['pipeline']
        webrtc = pipeline.get_by_name('webrtc')
        
        # Wait for remote description to be fully applied before flushing ICE
        promise = Gst.Promise.new()
        webrtc.emit('set-remote-description', answer, promise)
        promise.wait()
        self.peers[peer_id]['answer_applied'] = True

        # Flush ICE from browser that arrived before the answer
        for mline, cand in self.peers[peer_id].get('ice_queue', []):
            webrtc.emit('add-ice-candidate', mline, cand)
        self.peers[peer_id]['ice_queue'] = []
        self.get_logger().info(f'Set remote description (answer) for peer {peer_id}')

    def handle_ice_candidate(self, peer_id: str, message: dict):
        """Handle ICE candidate from browser."""
        if peer_id not in self.peers:
            return
        mline = message.get('sdpMLineIndex', 0)
        cand = message.get('candidate', '')
        if not self.peers[peer_id].get('answer_applied', False):
            self.peers[peer_id].setdefault('ice_queue', []).append((mline, cand))
            return
        pipeline = self.peers[peer_id]['pipeline']
        webrtc = pipeline.get_by_name('webrtc')
        webrtc.emit('add-ice-candidate', mline, cand)

    # -------------------------------------------------------------------------
    # Data channel handlers
    # -------------------------------------------------------------------------

    def on_data_channel(self, webrtc, channel, peer_id):
        """Handle incoming data channel from browser. Routes by channel label."""
        label = channel.get_property('label')
        self.get_logger().info(f'Data channel received for peer {peer_id}: {label}')
        
        if label == 'control':
            channel.connect('on-open', self._on_control_channel_open, peer_id)
            channel.connect('on-message-string', self._on_control_channel_message, peer_id)
        elif label == 'video':
            channel.connect('on-open', self._on_video_channel_open, peer_id)
            channel.connect('on-message-data', self._on_video_channel_data, peer_id)
        elif label == 'audio':
            channel.connect('on-open', self._on_audio_channel_open, peer_id)
            channel.connect('on-message-data', self._on_audio_channel_data, peer_id)
        else:
            # Unknown channel label — treat as control for backward compatibility
            self.get_logger().warn(f'Unknown data channel label: {label}, treating as control')
            channel.connect('on-open', self._on_control_channel_open, peer_id)
            channel.connect('on-message-string', self._on_control_channel_message, peer_id)

    def _on_control_channel_open(self, channel, peer_id):
        label = channel.get_property('label')
        self.get_logger().info(f'Control data channel opened for peer {peer_id}: {label}')
        if peer_id in self.peers:
            self.peers[peer_id]['data_channel'] = channel
        # Broadcast operator-side config so the webapp can mux. Re-read the
        # ROS param each time so toggling it without a restart works for
        # the NEXT operator that connects.
        enable_op_media = bool(self.get_parameter('enable_operator_media').value)
        self.send_data_channel_message(peer_id, {
            'type': 'config',
            'enable_operator_media': enable_op_media,
        })
        self.get_logger().info(
            f'Sent config to {peer_id}: enable_operator_media={enable_op_media}'
        )

    def _on_video_channel_open(self, channel, peer_id):
        self.get_logger().info(f'Video data channel opened for peer {peer_id}')
        if peer_id in self.peers:
            self.peers[peer_id]['video_channel'] = channel

    def _on_audio_channel_open(self, channel, peer_id):
        self.get_logger().info(f'Audio data channel opened for peer {peer_id}')
        if peer_id in self.peers:
            self.peers[peer_id]['audio_channel'] = channel

    def _on_control_channel_message(self, channel, message, peer_id):
        """Handle JSON control messages on the 'control' data channel."""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'teleop_command':
                self.process_teleop_command(data)
            elif data.get('type') == 'vr_teleop':
                # Flat 25-float atomic VR payload. Forward as-is on /vr_teleop;
                # humanoid_kinematics_node owns the single callback that does
                # head + dual-arm IK + grippers + cmd_vel publishes from it.
                v = data.get('v')
                if isinstance(v, list) and len(v) == 25:
                    ma = Float32MultiArray()
                    ma.data = [float(x) for x in v]
                    self.vr_teleop_pub.publish(ma)
                else:
                    self.get_logger().warn(
                        f'vr_teleop message ignored: expected 25-element list, '
                        f'got {type(v).__name__} (len={len(v) if isinstance(v, list) else "?"})'
                    )
            elif data.get('type') == 'camera_switch':
                self.switch_camera(data.get('camera', 'front'))
            elif data.get('type') == 'heartbeat':
                now = self.get_clock().now().nanoseconds / 1e9
                self.last_heartbeat_time = now
                self.send_data_channel_message(peer_id, {'type': 'heartbeat_ack', 'timestamp': data.get('timestamp')})
            elif data.get('type') == 'ping':
                self.send_data_channel_message(peer_id, {'type': 'pong', 'timestamp': data.get('timestamp')})
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse control message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing control message: {e}')

    def _on_video_channel_data(self, channel, data, peer_id):
        """Handle binary JPEG frames on the 'video' data channel.
        Published to /operator/video/compressed as CompressedImage."""
        try:
            if data is None:
                return
            # GLib.Bytes -> Python bytes
            jpeg_data = data.get_data()
            if not jpeg_data or len(jpeg_data) == 0:
                return
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'operator_camera_{peer_id}'
            msg.format = 'jpeg'
            msg.data = bytes(jpeg_data)
            
            self.operator_video_pub.publish(msg)
            self._operator_video_frame_count += 1
            if self._operator_video_frame_count <= 3 or self._operator_video_frame_count % 100 == 0:
                self.get_logger().info(
                    f'Published operator video frame #{self._operator_video_frame_count} '
                    f'({len(jpeg_data)} bytes) from {peer_id}'
                )
        except Exception as e:
            self.get_logger().error(f'Error publishing operator video: {e}')

    def _on_audio_channel_data(self, channel, data, peer_id):
        """Handle binary PCM audio chunks on the 'audio' data channel.
        Publishes raw PCM Int16 LE bytes to /operator/audio/raw and status
        to /operator/audio/status."""
        try:
            if data is None:
                return
            audio_data = data.get_data()
            if not audio_data or len(audio_data) == 0:
                return

            # Duck robot mic while operator audio is flowing (echo suppression)
            if self._mic_volume_elem is not None:
                self._mic_volume_elem.set_property('volume', 0.15)
                self._op_audio_active_until = time.monotonic() + 0.3

            # Publish raw PCM bytes for downstream consumers
            audio_msg = UInt8MultiArray()
            audio_msg.data = list(bytes(audio_data))
            self.operator_audio_pub.publish(audio_msg)

            # Publish status (lightweight, for monitoring)
            msg = String()
            msg.data = json.dumps({
                'peer_id': peer_id,
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'buffer_size': len(audio_data),
                'status': 'receiving'
            })
            self.operator_audio_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing operator audio: {e}')

    def _on_robot_audio_sample(self, appsink):
        """GStreamer callback: publish robot mic PCM to /robot/audio/raw."""
        sample = appsink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if ok:
            audio_msg = UInt8MultiArray()
            audio_msg.data = list(bytes(info.data))
            self.robot_audio_pub.publish(audio_msg)
            buf.unmap(info)
        return Gst.FlowReturn.OK

    # -------------------------------------------------------------------------
    # Teleop command processing
    # -------------------------------------------------------------------------

    def process_teleop_command(self, data: dict):
        """Convert DataChannel message to ROS TeleopCommand"""
        msg = TeleopCommand()
        msg.mode = data.get('mode', 'idle')
        msg.emergency_stop = data.get('emergency_stop', False)
        msg.timestamp_us = data.get('timestamp_us', 0)
        msg.sequence_number = data.get('sequence_number', 0)

        # Default gripper to -1.0 (sentinel for "no command") so messages
        # without gripper_position don't accidentally drive the gripper.
        msg.left_arm.gripper_position = -1.0
        msg.right_arm.gripper_position = -1.0

        # Parse left arm command
        if 'left_arm' in data:
            left_data = data['left_arm']
            msg.left_arm.command_type = left_data.get('command_type', '')
            msg.left_arm.arm_name = 'left'

            if 'joint_positions' in left_data:
                msg.left_arm.joint_positions = left_data['joint_positions']

            if 'end_effector_pose' in left_data:
                pose_data = left_data['end_effector_pose']
                msg.left_arm.end_effector_pose.position.x = pose_data['position']['x']
                msg.left_arm.end_effector_pose.position.y = pose_data['position']['y']
                msg.left_arm.end_effector_pose.position.z = pose_data['position']['z']
                msg.left_arm.end_effector_pose.orientation.w = pose_data['orientation']['w']
                msg.left_arm.end_effector_pose.orientation.x = pose_data['orientation']['x']
                msg.left_arm.end_effector_pose.orientation.y = pose_data['orientation']['y']
                msg.left_arm.end_effector_pose.orientation.z = pose_data['orientation']['z']

            if 'delta_position' in left_data:
                dp = left_data['delta_position']
                msg.left_arm.delta_position.x = float(dp.get('x', 0.0))
                msg.left_arm.delta_position.y = float(dp.get('y', 0.0))
                msg.left_arm.delta_position.z = float(dp.get('z', 0.0))
            if 'gripper_position' in left_data:
                msg.left_arm.gripper_position = float(left_data['gripper_position'])

        # Parse right arm command
        if 'right_arm' in data:
            right_data = data['right_arm']
            msg.right_arm.command_type = right_data.get('command_type', '')
            msg.right_arm.arm_name = 'right'

            if 'joint_positions' in right_data:
                msg.right_arm.joint_positions = right_data['joint_positions']

            if 'end_effector_pose' in right_data:
                pose_data = right_data['end_effector_pose']
                msg.right_arm.end_effector_pose.position.x = float(pose_data['position']['x'])
                msg.right_arm.end_effector_pose.position.y = float(pose_data['position']['y'])
                msg.right_arm.end_effector_pose.position.z = float(pose_data['position']['z'])
                msg.right_arm.end_effector_pose.orientation.w = float(pose_data['orientation']['w'])
                msg.right_arm.end_effector_pose.orientation.x = float(pose_data['orientation']['x'])
                msg.right_arm.end_effector_pose.orientation.y = float(pose_data['orientation']['y'])
                msg.right_arm.end_effector_pose.orientation.z = float(pose_data['orientation']['z'])

            if 'delta_position' in right_data:
                dp = right_data['delta_position']
                msg.right_arm.delta_position.x = float(dp.get('x', 0.0))
                msg.right_arm.delta_position.y = float(dp.get('y', 0.0))
                msg.right_arm.delta_position.z = float(dp.get('z', 0.0))
            if 'gripper_position' in right_data:
                msg.right_arm.gripper_position = float(right_data['gripper_position'])

        # Parse head command (thumbstick mode: delta_rotation → head_pose.position for yaw/pitch)
        if 'head' in data:
            head_data = data['head']
            if 'delta_rotation' in head_data:
                dr = head_data['delta_rotation']
                # Encode head deltas in head_pose.position (teleop_controller reads these)
                msg.head_pose.position.x = float(dr.get('yaw', 0.0))
                msg.head_pose.position.y = float(dr.get('pitch', 0.0))
                # position.z unused for head

        # VR pose fields (ik_control mode): full 6-DoF head + controller poses in Unity world frame.
        # humanoid_kinematics_node uses unity_pose_to_ros_se3 to convert; sending raw Unity coords is correct.
        def _set_pose(pose_msg, pose_data):
            pos = pose_data.get('position') or {}
            pose_msg.position.x = float(pos.get('x', 0.0))
            pose_msg.position.y = float(pos.get('y', 0.0))
            pose_msg.position.z = float(pos.get('z', 0.0))
            ori = pose_data.get('orientation') or {}
            pose_msg.orientation.x = float(ori.get('x', 0.0))
            pose_msg.orientation.y = float(ori.get('y', 0.0))
            pose_msg.orientation.z = float(ori.get('z', 0.0))
            pose_msg.orientation.w = float(ori.get('w', 1.0))

        if 'head_pose' in data:
            _set_pose(msg.head_pose, data['head_pose'])
        if 'left_controller_pose' in data:
            _set_pose(msg.left_controller_pose, data['left_controller_pose'])
        if 'right_controller_pose' in data:
            _set_pose(msg.right_controller_pose, data['right_controller_pose'])

        # Parse drive (wheels) command
        if 'drive' in data:
            drive_data = data['drive']
            msg.drive_linear = float(drive_data.get('linear', 0.0))
            msg.drive_angular = float(drive_data.get('angular', 0.0))

        self.teleop_pub.publish(msg)

    def send_data_channel_message(self, peer_id: str, data: dict):
        """Send message via control DataChannel."""
        channel = self.peers.get(peer_id, {}).get('data_channel')
        if channel:
            channel.emit('send-string', json.dumps(data))

    def send_signaling_message(self, peer_id: str, message: dict):
        """Send signaling message (SDP/ICE)"""
        msg = String()
        msg.data = json.dumps({
            'peer_id': peer_id,
            **message
        })
        self.signaling_pub.publish(msg)

    # -------------------------------------------------------------------------
    # Camera / ROS helpers
    # -------------------------------------------------------------------------

    def _on_camera_output(self, msg: CompressedImage):
        """Push /camera/output/compressed frames to all peers' appsrc when video_source=ros."""
        with self._peers_lock:
            peers_snapshot = [(pid, d.get('appsrc')) for pid, d in self.peers.items()]
        for peer_id, appsrc in peers_snapshot:
            if appsrc is None:
                continue
            try:
                buf = Gst.Buffer.new_wrapped(bytes(msg.data))
                ret = appsrc.emit('push-buffer', buf)
                self._send_frame_count += 1
                if self._send_frame_count <= 5 or self._send_frame_count % 300 == 0:
                    # Log pipeline state on first few frames and every 10s @30fps
                    pipeline = self.peers.get(peer_id, {}).get('pipeline')
                    pipe_state = pipeline.get_state(0)[1].value_nick if pipeline else '?'
                    self.get_logger().info(
                        f'[SEND-DIAG] frame={self._send_frame_count} peer={peer_id} '
                        f'push-ret={ret} jpeg_len={len(msg.data)} pipe_state={pipe_state}'
                    )
                    # Check pipeline bus for errors
                    if pipeline:
                        bus = pipeline.get_bus()
                        if bus:
                            bus_msg = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING)
                            if bus_msg:
                                if bus_msg.type == Gst.MessageType.ERROR:
                                    err, debug = bus_msg.parse_error()
                                    self.get_logger().error(f'[SEND-DIAG] Pipeline error: {err.message} | {debug}')
                                else:
                                    warn, debug = bus_msg.parse_warning()
                                    self.get_logger().warn(f'[SEND-DIAG] Pipeline warning: {warn.message} | {debug}')
            except Exception as e:
                self.get_logger().error(f'push buffer: {e}')

    def camera_switch_callback(self, msg: String):
        """Handle camera switch request"""
        self.switch_camera(msg.data)

    def switch_camera(self, camera: str):
        """Switch active camera and force keyframe"""
        self.get_logger().info(f'Switching camera to: {camera}')
        self.active_camera = camera
        
        for peer_id, peer_data in self.peers.items():
            pipeline = peer_data['pipeline']
            video_encoder = pipeline.get_by_name('video_encoder')
            if video_encoder:
                event = Gst.Event.new_custom(
                    Gst.EventType.CUSTOM_DOWNSTREAM,
                    Gst.Structure.new_from_string('GstForceKeyUnit')
                )
                video_encoder.send_event(event)

    # -------------------------------------------------------------------------
    # Peer lifecycle
    # -------------------------------------------------------------------------

    def add_peer(self, peer_id: str):
        """Add new WebRTC peer"""
        with self._peers_lock:
            if peer_id in self.peers:
                self.get_logger().warn(f'Peer {peer_id} already exists')
                return

        # Build the pipeline OUTSIDE the lock — create_pipeline can take
        # seconds (pactl probe, GStreamer element init) and must not block
        # _on_camera_output frame delivery.
        connection_type = self.get_connection_type(peer_id)
        pipeline = self.create_pipeline(peer_id)
        if not pipeline:
            return

        with self._peers_lock:
            # Re-check after releasing and re-acquiring (another thread may
            # have added the same peer while we were building the pipeline).
            if peer_id in self.peers:
                self.get_logger().warn(f'Peer {peer_id} already exists (added while pipeline was building)')
                return

            self.peers[peer_id] = {
                'connection_type': connection_type,
                'pipeline': pipeline,
                'data_channel': None,
                'video_channel': None,
                'audio_channel': None,
                'appsrc': pipeline.get_by_name('ros_video') if self.video_source == 'ros' and connection_type == 'send' else None,
                'ice_queue': [],
                'answer_applied': False,
                'negotiation_done': False,
                'pending_offer': None,
            }

        # Start pipeline
        ret = pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'Pipeline set_state(PLAYING) returned {ret} for peer {peer_id}')
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error(f'Pipeline state change to PLAYING failed for peer {peer_id}')
            with self._peers_lock:
                self.peers.pop(peer_id, None)
            return

        # Non-blocking bus error check (don't stall GLib main loop)
        bus = pipeline.get_bus()
        if bus:
            msg = bus.pop_filtered(Gst.MessageType.ERROR)
            if msg:
                err, debug = msg.parse_error()
                self.get_logger().error(f'Pipeline error for peer {peer_id}: {err.message}')
                with self._peers_lock:
                    self.peers.pop(peer_id, None)
                return

        # For send connections: GStreamer on-negotiation-needed signal fires
        # automatically when the pipeline reaches PLAYING — do NOT trigger
        # manually here, or we get duplicate offers that desync local/remote SDP.

        # Flush any offer that arrived before this peer was added
        pending = getattr(self, '_pending_offers', {}).pop(peer_id, None)
        if pending:
            self.get_logger().info(f'Flushing queued offer for {peer_id}')
            GLib.idle_add(lambda p=peer_id, s=pending: self.handle_offer(p, s))

        # Mark as active operator
        base_operator_id = self._extract_base_operator_id(peer_id)
        if not self.active_operator:
            now = self.get_clock().now().nanoseconds / 1e9
            self.active_operator = base_operator_id
            self.operator_connected_at = now
            self.last_heartbeat_time = now  # Initialize heartbeat so we don't immediately evict
            self.get_logger().info(f'Added peer {peer_id} (operator {base_operator_id} is now active)')
            self.publish_operator_status()
        else:
            self.get_logger().info(f'Added peer {peer_id} and started pipeline (operator {base_operator_id} adding another connection)')

    def remove_peer(self, peer_id: str):
        """Remove WebRTC peer"""
        with self._peers_lock:
            if peer_id not in self.peers:
                return
            pipeline = self.peers[peer_id]['pipeline']
            del self.peers[peer_id]
        # Tear down pipeline on background thread — set_state(NULL) can block
        # for seconds (DTLS shutdown) and would stall the GLib main loop,
        # preventing new operators from connecting.
        threading.Thread(
            target=lambda p=pipeline: p.set_state(Gst.State.NULL),
            daemon=True
        ).start()
        
        base_operator_id = self._extract_base_operator_id(peer_id)
        remaining_connections = [
            p for p in self.peers.keys()
            if self._extract_base_operator_id(p) == base_operator_id
        ]
        
        if self.active_operator == base_operator_id and len(remaining_connections) == 0:
            self.active_operator = None
            self.operator_connected_at = None
            self.last_heartbeat_time = None
            self.get_logger().info(f'Removed peer {peer_id} - operator {base_operator_id} fully disconnected, robot now free')
            self.publish_operator_status()
            # Restart service after a delay for a clean GStreamer slate.
            # 10s gives takeover time to complete; if no new operator joins,
            # restart cleans up any leaked GStreamer state.
            # Auto-restart is for production where systemd manages the
            # process — when launched interactively (test_local_full or a
            # plain `ros2 launch`), `systemctl restart robot-teleop` spawns a
            # duplicate stack instead of cycling the current one (it shells
            # out via sudo to a different unit's process tree). Gate it on
            # INVOCATION_ID, which systemd sets for every service it spawns.
            running_under_systemd = bool(os.environ.get('INVOCATION_ID'))
            if running_under_systemd:
                self.get_logger().info('Operator disconnected — will restart service in 10s unless new operator joins')
                def _maybe_restart():
                    if self.active_operator is None and len(self.peers) == 0:
                        self.get_logger().info('No new operator joined — restarting service')
                        subprocess.Popen(['sudo', 'systemctl', 'restart', 'robot-teleop'])
                    else:
                        self.get_logger().info('New operator joined — skipping restart')
                threading.Timer(10.0, _maybe_restart).start()
            else:
                self.get_logger().info('Operator disconnected — auto-restart skipped (not running under systemd)')
        else:
            self.get_logger().info(f'Removed peer {peer_id} ({len(remaining_connections)} connections remaining for operator {base_operator_id})')

    def _takeover_operator(self, old_operator_id: str, new_peer_id: str):
        """Handle operator takeover: remove all connections of old operator, add new"""
        self.get_logger().info(f'Operator takeover: removing all connections for {old_operator_id}, adding {new_peer_id}')
        # Remove all connections belonging to the old operator
        old_peers = [
            p for p in list(self.peers.keys())
            if self._extract_base_operator_id(p) == old_operator_id
        ]
        for p in old_peers:
            self.remove_peer(p)
        self.add_peer(new_peer_id)

    def _is_operator_stale(self) -> bool:
        """Check if the current operator's heartbeat has timed out."""
        if self.active_operator is None or self.last_heartbeat_time is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.last_heartbeat_time) > self.heartbeat_timeout_sec

    def _is_operator_expired(self) -> bool:
        """Check if the current operator has exceeded the max session time."""
        if self.active_operator is None or self.operator_connected_at is None:
            return False
        if self.operator_max_time_sec <= 0:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.operator_connected_at) > self.operator_max_time_sec

    def _evict_operator(self, reason: str):
        """Remove all connections for the active operator."""
        if self.active_operator is None:
            return
        operator_id = self.active_operator
        self.get_logger().info(f'Evicting operator {operator_id}: {reason}')
        peers_to_remove = [
            p for p in list(self.peers.keys())
            if self._extract_base_operator_id(p) == operator_id
        ]
        # Notify operator before tearing down
        for p in peers_to_remove:
            self.send_data_channel_message(p, {
                'type': 'session_ended',
                'reason': reason,
            })
        for p in peers_to_remove:
            GLib.idle_add(lambda pid=p: self.remove_peer(pid))

    def _periodic_check(self):
        """Runs every 1s: check heartbeat staleness, echo duck release, publish status.
        Session expiry only triggers takeover when a new operator joins (see _on_peer_joined)."""
        if self.active_operator is not None:
            if self._is_operator_stale():
                self._evict_operator('Heartbeat lost — connection appears stale.')
        # Release mic ducking when operator audio stops
        if (self._mic_volume_elem is not None
                and self._op_audio_active_until > 0
                and time.monotonic() > self._op_audio_active_until):
            self._mic_volume_elem.set_property('volume', 1.0)
            self._op_audio_active_until = 0.0
        self.publish_operator_status()

    def publish_operator_status(self):
        """Publish operator status for centralized server integration"""
        msg = String()
        current_time = self.get_clock().now().nanoseconds / 1e9
        status = {
            'status': 'busy' if self.active_operator else 'free',
            'operator_id': self.active_operator,
            'timestamp': current_time,
        }
        if self.operator_connected_at:
            status['connection_duration_sec'] = current_time - self.operator_connected_at
        
        msg.data = json.dumps(status)
        self.operator_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
