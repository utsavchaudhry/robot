#!/usr/bin/env python3
"""
Session Recorder — records teleop sessions as a single video file.

Composites the robot's active camera view (main) with the operator's webcam
as a picture-in-picture overlay, and merges robot + operator audio into one
mixed track.  Output is an MP4 (video via cv2.VideoWriter, audio via wave,
muxed with GStreamer on stop).

Automatically starts a new recording when an operator connects and stops
when they disconnect, producing one file per session.

Subscribes to:
  /camera/output/compressed   (sensor_msgs/CompressedImage) — robot JPEG frames
  /operator/video/compressed  (sensor_msgs/CompressedImage) — operator JPEG frames
  /operator/audio/raw         (std_msgs/UInt8MultiArray)    — PCM Int16 LE mono 48 kHz
  /robot/audio/raw            (std_msgs/UInt8MultiArray)    — PCM Int16 LE mono 48 kHz
  /robot/operator_status      (std_msgs/String)             — operator connect/disconnect

Services:
  session_recorder/record     (std_srvs/SetBool)  — manual start / stop override

Publishes:
  session_recorder/is_recording (std_msgs/Bool)    — current recording state
"""

import json
import os
import queue
import wave
import time
import threading
import numpy as np
import cv2

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8MultiArray, Bool, String
from std_srvs.srv import SetBool

try:
    import boto3
    HAS_BOTO3 = True
except ImportError:
    HAS_BOTO3 = False


class SessionRecorderNode(Node):

    def __init__(self):
        super().__init__('session_recorder')
        Gst.init(None)

        # ── Parameters ────────────────────────────────────────────────────
        # Output
        self.declare_parameter('output_dir', '~/teleop_recordings')
        self.declare_parameter('output_fps', 30)
        self.declare_parameter('output_width', 1280)
        self.declare_parameter('output_height', 720)
        self.declare_parameter('filename_prefix', 'session')

        # PiP overlay
        self.declare_parameter('pip_enabled', True)
        self.declare_parameter('pip_position', 'bottom-right')
        self.declare_parameter('pip_scale', 0.25)
        self.declare_parameter('pip_margin', 20)
        self.declare_parameter('pip_opacity', 1.0)
        self.declare_parameter('pip_border_width', 2)
        self.declare_parameter('pip_border_color', [255, 255, 255])
        self.declare_parameter('pip_corner_radius', 0)

        # Video encoding
        self.declare_parameter('video_fourcc', 'mp4v')

        # Audio
        self.declare_parameter('audio_enabled', True)
        self.declare_parameter('audio_sample_rate', 48000)
        self.declare_parameter('audio_channels', 1)
        self.declare_parameter('robot_audio_volume', 1.0)
        self.declare_parameter('operator_audio_volume', 1.0)

        # Behaviour
        self.declare_parameter('auto_record', True)

        # R2 sync (credentials via env: R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY)
        self.declare_parameter('r2_enabled', False)
        self.declare_parameter('r2_account_id', '')
        self.declare_parameter('r2_bucket', 'teleop-recordings')
        self.declare_parameter('r2_prefix', 'sessions')      # key prefix in bucket
        self.declare_parameter('delete_after_upload', False)

        # ── Read params ───────────────────────────────────────────────────
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.fps = self.get_parameter('output_fps').value
        self.out_w = self.get_parameter('output_width').value
        self.out_h = self.get_parameter('output_height').value
        self.filename_prefix = self.get_parameter('filename_prefix').value

        self.pip_enabled = self.get_parameter('pip_enabled').value
        self.pip_position = self.get_parameter('pip_position').value
        self.pip_scale = self.get_parameter('pip_scale').value
        self.pip_margin = self.get_parameter('pip_margin').value
        self.pip_opacity = self.get_parameter('pip_opacity').value
        self.pip_border_width = self.get_parameter('pip_border_width').value
        self.pip_border_color = tuple(self.get_parameter('pip_border_color').value)
        self.pip_corner_radius = self.get_parameter('pip_corner_radius').value

        self.video_fourcc = self.get_parameter('video_fourcc').value

        self.audio_enabled = self.get_parameter('audio_enabled').value
        self.sample_rate = self.get_parameter('audio_sample_rate').value
        self.audio_channels = self.get_parameter('audio_channels').value
        self.robot_audio_volume = self.get_parameter('robot_audio_volume').value
        self.operator_audio_volume = self.get_parameter('operator_audio_volume').value

        # ── State ─────────────────────────────────────────────────────────
        self._recording = False
        self._video_writer: cv2.VideoWriter | None = None
        self._audio_wav: wave.Wave_write | None = None
        self._frame_timer = None
        self._auto_record = self.get_parameter('auto_record').value

        # File paths for current recording
        self._video_path: str | None = None
        self._audio_path: str | None = None
        self._output_path: str | None = None

        # Operator session tracking
        self._operator_connected = False
        self._current_operator_id: str | None = None

        # Latest decoded frames
        self._robot_frame: np.ndarray | None = None
        self._operator_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        # Audio ring-buffers (PCM s16le bytes from ROS topics)
        self._op_audio_buf = bytearray()
        self._op_audio_lock = threading.Lock()
        self._robot_audio_buf = bytearray()
        self._robot_audio_lock = threading.Lock()

        # Audio writer thread
        self._audio_thread: threading.Thread | None = None
        self._audio_stop = threading.Event()

        # Stats
        self._frames_written = 0
        self._record_start_time = 0.0

        # ── R2 sync ──────────────────────────────────────────────────────
        self._r2_enabled = self.get_parameter('r2_enabled').value
        self._r2_account_id = self.get_parameter('r2_account_id').value
        self._r2_bucket = self.get_parameter('r2_bucket').value
        self._r2_prefix = self.get_parameter('r2_prefix').value
        self._delete_after_upload = self.get_parameter('delete_after_upload').value
        self._upload_queue: queue.Queue = queue.Queue()
        self._upload_thread: threading.Thread | None = None
        self._shutdown = threading.Event()

        if self._r2_enabled:
            if not HAS_BOTO3:
                self.get_logger().error('R2 sync enabled but boto3 not installed (pip3 install boto3)')
                self._r2_enabled = False
            elif not self._r2_account_id:
                self.get_logger().error('R2 sync enabled but r2_account_id not set')
                self._r2_enabled = False
            else:
                # Scan for any un-uploaded files from previous runs
                self._enqueue_pending_uploads()
                # Start upload thread (waits for idle periods)
                self._upload_thread = threading.Thread(
                    target=self._upload_loop, daemon=True)
                self._upload_thread.start()
                self.get_logger().info(
                    f'R2 sync enabled → s3://{self._r2_bucket}/{self._r2_prefix}/')

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, '/camera/output/compressed',
            self._on_robot_frame, 10)
        self.create_subscription(
            CompressedImage, '/operator/video/compressed',
            self._on_operator_frame, 10)
        self.create_subscription(
            String, '/robot/operator_status',
            self._on_operator_status, 10)
        if self.audio_enabled:
            self.create_subscription(
                UInt8MultiArray, '/operator/audio/raw',
                self._on_operator_audio, 50)
            self.create_subscription(
                UInt8MultiArray, '/robot/audio/raw',
                self._on_robot_audio, 50)

        # ── Service (manual override) ─────────────────────────────────────
        self.create_service(SetBool, 'session_recorder/record', self._toggle_cb)

        # ── Status publisher (1 Hz) ──────────────────────────────────────
        self._status_pub = self.create_publisher(Bool, 'session_recorder/is_recording', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Session recorder ready  output_dir={self.output_dir}  '
            f'resolution={self.out_w}x{self.out_h}@{self.fps}fps  '
            f'pip={self.pip_position}@{self.pip_scale}  audio={self.audio_enabled}')

    # ══════════════════════════════════════════════════════════════════════
    #  ROS callbacks
    # ══════════════════════════════════════════════════════════════════════

    def _on_robot_frame(self, msg: CompressedImage):
        frame = self._decode_jpeg(msg.data)
        if frame is not None:
            with self._frame_lock:
                self._robot_frame = frame

    def _on_operator_frame(self, msg: CompressedImage):
        frame = self._decode_jpeg(msg.data)
        if frame is not None:
            with self._frame_lock:
                self._operator_frame = frame

    def _on_operator_status(self, msg: String):
        """Auto start/stop recording based on operator connection state."""
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return

        is_busy = status.get('status') == 'busy'
        operator_id = status.get('operator_id')

        if is_busy and not self._operator_connected:
            self._operator_connected = True
            self._current_operator_id = operator_id
            self.get_logger().info(f'Operator connected: {operator_id}')
            if self._auto_record and not self._recording:
                self._start_recording()

        elif not is_busy and self._operator_connected:
            self._operator_connected = False
            prev = self._current_operator_id
            self._current_operator_id = None
            self.get_logger().info(f'Operator disconnected: {prev}')
            if self._auto_record and self._recording:
                self._stop_recording()
            with self._frame_lock:
                self._operator_frame = None

    def _on_operator_audio(self, msg: UInt8MultiArray):
        if not self._recording:
            return
        raw = bytes(msg.data)
        if len(raw) < 2:
            return
        with self._op_audio_lock:
            self._op_audio_buf.extend(raw)

    def _on_robot_audio(self, msg: UInt8MultiArray):
        if not self._recording:
            return
        raw = bytes(msg.data)
        if len(raw) < 2:
            return
        with self._robot_audio_lock:
            self._robot_audio_buf.extend(raw)

    def _toggle_cb(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            ok, info = self._start_recording()
        else:
            ok, info = self._stop_recording()
        response.success = ok
        response.message = info
        return response

    def _publish_status(self):
        self._status_pub.publish(Bool(data=self._recording))

    # ══════════════════════════════════════════════════════════════════════
    #  Recording lifecycle
    # ══════════════════════════════════════════════════════════════════════

    def _start_recording(self) -> tuple[bool, str]:
        if self._recording:
            return False, 'Already recording'

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = time.strftime('%Y-%m-%d_%H%M%S')
        base = os.path.join(self.output_dir, f'{self.filename_prefix}_{timestamp}')
        self._video_path = base + '_video.avi'
        self._audio_path = base + '_audio.wav'
        self._output_path = base + '.mp4'

        # Open cv2 video writer
        fourcc = cv2.VideoWriter_fourcc(*self.video_fourcc)
        self._video_writer = cv2.VideoWriter(
            self._video_path, fourcc, self.fps, (self.out_w, self.out_h))
        if not self._video_writer.isOpened():
            self.get_logger().error('cv2.VideoWriter failed to open')
            return False, 'VideoWriter failed'

        # Open wav file for mixed audio
        if self.audio_enabled:
            self._audio_wav = wave.open(self._audio_path, 'wb')
            self._audio_wav.setnchannels(self.audio_channels)
            self._audio_wav.setsampwidth(2)  # int16
            self._audio_wav.setframerate(self.sample_rate)

            self._audio_stop.clear()
            self._audio_thread = threading.Thread(
                target=self._audio_writer_loop, daemon=True)
            self._audio_thread.start()

        self._frames_written = 0
        self._record_start_time = time.monotonic()
        self._recording = True
        self._frame_timer = self.create_timer(1.0 / self.fps, self._write_frame)

        self.get_logger().info(f'Recording started: {self._output_path}')
        return True, self._output_path

    def _stop_recording(self) -> tuple[bool, str]:
        if not self._recording:
            return False, 'Not recording'

        self._recording = False

        # Stop frame timer
        if self._frame_timer is not None:
            self._frame_timer.cancel()
            self.destroy_timer(self._frame_timer)
            self._frame_timer = None

        # Stop audio thread
        self._audio_stop.set()
        if self._audio_thread is not None:
            self._audio_thread.join(timeout=3.0)
            self._audio_thread = None

        # Close writers
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None

        if self._audio_wav is not None:
            self._audio_wav.close()
            self._audio_wav = None

        # Clear audio buffers
        with self._op_audio_lock:
            self._op_audio_buf.clear()
        with self._robot_audio_lock:
            self._robot_audio_buf.clear()

        elapsed = time.monotonic() - self._record_start_time
        self.get_logger().info(
            f'Recording stopped. {self._frames_written} frames in {elapsed:.1f}s. Muxing...')

        # Mux video + audio into final MP4
        self._mux_to_mp4()

        # Queue for R2 upload
        if self._r2_enabled and self._output_path and os.path.exists(self._output_path):
            self._upload_queue.put(self._output_path)

        msg = f'Recording saved: {self._output_path}'
        self.get_logger().info(msg)
        return True, msg

    def _mux_to_mp4(self):
        """Mux .avi video + .wav audio into a single .mp4 using GStreamer.

        Uses decodebin on the AVI to auto-handle whatever codec cv2 used,
        then re-encodes to H.264 + AAC for a universally playable MP4.
        """
        has_audio = (self.audio_enabled
                     and self._audio_path
                     and os.path.exists(self._audio_path)
                     and os.path.getsize(self._audio_path) > 44)  # wav header = 44 bytes

        # Pick AAC encoder
        aac_enc = None
        if has_audio:
            for enc in ('voaacenc', 'avenc_aac'):
                if Gst.ElementFactory.find(enc):
                    aac_enc = enc
                    break
            if aac_enc is None:
                self.get_logger().warn('No AAC encoder — muxing video only')
                has_audio = False

        # Video: avidemux → decode → re-encode H.264
        video_branch = (
            f'filesrc location={self._video_path} ! avidemux ! decodebin ! '
            f'videoconvert ! video/x-raw,format=I420 ! '
            f'x264enc speed-preset=ultrafast tune=zerolatency ! '
            f'queue ! mux.')

        if has_audio:
            audio_branch = (
                f'filesrc location={self._audio_path} ! wavparse ! '
                f'audioconvert ! audioresample ! {aac_enc} ! queue ! mux.')
        else:
            audio_branch = ''

        pipeline_str = (
            f'qtmux name=mux ! filesink location={self._output_path} '
            f'{video_branch} {audio_branch}')

        try:
            self.get_logger().info(f'Muxing to {self._output_path} (audio={has_audio})')
            pipeline = Gst.parse_launch(pipeline_str)
            pipeline.set_state(Gst.State.PLAYING)
            bus = pipeline.get_bus()
            msg = bus.timed_pop_filtered(
                120 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
            if msg and msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self.get_logger().error(f'Mux failed: {err.message}')
                pipeline.set_state(Gst.State.NULL)
                return
            pipeline.set_state(Gst.State.NULL)

            # Clean up intermediate files
            self._safe_remove(self._video_path)
            self._safe_remove(self._audio_path)
        except Exception as e:
            self.get_logger().error(f'Mux exception: {e} — raw files kept')

    # ══════════════════════════════════════════════════════════════════════
    #  Frame compositing & writing
    # ══════════════════════════════════════════════════════════════════════

    def _write_frame(self):
        if not self._recording or self._video_writer is None:
            return

        with self._frame_lock:
            robot = self._robot_frame
            operator = self._operator_frame

        if robot is None:
            frame = np.zeros((self.out_h, self.out_w, 3), dtype=np.uint8)
        else:
            frame = self._fit_frame(robot, self.out_w, self.out_h)

        if self.pip_enabled and operator is not None:
            frame = self._overlay_pip(frame, operator)

        self._video_writer.write(frame)
        self._frames_written += 1

    def _overlay_pip(self, base: np.ndarray, pip_img: np.ndarray) -> np.ndarray:
        """Draw picture-in-picture overlay on base frame."""
        bh, bw = base.shape[:2]

        pip_w = int(bw * self.pip_scale)
        pip_h = int(pip_img.shape[0] * (pip_w / pip_img.shape[1]))

        pip_w = min(pip_w, bw - 2 * self.pip_margin)
        pip_h = min(pip_h, bh - 2 * self.pip_margin)
        if pip_w <= 0 or pip_h <= 0:
            return base

        pip_resized = cv2.resize(pip_img, (pip_w, pip_h))

        m = self.pip_margin
        pos = self.pip_position
        if pos == 'top-left':
            x, y = m, m
        elif pos == 'top-right':
            x, y = bw - pip_w - m, m
        elif pos == 'bottom-left':
            x, y = m, bh - pip_h - m
        else:  # bottom-right
            x, y = bw - pip_w - m, bh - pip_h - m

        # Corner radius mask
        if self.pip_corner_radius > 0:
            mask = np.zeros((pip_h, pip_w), dtype=np.uint8)
            r = self.pip_corner_radius
            cv2.rectangle(mask, (r, 0), (pip_w - r, pip_h), 255, -1)
            cv2.rectangle(mask, (0, r), (pip_w, pip_h - r), 255, -1)
            cv2.circle(mask, (r, r), r, 255, -1)
            cv2.circle(mask, (pip_w - r, r), r, 255, -1)
            cv2.circle(mask, (r, pip_h - r), r, 255, -1)
            cv2.circle(mask, (pip_w - r, pip_h - r), r, 255, -1)
        else:
            mask = None

        # Border
        bdr = self.pip_border_width
        if bdr > 0:
            if self.pip_corner_radius > 0:
                br = self.pip_corner_radius + bdr
                x0, y0 = x - bdr, y - bdr
                bw2, bh2 = pip_w + 2 * bdr, pip_h + 2 * bdr
                if x0 >= 0 and y0 >= 0 and x0 + bw2 <= bw and y0 + bh2 <= bh:
                    border_mask = np.zeros((bh2, bw2), dtype=np.uint8)
                    cv2.rectangle(border_mask, (br, 0), (bw2 - br, bh2), 255, -1)
                    cv2.rectangle(border_mask, (0, br), (bw2, bh2 - br), 255, -1)
                    cv2.circle(border_mask, (br, br), br, 255, -1)
                    cv2.circle(border_mask, (bw2 - br, br), br, 255, -1)
                    cv2.circle(border_mask, (br, bh2 - br), br, 255, -1)
                    cv2.circle(border_mask, (bw2 - br, bh2 - br), br, 255, -1)
                    roi = base[y0:y0 + bh2, x0:x0 + bw2]
                    border_color = np.array(self.pip_border_color, dtype=np.uint8)
                    roi[border_mask > 0] = border_color
            else:
                x0, y0 = max(0, x - bdr), max(0, y - bdr)
                x1, y1 = min(bw, x + pip_w + bdr), min(bh, y + pip_h + bdr)
                base[y0:y1, x0:x1] = self.pip_border_color

        # Composite PiP
        roi = base[y:y + pip_h, x:x + pip_w]
        if self.pip_opacity < 1.0:
            blended = cv2.addWeighted(
                pip_resized, self.pip_opacity, roi, 1.0 - self.pip_opacity, 0)
        else:
            blended = pip_resized

        if mask is not None:
            mask3 = cv2.merge([mask, mask, mask])
            roi[:] = np.where(mask3 > 0, blended, roi)
        else:
            roi[:] = blended

        return base

    @staticmethod
    def _fit_frame(src: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
        """Resize preserving aspect ratio, letterbox on black canvas."""
        sh, sw = src.shape[:2]
        scale = min(target_w / sw, target_h / sh)
        new_w = int(sw * scale)
        new_h = int(sh * scale)
        resized = cv2.resize(src, (new_w, new_h))
        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        x_off = (target_w - new_w) // 2
        y_off = (target_h - new_h) // 2
        canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized
        return canvas

    # ══════════════════════════════════════════════════════════════════════
    #  Audio mixing & writing
    # ══════════════════════════════════════════════════════════════════════

    def _audio_writer_loop(self):
        """Drain audio buffers using wall-clock timing to stay in sync with video.

        The video writer fires at 1/fps via a ROS timer driven by wall clock.
        We match that by computing exactly how many audio samples correspond to
        the elapsed wall-clock time, reading that many from the ring buffers
        (zero-padding any shortfall), and writing them to the WAV file.
        """
        start_time = time.monotonic()
        total_samples_written = 0

        try:
            while not self._audio_stop.is_set():
                elapsed = time.monotonic() - start_time
                target_samples = int(elapsed * self.sample_rate)
                samples_needed = target_samples - total_samples_written

                if samples_needed <= 0:
                    self._audio_stop.wait(timeout=0.01)
                    continue

                bytes_needed = samples_needed * 2 * self.audio_channels

                with self._robot_audio_lock:
                    robot_raw = bytes(self._robot_audio_buf[:bytes_needed])
                    del self._robot_audio_buf[:bytes_needed]

                with self._op_audio_lock:
                    op_raw = bytes(self._op_audio_buf[:bytes_needed])
                    del self._op_audio_buf[:bytes_needed]

                mixed = self._mix_audio(op_raw, robot_raw, bytes_needed)

                if self._audio_wav is not None:
                    self._audio_wav.writeframes(mixed)

                total_samples_written += samples_needed
                self._audio_stop.wait(timeout=0.02)
        except Exception as e:
            self.get_logger().error(f'Audio thread error: {e}')

    def _mix_audio(self, op_raw: bytes, robot_raw: bytes,
                   target_bytes: int) -> bytes:
        """Mix two PCM s16le streams, applying volume scaling."""
        target_samples = target_bytes // 2

        if len(op_raw) >= 2:
            op = np.frombuffer(op_raw, dtype=np.int16)
        else:
            op = np.zeros(0, dtype=np.int16)

        if len(robot_raw) >= 2:
            robot = np.frombuffer(robot_raw, dtype=np.int16)
        else:
            robot = np.zeros(0, dtype=np.int16)

        if len(op) < target_samples:
            op = np.pad(op, (0, target_samples - len(op)))
        else:
            op = op[:target_samples]

        if len(robot) < target_samples:
            robot = np.pad(robot, (0, target_samples - len(robot)))
        else:
            robot = robot[:target_samples]

        # Gentle noise gate: attenuate quiet 10ms sub-frames per-source
        gate_thresh = 120.0  # ~-48 dBFS RMS threshold
        gate_ratio = 0.15  # Reduce quiet frames to 15% (not zero)
        frame_sz = self.sample_rate // 100  # 480 samples = 10ms
        op_f = op.astype(np.float32)
        robot_f = robot.astype(np.float32)
        for arr in (op_f, robot_f):
            for i in range(0, target_samples, frame_sz):
                end = min(i + frame_sz, target_samples)
                chunk = arr[i:end]
                if len(chunk) > 0 and np.sqrt(np.mean(chunk ** 2)) < gate_thresh:
                    arr[i:end] *= gate_ratio

        mixed = (op_f * self.operator_audio_volume +
                 robot_f * self.robot_audio_volume)
        mixed = np.clip(mixed, -32768, 32767).astype(np.int16)
        return mixed.tobytes()

    # ══════════════════════════════════════════════════════════════════════
    #  R2 sync
    # ══════════════════════════════════════════════════════════════════════

    def _enqueue_pending_uploads(self):
        """Scan output_dir for .mp4 files not yet uploaded (no .uploaded marker)."""
        if not os.path.isdir(self.output_dir):
            return
        for f in sorted(os.listdir(self.output_dir)):
            if not f.endswith('.mp4'):
                continue
            full = os.path.join(self.output_dir, f)
            marker = full + '.uploaded'
            if not os.path.exists(marker):
                self._upload_queue.put(full)
                self.get_logger().info(f'Queued pending upload: {f}')

    def _upload_loop(self):
        """Background thread: upload queued files to R2 when no operator connected."""
        s3 = boto3.client(
            's3',
            endpoint_url=f'https://{self._r2_account_id}.r2.cloudflarestorage.com',
            aws_access_key_id=os.environ.get('R2_ACCESS_KEY_ID', ''),
            aws_secret_access_key=os.environ.get('R2_SECRET_ACCESS_KEY', ''),
            region_name='auto',
        )

        while not self._shutdown.is_set():
            # Wait for a file
            try:
                filepath = self._upload_queue.get(timeout=5.0)
            except queue.Empty:
                continue

            # Wait until no operator is connected
            while self._operator_connected and not self._shutdown.is_set():
                self._shutdown.wait(timeout=5.0)
            if self._shutdown.is_set():
                break

            if not os.path.exists(filepath):
                continue

            # Build R2 key: prefix/YYYY-MM-DD/filename.mp4
            filename = os.path.basename(filepath)
            # Extract date from filename like session_2026-04-04_155303.mp4
            parts = filename.replace('.mp4', '').split('_')
            date_str = parts[1] if len(parts) >= 3 else time.strftime('%Y-%m-%d')
            key = f'{self._r2_prefix}/{date_str}/{filename}'

            try:
                self.get_logger().info(f'Uploading {filename} → r2://{self._r2_bucket}/{key}')
                s3.upload_file(filepath, self._r2_bucket, key)

                # Mark as uploaded
                marker = filepath + '.uploaded'
                with open(marker, 'w') as f:
                    f.write(key)

                if self._delete_after_upload:
                    self._safe_remove(filepath)

                self.get_logger().info(f'Upload complete: {filename}')
            except Exception as e:
                self.get_logger().error(f'Upload failed for {filename}: {e}')
                # Re-queue for retry
                self._upload_queue.put(filepath)
                # Back off before retrying
                self._shutdown.wait(timeout=30.0)

    # ══════════════════════════════════════════════════════════════════════
    #  Helpers
    # ══════════════════════════════════════════════════════════════════════

    @staticmethod
    def _decode_jpeg(data) -> np.ndarray | None:
        arr = np.frombuffer(bytes(data), dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    @staticmethod
    def _safe_remove(path):
        try:
            if path and os.path.exists(path):
                os.remove(path)
        except OSError:
            pass

    def destroy_node(self):
        if self._recording:
            self._stop_recording()
        self._shutdown.set()
        if self._upload_thread is not None:
            self._upload_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SessionRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
