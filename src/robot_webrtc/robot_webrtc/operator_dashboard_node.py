#!/usr/bin/env python3
"""
Operator Dashboard — displays the operator's camera feed (JPEG via WebRTC)
when connected, or the robot's own camera feed when disconnected.

Uses OpenCV for fullscreen video display and GStreamer for audio playback.

Subscribes to:
  /operator/video/compressed   (sensor_msgs/CompressedImage) — operator JPEG frames
  /operator/audio/raw          (std_msgs/UInt8MultiArray)    — PCM Int16 LE mono 48 kHz
  /robot/operator_status       (std_msgs/String)             — operator connection state
  /camera/output/compressed    (sensor_msgs/CompressedImage) — robot camera JPEG from camera_manager
  /camera_status               (std_msgs/String)             — active camera + stream mode tracking
"""

import signal
import subprocess
import threading
import json
import time

import cv2
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt8MultiArray


AUDIO_SAMPLE_RATE = 48000
AUDIO_CHANNELS = 1
WINDOW_NAME = 'Robot Dashboard'


def _get_screen_size():
    """Query screen resolution from the system. Fallback 1920x1080."""
    # Try Linux framebuffer (works on Jetson regardless of Wayland/X11)
    try:
        with open('/sys/class/graphics/fb0/virtual_size') as f:
            w, h = f.read().strip().split(',')
            return int(w), int(h)
    except Exception:
        pass
    # Try xrandr (current resolution marked with *)
    try:
        import subprocess
        out = subprocess.check_output(
            ['xrandr', '--current'], text=True, timeout=2)
        for line in out.splitlines():
            if '*' in line:
                res = line.split()[0]
                w, h = res.split('x')
                return int(w), int(h)
    except Exception:
        pass
    return 1920, 1080


def _gst_element_available(name: str) -> bool:
    Gst.init(None)
    return Gst.ElementFactory.find(name) is not None


class OperatorDashboardNode(Node):
    """ROS 2 node with OpenCV video display + GStreamer audio playback."""

    def __init__(self):
        super().__init__('operator_dashboard')
        self.get_logger().info('Operator dashboard node starting…')

        Gst.init(None)

        # State
        self.operator_connected: bool = False
        self._active_camera: str = 'stereo'
        self._stream_mode: str = 'flat'

        # Display frame (written by ROS callbacks, read by main thread)
        self._display_frame = None

        # FPS tracking
        self._fps_count = 0
        self._fps_display = 0
        self._fps_time = time.monotonic()

        # ── Hardware JPEG decoder ─────────────────────────────────────────
        self._use_hw_jpeg_dec = _gst_element_available('nvjpegdec')
        if self._use_hw_jpeg_dec:
            self._init_hw_jpeg_decoder()
            self.get_logger().info('Using NVIDIA nvjpegdec for hardware JPEG decoding')
        else:
            self._hw_dec_pipeline = None
            self.get_logger().info('nvjpegdec not available, using cv2.imdecode (CPU)')

        # ── Audio playback pipeline ───────────────────────────────────────
        self._audio_pipeline = None
        self._audio_src = None
        self._audio_ts = 0
        self._init_audio_pipeline()

        # ── ROS subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, 'operator/video/compressed', self._on_video, 10)
        self.create_subscription(
            UInt8MultiArray, 'operator/audio/raw', self._on_audio, 10)
        self.create_subscription(
            String, 'robot/operator_status', self._on_operator_status, 10)
        self.create_subscription(
            CompressedImage, 'camera/output/compressed', self._on_robot_video, 10)
        self.create_subscription(
            String, 'camera_status', self._on_camera_status, 10)

        # ── FPS update timer (1 Hz) ───────────────────────────────────────
        self.create_timer(1.0, self._update_fps)

        self.get_logger().info('Operator dashboard ready (OpenCV fullscreen display)')

    # ── Audio setup ────────────────────────────────────────────────────────

    # ALSA card names that are NOT real speakers (HDMI, built-in SoC, etc.)
    _SKIP_SINKS = ['hdmi', 'tegra', 'hda nvidia', 'hda intel', 'hda ati',
                   'displayport', 'spdif']

    def _auto_detect_audio_output(self):
        """Auto-detect a USB audio playback device via aplay -l.

        Skips HDMI / built-in sinks (no physical speaker) and only
        returns a device whose card path lives under /sys/…/usb*,
        confirming it is a real USB-connected speaker.

        Returns a 'plughw:N,M' string or None.
        """
        try:
            result = subprocess.run(
                ['aplay', '-l'], capture_output=True, text=True, timeout=3
            )
            if result.returncode != 0:
                self.get_logger().warn(
                    f'aplay -l failed (rc={result.returncode}): {result.stderr.strip()}')
                return None

            self.get_logger().info(f'Available playback devices:\n{result.stdout}')

            candidates = []
            for line in result.stdout.splitlines():
                if not line.startswith('card '):
                    continue
                low = line.lower()
                # Skip known non-speaker sinks
                if any(s in low for s in self._SKIP_SINKS):
                    self.get_logger().info(f'  skip (non-speaker): {line.strip()}')
                    continue
                parsed = self._parse_alsa_device_line(line)
                if parsed:
                    candidates.append((parsed, line.strip()))

            if not candidates:
                self.get_logger().warn('No non-HDMI playback devices found in aplay -l')
                return None

            # Verify each candidate is actually USB-connected via sysfs
            for dev, desc in candidates:
                card_num = dev.split(':')[1].split(',')[0]
                if self._is_usb_audio_card(card_num):
                    self.get_logger().info(
                        f'Auto-detected USB speaker: {dev}  ({desc})')
                    return dev
                else:
                    self.get_logger().info(
                        f'  skip (not USB-connected): {dev}  ({desc})')

            # If sysfs check is inconclusive, accept any remaining candidate
            # that has 'usb' in its aplay description
            for dev, desc in candidates:
                if 'usb' in desc.lower():
                    self.get_logger().info(
                        f'Auto-detected USB speaker (name match): {dev}  ({desc})')
                    return dev

            self.get_logger().warn(
                f'Found {len(candidates)} playback device(s) but none are USB-connected')

        except Exception as e:
            self.get_logger().warn(f'Audio output auto-detect failed: {e}')
        return None

    @staticmethod
    def _is_usb_audio_card(card_num: str) -> bool:
        """Check /sys/class/sound/cardN to see if the device is on a USB bus."""
        import os
        try:
            real = os.path.realpath(f'/sys/class/sound/card{card_num}')
            return '/usb' in real
        except Exception:
            return False

    @staticmethod
    def _parse_alsa_device_line(line: str):
        """Parse 'card N: ... device M: ...' into 'plughw:N,M'.
        Uses plughw so ALSA handles format conversion automatically."""
        try:
            card_num = line.split(':')[0].replace('card ', '').strip()
            dev_part = line.split('device ')[-1] if 'device ' in line else '0'
            dev_num = dev_part.split(':')[0].strip()
            return f'plughw:{card_num},{dev_num}'
        except Exception:
            return None

    def _init_audio_pipeline(self):
        """Build and start the GStreamer audio playback pipeline.

        Only targets the auto-detected USB speaker.  Does NOT fall back
        to autoaudiosink, which would silently pick HDMI on Jetson.
        """
        alsa_dev = self._auto_detect_audio_output()

        if not alsa_dev:
            self.get_logger().error(
                'No USB speaker detected — operator audio will not play. '
                'Check the speaker is plugged in (aplay -l on robot).')
            return

        audio_str = (
            f'appsrc name=audiosrc is-live=true format=time '
            f'caps=audio/x-raw,format=S16LE,rate={AUDIO_SAMPLE_RATE},'
            f'channels={AUDIO_CHANNELS},layout=interleaved '
            f'! queue max-size-time=150000000 min-threshold-time=50000000 '
            f'! audioconvert ! audioresample '
            f'! alsasink device={alsa_dev} sync=false'
        )
        try:
            pipeline = Gst.parse_launch(audio_str)
            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error(
                    f'alsasink {alsa_dev} failed to start — is the speaker connected?')
                pipeline.set_state(Gst.State.NULL)
                return
            self._audio_pipeline = pipeline
            self._audio_src = pipeline.get_by_name('audiosrc')
            self._audio_ts = 0
            self.get_logger().info(f'Audio playback started on {alsa_dev}')
        except Exception as e:
            self.get_logger().error(f'Audio pipeline failed ({alsa_dev}): {e}')

    # ── FPS counter ───────────────────────────────────────────────────────

    def _update_fps(self):
        now = time.monotonic()
        elapsed = now - self._fps_time
        if elapsed > 0:
            self._fps_display = int(self._fps_count / elapsed)
        self._fps_count = 0
        self._fps_time = now

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _on_operator_status(self, msg: String):
        try:
            status = json.loads(msg.data)
            self.operator_connected = status.get('status') == 'busy'
        except (json.JSONDecodeError, KeyError):
            pass

    def _on_camera_status(self, msg: String):
        data = msg.data.strip()
        if data.startswith('active:'):
            self._active_camera = data.split(':', 1)[1]
        elif data.startswith('stream_mode:'):
            self._stream_mode = data.split(':', 1)[1]

    # ── JPEG decoding ─────────────────────────────────────────────────────

    def _init_hw_jpeg_decoder(self):
        pipeline_str = (
            'appsrc name=src emit-signals=false is-live=true '
            'caps=image/jpeg '
            '! nvjpegdec '
            '! videoconvert '
            '! video/x-raw,format=BGR '
            '! appsink name=sink emit-signals=false sync=false'
        )
        self._hw_dec_pipeline = Gst.parse_launch(pipeline_str)
        self._hw_dec_src = self._hw_dec_pipeline.get_by_name('src')
        self._hw_dec_sink = self._hw_dec_pipeline.get_by_name('sink')
        self._hw_dec_pipeline.set_state(Gst.State.PLAYING)

    def _jpeg_decode(self, jpeg_bytes: bytes):
        if self._use_hw_jpeg_dec and self._hw_dec_pipeline is not None:
            try:
                buf = Gst.Buffer.new_wrapped(jpeg_bytes)
                ret = self._hw_dec_src.emit('push-buffer', buf)
                if ret != Gst.FlowReturn.OK:
                    raise RuntimeError(f'push-buffer returned {ret}')
                sample = self._hw_dec_sink.emit('pull-sample')
                if sample is None:
                    raise RuntimeError('pull-sample returned None')
                caps = sample.get_caps()
                w = caps.get_structure(0).get_value('width')
                h = caps.get_structure(0).get_value('height')
                out_buf = sample.get_buffer()
                ok, info = out_buf.map(Gst.MapFlags.READ)
                if not ok:
                    raise RuntimeError('buffer map failed')
                frame = np.frombuffer(info.data, np.uint8).reshape((h, w, 3)).copy()
                out_buf.unmap(info)
                return frame
            except Exception as e:
                self.get_logger().warn(f'HW decode failed, CPU fallback: {e}')
                self._use_hw_jpeg_dec = False
                if self._hw_dec_pipeline is not None:
                    self._hw_dec_pipeline.set_state(Gst.State.NULL)

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # ── Video callbacks ───────────────────────────────────────────────────

    def _on_robot_video(self, msg: CompressedImage):
        """Decode JPEG, crop right eye if SBS, store for display.
        Only active when operator is NOT connected."""
        if self.operator_connected:
            return
        frame = self._jpeg_decode(bytes(msg.data))
        if frame is None:
            return

        # In VR mode the output is SBS; crop to right half
        if self._stream_mode == 'vr':
            frame = frame[:, frame.shape[1] // 2:]

        self._fps_count += 1
        self._display_frame = frame

    def _on_video(self, msg: CompressedImage):
        """Operator video — decode JPEG and store for display."""
        if not self.operator_connected:
            self.operator_connected = True
            self.get_logger().info('Operator detected (video stream arrived)')
        frame = self._jpeg_decode(bytes(msg.data))
        if frame is None:
            return
        self._fps_count += 1
        self._display_frame = frame

    def _on_audio(self, msg: UInt8MultiArray):
        if self._audio_src is None:
            return
        raw = bytes(msg.data)
        if len(raw) < 2:
            return
        buf = Gst.Buffer.new_wrapped(raw)
        num_samples = len(raw) // 2
        duration = Gst.util_uint64_scale(
            num_samples, Gst.SECOND, AUDIO_SAMPLE_RATE)
        buf.pts = self._audio_ts
        buf.duration = duration
        self._audio_ts += duration
        self._audio_src.emit('push-buffer', buf)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._audio_pipeline is not None:
            self._audio_pipeline.set_state(Gst.State.NULL)
        if getattr(self, '_hw_dec_pipeline', None) is not None:
            self._hw_dec_pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorDashboardNode()

    # ROS spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # OpenCV fullscreen display on main thread
    screen_w, screen_h = _get_screen_size()
    node.get_logger().info(f'Display size: {screen_w}x{screen_h}')

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while rclpy.ok():
            frame = node._display_frame
            if frame is not None:
                # Scale preserving aspect ratio, filling one axis
                fh, fw = frame.shape[:2]
                scale = min(screen_w / fw, screen_h / fh)
                new_w = int(fw * scale)
                new_h = int(fh * scale)
                resized = cv2.resize(frame, (new_w, new_h))

                # Center on black canvas
                display = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)
                x_off = (screen_w - new_w) // 2
                y_off = (screen_h - new_h) // 2
                display[y_off:y_off + new_h, x_off:x_off + new_w] = resized

                # Draw status overlay
                if node.operator_connected:
                    label = f'OPERATOR FEED | {node._fps_display} fps'
                else:
                    label = f'ROBOT CAMERA | {node._fps_display} fps'
                cv2.putText(display, label, (10, screen_h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(33) & 0xFF
            if key == 27:  # ESC to quit
                break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
