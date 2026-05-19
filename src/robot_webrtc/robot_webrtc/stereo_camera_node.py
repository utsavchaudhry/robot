#!/usr/bin/env python3
"""
Camera capture node — opens a single /dev/video* device (MJPEG-preferred),
JPEG-encodes each frame, and publishes a CompressedImage straight to the
topic WebRTC consumes (`/camera/output/compressed`).

Designed for VR.Cam-style USB stereo cameras that deliver an already-packed
2:1 SBS frame on one /dev/video* node — we serve the whole frame as-is and
let the frontend (webapp) decide how to render it (right-eye crop on flat
screens, per-eye sampling in VR).

This replaces the older split + recombine pipeline (stereo_camera_node →
camera_manager_node) which was a frequent source of bugs (black frames when
the wrong device was opened, slow throughput when one side stalled, etc.).
"""

import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_camera_node")

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 2400)
        self.declare_parameter("height", 1200)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("output_topic", "/camera/output/compressed")
        # MJPG fourcc is what VR.Cam-class cameras need to advertise the
        # higher SBS resolutions. Set to 'NONE' to skip the fourcc set call.
        self.declare_parameter("fourcc", "MJPG")
        # Resize captured frames to this size before JPEG-encoding and
        # publishing. Set publish_width=0 to skip the resize (publish at
        # native capture size). Keeping the published frames small saves CPU
        # on the receiving end (webrtc_node's jpegdec + x264enc) and shrinks
        # the ROS topic payload too.
        self.declare_parameter("publish_width", 1600)
        self.declare_parameter("publish_height", 800)

        self.device = self.get_parameter("device").value.strip()
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        topic = self.get_parameter("output_topic").value
        self.fourcc = self.get_parameter("fourcc").value.strip()
        self.publish_width = int(self.get_parameter("publish_width").value)
        self.publish_height = int(self.get_parameter("publish_height").value)

        self.pub = self.create_publisher(CompressedImage, topic, 10)
        self.cap = None
        self._open_device()

        # Frame-counter so the user gets quick feedback that frames are flowing.
        self._frame_count = 0
        self._first_frame_logged = False
        self._last_perf_log = time.monotonic()

        self.timer = self.create_timer(1.0 / max(1.0, self.fps), self.timer_cb)

        self.get_logger().info(
            f"Capture: {self.device} at {self.width}x{self.height} @ {self.fps}fps "
            f"(fourcc={self.fourcc or 'auto'}, jpeg_q={self.jpeg_quality}) → {topic}"
        )

    def _open_device(self):
        if not os.path.exists(self.device):
            self.get_logger().error(f"Device {self.device} does not exist")
            return
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"Could not open {self.device}")
            return
        # Order matters on many UVC drivers: set fourcc FIRST, then resolution.
        # Otherwise the resolution set call falls back to whatever the default
        # raw format supports (often a much smaller size or YUYV-only).
        if self.fourcc and self.fourcc.upper() != "NONE":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Log what the driver actually accepted vs what we requested — useful
        # when investigating "I asked for 2400x1200 but topic_hz is wrong".
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Opened {self.device}: driver says {actual_w}x{actual_h} @ {actual_fps:.1f}fps"
        )
        self.cap = cap

    def timer_cb(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        if not self._first_frame_logged:
            h, w = frame.shape[:2]
            self.get_logger().info(
                f"First frame from {self.device}: {w}x{h} (aspect {w/max(1,h):.2f})"
            )
            self._first_frame_logged = True

        # Resize before JPEG-encode so downstream (webrtc_node's jpegdec +
        # x264enc) gets a smaller raster. The 2400x1200 native size was
        # tipping over the LattePanda's CPU under combined load.
        if self.publish_width > 0 and self.publish_height > 0:
            h, w = frame.shape[:2]
            if (w, h) != (self.publish_width, self.publish_height):
                frame = cv2.resize(
                    frame,
                    (self.publish_width, self.publish_height),
                    interpolation=cv2.INTER_LINEAR,
                )

        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self.pub.publish(msg)

        self._frame_count += 1
        now = time.monotonic()
        if now - self._last_perf_log >= 5.0:
            fps = self._frame_count / (now - self._last_perf_log)
            self.get_logger().info(f"PERF {fps:.1f} fps over last 5s")
            self._frame_count = 0
            self._last_perf_log = now

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
