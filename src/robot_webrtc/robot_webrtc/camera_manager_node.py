#!/usr/bin/env python3
"""
Camera Manager Node — subscribes to raw Image topics from all cameras,
merges them into a 2:1 Side-by-Side (left|right) frame, and publishes a
single CompressedImage output for WebRTC.

Single pipeline only: ALWAYS produces 2:1 SBS. The frontend is responsible
for any cropping (showing one eye on flat screens) or per-eye splitting (in
VR). No stream_mode / flat_eye knobs to fight with.

When active_camera is 'stereo', uses the stereo left/right feeds.
When active_camera is a mono camera (front/back/arm/feet), duplicates that
frame to both eyes so the output stays a consistent 2:1 SBS.

Subscribes to:
  /camera/{front,back,arm,feet}/image_raw  (sensor_msgs/Image)
  /camera/stereo/{left,right}/image_raw    (sensor_msgs/Image)
  camera_switch  (std_msgs/String) — e.g. 'front', 'stereo'

Publishes:
  camera/output/compressed  (sensor_msgs/CompressedImage) — JPEG at 30 fps
  camera_status             (std_msgs/String)
"""

import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Optional, Dict


def _gst_element_available(name: str) -> bool:
    """Check if a GStreamer element/plugin is available."""
    Gst.init(None)
    return Gst.ElementFactory.find(name) is not None


class CameraManagerNode(Node):
    """
    ROS2 Node for managing multiple cameras and preparing stereo SBS streaming
    """

    def __init__(self):
        super().__init__('camera_manager_node')

        # Declare parameters
        self.declare_parameter('front_camera_topic', '/camera/front/image_raw')
        self.declare_parameter('back_camera_topic', '/camera/back/image_raw')
        self.declare_parameter('arm_camera_topic', '/camera/arm/image_raw')
        self.declare_parameter('feet_camera_topic', '/camera/feet/image_raw')
        self.declare_parameter('stereo_left_topic', '/camera/stereo/left/image_raw')
        self.declare_parameter('stereo_right_topic', '/camera/stereo/right/image_raw')
        self.declare_parameter('output_width', 2560)
        self.declare_parameter('output_height', 720)
        self.declare_parameter('active_camera', 'front')
        self.declare_parameter('jpeg_quality', 85)

        # Get parameters
        self.front_camera_topic = self.get_parameter('front_camera_topic').value
        self.back_camera_topic = self.get_parameter('back_camera_topic').value
        self.arm_camera_topic = self.get_parameter('arm_camera_topic').value
        self.feet_camera_topic = self.get_parameter('feet_camera_topic').value
        self.stereo_left_topic = self.get_parameter('stereo_left_topic').value
        self.stereo_right_topic = self.get_parameter('stereo_right_topic').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        self.active_camera = self.get_parameter('active_camera').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.bridge = CvBridge()

        # Latest images from each camera
        self.camera_images: Dict[str, np.ndarray] = {
            'front': None,
            'back': None,
            'arm': None,
            'feet': None
        }
        
        # Latest stereo images
        self.stereo_left: Optional[np.ndarray] = None
        self.stereo_right: Optional[np.ndarray] = None

        # Create subscribers for each camera
        self.front_sub = self.create_subscription(
            Image,
            self.front_camera_topic,
            lambda msg: self.camera_callback(msg, 'front'),
            10
        )

        self.back_sub = self.create_subscription(
            Image,
            self.back_camera_topic,
            lambda msg: self.camera_callback(msg, 'back'),
            10
        )

        self.arm_sub = self.create_subscription(
            Image,
            self.arm_camera_topic,
            lambda msg: self.camera_callback(msg, 'arm'),
            10
        )

        self.feet_sub = self.create_subscription(
            Image,
            self.feet_camera_topic,
            lambda msg: self.camera_callback(msg, 'feet'),
            10
        )

        # Stereo camera subscribers
        self.stereo_left_sub = self.create_subscription(
            Image,
            self.stereo_left_topic,
            self.stereo_left_callback,
            10
        )

        self.stereo_right_sub = self.create_subscription(
            Image,
            self.stereo_right_topic,
            self.stereo_right_callback,
            10
        )

        # Subscriber for camera switch commands
        self.switch_sub = self.create_subscription(
            String,
            'camera_switch',
            self.switch_callback,
            10
        )

        # Publisher for output video stream (to be consumed by WebRTC)
        self.output_pub = self.create_publisher(
            CompressedImage,
            'camera/output/compressed',
            10
        )

        # Publisher for camera status
        self.status_pub = self.create_publisher(
            String,
            'camera_status',
            10
        )

        # Timer for processing and publishing output
        self.timer = self.create_timer(1.0 / 30.0, self.process_and_publish)  # 30 FPS

        # Republish status periodically so late-joining nodes pick it up
        self._status_timer = self.create_timer(2.0, self._publish_status)

        # ── Hardware JPEG encoder (NVIDIA nvjpegenc via GStreamer) ────────
        Gst.init(None)
        self._use_hw_jpeg = _gst_element_available('nvjpegenc')
        if self._use_hw_jpeg:
            self._init_hw_jpeg_encoder()
            self.get_logger().info('Using NVIDIA nvjpegenc for hardware JPEG encoding')
        else:
            self._hw_enc_pipeline = None
            self.get_logger().info('nvjpegenc not available, using cv2.imencode (CPU)')

        # ── Timing instrumentation ────────────────────────────────────────
        self._perf_count = 0
        self._perf_sbs = 0.0
        self._perf_resize = 0.0
        self._perf_encode = 0.0
        self._perf_total = 0.0

        # Publish initial state so late-joining subscribers know the active camera.
        status = String()
        status.data = f'active:{self.active_camera}'
        self.status_pub.publish(status)

        self.get_logger().info('Camera Manager Node initialized')
        self.get_logger().info(f'Active camera: {self.active_camera}')
        self.get_logger().info(f'Output: 2:1 SBS at {self.output_width}x{self.output_height} (frontend handles crop/split)')
        self.get_logger().info(f'OpenCV: {cv2.__file__}')

    def _publish_status(self):
        msg = String()
        msg.data = f'active:{self.active_camera}'
        self.status_pub.publish(msg)

    def camera_callback(self, msg: Image, camera_name: str):
        """Store latest image from camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_images[camera_name] = cv_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from {camera_name}: {e}')

    def stereo_left_callback(self, msg: Image):
        """Store latest left stereo image"""
        try:
            self.stereo_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert stereo left image: {e}')

    def stereo_right_callback(self, msg: Image):
        """Store latest right stereo image"""
        try:
            self.stereo_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert stereo right image: {e}')

    def switch_callback(self, msg: String):
        """Handle camera switch command"""
        new_camera = msg.data.lower()
        
        valid_cameras = ['front', 'back', 'arm', 'feet', 'stereo']
        if new_camera in valid_cameras:
            old_camera = self.active_camera
            self.active_camera = new_camera
            self.get_logger().info(f'Switched camera from {old_camera} to {new_camera}')
            
            # Publish status
            status_msg = String()
            status_msg.data = f'active:{new_camera}'
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().warn(f'Invalid camera name: {new_camera}')

    def create_sbs_image(self, left_img: np.ndarray, right_img: np.ndarray) -> np.ndarray:
        """
        Create Side-by-Side stereo image
        Left half = left eye, Right half = right eye
        """
        # Ensure both images have the same height
        if left_img.shape[0] != right_img.shape[0]:
            target_height = min(left_img.shape[0], right_img.shape[0])
            left_img = cv2.resize(left_img, (left_img.shape[1], target_height))
            right_img = cv2.resize(right_img, (right_img.shape[1], target_height))

        # Concatenate horizontally
        sbs_image = np.hstack((left_img, right_img))
        
        return sbs_image

    def resize_image(self, img: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
        """Resize image to target dimensions while maintaining aspect ratio"""
        h, w = img.shape[:2]
        
        # Calculate scaling to fit within target dimensions
        scale = min(target_width / w, target_height / h)
        
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Create canvas and center the image
        canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        
        y_offset = (target_height - new_h) // 2
        x_offset = (target_width - new_w) // 2
        
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        return canvas

    def _init_hw_jpeg_encoder(self):
        """Create a persistent GStreamer pipeline: appsrc → nvjpegenc → appsink."""
        pipeline_str = (
            'appsrc name=src emit-signals=false is-live=true format=time '
            'caps=video/x-raw,format=BGR '
            '! videoconvert '
            f'! nvjpegenc quality={self.jpeg_quality} '
            '! appsink name=sink emit-signals=false sync=false'
        )
        self._hw_enc_pipeline = Gst.parse_launch(pipeline_str)
        self._hw_enc_src = self._hw_enc_pipeline.get_by_name('src')
        self._hw_enc_sink = self._hw_enc_pipeline.get_by_name('sink')
        self._hw_enc_pipeline.set_state(Gst.State.PLAYING)

    def _jpeg_encode(self, image: np.ndarray) -> Optional[bytes]:
        """Encode BGR image to JPEG using hardware (nvjpegenc) or CPU fallback."""
        if self._use_hw_jpeg and self._hw_enc_pipeline is not None:
            try:
                h, w, ch = image.shape
                data = image.tobytes()
                buf = Gst.Buffer.new_wrapped(data)
                buf.pts = Gst.CLOCK_TIME_NONE
                buf.duration = Gst.CLOCK_TIME_NONE

                # Update caps with current frame dimensions
                caps = Gst.Caps.from_string(
                    f'video/x-raw,format=BGR,width={w},height={h},framerate=30/1'
                )
                self._hw_enc_src.set_property('caps', caps)

                ret = self._hw_enc_src.emit('push-buffer', buf)
                if ret != Gst.FlowReturn.OK:
                    raise RuntimeError(f'push-buffer returned {ret}')

                sample = self._hw_enc_sink.emit('pull-sample')
                if sample is None:
                    raise RuntimeError('pull-sample returned None')

                out_buf = sample.get_buffer()
                success, map_info = out_buf.map(Gst.MapFlags.READ)
                if not success:
                    raise RuntimeError('buffer map failed')
                jpeg_bytes = bytes(map_info.data)
                out_buf.unmap(map_info)
                return jpeg_bytes
            except Exception as e:
                self.get_logger().warn(f'HW JPEG encode failed, falling back to CPU: {e}')
                self._use_hw_jpeg = False
                if self._hw_enc_pipeline is not None:
                    self._hw_enc_pipeline.set_state(Gst.State.NULL)

        # CPU fallback
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        _, jpeg_data = cv2.imencode('.jpg', image, encode_param)
        return jpeg_data.tobytes()

    def process_and_publish(self):
        """Process active camera and publish a 2:1 SBS frame.
        Always 2:1 — the frontend handles cropping / per-eye splitting.
        """
        t_start = time.monotonic()
        output_image = None
        t_sbs = 0.0
        t_resize = 0.0

        # Pick the left and right inputs. Stereo cam supplies two halves;
        # any other camera supplies one image that we duplicate to both eyes.
        if self.active_camera == 'stereo':
            left = self.stereo_left
            right = self.stereo_right
            # If one eye is missing, fall back to the other so we still emit a
            # frame (better than blanking the output until both arrive).
            if left is None:
                left = right
            elif right is None:
                right = left
        else:
            mono = self.camera_images.get(self.active_camera)
            left = right = mono

        if left is not None and right is not None:
            t0 = time.monotonic()
            sbs_image = self.create_sbs_image(left, right)
            t_sbs = time.monotonic() - t0
            t0 = time.monotonic()
            output_image = self.resize_image(sbs_image, self.output_width, self.output_height)
            t_resize = time.monotonic() - t0

        # Publish output
        t_encode = 0.0
        if output_image is not None:
            try:
                t0 = time.monotonic()
                jpeg_bytes = self._jpeg_encode(output_image)
                t_encode = time.monotonic() - t0

                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'{self.active_camera}_camera'
                msg.format = 'jpeg'
                msg.data = jpeg_bytes

                self.output_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f'Failed to publish output image: {e}')

        # ── Perf logging (every 30 frames) ───────────────────────────────
        t_total = time.monotonic() - t_start
        self._perf_sbs += t_sbs
        self._perf_resize += t_resize
        self._perf_encode += t_encode
        self._perf_total += t_total
        self._perf_count += 1
        if self._perf_count >= 30:
            n = self._perf_count
            self.get_logger().info(
                f'PERF [30 frames] sbs={self._perf_sbs/n*1000:.1f}ms '
                f'resize={self._perf_resize/n*1000:.1f}ms '
                f'encode={self._perf_encode/n*1000:.1f}ms '
                f'total={self._perf_total/n*1000:.1f}ms '
                f'(max_fps={n/self._perf_total:.0f})'
            )
            self._perf_count = 0
            self._perf_sbs = 0.0
            self._perf_resize = 0.0
            self._perf_encode = 0.0
            self._perf_total = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._hw_enc_pipeline is not None:
            node._hw_enc_pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
