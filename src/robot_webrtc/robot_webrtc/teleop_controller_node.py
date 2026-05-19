#!/usr/bin/env python3
"""
Teleop Controller Node: multiplexes VR (absolute pose) and flat screen (thumbstick deltas).

Publishes target poses to topics for the IK node to consume at its own rate:
  /left_arm/ik_target   (geometry_msgs/Pose)
  /right_arm/ik_target  (geometry_msgs/Pose)
  head/joint_commands   (sensor_msgs/JointState)  — direct 2-DOF yaw/pitch

Control mapping (flat screen / thumbstick mode, from robot's POV):
  Joystick horizontal (delta_position.x) → base_link Y (left/right)
  Joystick vertical   (delta_position.y) → base_link Z (up/down)
  1D joystick         (delta_position.z) → base_link X (forward/backward)

Safety features:
  - Linear EE velocity clamping (max_linear_velocity, m/s)
  - Head angular velocity clamping (max_angular_velocity, rad/s)
  - Smooth return to default on operator disconnect (disconnect_timeout)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from robot_interfaces.msg import TeleopCommand
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header, Empty, String
import numpy as np


class _OneEuroFilter:
    """Adaptive low-pass filter for head smoothing (mirrors robot_kinematics version)."""

    def __init__(self, n: int, rate: float,
                 min_cutoff: float = 1.5, beta: float = 0.01, d_cutoff: float = 1.0):
        self._min_cutoff = min_cutoff
        self._beta = beta
        self._d_cutoff = d_cutoff
        self._rate = rate
        self._x_prev = np.zeros(n)
        self._dx_prev = np.zeros(n)
        self._initialized = False

    def filter(self, x: np.ndarray) -> np.ndarray:
        if not self._initialized:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            self._initialized = True
            return x.copy()
        a_d = 1.0 / (1.0 + 1.0 / (2.0 * math.pi * self._d_cutoff * (1.0 / self._rate)))
        dx = (x - self._x_prev) * self._rate
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self._min_cutoff + self._beta * np.abs(dx_hat)
        te = 1.0 / self._rate
        tau = 1.0 / (2.0 * math.pi * cutoff)
        a = 1.0 / (1.0 + tau / te)
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev = x_hat.copy()
        self._dx_prev = dx_hat.copy()
        return x_hat

    def reset(self, x: np.ndarray):
        self._x_prev = x.copy()
        self._dx_prev = np.zeros_like(x)
        self._initialized = True


class TeleopControllerNode(Node):
    def __init__(self):
        super().__init__('teleop_controller_node')

        # Workspace bounds
        self.declare_parameter('min_x', 0.05)
        self.declare_parameter('max_x', 0.45)
        self.declare_parameter('min_z', -0.1)
        self.declare_parameter('max_z', 0.45)
        self.declare_parameter('min_y', -0.4)
        self.declare_parameter('max_y', 0.4)

        # Gains
        self.declare_parameter('delta_scale', 0.5)
        self.declare_parameter('head_scale', 1.0)

        # Safety limits
        self.declare_parameter('max_linear_velocity', 0.3)     # m/s for EE target
        self.declare_parameter('max_angular_velocity', 1.5)    # rad/s for head
        self.declare_parameter('return_home_alpha', 0.03)      # exponential smoothing per tick

        # Drive (wheels) scaling
        self.declare_parameter('max_drive_linear', 1.0)      # scale factor for linear velocity
        self.declare_parameter('max_drive_angular', 1.0)     # scale factor for angular velocity

        # Safety watchdog
        self.declare_parameter('command_timeout', 0.5)       # seconds without teleop → e-stop

        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.delta_scale = self.get_parameter('delta_scale').value
        self.head_scale = self.get_parameter('head_scale').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.return_home_alpha = self.get_parameter('return_home_alpha').value
        self.max_drive_linear = self.get_parameter('max_drive_linear').value
        self.max_drive_angular = self.get_parameter('max_drive_angular').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # Assumed command rate for per-tick limits
        self._cmd_dt = 0.02  # 50 Hz

        # Current target poses in base_link — fallback until FK arrives
        self.target_poses = {
            'left': self._make_pose(0.2, 0.2, 0.2),
            'right': self._make_pose(0.2, -0.2, 0.2),
        }
        self._targets_initialized = {'left': False, 'right': False}

        # Home poses — set once from first FK (default joint angles)
        self._home_poses = {
            'left': self._make_pose(0.2, 0.2, 0.2),
            'right': self._make_pose(0.2, -0.2, 0.2),
        }
        self._home_set = set()

        # Head state
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.head_yaw_limit = 1.5
        self.head_pitch_limit = 0.8

        # Operator connection state (driven by WebRTC status topic)
        self._operator_connected = False
        self._returning_home = False
        self._reset_in_progress = False  # True when user pressed Default Pose
        self._last_teleop_time = None    # ROS time of last teleop command
        self._commanding = False         # True while teleop commands are flowing

        # Subscriptions
        self.teleop_sub = self.create_subscription(
            TeleopCommand, 'teleop_commands', self.teleop_callback, 10
        )
        # WebRTC operator status — drives disconnect detection
        self.create_subscription(
            String, 'robot/operator_status', self._on_operator_status, 10
        )

        # Publishers
        self.left_target_pub = self.create_publisher(Pose, 'left_arm/ik_target', 10)
        self.right_target_pub = self.create_publisher(Pose, 'right_arm/ik_target', 10)
        self.head_pub = self.create_publisher(JointState, 'head/joint_commands', 10)
        self.left_gripper_pub = self.create_publisher(Float64, 'left_arm/gripper_command', 10)
        self.right_gripper_pub = self.create_publisher(Float64, 'right_arm/gripper_command', 10)
        self.reset_pub = self.create_publisher(Empty, 'reset_to_default', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # FK pose subscriptions (TRANSIENT_LOCAL)
        fk_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Pose, 'left_arm/current_pose',
                                 lambda msg: self._on_fk_pose('left', msg), fk_qos)
        self.create_subscription(Pose, 'right_arm/current_pose',
                                 lambda msg: self._on_fk_pose('right', msg), fk_qos)

        # Head smoothing filter (One Euro, 2-DOF: yaw + pitch)
        self._head_filter = _OneEuroFilter(
            n=2, rate=1.0 / self._cmd_dt,  # 50 Hz
            min_cutoff=1.5, beta=0.01, d_cutoff=1.0,
        )

        # Timer for smooth return-to-home (50 Hz, only active when _returning_home)
        self._return_home_timer = self.create_timer(self._cmd_dt, self._return_home_tick)

        # Safety watchdog (20 Hz) — e-stop when teleop commands stop flowing
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_tick)

        self.get_logger().info('Teleop Controller Node initialized')
        self.get_logger().info(f'  X(fwd): [{self.min_x:.2f}, {self.max_x:.2f}]')
        self.get_logger().info(f'  Y(side): [{self.min_y:.2f}, {self.max_y:.2f}]')
        self.get_logger().info(f'  Z(up): [{self.min_z:.2f}, {self.max_z:.2f}]')
        self.get_logger().info(f'  max_linear_vel: {self.max_linear_velocity} m/s')
        self.get_logger().info(f'  max_angular_vel: {self.max_angular_velocity} rad/s')
        self.get_logger().info(f'  command_timeout (watchdog): {self.command_timeout}s')
        self.get_logger().info(f'  disconnect via robot/operator_status topic')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_pose(self, x: float, y: float, z: float) -> Pose:
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.w = 1.0
        return p

    def _on_fk_pose(self, arm_name: str, msg: Pose):
        """Update target from FK; capture first FK as home pose."""
        self.target_poses[arm_name] = self._make_pose(
            msg.position.x, msg.position.y, msg.position.z
        )
        self._targets_initialized[arm_name] = True
        # Capture first FK as home (from default joint angles on startup)
        if arm_name not in self._home_set:
            self._home_poses[arm_name] = self._make_pose(
                msg.position.x, msg.position.y, msg.position.z
            )
            self._home_set.add(arm_name)
        self.get_logger().info(
            f'FK sync {arm_name}: ({msg.position.x:.4f}, {msg.position.y:.4f}, {msg.position.z:.4f})'
        )

    # ------------------------------------------------------------------
    # Velocity clamping
    # ------------------------------------------------------------------

    def _clamp_linear_delta(self, dx: float, dy: float, dz: float) -> tuple:
        """Clamp (dx, dy, dz) so the resulting velocity stays within max_linear_velocity."""
        max_delta = self.max_linear_velocity * self._cmd_dt
        mag = np.sqrt(dx * dx + dy * dy + dz * dz)
        if mag > max_delta and mag > 1e-9:
            scale = max_delta / mag
            return dx * scale, dy * scale, dz * scale
        return dx, dy, dz

    def _clamp_angular_delta(self, delta: float) -> float:
        """Clamp an angular delta to max_angular_velocity."""
        max_delta = self.max_angular_velocity * self._cmd_dt
        return float(np.clip(delta, -max_delta, max_delta))

    # ------------------------------------------------------------------
    # Main dispatch
    # ------------------------------------------------------------------

    def teleop_callback(self, msg: TeleopCommand):
        # Track command flow for safety watchdog
        self._last_teleop_time = self.get_clock().now().nanoseconds / 1e9
        self._commanding = True

        mode = msg.mode.strip().lower()

        if mode == 'reset_to_default':
            self._handle_reset()
            return

        # Block normal teleop while smoothly returning to home/default
        if self._returning_home:
            return

        if mode == 'ik_control':
            self._handle_ik_control(msg)
        elif mode in ('velocity_control', 'flat_control', 'thumbstick_control'):
            self._handle_delta_control(msg)
            self._handle_head_control(msg)

        # Drive applies in both VR (ik_control) and flat/thumbstick modes —
        # VR sends `drive` alongside ik_control when the right thumbstick is
        # pushed. _handle_drive_control early-returns when both axes are
        # near-zero, so this is safe to run unconditionally.
        self._handle_drive_control(msg)

    # ------------------------------------------------------------------
    # Reset (smooth — same interpolation as disconnect return-to-home)
    # ------------------------------------------------------------------

    def _handle_reset(self):
        if self._returning_home:
            return  # already returning
        self.get_logger().info('Reset to default pose — smooth return')
        self._returning_home = True
        self._reset_in_progress = True
        self._stop_wheels()

    # ------------------------------------------------------------------
    # VR / absolute
    # ------------------------------------------------------------------

    def _handle_ik_control(self, msg: TeleopCommand):
        for arm_cmd, arm_name in [(msg.left_arm, 'left'), (msg.right_arm, 'right')]:
            if arm_cmd.command_type == 'end_effector_pose':
                self.target_poses[arm_name] = arm_cmd.end_effector_pose
                self._publish_target(arm_name)
            if arm_cmd.gripper_position >= 0:
                self._send_gripper(arm_name, arm_cmd.gripper_position)

    # ------------------------------------------------------------------
    # Flat-screen / thumbstick
    # ------------------------------------------------------------------

    def _handle_delta_control(self, msg: TeleopCommand):
        """
        All axes are delta-based and velocity-clamped:
          delta_position.x → Y (left/right)   from 2D joystick
          delta_position.y → Z (up/down)       from 2D joystick
          delta_position.z → X (forward/back)  from 1D joystick
        """
        for arm_cmd, arm_name in [(msg.left_arm, 'left'), (msg.right_arm, 'right')]:
            if arm_cmd.command_type != 'thumbstick_delta':
                continue

            # Map joystick deltas to workspace axes
            dx = arm_cmd.delta_position.z * self.delta_scale    # 1D fwd/back → X
            dy = -arm_cmd.delta_position.x * self.delta_scale   # 2D horizontal → Y
            dz = -arm_cmd.delta_position.y * self.delta_scale   # 2D vertical → Z

            # Velocity-clamp the combined delta
            dx, dy, dz = self._clamp_linear_delta(dx, dy, dz)

            self.target_poses[arm_name].position.x += dx
            self.target_poses[arm_name].position.y += dy
            self.target_poses[arm_name].position.z += dz

            # Clamp workspace
            self.target_poses[arm_name].position.x = float(np.clip(
                self.target_poses[arm_name].position.x, self.min_x, self.max_x))
            self.target_poses[arm_name].position.y = float(np.clip(
                self.target_poses[arm_name].position.y, self.min_y, self.max_y))
            self.target_poses[arm_name].position.z = float(np.clip(
                self.target_poses[arm_name].position.z, self.min_z, self.max_z))

            self._publish_target(arm_name)

            if arm_cmd.gripper_position >= 0:
                self._send_gripper(arm_name, arm_cmd.gripper_position)

    # ------------------------------------------------------------------
    # Head (yaw FLIPPED so joystick-right = robot looks right)
    # ------------------------------------------------------------------

    def _handle_head_control(self, msg: TeleopCommand):
        yaw_delta = msg.head_pose.position.x
        pitch_delta = msg.head_pose.position.y

        if abs(yaw_delta) < 1e-6 and abs(pitch_delta) < 1e-6:
            return

        # Negate yaw so joystick-right = robot yaw-right
        yaw_change = self._clamp_angular_delta(-yaw_delta * self.head_scale)
        pitch_change = self._clamp_angular_delta(pitch_delta * self.head_scale)

        self.head_yaw += yaw_change
        self.head_pitch += pitch_change

        self.head_yaw = float(np.clip(self.head_yaw, -self.head_yaw_limit, self.head_yaw_limit))
        self.head_pitch = float(np.clip(self.head_pitch, -self.head_pitch_limit, self.head_pitch_limit))

        self._publish_head()

    # ------------------------------------------------------------------
    # Drive (wheels)
    # ------------------------------------------------------------------

    def _handle_drive_control(self, msg: TeleopCommand):
        if abs(msg.drive_linear) < 1e-6 and abs(msg.drive_angular) < 1e-6:
            return
        twist = Twist()
        twist.linear.x = float(np.clip(
            msg.drive_linear * self.max_drive_linear, -self.max_drive_linear, self.max_drive_linear))
        twist.angular.z = float(np.clip(
            msg.drive_angular * self.max_drive_angular, -self.max_drive_angular, self.max_drive_angular))
        self.cmd_vel_pub.publish(twist)

    def _stop_wheels(self):
        self.cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Safety watchdog (wheels only)
    # ------------------------------------------------------------------

    def _watchdog_tick(self):
        """Fires at 20 Hz. Stop wheels if teleop commands were flowing and stop."""
        if not self._commanding or self._last_teleop_time is None:
            return
        if self._returning_home:
            return  # user-initiated return in progress
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_teleop_time) > self.command_timeout:
            self.get_logger().warn(
                f'WATCHDOG: no teleop for >{self.command_timeout:.1f}s — stopping wheels')
            self._commanding = False
            self._stop_wheels()

    # ------------------------------------------------------------------
    # Operator connection status (from WebRTC node)
    # ------------------------------------------------------------------

    def _on_operator_status(self, msg: String):
        """React to WebRTC operator connect/disconnect events."""
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        connected = status.get('status') == 'busy'

        if connected and not self._operator_connected:
            # Operator just connected
            self._operator_connected = True
            # Only cancel disconnect-triggered return; never cancel user-initiated reset
            if self._returning_home and not self._reset_in_progress:
                self._returning_home = False
                self.get_logger().info('Operator connected — cancelling return-to-home')
            self.get_logger().info('Operator connected')

        elif not connected and self._operator_connected:
            # Operator disconnected — stop wheels + smooth return to home
            self._operator_connected = False
            self._commanding = False
            self._stop_wheels()
            if not self._returning_home:
                self._returning_home = True
                self.get_logger().info('Operator disconnected — wheels stopped, returning to home')

    def _return_home_tick(self):
        """Called at 50 Hz. While returning home, gradually interpolate toward default."""
        if not self._returning_home:
            return

        alpha = self.return_home_alpha
        done = True

        # Interpolate arms toward home, publish as IK targets
        for arm_name in ['left', 'right']:
            home = self._home_poses[arm_name]
            cur = self.target_poses[arm_name]

            dx = alpha * (home.position.x - cur.position.x)
            dy = alpha * (home.position.y - cur.position.y)
            dz = alpha * (home.position.z - cur.position.z)

            # Velocity-clamp the return motion too
            dx, dy, dz = self._clamp_linear_delta(dx, dy, dz)

            cur.position.x += dx
            cur.position.y += dy
            cur.position.z += dz

            self._publish_target(arm_name)

            dist = np.sqrt(
                (cur.position.x - home.position.x) ** 2 +
                (cur.position.y - home.position.y) ** 2 +
                (cur.position.z - home.position.z) ** 2
            )
            if dist > 0.003:
                done = False

        # Interpolate head toward (0, 0)
        yaw_delta = self._clamp_angular_delta(alpha * (0.0 - self.head_yaw))
        pitch_delta = self._clamp_angular_delta(alpha * (0.0 - self.head_pitch))
        self.head_yaw += yaw_delta
        self.head_pitch += pitch_delta

        if abs(self.head_yaw) > 0.005 or abs(self.head_pitch) > 0.005:
            done = False

        self._publish_head()

        if done:
            self.head_yaw = 0.0
            self.head_pitch = 0.0
            self._head_filter.reset(np.array([0.0, 0.0]))
            self._returning_home = False
            self._reset_in_progress = False
            self._commanding = False
            self.get_logger().info('Return to home complete')

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_target(self, arm_name: str):
        pose = self.target_poses[arm_name]
        if arm_name == 'left':
            self.left_target_pub.publish(pose)
        elif arm_name == 'right':
            self.right_target_pub.publish(pose)

    def _publish_head(self):
        raw = np.array([self.head_yaw, self.head_pitch])
        filtered = self._head_filter.filter(raw)
        head_msg = JointState()
        head_msg.header = Header()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.name = ['head_yaw', 'head_pitch']
        head_msg.position = filtered.tolist()
        self.head_pub.publish(head_msg)

    def _send_gripper(self, arm_name: str, position: float):
        msg = Float64()
        msg.data = position
        if arm_name == 'left':
            self.left_gripper_pub.publish(msg)
        elif arm_name == 'right':
            self.right_gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopControllerNode()
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


if __name__ == '__main__':
    main()
