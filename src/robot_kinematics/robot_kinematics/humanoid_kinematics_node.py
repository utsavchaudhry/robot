#!/usr/bin/env python3
"""
ROS2 Node for Arm Kinematics using Pink IK

Provides two control interfaces:
  1. Topic-based (primary for real-time teleop):
     Subscribes to /left_arm/ik_target and /right_arm/ik_target (Pose),
     stores the latest target, and a timer at update_rate Hz solves IK
     incrementally (few iterations, smooth motion, drops stale targets).

  2. Action server (backward-compatible for one-shot goals):
     /compute_ik action for batch IK computation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import math
import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Empty, Float32MultiArray, Float64


# =====================================================================
# Joint Angle Filters
# =====================================================================

class OneEuroFilter:
    """
    One Euro Filter — adaptive low-pass for real-time signals.
    Smooths aggressively when the signal is slow (kills oscillation),
    lets fast motion through with minimal lag (preserves responsiveness).

    Params:
        min_cutoff: Minimum cutoff frequency (Hz). Lower = smoother when still.
        beta:       Speed coefficient. Higher = less lag during fast motion.
        d_cutoff:   Cutoff for derivative smoothing (Hz). Usually left at 1.0.
    """

    def __init__(self, n_joints: int, rate: float,
                 min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0):
        self._min_cutoff = min_cutoff
        self._beta = beta
        self._d_cutoff = d_cutoff
        self._rate = rate
        self._x_prev = np.zeros(n_joints)
        self._dx_prev = np.zeros(n_joints)
        self._initialized = False

    def _alpha(self, cutoff: float) -> float:
        te = 1.0 / self._rate
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / te)

    def filter(self, x: np.ndarray) -> np.ndarray:
        if not self._initialized:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            self._initialized = True
            return x.copy()

        # Derivative (smoothed)
        a_d = self._alpha(self._d_cutoff)
        dx = (x - self._x_prev) * self._rate
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev

        # Adaptive cutoff
        cutoff = self._min_cutoff + self._beta * np.abs(dx_hat)

        # Filtered signal (per-joint adaptive alpha)
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


class EMAFilter:
    """Simple Exponential Moving Average filter."""

    def __init__(self, n_joints: int, alpha: float = 0.3):
        self._alpha = alpha
        self._prev = np.zeros(n_joints)
        self._initialized = False

    def filter(self, x: np.ndarray) -> np.ndarray:
        if not self._initialized:
            self._prev = x.copy()
            self._initialized = True
            return x.copy()
        self._prev = self._alpha * x + (1.0 - self._alpha) * self._prev
        return self._prev.copy()

    def reset(self, x: np.ndarray):
        self._prev = x.copy()
        self._initialized = True

from robot_interfaces.action import ComputeIK
from robot_interfaces.msg import TeleopCommand, ArmCommand
from robot_kinematics.humanoid_kinematics import HumanoidKinematics
from robot_kinematics.unity_ros_transform import unity_pose_to_ros_se3
from robot_kinematics.default_pose_loader import load_default_pose

from ament_index_python.packages import get_package_share_directory
import os
import tempfile

try:
    from xacro import process_file as xacro_process_file
except ImportError:
    xacro_process_file = None


def _ensure_urdf(path: str) -> str:
    """If path ends with .xacro, expand to a temp .urdf and return that path; else return path."""
    if not path or not path.endswith('.xacro'):
        return path
    if xacro_process_file is None:
        raise RuntimeError("xacro is required to use .xacro URDFs; install the xacro package")
    xml = xacro_process_file(path).toxml()
    f = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', prefix='arm_', delete=False)
    f.write(xml)
    f.close()
    return f.name


class ArmKinematicsNode(Node):
    """ROS2 Node for computing inverse kinematics for robot arms"""

    def __init__(self):
        super().__init__('humanoid_kinematics_node')

        # Declare parameters
        self.declare_parameter('left_end_effector', 'left_end_effector')
        self.declare_parameter('right_end_effector', 'right_end_effector')
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('realtime_iterations', 3)  # IK iterations per timer tick
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s — safety clamp on IK output
        # IK solver costs (higher → stronger tracking)
        self.declare_parameter('position_cost', 1.0)
        self.declare_parameter('orientation_cost', 0.0)
        self.declare_parameter('regularization_cost', 0.001)
        self.declare_parameter('learning_rate', 0.1)
        # Joint smoothing filter
        self.declare_parameter('filter_type', 'one_euro')       # 'none', 'ema', 'one_euro'
        self.declare_parameter('ema_alpha', 0.3)                # EMA: 0-1, lower = smoother
        self.declare_parameter('one_euro_min_cutoff', 1.5)      # One Euro: Hz, lower = smoother when still
        self.declare_parameter('one_euro_beta', 0.01)           # One Euro: higher = less lag when fast
        self.declare_parameter('one_euro_d_cutoff', 1.0)        # One Euro: derivative smoothing

        # VR teleop safety limits (apply to /vr_teleop atomic path).
        # Defaults match the values teleop_controller_node uses for the legacy
        # delta-control path so behaviour is consistent across both inputs.
        self.declare_parameter('vr_workspace_min_x', 0.1)
        self.declare_parameter('vr_workspace_max_x', 0.5)
        self.declare_parameter('vr_workspace_min_y', -0.4)
        self.declare_parameter('vr_workspace_max_y', 0.4)
        self.declare_parameter('vr_workspace_min_z', -0.1)
        self.declare_parameter('vr_workspace_max_z', 0.45)
        self.declare_parameter('vr_head_yaw_limit', 1.5)      # rad — neck stop
        self.declare_parameter('vr_head_pitch_limit', 0.8)    # rad — neck stop
        self.declare_parameter('vr_max_drive_linear', 0.3)    # m/s at thumbstick=1
        self.declare_parameter('vr_max_drive_angular', 1.5)   # rad/s at thumbstick=1
        self.declare_parameter('vr_command_timeout', 0.5)     # s — wheels stop if no VR pkt

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.realtime_iterations = self.get_parameter('realtime_iterations').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.position_cost = self.get_parameter('position_cost').value
        self.orientation_cost = self.get_parameter('orientation_cost').value
        self.regularization_cost = self.get_parameter('regularization_cost').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.filter_type = self.get_parameter('filter_type').value

        # Use single humanoid URDF for whole-body IK
        pkg_path = get_package_share_directory('robot_description')
        humanoid_urdf_path = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')

        # Expand .xacro to .urdf
        humanoid_urdf_path = _ensure_urdf(humanoid_urdf_path)
        self.get_logger().info(f'Using unified humanoid URDF: {humanoid_urdf_path}')

        # Joint configuration for whole-body IK
        joint_names = {
            'left': [
                'left_shoulder_pitch', 'left_shoulder_yaw', 'left_shoulder_roll',
                'left_elbow_flex', 'left_wrist_roll', 'left_wrist_yaw', 'left_hand_wrist_pitch'
            ],
            'right': [
                'right_shoulder_pitch', 'right_shoulder_yaw', 'right_shoulder_roll',
                'right_elbow_flex', 'right_wrist_roll', 'right_wrist_yaw', 'right_hand_wrist_pitch'
            ],
            'head': ['head_yaw', 'head_pitch']
        }

        end_effector_links = {
            'left': self.get_parameter('left_end_effector').value,
            'right': self.get_parameter('right_end_effector').value,
        }

        # Load default pose from YAML configuration
        try:
            initial_joint_positions = load_default_pose()
            self.get_logger().info('Loaded default pose from configuration file')
        except Exception as e:
            self.get_logger().error(f'Failed to load default pose config: {e}')
            self.get_logger().warn('Using fallback default pose')
            head_initial = np.array([0.0, 0.0], dtype=float)
            left_arm_initial = np.array(
                [0.0, -1.5635088, 0.0, -2.98, 0.8987049384369203, 0.0, -0.0001919862177193199],
                dtype=float
            )
            right_arm_initial = np.array(
                [0.0, 1.5711316, 0.0, -0.16, 0.8987049384369203, 0.0, -0.0001919862177193199],
                dtype=float
            )
            initial_joint_positions = {
                'left': left_arm_initial,
                'right': right_arm_initial,
                'head': head_initial,
            }

        # Initialize single whole-body IK solver
        try:
            self.humanoid_ik = HumanoidKinematics(
                urdf_path=humanoid_urdf_path,
                end_effector_links=end_effector_links,
                joint_names=joint_names,
                initial_joint_positions=initial_joint_positions,
                position_cost=self.position_cost,
                orientation_cost=self.orientation_cost,
                regularization_cost=self.regularization_cost,
                robot_joint_fq=self.update_rate,
                learning_rate=self.learning_rate,
                debug_mode=self.debug_mode,
            )
            self.get_logger().info('Humanoid kinematics initialized (unified humanoid URDF)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize humanoid kinematics: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.humanoid_ik = None

        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # =====================================================================
        # Topic-based IK targets (primary for real-time teleop)
        # =====================================================================
        self._left_target = None   # Latest Pose or None
        self._right_target = None  # Latest Pose or None
        self._left_dirty = False
        self._right_dirty = False

        self.create_subscription(
            Pose, 'left_arm/ik_target', self._left_target_cb, 10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Pose, 'right_arm/ik_target', self._right_target_cb, 10,
            callback_group=self.callback_group
        )

        # Timer for incremental IK solve at update_rate
        self._ik_timer = self.create_timer(
            1.0 / self.update_rate, self._ik_timer_callback,
            callback_group=self.callback_group
        )

        # =====================================================================
        # Action server (backward-compatible for one-shot goals)
        # =====================================================================
        self._action_server = ActionServer(
            self,
            ComputeIK,
            'compute_ik',
            execute_callback=self.execute_ik_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # =====================================================================
        # Subscriber for VR teleop commands (ik_control mode)
        # =====================================================================
        self.teleop_sub = self.create_subscription(
            TeleopCommand,
            'teleop_commands',
            self.teleop_callback,
            10,
            callback_group=self.callback_group
        )

        # =====================================================================
        # Publishers for joint commands
        # =====================================================================
        self.head_joint_pub = self.create_publisher(JointState, 'head/joint_commands', 10)
        self.left_joint_pub = self.create_publisher(JointState, 'left_arm/joint_commands', 10)
        self.right_joint_pub = self.create_publisher(JointState, 'right_arm/joint_commands', 10)

        # Drive + grippers — published from the atomic VR callback so head,
        # arms, grippers, and wheels are all driven from the same callback
        # invocation. No more multi-subscriber races.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.left_gripper_pub = self.create_publisher(Float64, 'left_arm/gripper_command', 10)
        self.right_gripper_pub = self.create_publisher(Float64, 'right_arm/gripper_command', 10)

        # =====================================================================
        # Atomic VR teleop subscription — 25-float payload from webrtc_node.
        # MutuallyExclusiveCallbackGroup keeps callbacks serialized so the
        # shared `self._configuration` can't be clobbered by parallel solves.
        # =====================================================================
        self._vr_cb_group = MutuallyExclusiveCallbackGroup()
        self.vr_teleop_sub = self.create_subscription(
            Float32MultiArray, 'vr_teleop',
            self.vr_teleop_callback,
            QoSProfile(depth=1),   # drop stale frames rather than queue
            callback_group=self._vr_cb_group,
        )

        # VR safety state — read once at boot; treat the YAML as authoritative.
        self._vr_ws_min = np.array([
            self.get_parameter('vr_workspace_min_x').value,
            self.get_parameter('vr_workspace_min_y').value,
            self.get_parameter('vr_workspace_min_z').value,
        ], dtype=float)
        self._vr_ws_max = np.array([
            self.get_parameter('vr_workspace_max_x').value,
            self.get_parameter('vr_workspace_max_y').value,
            self.get_parameter('vr_workspace_max_z').value,
        ], dtype=float)
        self._vr_head_yaw_lim   = float(self.get_parameter('vr_head_yaw_limit').value)
        self._vr_head_pitch_lim = float(self.get_parameter('vr_head_pitch_limit').value)
        self._vr_max_drive_lin  = float(self.get_parameter('vr_max_drive_linear').value)
        self._vr_max_drive_ang  = float(self.get_parameter('vr_max_drive_angular').value)
        self._vr_cmd_timeout    = float(self.get_parameter('vr_command_timeout').value)

        # 2-DOF One Euro filter for HMD yaw/pitch — kills the controller jitter
        # the operator wouldn't notice in their headset but the motors would.
        # Same hyperparams as the arm filter; head only has 2 dims.
        self._vr_head_filter = OneEuroFilter(
            n_joints=2,
            rate=self.update_rate,
            min_cutoff=self.get_parameter('one_euro_min_cutoff').value,
            beta=self.get_parameter('one_euro_beta').value,
            d_cutoff=self.get_parameter('one_euro_d_cutoff').value,
        )
        # Wheel watchdog — if VR packets stop, send zero Twist. Checked at
        # 20Hz, which is >> the 0.5s default timeout.
        self._vr_last_msg_time = None
        self._vr_wheels_zeroed_since_timeout = True   # avoids spam-zeroing /cmd_vel forever
        self._vr_watchdog_timer = self.create_timer(0.05, self._vr_watchdog_tick)

        # Joint state subscribers (for updating IK seed from real hardware)
        self.left_joint_state_sub = self.create_subscription(
            JointState, 'left_arm/joint_states', self.left_joint_state_callback, 10,
            callback_group=self.callback_group
        )
        self.right_joint_state_sub = self.create_subscription(
            JointState, 'right_arm/joint_states', self.right_joint_state_callback, 10,
            callback_group=self.callback_group
        )

        # FK pose publishers — TRANSIENT_LOCAL so late-joiners get last value
        fk_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.left_pose_pub = self.create_publisher(Pose, 'left_arm/current_pose', fk_qos)
        self.right_pose_pub = self.create_publisher(Pose, 'right_arm/current_pose', fk_qos)

        # Subscribe to reset signal from teleop_controller
        self.create_subscription(
            Empty, 'reset_to_default', self._handle_reset_to_default, 10,
            callback_group=self.callback_group
        )

        # Store default pose for startup publishing
        self._default_pose = initial_joint_positions
        self._sent_default_pose = False
        self._default_pose_timer = self.create_timer(0.1, self._publish_default_pose_once)

        # Previous joint positions for velocity clamping
        self._prev_joints = {}  # arm_name → np.ndarray

        # Joint smoothing filters (per-arm)
        self._joint_filters = {}  # arm_name → filter instance
        n_arm_joints = 7
        for arm in ['left', 'right']:
            if self.filter_type == 'one_euro':
                self._joint_filters[arm] = OneEuroFilter(
                    n_arm_joints, self.update_rate,
                    min_cutoff=self.get_parameter('one_euro_min_cutoff').value,
                    beta=self.get_parameter('one_euro_beta').value,
                    d_cutoff=self.get_parameter('one_euro_d_cutoff').value,
                )
            elif self.filter_type == 'ema':
                self._joint_filters[arm] = EMAFilter(
                    n_arm_joints,
                    alpha=self.get_parameter('ema_alpha').value,
                )
            else:
                self._joint_filters[arm] = None

        self.get_logger().info(f'IK timer at {self.update_rate} Hz, {self.realtime_iterations} iterations/step')
        self.get_logger().info(f'Max joint velocity: {self.max_joint_velocity} rad/s')
        self.get_logger().info(f'Joint filter: {self.filter_type}')
        self.get_logger().info('Arm Kinematics Node initialized')

    # ------------------------------------------------------------------
    # Topic-based IK target callbacks
    # ------------------------------------------------------------------
    def _left_target_cb(self, msg: Pose):
        self._left_target = msg
        self._left_dirty = True

    def _right_target_cb(self, msg: Pose):
        self._right_target = msg
        self._right_dirty = True

    def _ik_timer_callback(self):
        """
        Periodic IK solve: uses ONLY the latest target for each arm.
        Runs a small number of iterations for smooth incremental motion.
        Drops stale targets — never builds a backlog.
        """
        if self.humanoid_ik is None:
            return

        # Collect dirty targets
        target_poses = {}
        if self._left_dirty and self._left_target is not None:
            target_poses['left'] = self.pose_to_se3(self._left_target)
            self._left_dirty = False
        if self._right_dirty and self._right_target is not None:
            target_poses['right'] = self.pose_to_se3(self._right_target)
            self._right_dirty = False

        if not target_poses:
            return  # Nothing new to solve

        try:
            joint_solutions = self.humanoid_ik.compute_ik(
                target_poses,
                iterations=self.realtime_iterations
            )

            dt = 1.0 / self.update_rate
            max_delta = self.max_joint_velocity * dt

            # Publish joint commands: velocity clamp → filter → publish
            for arm_name in target_poses:
                joint_positions = joint_solutions[arm_name]
                if np.any(np.isnan(joint_positions)) or np.any(np.isinf(joint_positions)):
                    continue

                # 1. Clamp joint velocity: limit change per tick
                if arm_name in self._prev_joints:
                    delta = joint_positions - self._prev_joints[arm_name]
                    delta = np.clip(delta, -max_delta, max_delta)
                    joint_positions = self._prev_joints[arm_name] + delta

                self._prev_joints[arm_name] = joint_positions.copy()

                # 2. Smooth with filter (kills oscillation, preserves fast motion)
                filt = self._joint_filters.get(arm_name)
                if filt is not None:
                    joint_positions = filt.filter(joint_positions)

                self.publish_joint_command(joint_positions, arm_name)

        except Exception as e:
            if self.debug_mode:
                self.get_logger().error(f'IK timer error: {e}')

    # ------------------------------------------------------------------
    # Action server callbacks (backward-compatible)
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        """Accept or reject IK goal"""
        self.get_logger().info(f'Received IK goal for {goal_request.arm_name} arm')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation request"""
        self.get_logger().info('IK goal cancelled')
        return CancelResponse.ACCEPT

    def pose_to_se3(self, pose: Pose) -> pin.SE3:
        """Convert ROS Pose message to Pinocchio SE3 transform (position-only)"""
        translation = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        rotation = np.eye(3)
        return pin.SE3(rotation, translation)

    async def execute_ik_callback(self, goal_handle):
        """Execute IK computation action (used for one-shot goals, not real-time)"""
        self.get_logger().info('Executing IK computation...')

        request = goal_handle.request
        feedback_msg = ComputeIK.Feedback()

        if self.humanoid_ik is None:
            self.get_logger().error('Humanoid IK not initialized')
            goal_handle.abort()
            result = ComputeIK.Result()
            result.success = False
            result.message = 'Humanoid IK not initialized'
            return result

        target_se3 = self.pose_to_se3(request.target_pose)

        try:
            target_poses = {request.arm_name: target_se3}
            joint_solutions = self.humanoid_ik.compute_ik(target_poses, iterations=50)
            joint_positions = joint_solutions[request.arm_name]

            if np.any(np.isnan(joint_positions)) or np.any(np.isinf(joint_positions)):
                self.get_logger().warn(f'IK returned NaN/Inf for {request.arm_name} arm')
                goal_handle.abort()
                result = ComputeIK.Result()
                result.success = False
                result.message = 'IK returned invalid joint positions'
                return result

            feedback_msg.iteration = 1
            feedback_msg.position_error = 0.0
            feedback_msg.orientation_error = 0.0
            goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()
            result = ComputeIK.Result()
            result.joint_positions = joint_positions.tolist()
            result.success = True
            result.message = 'IK computation successful'
            self.get_logger().info(f'IK succeeded for {request.arm_name} arm')

            self.publish_joint_command(joint_positions, request.arm_name)
            return result

        except Exception as e:
            self.get_logger().error(f'IK computation error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            result = ComputeIK.Result()
            result.success = False
            result.message = str(e)
            return result

    # ------------------------------------------------------------------
    # VR teleop callback (ik_control mode only)
    # ------------------------------------------------------------------
    def extract_head_angles(self, T_head: pin.SE3) -> tuple:
        """Extract yaw and pitch from a ROS-frame head SE3.

        Caller must pass an SE3 already converted from the WebXR/Unity wire
        frame via `unity_pose_to_ros_se3`. Reading rpy directly off the raw
        Unity quaternion mis-labels the axes (Unity Y-up vs ROS Z-up), which
        is what caused HMD yaw to drive robot pitch.

        Pitch sign is inverted: with the corrected ROS-frame rotation, the
        HMD's "look down" produces a positive rpy[1], but the robot's
        head_pitch servo expects "look down" = negative. Flipping here keeps
        the protocol-facing yaw/pitch in the same direction as the operator.

        Returns (yaw_around_Z, pitch_around_Y_flipped).
        """
        rpy = pin.rpy.matrixToRpy(T_head.rotation)
        return float(rpy[2]), float(-rpy[1])

    # ------------------------------------------------------------------
    # Atomic VR teleop — one Float32MultiArray, one callback, one solve,
    # everything published from here. This is the only path the WebXR
    # client takes; the older TeleopCommand-based teleop_callback below is
    # retained only for legacy/Unity clients and is a no-op for ik_control.
    # ------------------------------------------------------------------
    def vr_teleop_callback(self, msg: Float32MultiArray):
        v = msg.data
        if len(v) != 25:
            return  # webrtc_node already logs the length mismatch

        # --- 1. Decode (no side effects yet) ---------------------------------
        head_dict = {
            'position':    {'x': float(v[0]),  'y': float(v[1]),  'z': float(v[2])},
            'rotation':    {'x': float(v[3]),  'y': float(v[4]),  'z': float(v[5]),  'w': float(v[6])},
        }
        left_dict = {
            'position':    {'x': float(v[7]),  'y': float(v[8]),  'z': float(v[9])},
            'rotation':    {'x': float(v[10]), 'y': float(v[11]), 'z': float(v[12]), 'w': float(v[13])},
        }
        right_dict = {
            'position':    {'x': float(v[14]), 'y': float(v[15]), 'z': float(v[16])},
            'rotation':    {'x': float(v[17]), 'y': float(v[18]), 'z': float(v[19]), 'w': float(v[20])},
        }
        left_grip     = float(v[21])
        right_grip    = float(v[22])
        drive_linear  = float(v[23])
        drive_angular = float(v[24])

        T_world_head = unity_pose_to_ros_se3(head_dict)
        T_head_world = T_world_head.inverse()

        # Update watchdog timestamp — wheels stop in _vr_watchdog_tick if this
        # falls more than vr_command_timeout seconds behind wall time.
        self._vr_last_msg_time = self.get_clock().now().nanoseconds / 1e9
        self._vr_wheels_zeroed_since_timeout = False

        # --- 2. Head (HMD yaw/pitch) — clamp + One Euro filter --------------
        head_yaw, head_pitch = self.extract_head_angles(T_world_head)
        head_yaw   = float(np.clip(head_yaw,   -self._vr_head_yaw_lim,   self._vr_head_yaw_lim))
        head_pitch = float(np.clip(head_pitch, -self._vr_head_pitch_lim, self._vr_head_pitch_lim))
        head_smoothed = self._vr_head_filter.filter(np.array([head_yaw, head_pitch]))
        self.publish_head_command(float(head_smoothed[0]), float(head_smoothed[1]))

        # --- 3. Dual-arm IK — POSITION ONLY, single compute_ik call ---------
        if self.humanoid_ik is not None:
            T_world_left  = pin.SE3(np.eye(3), unity_pose_to_ros_se3(left_dict).translation)
            T_world_right = pin.SE3(np.eye(3), unity_pose_to_ros_se3(right_dict).translation)
            T_head_left  = T_head_world * T_world_left
            T_head_right = T_head_world * T_world_right

            # Axis fix: WebXR target was lateral-mirrored relative to the
            # robot's base_link (operator-right mapped onto robot-left because
            # of the Unity→ROS axis remap). Negate only Y. X (forward) is
            # already correct; flipping it sent targets behind the robot's
            # back. Z (up) was always right.
            #
            # Workspace clamp: prevents the IK from chasing positions the arm
            # physically can't reach — at the edges the QP would saturate at
            # joint limits and produce oscillating/jumpy solutions.
            def _flip_y_and_clamp(T: pin.SE3) -> pin.SE3:
                t = T.translation
                clamped = np.clip(
                    np.array([t[0], -t[1], t[2]]),
                    self._vr_ws_min,
                    self._vr_ws_max,
                )
                return pin.SE3(T.rotation, clamped)
            T_head_left  = _flip_y_and_clamp(T_head_left)
            T_head_right = _flip_y_and_clamp(T_head_right)
            joint_solutions = self.humanoid_ik.compute_ik(
                {'left': T_head_left, 'right': T_head_right},
                iterations=10,
            )
            dt = 1.0 / self.update_rate
            max_delta = self.max_joint_velocity * dt
            for arm_name in ('left', 'right'):
                jp = joint_solutions.get(arm_name)
                if jp is None or np.any(np.isnan(jp)) or np.any(np.isinf(jp)):
                    continue
                if arm_name in self._prev_joints:
                    delta = jp - self._prev_joints[arm_name]
                    delta = np.clip(delta, -max_delta, max_delta)
                    jp = self._prev_joints[arm_name] + delta
                self._prev_joints[arm_name] = jp.copy()
                filt = self._joint_filters.get(arm_name)
                if filt is not None:
                    jp = filt.filter(jp)
                self.publish_joint_command(jp, arm_name)

        # --- 4. Grippers (-1.0 sentinel = no command) ------------------------
        if left_grip >= 0.0:
            gmsg = Float64()
            gmsg.data = left_grip
            self.left_gripper_pub.publish(gmsg)
        if right_grip >= 0.0:
            gmsg = Float64()
            gmsg.data = right_grip
            self.right_gripper_pub.publish(gmsg)

        # --- 5. Drive (/cmd_vel) — scale [-1, 1] stick by configured caps,
        # then clamp belt-and-suspenders. Zero is a valid command (stop).
        twist = Twist()
        twist.linear.x  = float(np.clip(
            drive_linear  * self._vr_max_drive_lin,
            -self._vr_max_drive_lin, self._vr_max_drive_lin))
        twist.angular.z = float(np.clip(
            drive_angular * self._vr_max_drive_ang,
            -self._vr_max_drive_ang, self._vr_max_drive_ang))
        self.cmd_vel_pub.publish(twist)

    def _vr_watchdog_tick(self):
        """Stop wheels if VR packets stop arriving.

        Fires at 20Hz. If the last vr_teleop message is older than the
        configured timeout, publish a zero Twist exactly once and then go
        quiet so we don't spam /cmd_vel. The flag resets the moment a new
        VR packet lands."""
        if self._vr_last_msg_time is None or self._vr_wheels_zeroed_since_timeout:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._vr_last_msg_time) > self._vr_cmd_timeout:
            self.cmd_vel_pub.publish(Twist())
            self._vr_wheels_zeroed_since_timeout = True
            self.get_logger().warn(
                f'VR watchdog: no vr_teleop for >{self._vr_cmd_timeout:.1f}s — wheels stopped')

    def teleop_callback(self, msg: TeleopCommand):
        """
        Handle teleop commands from Unity operator via WebRTC (VR tracking).
        Only processes ik_control mode; thumbstick_control is handled via
        the teleop_controller_node → topic-based targets path.

        NOTE: The WebXR client now sends via /vr_teleop (vr_teleop_callback
        above), not /teleop_commands, so this method's ik_control branch is
        effectively dead for the modern client. Kept for legacy Unity
        clients that still emit nested TeleopCommand JSON.
        """
        if msg.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP received!')
            return

        # VR Mode only
        if msg.mode == 'ik_control':
            unity_head_dict = {
                'position': {'x': msg.head_pose.position.x, 'y': msg.head_pose.position.y, 'z': msg.head_pose.position.z},
                'rotation': {'x': msg.head_pose.orientation.x, 'y': msg.head_pose.orientation.y,
                           'z': msg.head_pose.orientation.z, 'w': msg.head_pose.orientation.w}
            }
            T_world_head = unity_pose_to_ros_se3(unity_head_dict)
            T_head_world = T_world_head.inverse()

            # Publish head first so HMD tracking isn't gated by the IK solve
            # for both arms (~50 pinocchio iterations × 2). The head is a
            # direct passthrough — no IK — so it should land on the bus
            # ahead of the slower arm path.
            head_yaw, head_pitch = self.extract_head_angles(T_world_head)
            self.publish_head_command(head_yaw, head_pitch)

            # Hard-lock orientation in the IK target — only position is fed
            # to the solver. orientation_cost=0 in teleop_config.yaml already
            # tells Pink to ignore orientation, but we ALSO zero out the
            # rotation here so a stale/jittery controller quaternion can't
            # disturb the QP through any other code path. Resolves the
            # wrist-oscillation seen in the first VR arm test.
            def _position_only(unity_pose) -> pin.SE3:
                full = unity_pose_to_ros_se3(unity_pose)
                return pin.SE3(np.eye(3), full.translation)

            # IK target = controller POSITION RELATIVE TO HEAD, then treated
            # as relative to robot's base_link (approximation that the robot's
            # chest/shoulder sits at base_link origin). The previous chain
            #     T_base_left = T_world_head * (T_head_world * T_world_left)
            # algebraically reduces to T_world_left — i.e. the operator's raw
            # world position in their playspace. That put targets at the
            # operator's Y-up world height (~1.4–1.7m) which is far outside
            # the robot's workspace (max_z=0.45). IK saturated, flipped between
            # local optima, and the arm flailed. Using T_head_left as the
            # target keeps things in a reach-sized box around base_link.
            #
            # iterations=self.realtime_iterations (= 1 from teleop_config.yaml)
            # instead of hard-coded 50: at 50Hz incoming VR commands, each
            # 50-iter solve cost ~25-40ms on the LattePanda; calling
            # teleop_callback at 50Hz with two arms = 100 iters / 50Hz = 100%
            # CPU on a single thread and the message queue backed up, which is
            # why head appeared to "snap" instead of stream smoothly.
            # Solve BOTH arms in a single compute_ik call. The earlier
            # version split into two sequential calls (left, then right)
            # which under MultiThreadedExecutor + ReentrantCallbackGroup
            # raced two concurrent invocations of this callback through the
            # shared self._configuration — one was mid-left-solve while
            # another was mid-right-solve, scrambling each other's state.
            # Symptoms: right arm publish count was ~2× left's, right arm
            # joints jumped 0.5 rad in 30ms (11× the configured velocity
            # limit), left arm "barely moved". Pink is designed to solve
            # multiple frame tasks together; the _ik_timer_callback already
            # uses that pattern.
            if self.humanoid_ik is not None:
                unity_left_dict = {
                    'position': {'x': msg.left_controller_pose.position.x, 'y': msg.left_controller_pose.position.y, 'z': msg.left_controller_pose.position.z},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                }
                unity_right_dict = {
                    'position': {'x': msg.right_controller_pose.position.x, 'y': msg.right_controller_pose.position.y, 'z': msg.right_controller_pose.position.z},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                }
                T_world_left = _position_only(unity_left_dict)
                T_world_right = _position_only(unity_right_dict)
                T_head_left = T_head_world * T_world_left
                T_head_right = T_head_world * T_world_right
                target_poses = {'left': T_head_left, 'right': T_head_right}

                # iter count: 10 was a compromise per-arm; with both arms in
                # one solve it's still ~10-15ms total on LattePanda.
                joint_solutions = self.humanoid_ik.compute_ik(target_poses, iterations=10)

                # Apply the same post-processing that _ik_timer_callback does
                # — velocity clamping (max_joint_velocity) + One Euro filter
                # — so VR-path commands don't jump 0.5 rad between frames.
                dt = 1.0 / self.update_rate
                max_delta = self.max_joint_velocity * dt
                for arm_name in ('left', 'right'):
                    joint_positions = joint_solutions[arm_name]
                    if np.any(np.isnan(joint_positions)) or np.any(np.isinf(joint_positions)):
                        continue
                    if arm_name in self._prev_joints:
                        delta = joint_positions - self._prev_joints[arm_name]
                        delta = np.clip(delta, -max_delta, max_delta)
                        joint_positions = self._prev_joints[arm_name] + delta
                    self._prev_joints[arm_name] = joint_positions.copy()
                    filt = self._joint_filters.get(arm_name)
                    if filt is not None:
                        joint_positions = filt.filter(joint_positions)
                    self.publish_joint_command(joint_positions, arm_name)

        # Direct arm commands (end_effector_pose / joint_positions)
        elif msg.left_arm.command_type == 'end_effector_pose':
            if self.humanoid_ik is not None:
                target_se3 = self.pose_to_se3(msg.left_arm.end_effector_pose)
                target_poses = {'left': target_se3}
                joint_solutions = self.humanoid_ik.compute_ik(target_poses, iterations=50)
                joint_positions = joint_solutions['left']
                self.publish_joint_command(joint_positions, 'left')

        elif msg.left_arm.command_type == 'joint_positions':
            self.publish_joint_command(msg.left_arm.joint_positions, 'left')

        if msg.right_arm.command_type == 'end_effector_pose':
            if self.humanoid_ik is not None:
                target_se3 = self.pose_to_se3(msg.right_arm.end_effector_pose)
                target_poses = {'right': target_se3}
                joint_solutions = self.humanoid_ik.compute_ik(target_poses, iterations=50)
                joint_positions = joint_solutions['right']
                self.publish_joint_command(joint_positions, 'right')

        elif msg.right_arm.command_type == 'joint_positions':
            self.publish_joint_command(msg.right_arm.joint_positions, 'right')

    # ------------------------------------------------------------------
    # Joint command publishers
    # ------------------------------------------------------------------
    def publish_head_command(self, yaw: float, pitch: float):
        """Publish head joint commands"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_yaw', 'head_pitch']
        msg.position = [yaw, pitch]
        self.head_joint_pub.publish(msg)

    def publish_joint_command(self, joint_positions, arm_name: str):
        """Publish joint commands"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        if arm_name == 'left':
            msg.name = [
                'left_shoulder_pitch', 'left_shoulder_yaw', 'left_shoulder_roll',
                'left_elbow_flex', 'left_wrist_roll', 'left_wrist_yaw', 'left_hand_wrist_pitch'
            ]
            msg.position = joint_positions if isinstance(joint_positions, list) else joint_positions.tolist()
            if self.debug_mode:
                self.get_logger().info(f'Publishing left arm joints: {msg.position}')
            self.left_joint_pub.publish(msg)
        elif arm_name == 'right':
            msg.name = [
                'right_shoulder_pitch', 'right_shoulder_yaw', 'right_shoulder_roll',
                'right_elbow_flex', 'right_wrist_roll', 'right_wrist_yaw', 'right_hand_wrist_pitch'
            ]
            msg.position = joint_positions if isinstance(joint_positions, list) else joint_positions.tolist()
            if self.debug_mode:
                self.get_logger().info(f'Publishing right arm joints: {msg.position}')
            self.right_joint_pub.publish(msg)

    def _publish_default_pose_once(self):
        """Publish the default pose once on startup, then compute FK and publish EE poses."""
        if self._sent_default_pose:
            return
        self._sent_default_pose = True

        self.get_logger().info('Publishing default pose (arms-down position)...')

        if self.humanoid_ik is not None:
            self.humanoid_ik.set_configuration(self._default_pose)

        self.publish_head_command(self._default_pose['head'][0], self._default_pose['head'][1])
        self.publish_joint_command(self._default_pose['left'], 'left')
        self.publish_joint_command(self._default_pose['right'], 'right')

        # Initialize velocity clamping + filter state from default pose
        for arm in ['left', 'right']:
            self._prev_joints[arm] = np.array(self._default_pose[arm], dtype=float)
            filt = self._joint_filters.get(arm)
            if filt is not None:
                filt.reset(self._prev_joints[arm])

        # Compute FK and publish EE poses so teleop_controller knows starting positions
        self._publish_current_ee_poses()

        self.get_logger().info('Default pose published successfully')
        self._default_pose_timer.cancel()

    # ------------------------------------------------------------------
    # Reset to default pose
    # ------------------------------------------------------------------
    def _handle_reset_to_default(self, msg):
        """Hard-reset joint angles to saved default, compute FK, publish everything."""
        self.get_logger().info('Reset to default pose received')
        if self.humanoid_ik is None:
            return

        # Hard-reset the IK solver's internal configuration to the default joint angles
        self.humanoid_ik.set_configuration(self._default_pose)

        # Publish joint commands
        self.publish_head_command(self._default_pose['head'][0], self._default_pose['head'][1])
        self.publish_joint_command(self._default_pose['left'], 'left')
        self.publish_joint_command(self._default_pose['right'], 'right')

        # Reset velocity clamping + filter state
        for arm in ['left', 'right']:
            self._prev_joints[arm] = np.array(self._default_pose[arm], dtype=float)
            filt = self._joint_filters.get(arm)
            if filt is not None:
                filt.reset(self._prev_joints[arm])

        # Compute FK from the reset configuration and publish EE poses
        self._publish_current_ee_poses()

        # Clear any stale IK targets — prevents old targets from causing motion
        self._left_dirty = False
        self._right_dirty = False
        self._left_target = None
        self._right_target = None

        self.get_logger().info('Default pose restored and FK published')

    # ------------------------------------------------------------------
    # FK → EE pose publisher
    # ------------------------------------------------------------------
    def _se3_to_pose(self, se3) -> Pose:
        """Convert Pinocchio SE3 transform to ROS Pose message."""
        p = Pose()
        t = se3.translation
        p.position.x = float(t[0])
        p.position.y = float(t[1])
        p.position.z = float(t[2])
        quat = pin.Quaternion(se3.rotation)
        # Pinocchio Quaternion exposes coeffs as properties, not methods
        p.orientation.x = float(quat.x)
        p.orientation.y = float(quat.y)
        p.orientation.z = float(quat.z)
        p.orientation.w = float(quat.w)
        return p

    def _publish_current_ee_poses(self):
        """Compute FK and publish current end-effector poses."""
        if self.humanoid_ik is None:
            return
        ee_poses = self.humanoid_ik.get_current_poses()
        for limb, se3 in ee_poses.items():
            pose_msg = self._se3_to_pose(se3)
            if limb == 'left':
                self.left_pose_pub.publish(pose_msg)
            elif limb == 'right':
                self.right_pose_pub.publish(pose_msg)
            self.get_logger().info(
                f'FK {limb}: ({se3.translation[0]:.4f}, {se3.translation[1]:.4f}, {se3.translation[2]:.4f})'
            )

    def left_joint_state_callback(self, msg: JointState):
        """Update humanoid IK solver with current left arm joint state"""
        pass

    def right_joint_state_callback(self, msg: JointState):
        """Update humanoid IK solver with current right arm joint state"""
        pass


def main(args=None):
    rclpy.init(args=args)

    node = ArmKinematicsNode()

    executor = MultiThreadedExecutor()
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
