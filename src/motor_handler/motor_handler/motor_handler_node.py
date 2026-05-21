from typing import List, Dict, Optional
from functools import partial

import os
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory

from motor_handler.esp32 import ESP32, XiaomiESP32, discover_devices
from motor_handler.motor import Motor


def load_servo_config(path: str) -> list:
    """Load servo config YAML, return list of dicts."""
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get("servos", [])


class MotorHandler(Node):
    def __init__(self):
        super().__init__('motor_handler')
        self.get_logger().info("Setting up motor handler...")

        # --- Config file parameter -----------------------------------------
        # Per-robot servo calibration lives in
        #   share/motor_handler/config/servo_config_<variant>.yaml
        # The variant is picked from the ROBOT_VARIANT env var (a=shop,
        # b=test/dev). Default 'b' so a fresh checkout / unconfigured robot
        # uses the dev calibration rather than silently driving with the
        # shop robot's (different) calibration. Each robot should set
        # ROBOT_VARIANT in /etc/environment or the systemd unit's
        # Environment= line — see the robot README. Explicit launch param
        # `-p servo_config:=/full/path/to.yaml` still wins over the env var.
        variant = os.environ.get('ROBOT_VARIANT', 'b').strip().lower()
        default_config = os.path.join(
            get_package_share_directory('motor_handler'),
            'config', f'servo_config_{variant}.yaml')
        self.declare_parameter('servo_config', default_config)
        config_path = (self.get_parameter('servo_config')
                       .get_parameter_value().string_value)
        self.get_logger().info(
            f"Loading config: {config_path} (ROBOT_VARIANT={variant!r})")
        servo_config = load_servo_config(config_path)

        # --- Enabled joints parameter --------------------------------------
        # Only joints in this list will receive move commands.
        # Position readback still works for all discovered joints.
        #
        # Default: head + shoulders (safe for remote testing).
        # Test a single joint:
        #   --ros-args -p enabled_joints:="['right_elbow_flex']"
        # Enable everything:
        #   --ros-args -p enabled_joints:="['all']"
        self.declare_parameter('enabled_joints', ['all'])
        enabled_list = (self.get_parameter('enabled_joints')
                        .get_parameter_value().string_array_value)
        enable_all = 'all' in enabled_list
        self.get_logger().info(
            f"Enabled joints: {'ALL' if enable_all else enabled_list}")

        # --- Discover ESP32s -----------------------------------------------
        self.get_logger().info("Starting ESP32 discovery...")
        device_map = discover_devices(self.get_logger())
        self.get_logger().info(
            f"Discovery complete. Found devices: {list(device_map.keys())}")

        self.servo_esps: Dict[str, ESP32] = {}
        self.xiaomi: Optional[XiaomiESP32] = None

        for identity, ser in device_map.items():
            if identity in ("tc", "m"):
                esp = ESP32(ser, identity)
                prefix = "t:" if identity == "tc" else ""
                esp.get_ids(prefix=prefix)
                self.get_logger().info(
                    f"'{identity}' on {ser.port}: servos {esp.ids}")
                self.servo_esps[identity] = esp
            elif identity == "xiaomi":
                self.xiaomi = XiaomiESP32(ser)
                self.get_logger().info(f"Xiaomi CyberGear on {ser.port}")

        # --- Register SCS motor IDs from config (not found by STS ping) -----
        for esp_key, esp in self.servo_esps.items():
            scs_ids = [
                entry["servo_id"]
                for entry in servo_config
                if entry.get("esp") == esp_key
                and entry.get("prefix", "") == "c:"
            ]
            if scs_ids:
                esp.add_ids(scs_ids)
                self.get_logger().info(
                    f"Registered SCS motor IDs on '{esp_key}': {scs_ids}")

        # Start background polling threads
        for esp in self.servo_esps.values():
            esp.start()
        if self.xiaomi:
            self.xiaomi.start()

        # --- Build motors from config --------------------------------------
        self.motors: List[Motor] = []
        self._motor_by_name: Dict[str, Motor] = {}

        for entry in servo_config:
            name = entry["name"]
            esp_key = entry["esp"]
            sid = entry["servo_id"]

            if esp_key not in self.servo_esps:
                self.get_logger().warn(
                    f"Skipping {name}: ESP32 '{esp_key}' not found")
                continue
            esp = self.servo_esps[esp_key]
            if sid not in esp.ids:
                self.get_logger().warn(
                    f"Skipping {name}: servo {sid} not found on '{esp_key}'")
                continue

            is_enabled = enable_all or name in enabled_list
            motor = Motor(
                name=name, esp=esp, servo_id=sid,
                angle_min=entry["angle_min"],
                angle_max=entry["angle_max"],
                esp_min=entry["esp_min"],
                esp_max=entry["esp_max"],
                cmd_prefix=entry.get("prefix", ""),
                flip=entry.get("flip", False),
                offset=entry.get("offset", 0.0),
                enabled=is_enabled)
            self.motors.append(motor)
            self._motor_by_name[name] = motor
            tag = "ENABLED" if is_enabled else "read-only"
            self.get_logger().info(
                f"  {name} -> servo {sid} on '{esp_key}' [{tag}]")

        # --- ROS2 topics ---------------------------------------------------
        self._head_sub = self.create_subscription(
            JointState, 'head/joint_commands', self._joint_cmd_cb, 10)
        self._head_pub = self.create_publisher(
            JointState, 'head/joint_states', 10)

        self._rarm_sub = self.create_subscription(
            JointState, 'right_arm/joint_commands', self._joint_cmd_cb, 10)
        self._rarm_pub = self.create_publisher(
            JointState, 'right_arm/joint_states', 10)

        self._larm_sub = self.create_subscription(
            JointState, 'left_arm/joint_commands', self._joint_cmd_cb, 10)
        self._larm_pub = self.create_publisher(
            JointState, 'left_arm/joint_states', 10)

        # --- Gripper commands (Float64, 0.0=open 1.0=close) -----------------
        self._left_gripper_motors = [
            n for n in self._motor_by_name if n == 'left_gripper']
        self._right_gripper_motors = [
            n for n in self._motor_by_name
            if n.startswith('right_finger_') or n == 'right_gripper']

        # Dedupe: clients (VR / flatscreen) re-send gripper state on every
        # input frame at 60-90 Hz. Without dedup, that fans out to 16 finger
        # servos × 60 Hz = ~1k serial writes/sec to the ESP — saturates the
        # USB controller and starves camera capture, causing video freeze
        # while head/wheel still work. Track the last value per side and
        # skip the per-motor write loop when the requested value hasn't
        # meaningfully changed.
        self._last_gripper_val = {'left': None, 'right': None}
        self._GRIPPER_EPS = 0.01

        if self._left_gripper_motors:
            self.create_subscription(
                Float64, 'left_arm/gripper_command',
                partial(self._gripper_cmd_cb,
                        motor_names=self._left_gripper_motors,
                        side='left'), 10)
            self.get_logger().info(
                f"Left gripper motors: {self._left_gripper_motors}")

        if self._right_gripper_motors:
            self.create_subscription(
                Float64, 'right_arm/gripper_command',
                partial(self._gripper_cmd_cb,
                        motor_names=self._right_gripper_motors,
                        side='right'), 10)
            self.get_logger().info(
                f"Right gripper motors: {self._right_gripper_motors}")

        # --- Wheels (cmd_vel → Xiaomi CyberGear) ----------------------------
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('max_turn_speed', 1.5)
        self.declare_parameter('backward_speed_pct', 0.2)  # 0-1, scales backward
        self.declare_parameter('max_wheel_speed', 3.0)     # max value sent to ESP32
        self._max_linear_speed = self.get_parameter('max_linear_speed').value
        self._max_turn_speed = self.get_parameter('max_turn_speed').value
        self._backward_speed_pct = self.get_parameter('backward_speed_pct').value
        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value

        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self._last_cmd_vel_time = self.get_clock().now()

        if not self.motors:
            self.get_logger().error(
                "NO MOTORS were created — commands will have no effect. "
                "Check ESP32 connections and servo_config.yaml.")
        else:
            self.get_logger().info(
                f"Motor handler ready: {len(self.motors)} motors "
                f"({sum(1 for m in self.motors if m.enabled)} enabled)")

        # --- Send default pose on startup (prevents jerk) -------------------
        self._send_default_pose()

        # --- Timers --------------------------------------------------------
        self._request_idx = 0  # round-robin index for staggered position reads
        self._request_timer = self.create_timer(0.05, self._request_positions)
        self._publish_timer = self.create_timer(0.05, self._publish_joint_states)
        self._wheel_timeout_timer = self.create_timer(0.1, self._wheel_timeout_check)

    # --- Default pose on startup ------------------------------------------

    def _send_default_pose(self):
        """Command enabled motors to the default pose so they don't jerk."""
        try:
            pose_path = os.path.join(
                get_package_share_directory('robot_bringup'),
                'config', 'default_pose.yaml')
            with open(pose_path) as f:
                pose = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f"Could not load default pose: {e}")
            return

        angles = {}
        for name, val in pose.get('head', {}).items():
            angles[f'head_{name}'] = float(val)
        for name, val in pose.get('left_arm', {}).items():
            angles[f'left_{name}'] = float(val)
        for name, val in pose.get('right_arm', {}).items():
            angles[f'right_{name}'] = float(val)

        sent = 0
        for joint_name, angle in angles.items():
            motor = self._motor_by_name.get(joint_name)
            if motor and motor.enabled:
                motor.set_angle(angle)
                sent += 1

        if sent:
            self.get_logger().info(
                f"Default pose sent to {sent} enabled motor(s)")

    # --- Joint command callback (shared by all topics) ---------------------

    def _joint_cmd_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            motor = self._motor_by_name.get(name)
            if motor:
                motor.set_angle(pos)

    def _gripper_cmd_cb(self, msg: Float64, motor_names: list, side: str):
        """Map normalized 0.0 (open) – 1.0 (close) to each motor's angle range.

        Skips the per-motor write when the requested value matches the last
        commanded value (within _GRIPPER_EPS). See the dedup comment in
        __init__ — this prevents the gripper-flood video-freeze symptom.
        """
        val = max(0.0, min(1.0, msg.data))
        last = self._last_gripper_val.get(side)
        if last is not None and abs(val - last) < self._GRIPPER_EPS:
            return
        self._last_gripper_val[side] = val
        for name in motor_names:
            motor = self._motor_by_name.get(name)
            if motor:
                angle = motor.angle_min + val * (motor.angle_max - motor.angle_min)
                motor.set_angle(angle)

    # --- Wheel command handling --------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        if not self.xiaomi:
            return
        self._last_cmd_vel_time = self.get_clock().now()

        # linear.x = forward/backward (-1..1), angular.z = turn (-1..1)
        forward = msg.linear.x
        turn = msg.angular.z

        # Scale forward speed (reduce when going backward)
        if forward < 0:
            forward_speed = forward * self._max_linear_speed * self._backward_speed_pct
        else:
            forward_speed = forward * self._max_linear_speed

        turn_speed = -turn * self._max_turn_speed

        # Differential drive: left = forward + turn, right = forward - turn
        left = forward_speed + turn_speed
        right = forward_speed - turn_speed

        # Clamp to max wheel speed
        left = max(-self._max_wheel_speed, min(self._max_wheel_speed, left))
        right = max(-self._max_wheel_speed, min(self._max_wheel_speed, right))
        self.xiaomi.set_speed(left, right)

    def _wheel_timeout_check(self):
        """Stop wheels if no cmd_vel received for 0.2s."""
        if not self.xiaomi:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > 0.2:
            self.xiaomi.set_speed(0.0, 0.0)

    # --- Async position request / publish ----------------------------------

    def _request_positions(self):
        if not self.motors:
            return
        motor = self.motors[self._request_idx % len(self.motors)]
        motor.esp.request_pos(motor.esp_id, prefix=motor.cmd_prefix)
        self._request_idx += 1

    def _publish_joint_states(self):
        head_names, head_pos = [], []
        rarm_names, rarm_pos = [], []
        larm_names, larm_pos = [], []

        for motor in self.motors:
            angle = motor.get_angle()
            if angle is None:
                continue
            if motor.name.startswith("head"):
                head_names.append(motor.name)
                head_pos.append(angle)
            elif motor.name.startswith("right"):
                rarm_names.append(motor.name)
                rarm_pos.append(angle)
            elif motor.name.startswith("left"):
                larm_names.append(motor.name)
                larm_pos.append(angle)

        now = self.get_clock().now().to_msg()

        if head_pos:
            msg = JointState()
            msg.header.stamp = now
            msg.name = head_names
            msg.position = head_pos
            self._head_pub.publish(msg)

        if rarm_pos:
            msg = JointState()
            msg.header.stamp = now
            msg.name = rarm_names
            msg.position = rarm_pos
            self._rarm_pub.publish(msg)

        if larm_pos:
            msg = JointState()
            msg.header.stamp = now
            msg.name = larm_names
            msg.position = larm_pos
            self._larm_pub.publish(msg)

    # --- Cleanup -----------------------------------------------------------

    def destroy_node(self):
        for esp in self.servo_esps.values():
            esp.stop()
        if self.xiaomi:
            self.xiaomi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorHandler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
