#!/usr/bin/env python3
"""
Teleop Data Logger Node
=======================
Subscribes to control commands and IK joint outputs, writing them to
timestamped CSV files for offline analysis.

Logs are stored in a persistent location, organized per session:

    ~/teleop_logs/
    └── <YYYY-MM-DD_HHMMSS>/          # one directory per launch session
        ├── session_info.json          # launch metadata
        ├── teleop_commands/
        │   └── teleop_commands.csv    # every TeleopCommand from WebRTC
        ├── joint_states/
        │   └── joint_states.csv       # every /joint_states message (IK output)
        └── session/
            └── session.jsonl          # both events interleaved as JSON-lines

Usage:
    ros2 run robot_bringup teleop_data_logger.py
    ros2 run robot_bringup teleop_data_logger.py --ros-args -p log_dir:=/tmp/my_logs
"""

import os
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import TeleopCommand
from sensor_msgs.msg import JointState


class TeleopDataLogger(Node):
    """Logs teleop commands and joint states to CSV files."""

    # Column names for the teleop command CSV
    TELEOP_COLUMNS = [
        'time_sec',
        'sequence_number',
        'timestamp_us',
        'mode',
        'emergency_stop',
        # Head pose
        'head_pos_x', 'head_pos_y', 'head_pos_z',
        'head_ori_x', 'head_ori_y', 'head_ori_z', 'head_ori_w',
        # Left arm
        'left_command_type',
        'left_delta_x', 'left_delta_y', 'left_delta_z',
        'left_gripper',
        # Right arm
        'right_command_type',
        'right_delta_x', 'right_delta_y', 'right_delta_z',
        'right_gripper',
    ]

    def __init__(self):
        super().__init__('teleop_data_logger')

        # Parameters
        self.declare_parameter('log_dir', os.path.expanduser('~/teleop_logs'))
        self.declare_parameter('flush_interval', 1.0)  # seconds between fflush

        base_log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        flush_interval = self.get_parameter('flush_interval').value

        # Create a per-session directory named by timestamp
        session_ts = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        session_dir = os.path.join(base_log_dir, session_ts)

        # Create subdirectories organized by data source
        teleop_dir = os.path.join(session_dir, 'teleop_commands')
        joint_dir = os.path.join(session_dir, 'joint_states')
        session_log_dir = os.path.join(session_dir, 'session')

        for d in [teleop_dir, joint_dir, session_log_dir]:
            os.makedirs(d, exist_ok=True)

        # Write session metadata
        session_info = {
            'session_start': datetime.now().isoformat(),
            'session_id': session_ts,
            'ros_log_dir': os.environ.get('ROS_LOG_DIR', ''),
            'hostname': os.uname().nodename,
            'files': {
                'teleop_commands': 'teleop_commands/teleop_commands.csv',
                'joint_states': 'joint_states/joint_states.csv',
                'session_jsonl': 'session/session.jsonl',
            },
        }
        with open(os.path.join(session_dir, 'session_info.json'), 'w') as f:
            json.dump(session_info, f, indent=2)

        # ---- Teleop commands CSV ----
        self._teleop_path = os.path.join(teleop_dir, 'teleop_commands.csv')
        self._teleop_file = open(self._teleop_path, 'w', newline='')
        self._teleop_writer = csv.writer(self._teleop_file)
        self._teleop_writer.writerow(self.TELEOP_COLUMNS)

        # ---- Joint states CSV ----
        self._joint_path = os.path.join(joint_dir, 'joint_states.csv')
        self._joint_file = open(self._joint_path, 'w', newline='')
        self._joint_writer = csv.writer(self._joint_file)
        self._joint_header_written = False  # Write header on first message (dynamic joint names)

        # ---- Combined JSON-lines log (full structured data) ----
        self._jsonl_path = os.path.join(session_log_dir, 'session.jsonl')
        self._jsonl_file = open(self._jsonl_path, 'w')

        # Store session dir for shutdown summary
        self._session_dir = session_dir

        # Counters
        self._teleop_count = 0
        self._joint_count = 0

        # Subscriptions
        self.create_subscription(TeleopCommand, 'teleop_commands', self._teleop_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)

        # Periodic flush
        self.create_timer(flush_interval, self._flush)

        self.get_logger().info(f'Teleop Data Logger started')
        self.get_logger().info(f'  Session dir: {session_dir}')
        self.get_logger().info(f'  teleop_commands/ — WebRTC teleop commands')
        self.get_logger().info(f'  joint_states/    — IK joint output')
        self.get_logger().info(f'  session/         — combined JSONL log')

    # ------------------------------------------------------------------
    # Teleop command callback
    # ------------------------------------------------------------------
    def _teleop_cb(self, msg: TeleopCommand):
        t = self._now_sec()

        row = [
            f'{t:.6f}',
            msg.sequence_number,
            msg.timestamp_us,
            msg.mode,
            int(msg.emergency_stop),
            # Head pose
            msg.head_pose.position.x,
            msg.head_pose.position.y,
            msg.head_pose.position.z,
            msg.head_pose.orientation.x,
            msg.head_pose.orientation.y,
            msg.head_pose.orientation.z,
            msg.head_pose.orientation.w,
            # Left arm
            msg.left_arm.command_type,
            msg.left_arm.delta_position.x,
            msg.left_arm.delta_position.y,
            msg.left_arm.delta_position.z,
            msg.left_arm.gripper_position,
            # Right arm
            msg.right_arm.command_type,
            msg.right_arm.delta_position.x,
            msg.right_arm.delta_position.y,
            msg.right_arm.delta_position.z,
            msg.right_arm.gripper_position,
        ]
        self._teleop_writer.writerow(row)
        self._teleop_count += 1

        # Also write to JSONL
        self._write_jsonl('teleop_command', t, {
            'mode': msg.mode,
            'sequence_number': msg.sequence_number,
            'timestamp_us': msg.timestamp_us,
            'emergency_stop': msg.emergency_stop,
            'head_pose': self._pose_dict(msg.head_pose),
            'left_arm': self._arm_dict(msg.left_arm),
            'right_arm': self._arm_dict(msg.right_arm),
        })

        if self._teleop_count % 100 == 0:
            self.get_logger().info(f'Logged {self._teleop_count} teleop commands')

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------
    def _joint_cb(self, msg: JointState):
        t = self._now_sec()

        # Write CSV header dynamically on first message (joint names vary)
        if not self._joint_header_written:
            header = ['time_sec'] + list(msg.name)
            self._joint_writer.writerow(header)
            self._joint_header_written = True
            self._joint_names = list(msg.name)

        positions = list(msg.position)
        row = [f'{t:.6f}'] + [f'{p:.6f}' for p in positions]
        self._joint_writer.writerow(row)
        self._joint_count += 1

        # Also write to JSONL
        joint_dict = {}
        for name, pos in zip(msg.name, msg.position):
            joint_dict[name] = round(pos, 6)
        self._write_jsonl('joint_state', t, joint_dict)

        if self._joint_count % 100 == 0:
            self.get_logger().info(f'Logged {self._joint_count} joint states')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _now_sec(self) -> float:
        """Current ROS time as float seconds."""
        stamp = self.get_clock().now().to_msg()
        return stamp.sec + stamp.nanosec * 1e-9

    def _pose_dict(self, pose) -> dict:
        return {
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {
                'x': pose.orientation.x, 'y': pose.orientation.y,
                'z': pose.orientation.z, 'w': pose.orientation.w,
            },
        }

    def _arm_dict(self, arm) -> dict:
        return {
            'command_type': arm.command_type,
            'arm_name': arm.arm_name,
            'delta_position': {
                'x': arm.delta_position.x,
                'y': arm.delta_position.y,
                'z': arm.delta_position.z,
            },
            'gripper_position': arm.gripper_position,
        }

    def _write_jsonl(self, event_type: str, time_sec: float, data: dict):
        record = {
            'event': event_type,
            'time_sec': round(time_sec, 6),
            'data': data,
        }
        self._jsonl_file.write(json.dumps(record) + '\n')

    def _flush(self):
        """Periodically flush files to disk."""
        self._teleop_file.flush()
        self._joint_file.flush()
        self._jsonl_file.flush()

    def destroy_node(self):
        """Close files and update session metadata on shutdown."""
        self.get_logger().info(
            f'Logger shutting down. Logged {self._teleop_count} teleop commands, '
            f'{self._joint_count} joint states.'
        )
        self.get_logger().info(f'  Session dir: {self._session_dir}')

        # Update session_info.json with final stats
        info_path = os.path.join(self._session_dir, 'session_info.json')
        try:
            with open(info_path, 'r') as f:
                info = json.load(f)
            info['session_end'] = datetime.now().isoformat()
            info['stats'] = {
                'teleop_commands_logged': self._teleop_count,
                'joint_states_logged': self._joint_count,
            }
            with open(info_path, 'w') as f:
                json.dump(info, f, indent=2)
        except Exception:
            pass

        self._teleop_file.close()
        self._joint_file.close()
        self._jsonl_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopDataLogger()
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
