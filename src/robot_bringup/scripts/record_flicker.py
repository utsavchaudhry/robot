#!/usr/bin/env python3
"""
Capture every joint command + the teleop input that drove it, one JSON object
per message, to a single file. Designed for diagnosing post-VR-connect flicker
or IK weirdness.

Usage:
    ./record_flicker.py [out_path] [duration_seconds]
    # defaults: /tmp/flicker.jsonl, runs until Ctrl-C

Topics recorded:
    /teleop_commands              — VR input from operator
    /joint_states                 — relay output that drives RViz / TF
    /head/joint_commands          — IK head output (yaw, pitch)
    /left_arm/joint_commands      — IK left arm output (7 joints)
    /right_arm/joint_commands     — IK right arm output (7 joints)
    /left_arm/ik_target           — teleop_controller's IK target (if any)
    /right_arm/ik_target          — same, right arm

Each line of the output file is `{"t": <monotonic_seconds_since_start>,
"topic": <topic>, ...}` so it's easy to slice + plot + diff.
"""

import json
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from robot_interfaces.msg import TeleopCommand


def _pose_dict(p: Pose) -> dict:
    return {
        "pos": [p.position.x, p.position.y, p.position.z],
        "rot": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
    }


class FlickerRecorder(Node):
    def __init__(self, out_path: str):
        super().__init__("flicker_recorder")
        self._t0 = time.monotonic()
        self._f = open(out_path, "w", buffering=1)  # line-buffered so we don't lose data on SIGINT
        self.get_logger().info(f"Writing JSONL → {out_path}")

        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._js("joint_states", m), 50)
        self.create_subscription(JointState, "/left_arm/joint_commands",
                                 lambda m: self._js("left_arm/joint_commands", m), 50)
        self.create_subscription(JointState, "/right_arm/joint_commands",
                                 lambda m: self._js("right_arm/joint_commands", m), 50)
        self.create_subscription(JointState, "/head/joint_commands",
                                 lambda m: self._js("head/joint_commands", m), 50)
        self.create_subscription(Pose, "/left_arm/ik_target",
                                 lambda m: self._pose("left_arm/ik_target", m), 50)
        self.create_subscription(Pose, "/right_arm/ik_target",
                                 lambda m: self._pose("right_arm/ik_target", m), 50)
        self.create_subscription(TeleopCommand, "/teleop_commands",
                                 self._teleop, 50)

    def _t(self) -> float:
        return time.monotonic() - self._t0

    def _emit(self, rec: dict):
        self._f.write(json.dumps(rec) + "\n")

    def _js(self, topic: str, m: JointState):
        self._emit({
            "t": self._t(),
            "topic": topic,
            "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "name": list(m.name),
            "position": [float(p) for p in m.position],
        })

    def _pose(self, topic: str, m: Pose):
        self._emit({
            "t": self._t(),
            "topic": topic,
            **_pose_dict(m),
        })

    def _teleop(self, m: TeleopCommand):
        self._emit({
            "t": self._t(),
            "topic": "teleop_commands",
            "mode": m.mode,
            "seq": int(m.sequence_number),
            "head": _pose_dict(m.head_pose),
            "left_ctrl": _pose_dict(m.left_controller_pose),
            "right_ctrl": _pose_dict(m.right_controller_pose),
            "left_arm_cmd_type": m.left_arm.command_type,
            "left_gripper": float(m.left_arm.gripper_position),
            "right_arm_cmd_type": m.right_arm.command_type,
            "right_gripper": float(m.right_arm.gripper_position),
            "drive": [float(m.drive_linear), float(m.drive_angular)],
        })

    def close(self):
        self._f.close()


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/flicker.jsonl"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rclpy.init()
    node = FlickerRecorder(out_path)

    try:
        if duration is None:
            print(f"Recording to {out_path}. Press Ctrl-C to stop.", flush=True)
            rclpy.spin(node)
        else:
            print(f"Recording to {out_path} for {duration}s.", flush=True)
            t_end = time.monotonic() + duration
            while time.monotonic() < t_end:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print(f"Done. {out_path}")


if __name__ == "__main__":
    main()
