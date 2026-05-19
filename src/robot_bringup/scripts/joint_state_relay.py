#!/usr/bin/env python3
"""
Relays per-group joint topics to /joint_states for RViz RobotModel.

Parameters:
    source (str): 'commands' — relay joint_commands (IK/teleop output, default)
                  'states'   — relay joint_states  (motor readback from hardware)
    swap_left_right (bool): swap left/right arm data (diagnostic)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.declare_parameter('swap_left_right', False)
        self.declare_parameter('source', 'commands')
        self._swap = self.get_parameter('swap_left_right').value
        source = self.get_parameter('source').value
        suffix = 'joint_states' if source == 'states' else 'joint_commands'
        self.get_logger().info(f"Relaying from */{ suffix } to /joint_states")
        self._head = None
        self._left = None
        self._right = None
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(
            JointState, f'head/{suffix}', self._head_cb, 10
        )
        self.create_subscription(
            JointState, f'left_arm/{suffix}', self._left_cb, 10
        )
        self.create_subscription(
            JointState, f'right_arm/{suffix}', self._right_cb, 10
        )
        # Publish at 20 Hz so RViz/robot_state_publisher get updates
        self._timer = self.create_timer(0.05, self._publish)
        if self._swap:
            self.get_logger().warn('swap_left_right=True: left/right joint data are swapped (diagnostic)')

    def _head_cb(self, msg: JointState):
        self._head = msg

    def _left_cb(self, msg: JointState):
        self._left = msg

    def _right_cb(self, msg: JointState):
        self._right = msg

    def _publish(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ''
        
        # Build as lists, then assign once
        names = []
        positions = []
        
        # Head joints
        if self._head:
            n, p = list(self._head.name), list(self._head.position)
            names.extend(n)
            positions.extend(p[:len(n)] if len(p) >= len(n) else p + [0.0] * (len(n) - len(p)))
        else:
            names.extend(['head_yaw', 'head_pitch'])
            positions.extend([0.0, 0.0])

        # Left/right: when swap_left_right, use right's positions for left joints and left's for right (diagnostic)
        left_data = self._right if self._swap else self._left
        right_data = self._left if self._swap else self._right

        left_names = [
            'left_shoulder_pitch', 'left_shoulder_yaw', 'left_shoulder_roll',
            'left_elbow_flex', 'left_wrist_roll', 'left_wrist_yaw', 'left_hand_wrist_pitch'
        ]
        right_names = [
            'right_shoulder_pitch', 'right_shoulder_yaw', 'right_shoulder_roll',
            'right_elbow_flex', 'right_wrist_roll', 'right_wrist_yaw', 'right_hand_wrist_pitch'
        ]

        # Default 7-DOF pose when no data: elbow_flex in [-2.53,-0.16]; use -0.5 to avoid over-bent elbow
        default_arm = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0]

        # Left arm: always left joint names; positions from left_data (or right when swapped)
        if left_data and len(left_data.position) >= 7:
            p = list(left_data.position)[:7]
            names.extend(left_names)
            positions.extend(p)
        else:
            names.extend(left_names)
            positions.extend(default_arm)

        # Right arm: always right joint names; positions from right_data (or left when swapped)
        if right_data and len(right_data.position) >= 7:
            p = list(right_data.position)[:7]
            names.extend(right_names)
            positions.extend(p)
        else:
            names.extend(right_names)
            positions.extend(default_arm)
        
        out.name = names
        out.position = positions
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
