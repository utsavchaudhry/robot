#!/usr/bin/env python3
"""
Relays /joint_states (from joint_state_publisher_gui sliders) to the
per-group command topics that motor_handler subscribes to.

motor_handler's _joint_cmd_cb looks up motors by name regardless of topic,
so we can publish all joints to a single command topic. For clarity we
split by name prefix.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SliderCommandRelay(Node):
    def __init__(self):
        super().__init__('slider_command_relay')
        self.create_subscription(JointState, 'joint_states', self._cb, 10)
        self._head_pub = self.create_publisher(
            JointState, 'head/joint_commands', 10)
        self._right_pub = self.create_publisher(
            JointState, 'right_arm/joint_commands', 10)
        self._left_pub = self.create_publisher(
            JointState, 'left_arm/joint_commands', 10)
        self.get_logger().info('Relaying /joint_states -> motor command topics')

    def _cb(self, msg: JointState):
        head = JointState()
        right = JointState()
        left = JointState()
        head.header = right.header = left.header = msg.header

        for name, pos in zip(msg.name, msg.position):
            if name.startswith('head'):
                head.name.append(name)
                head.position.append(pos)
            elif name.startswith('right'):
                right.name.append(name)
                right.position.append(pos)
            elif name.startswith('left'):
                left.name.append(name)
                left.position.append(pos)

        if head.name:
            self._head_pub.publish(head)
        if right.name:
            self._right_pub.publish(right)
        if left.name:
            self._left_pub.publish(left)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SliderCommandRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
