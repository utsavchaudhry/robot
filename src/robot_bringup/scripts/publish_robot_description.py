#!/usr/bin/env python3
"""Publishes robot_description to /robot_description for RViz RobotModel."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('publish_robot_description')
    node.declare_parameter('robot_description', '')
    udef = node.get_parameter('robot_description').value
    # TRANSIENT_LOCAL so RViz (and late joiners) receive the description
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, 'robot_description', qos)
    msg = String()
    msg.data = udef

    # Publish once immediately and once after 1.5s for late-joining RViz.
    # Do NOT repeat: joint_state_publisher (and _gui) re-run configure_robot() on
    # every /robot_description message, which resets all joint positions to zero
    # and causes the model to "center" at the publish frequency.
    pub.publish(msg)

    def once_for_late_joiners():
        pub.publish(msg)
        late_timer.cancel()

    late_timer = node.create_timer(1.5, once_for_late_joiners)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
