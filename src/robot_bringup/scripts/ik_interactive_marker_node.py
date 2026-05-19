#!/usr/bin/env python3
"""
Interactive marker for IK target: drag the 6-DOF control in RViz to send ComputeIK goals.
Requires: ros-humble-interactive-markers, ros-humble-visualization-msgs.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    InteractiveMarkerUpdate,
    Marker,
)
from std_msgs.msg import Header
from rclpy.action import ActionClient
from robot_interfaces.action import ComputeIK


class IKInteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('ik_interactive_marker_node')
        self._seq = 0
        self._pub_update = self.create_publisher(
            InteractiveMarkerUpdate, 'ik_target/update', 10
        )
        self.create_subscription(
            InteractiveMarkerFeedback,
            'ik_target/feedback',
            self._feedback_cb,
            10
        )
        self._action = ActionClient(self, ComputeIK, 'compute_ik')
        self._marker_name = 'ik_target_pose'
        self._frame_id = 'base_link'  # Humanoid torso frame
        self._arm = 'left'
        # Throttle: max 5 goals/sec while dragging
        self._min_interval = 0.2
        self._last_sent = 0.0

        self._publish_marker()
        self.get_logger().info(
            'IK interactive marker running. In RViz add "InteractiveMarkers" '
            'display and set Update Topic to /ik_target/update. Drag the control to run IK.'
        )

    def _make_marker(self) -> InteractiveMarker:
        m = InteractiveMarker()
        m.header = Header()
        m.header.frame_id = self._frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = self._marker_name
        m.description = 'IK target: drag to move'
        m.pose.position.x = 0.3
        m.pose.position.y = 0.2
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.scale = 0.15

        c = InteractiveMarkerControl()
        c.name = 'move_rotate_3d'
        c.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        c.orientation_mode = InteractiveMarkerControl.INHERIT
        c.always_visible = True
        # Small sphere so the control is visible
        s = Marker()
        s.type = Marker.SPHERE
        s.scale.x = 0.05
        s.scale.y = 0.05
        s.scale.z = 0.05
        s.color.r = 0.2
        s.color.g = 0.8
        s.color.b = 0.2
        s.color.a = 0.8
        c.markers.append(s)
        m.controls.append(c)
        return m

    def _publish_marker(self):
        update = InteractiveMarkerUpdate()
        update.server_id = 'ik_target'
        update.seq_num = self._seq
        self._seq += 1
        update.type = InteractiveMarkerUpdate.UPDATE
        update.markers = [self._make_marker()]
        update.markers[0].header.stamp = self.get_clock().now().to_msg()
        self._pub_update.publish(update)

    def _feedback_cb(self, msg: InteractiveMarkerFeedback):
        if msg.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_sent < self._min_interval:
            return
        self._last_sent = now

        goal = ComputeIK.Goal()
        goal.target_pose.position = msg.pose.position
        goal.target_pose.orientation = msg.pose.orientation
        goal.arm_name = self._arm

        if not self._action.wait_for_server(timeout_sec=0.5):
            return
        future = self._action.send_goal_async(goal)
        future.add_done_callback(lambda f: None)  # fire-and-forget


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IKInteractiveMarkerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
