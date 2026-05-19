#!/usr/bin/env bash
# Kill every ROS process from this robot stack and reset the ros2 daemon.
# Use when you've ended up with duplicate node names (e.g. systemd + manual
# launch both running). Safe to run any time the robot is idle.

set -uo pipefail

sudo systemctl stop robot-teleop || true

sudo pkill -9 -f 'webrtc_node|teleop_controller_node|humanoid_kinematics_node' || true
sudo pkill -9 -f 'signaling_bridge|signaling_server' || true
sudo pkill -9 -f 'joint_state_relay|publish_robot_description' || true
sudo pkill -9 -f 'teleop_data_logger|motor_handler_node|robot_state_publisher' || true
sudo pkill -9 -f 'session_recorder_node|stereo_camera_node' || true
sudo pkill -9 -f 'ros2 launch' || true

sleep 1

ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "Surviving nodes:"
ros2 node list | sed 's/^/  /' || true

echo "Surviving processes:"
pgrep -af 'webrtc_node|teleop_controller_node|humanoid_kinematics_node|signaling|joint_state_relay|publish_robot_description|teleop_data_logger|motor_handler|robot_state_publisher|session_recorder|stereo_camera|ros2 launch' | sed 's/^/  /' || echo "  (none)"
