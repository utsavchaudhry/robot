# Command for arm control via WebRTC DataChannel
string command_type  # "joint_positions", "end_effector_pose", "gripper", "thumbstick_delta"
string arm_name      # "left", "right", "both"
float64[] joint_positions
geometry_msgs/Pose end_effector_pose
float64 gripper_position

# For flat screen thumbstick control (command_type="thumbstick_delta")
geometry_msgs/Vector3 delta_position      # thumbstick X/Y/Z deltas (-1 to 1, scaled)
