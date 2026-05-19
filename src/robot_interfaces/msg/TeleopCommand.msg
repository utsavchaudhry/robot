# High-level teleop command from Unity operator (VR tracking)
string mode  # "joint_control", "ik_control", "velocity_control", "idle"

# VR poses in Unity world frame (position + quaternion)
geometry_msgs/Pose head_pose           # VR headset tracking
geometry_msgs/Pose left_controller_pose   # Left VR controller
geometry_msgs/Pose right_controller_pose  # Right VR controller

# Arm commands (for backward compatibility or direct joint control)
ArmCommand left_arm
ArmCommand right_arm

# Drive (wheels) — normalized velocity, -1 to +1
float64 drive_linear    # forward/backward
float64 drive_angular   # turn left/right

bool emergency_stop
uint64 timestamp_us
uint32 sequence_number
