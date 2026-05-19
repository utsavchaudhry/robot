"""
Unity ↔ ROS Coordinate System Transformation

Unity: Left-handed, Y-up, Z-forward
  X-right, Y-up, Z-forward

ROS: Right-handed, Z-up, X-forward  
  X-forward, Y-left, Z-up

Transformation:
  ROS_X = Unity_Z
  ROS_Y = Unity_X
  ROS_Z = Unity_Y
"""

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose, Point, Quaternion


def unity_to_ros_position(unity_pos: np.ndarray) -> np.ndarray:
    """
    Convert Unity position (X-right, Y-up, Z-forward) to ROS (X-forward, Y-left, Z-up)
    
    Args:
        unity_pos: [x, y, z] in Unity coordinates
        
    Returns:
        [x, y, z] in ROS coordinates
    """
    return np.array([
        unity_pos[2],   # ROS X = Unity Z (forward)
        unity_pos[0],   # ROS Y = Unity X (left, but Unity X is right, so this maps right to left)
        unity_pos[1]    # ROS Z = Unity Y (up)
    ])


def ros_to_unity_position(ros_pos: np.ndarray) -> np.ndarray:
    """
    Convert ROS position to Unity position
    
    Args:
        ros_pos: [x, y, z] in ROS coordinates
        
    Returns:
        [x, y, z] in Unity coordinates
    """
    return np.array([
        ros_pos[1],     # Unity X = ROS Y
        ros_pos[2],     # Unity Y = ROS Z
        ros_pos[0]      # Unity Z = ROS X
    ])


def unity_to_ros_quaternion(unity_quat: np.ndarray) -> np.ndarray:
    """
    Convert Unity quaternion to ROS quaternion with coordinate system change
    
    Unity uses left-handed, so we need to:
    1. Swap axes to match ROS coordinate system
    2. Adjust for handedness change
    
    Args:
        unity_quat: [x, y, z, w] in Unity coordinates
        
    Returns:
        [x, y, z, w] in ROS coordinates
    """
    # Unity quaternion [x, y, z, w]
    ux, uy, uz, uw = unity_quat
    
    # Convert: Unity(X,Y,Z) → ROS(Z,X,Y)
    # For quaternions, we also need to negate one component for handedness
    return np.array([
        uz,   # ROS qx = Unity qz
        ux,   # ROS qy = Unity qx  
        uy,   # ROS qz = Unity qy
        -uw   # ROS qw = -Unity qw (handedness flip)
    ])


def ros_to_unity_quaternion(ros_quat: np.ndarray) -> np.ndarray:
    """
    Convert ROS quaternion to Unity quaternion
    
    Args:
        ros_quat: [x, y, z, w] in ROS coordinates
        
    Returns:
        [x, y, z, w] in Unity coordinates
    """
    rx, ry, rz, rw = ros_quat
    
    return np.array([
        ry,   # Unity qx = ROS qy
        rz,   # Unity qy = ROS qz
        rx,   # Unity qz = ROS qx
        -rw   # Unity qw = -ROS qw
    ])


def unity_pose_to_ros_se3(unity_pose: dict) -> pin.SE3:
    """
    Convert Unity pose (position + quaternion) to ROS Pinocchio SE3
    
    Args:
        unity_pose: dict with 'position' {x,y,z} and 'rotation' {x,y,z,w}
        
    Returns:
        Pinocchio SE3 transform in ROS coordinates
    """
    # Convert position
    u_pos = np.array([unity_pose['position']['x'], 
                      unity_pose['position']['y'], 
                      unity_pose['position']['z']])
    ros_pos = unity_to_ros_position(u_pos)
    
    # Convert quaternion
    u_quat = np.array([unity_pose['rotation']['x'],
                       unity_pose['rotation']['y'],
                       unity_pose['rotation']['z'],
                       unity_pose['rotation']['w']])
    ros_quat = unity_to_ros_quaternion(u_quat)
    
    # Build SE3
    quat = pin.Quaternion(ros_quat[3], ros_quat[0], ros_quat[1], ros_quat[2])  # w, x, y, z
    rotation = quat.toRotationMatrix()
    
    return pin.SE3(rotation, ros_pos)


def ros_pose_msg_to_unity_pose(ros_pose: Pose) -> dict:
    """
    Convert ROS Pose message to Unity pose dict
    
    Args:
        ros_pose: ROS geometry_msgs/Pose
        
    Returns:
        Unity pose dict with position and rotation
    """
    ros_pos = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
    ros_quat = np.array([ros_pose.orientation.x, ros_pose.orientation.y, 
                         ros_pose.orientation.z, ros_pose.orientation.w])
    
    unity_pos = ros_to_unity_position(ros_pos)
    unity_quat = ros_to_unity_quaternion(ros_quat)
    
    return {
        'position': {'x': unity_pos[0], 'y': unity_pos[1], 'z': unity_pos[2]},
        'rotation': {'x': unity_quat[0], 'y': unity_quat[1], 'z': unity_quat[2], 'w': unity_quat[3]}
    }


def unity_joint_angles_to_ros(unity_angles: np.ndarray, joint_axes_unity: list) -> np.ndarray:
    """
    Convert Unity joint angles to ROS joint angles accounting for axis transformations
    
    Args:
        unity_angles: Joint angles in Unity (radians)
        joint_axes_unity: List of axis vectors for each joint in Unity coords
        
    Returns:
        Joint angles in ROS coordinates
    """
    # For each joint, if the axis changed direction in the coordinate transform,
    # we may need to negate the angle
    # This is joint-specific and depends on the URDF structure
    
    # TODO: Implement per-joint axis mapping based on URDF analysis
    return unity_angles  # Placeholder - needs joint-specific mapping
