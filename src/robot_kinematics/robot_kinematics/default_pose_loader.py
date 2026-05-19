"""
Utility module for loading default robot pose from YAML configuration.
Centralizes default joint angle values to avoid hardcoding.
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Dict
from ament_index_python.packages import get_package_share_directory


def load_default_pose(config_path: str = None) -> Dict[str, np.ndarray]:
    """
    Load default joint pose from YAML configuration file.
    
    Args:
        config_path: Optional path to YAML file. If None, uses default location.
        
    Returns:
        Dictionary with keys 'head', 'left', 'right' mapping to numpy arrays
        of joint angles in the order expected by the kinematics solver.
        
    Example:
        >>> pose = load_default_pose()
        >>> left_arm_angles = pose['left']  # [shoulder_pitch, shoulder_yaw, ...]
        >>> head_angles = pose['head']     # [yaw, pitch]
    """
    if config_path is None:
        # Use default location in robot_bringup package
        pkg_path = get_package_share_directory('robot_bringup')
        config_path = Path(pkg_path) / 'config' / 'default_pose.yaml'
    
    config_path = Path(config_path)
    
    if not config_path.exists():
        raise FileNotFoundError(
            f"Default pose config file not found: {config_path}\n"
            f"Please create the config file or specify a valid path."
        )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract joint angles in the correct order
    # Joint order per arm: [shoulder_pitch, shoulder_yaw, shoulder_roll, elbow_flex, wrist_roll, wrist_yaw, hand_wrist_pitch]
    left_arm = np.array([
        config['left_arm']['shoulder_pitch'],
        config['left_arm']['shoulder_yaw'],
        config['left_arm']['shoulder_roll'],
        config['left_arm']['elbow_flex'],
        config['left_arm']['wrist_roll'],
        config['left_arm']['wrist_yaw'],
        config['left_arm']['hand_wrist_pitch'],
    ], dtype=float)
    
    right_arm = np.array([
        config['right_arm']['shoulder_pitch'],
        config['right_arm']['shoulder_yaw'],
        config['right_arm']['shoulder_roll'],
        config['right_arm']['elbow_flex'],
        config['right_arm']['wrist_roll'],
        config['right_arm']['wrist_yaw'],
        config['right_arm']['hand_wrist_pitch'],
    ], dtype=float)
    
    # Head: [yaw, pitch]
    head = np.array([
        config['head']['yaw'],
        config['head']['pitch'],
    ], dtype=float)
    
    return {
        'left': left_arm,
        'right': right_arm,
        'head': head,
    }


def get_default_pose_dict() -> Dict[str, Dict[str, float]]:
    """
    Get default pose as a dictionary of dictionaries (for YAML compatibility).
    
    Returns:
        Dictionary with structure matching the YAML file format.
    """
    pose = load_default_pose()
    
    return {
        'head': {
            'yaw': float(pose['head'][0]),
            'pitch': float(pose['head'][1]),
        },
        'left_arm': {
            'shoulder_pitch': float(pose['left'][0]),
            'shoulder_yaw': float(pose['left'][1]),
            'shoulder_roll': float(pose['left'][2]),
            'elbow_flex': float(pose['left'][3]),
            'wrist_roll': float(pose['left'][4]),
            'wrist_yaw': float(pose['left'][5]),
            'hand_wrist_pitch': float(pose['left'][6]),
        },
        'right_arm': {
            'shoulder_pitch': float(pose['right'][0]),
            'shoulder_yaw': float(pose['right'][1]),
            'shoulder_roll': float(pose['right'][2]),
            'elbow_flex': float(pose['right'][3]),
            'wrist_roll': float(pose['right'][4]),
            'wrist_yaw': float(pose['right'][5]),
            'hand_wrist_pitch': float(pose['right'][6]),
        },
    }
