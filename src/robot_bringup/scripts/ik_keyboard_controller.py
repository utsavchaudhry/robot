#!/usr/bin/env python3
"""
Keyboard controller for moving both arms' end effectors using IK.
Simple keyboard interface to move left/right arm end effectors in X, Y, Z directions.

Controls:
  - WASD: Move left arm (W/S: Z, A/D: Y, Q/E: X)
  - IJKL: Move right arm (I/K: Z, J/L: Y, U/O: X)
  - Arrow keys: Move both arms together (Up/Down: Z, Left/Right: Y, PageUp/PageDown: X)
  - R: Reset both arms to initial pose
  - T: Toggle between left/right arm (for fine control)
  - +/-: Increase/decrease step size
  - ESC or Ctrl+C: Exit

Requires: keyboard library (pip install keyboard)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_interfaces.action import ComputeIK
from robot_kinematics.humanoid_kinematics import HumanoidKinematics
from robot_kinematics.default_pose_loader import load_default_pose
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import numpy as np
import threading
import sys

try:
    from xacro import process_file as xacro_process_file
except ImportError:
    xacro_process_file = None

try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("WARNING: keyboard library not installed. Install with: pip3 install keyboard")
    print("On Linux, you may need: sudo apt install python3-keyboard")


def _ensure_urdf(path: str) -> str:
    """If path ends with .xacro, expand to a temp .urdf and return that path; else return path."""
    if not path or not path.endswith('.xacro'):
        return path
    if xacro_process_file is None:
        raise RuntimeError("xacro is required to use .xacro URDFs; install the xacro package")
    xml = xacro_process_file(path).toxml()
    f = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', prefix='arm_', delete=False)
    f.write(xml)
    f.close()
    return f.name


def _compute_default_ee_poses():
    """Compute end-effector positions from default joint angles loaded from config"""
    # Load default joint angles from YAML configuration
    try:
        initial_joint_positions = load_default_pose()
    except Exception as e:
        print(f"Warning: Could not load default pose config: {e}")
        print("Using fallback default pose.")
        # Fallback to hardcoded values if config fails
        head_initial = np.array([0.0, 0.0], dtype=float)
        left_arm_initial = np.array(
            [0.0, -1.5635088, 0.0, -2.98, 0.8987049384369203, 0.0, -0.0001919862177193199],
            dtype=float
        )
        right_arm_initial = np.array(
            [0.0, 1.5711316, 0.0, -0.16, 0.8987049384369203, 0.0, -0.0001919862177193199],
            dtype=float
        )
        initial_joint_positions = {
            'left': left_arm_initial,
            'right': right_arm_initial,
            'head': head_initial,
        }
    
    # Joint names
    joint_names = {
        'left': [
            'left_shoulder_pitch', 'left_shoulder_yaw', 'left_shoulder_roll',
            'left_elbow_flex', 'left_wrist_roll', 'left_wrist_yaw', 'left_hand_wrist_pitch'
        ],
        'right': [
            'right_shoulder_pitch', 'right_shoulder_yaw', 'right_shoulder_roll',
            'right_elbow_flex', 'right_wrist_roll', 'right_wrist_yaw', 'right_hand_wrist_pitch'
        ],
        'head': ['head_yaw', 'head_pitch']
    }
    
    end_effector_links = {
        'left': 'left_end_effector',
        'right': 'right_end_effector',
    }
    
    # Load URDF and compute FK
    try:
        pkg_path = get_package_share_directory('robot_description')
        urdf_path = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')
        urdf_path = _ensure_urdf(urdf_path)
        
        # Initialize kinematics solver to compute FK
        ik_solver = HumanoidKinematics(
            urdf_path=urdf_path,
            end_effector_links=end_effector_links,
            joint_names=joint_names,
            initial_joint_positions=initial_joint_positions,
            position_cost=1.0,
            orientation_cost=0.0,
            regularization_cost=0.001,
            robot_joint_fq=50.0,
            debug_mode=False,
        )
        
        # Get current poses (which are at the default configuration)
        poses = ik_solver.get_current_poses()
        
        left_pos = poses['left'].translation
        right_pos = poses['right'].translation
        
        return left_pos, right_pos
    except Exception as e:
        print(f"Warning: Could not compute FK from default pose: {e}")
        print("Using approximate default positions.")
        # Fallback to approximate positions (will be updated when IK runs)
        return np.array([0.0, 0.15, 0.15]), np.array([0.0, -0.15, 0.15])


class IKKeyboardController(Node):
    def __init__(self):
        super().__init__('ik_keyboard_controller')
        
        # Action clients for IK computation
        self._ik_action_client = ActionClient(self, ComputeIK, 'compute_ik')
        
        # Wait for action server
        self.get_logger().info('Waiting for IK action server...')
        if not self._ik_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('IK action server not available!')
            return
        self.get_logger().info('IK action server connected!')
        
        # Compute default end-effector poses from default joint angles (once, cached)
        self.get_logger().info('Computing default end-effector poses from default joint angles...')
        self._default_left_pos, self._default_right_pos = _compute_default_ee_poses()
        
        # Current end effector poses (in base_link frame)
        # These are computed from the default pose joint angles to match the IK node's default pose
        self.left_ee_pose = Pose()
        self.left_ee_pose.position = Point(
            x=float(self._default_left_pos[0]), 
            y=float(self._default_left_pos[1]), 
            z=float(self._default_left_pos[2])
        )
        self.left_ee_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        
        self.right_ee_pose = Pose()
        self.right_ee_pose.position = Point(
            x=float(self._default_right_pos[0]), 
            y=float(self._default_right_pos[1]), 
            z=float(self._default_right_pos[2])
        )
        self.right_ee_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        
        self.get_logger().info(f'Default left EE position: ({self._default_left_pos[0]:.4f}, {self._default_left_pos[1]:.4f}, {self._default_left_pos[2]:.4f})')
        self.get_logger().info(f'Default right EE position: ({self._default_right_pos[0]:.4f}, {self._default_right_pos[1]:.4f}, {self._default_right_pos[2]:.4f})')
        
        # Step size for movement (in meters)
        self.step_size = 0.02  # 2cm per step
        
        # Current active arm for fine control
        self.active_arm = 'left'
        
        # Thread for keyboard input
        self._keyboard_thread = None
        self._running = True
        
        if KEYBOARD_AVAILABLE:
            self._start_keyboard_listener()
            self._print_instructions()
        else:
            self.get_logger().error('Keyboard library not available. Cannot start controller.')
    
    def _print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("IK Keyboard Controller - End Effector Movement")
        print("="*60)
        print("LEFT ARM:")
        print("  W/S: Move up/down (Z-axis)")
        print("  A/D: Move left/right (Y-axis)")
        print("  Q/E: Move forward/backward (X-axis)")
        print("\nRIGHT ARM:")
        print("  I/K: Move up/down (Z-axis)")
        print("  J/L: Move left/right (Y-axis)")
        print("  U/O: Move forward/backward (X-axis)")
        print("\nBOTH ARMS (synchronized):")
        print("  ↑/↓: Move up/down (Z-axis)")
        print("  ←/→: Move left/right (Y-axis)")
        print("  PageUp/PageDown: Move forward/backward (X-axis)")
        print("\nOTHER:")
        print("  T: Toggle active arm (left/right)")
        print("  +/-: Increase/decrease step size")
        print("  R: Reset both arms to initial pose")
        print("  ESC or Ctrl+C: Exit")
        print("="*60)
        print(f"Current step size: {self.step_size*100:.1f} cm")
        print(f"Active arm: {self.active_arm}")
        print("="*60 + "\n")
    
    def _start_keyboard_listener(self):
        """Start keyboard listener in a separate thread"""
        def keyboard_loop():
            while self._running:
                try:
                    # Left arm controls
                    if keyboard.is_pressed('w'):
                        self._move_ee('left', 'z', self.step_size)
                        self._wait_key_release('w')
                    elif keyboard.is_pressed('s'):
                        self._move_ee('left', 'z', -self.step_size)
                        self._wait_key_release('s')
                    elif keyboard.is_pressed('a'):
                        self._move_ee('left', 'y', self.step_size)
                        self._wait_key_release('a')
                    elif keyboard.is_pressed('d'):
                        self._move_ee('left', 'y', -self.step_size)
                        self._wait_key_release('d')
                    elif keyboard.is_pressed('q'):
                        self._move_ee('left', 'x', self.step_size)
                        self._wait_key_release('q')
                    elif keyboard.is_pressed('e'):
                        self._move_ee('left', 'x', -self.step_size)
                        self._wait_key_release('e')
                    
                    # Right arm controls
                    elif keyboard.is_pressed('i'):
                        self._move_ee('right', 'z', self.step_size)
                        self._wait_key_release('i')
                    elif keyboard.is_pressed('k'):
                        self._move_ee('right', 'z', -self.step_size)
                        self._wait_key_release('k')
                    elif keyboard.is_pressed('j'):
                        self._move_ee('right', 'y', self.step_size)
                        self._wait_key_release('j')
                    elif keyboard.is_pressed('l'):
                        self._move_ee('right', 'y', -self.step_size)
                        self._wait_key_release('l')
                    elif keyboard.is_pressed('u'):
                        self._move_ee('right', 'x', self.step_size)
                        self._wait_key_release('u')
                    elif keyboard.is_pressed('o'):
                        self._move_ee('right', 'x', -self.step_size)
                        self._wait_key_release('o')
                    
                    # Both arms synchronized
                    elif keyboard.is_pressed('up'):
                        self._move_ee('left', 'z', self.step_size)
                        self._move_ee('right', 'z', self.step_size)
                        self._wait_key_release('up')
                    elif keyboard.is_pressed('down'):
                        self._move_ee('left', 'z', -self.step_size)
                        self._move_ee('right', 'z', -self.step_size)
                        self._wait_key_release('down')
                    elif keyboard.is_pressed('left'):
                        self._move_ee('left', 'y', self.step_size)
                        self._move_ee('right', 'y', -self.step_size)  # Opposite for symmetry
                        self._wait_key_release('left')
                    elif keyboard.is_pressed('right'):
                        self._move_ee('left', 'y', -self.step_size)
                        self._move_ee('right', 'y', self.step_size)  # Opposite for symmetry
                        self._wait_key_release('right')
                    elif keyboard.is_pressed('page up'):
                        self._move_ee('left', 'x', self.step_size)
                        self._move_ee('right', 'x', self.step_size)
                        self._wait_key_release('page up')
                    elif keyboard.is_pressed('page down'):
                        self._move_ee('left', 'x', -self.step_size)
                        self._move_ee('right', 'x', -self.step_size)
                        self._wait_key_release('page down')
                    
                    # Other controls
                    elif keyboard.is_pressed('t'):
                        self._toggle_arm()
                        self._wait_key_release('t')
                    elif keyboard.is_pressed('+') or keyboard.is_pressed('='):
                        self.step_size = min(0.1, self.step_size + 0.01)
                        self.get_logger().info(f'Step size: {self.step_size*100:.1f} cm')
                        self._wait_key_release('+')
                        self._wait_key_release('=')
                    elif keyboard.is_pressed('-'):
                        self.step_size = max(0.01, self.step_size - 0.01)
                        self.get_logger().info(f'Step size: {self.step_size*100:.1f} cm')
                        self._wait_key_release('-')
                    elif keyboard.is_pressed('r'):
                        self._reset_poses()
                        self._wait_key_release('r')
                    elif keyboard.is_pressed('esc'):
                        self._running = False
                        break
                    
                except Exception as e:
                    self.get_logger().error(f'Keyboard error: {e}')
        
        self._keyboard_thread = threading.Thread(target=keyboard_loop, daemon=True)
        self._keyboard_thread.start()
    
    def _wait_key_release(self, key):
        """Wait for key to be released to avoid rapid repeats"""
        import time
        while keyboard.is_pressed(key):
            time.sleep(0.05)
        time.sleep(0.1)  # Small delay after release
    
    def _toggle_arm(self):
        """Toggle active arm"""
        self.active_arm = 'right' if self.active_arm == 'left' else 'left'
        self.get_logger().info(f'Active arm: {self.active_arm}')
    
    def _reset_poses(self):
        """Reset both arms to default pose (arms-down position)"""
        # Use cached default poses (computed at startup)
        self.left_ee_pose.position = Point(
            x=float(self._default_left_pos[0]), 
            y=float(self._default_left_pos[1]), 
            z=float(self._default_left_pos[2])
        )
        self.left_ee_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.right_ee_pose.position = Point(
            x=float(self._default_right_pos[0]), 
            y=float(self._default_right_pos[1]), 
            z=float(self._default_right_pos[2])
        )
        self.right_ee_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        
        self._send_ik_goal('left', self.left_ee_pose)
        self._send_ik_goal('right', self.right_ee_pose)
        self.get_logger().info('Reset both arms to default pose (arms-down position)')
    
    def _move_ee(self, arm: str, axis: str, delta: float):
        """Move end effector along specified axis"""
        if arm == 'left':
            pose = self.left_ee_pose
        else:
            pose = self.right_ee_pose
        
        if axis == 'x':
            pose.position.x += delta
        elif axis == 'y':
            pose.position.y += delta
        elif axis == 'z':
            pose.position.z += delta
        
        # Update stored pose
        if arm == 'left':
            self.left_ee_pose = pose
        else:
            self.right_ee_pose = pose
        
        # Send IK goal
        self._send_ik_goal(arm, pose)
    
    def _send_ik_goal(self, arm: str, pose: Pose):
        """Send IK goal to action server"""
        goal_msg = ComputeIK.Goal()
        goal_msg.arm_name = arm
        goal_msg.target_pose = pose
        
        # Send goal asynchronously
        future = self._ik_action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._goal_response_callback(f, arm))
    
    def _goal_response_callback(self, future, arm: str):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{arm} arm IK goal rejected')
            return
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._goal_result_callback(f, arm))
    
    def _goal_result_callback(self, future, arm: str):
        """Handle goal result"""
        result = future.result().result
        if result.success:
            self.get_logger().debug(f'{arm} arm IK succeeded')
        else:
            self.get_logger().warn(f'{arm} arm IK failed: {result.message}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self._running = False
        if self._keyboard_thread:
            self._keyboard_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    if not KEYBOARD_AVAILABLE:
        print("\nERROR: keyboard library not available!")
        print("Install with: pip3 install keyboard")
        print("On Linux, you may need: sudo apt install python3-keyboard")
        print("\nNote: This script requires root/sudo privileges on Linux for keyboard input.")
        return
    
    rclpy.init(args=args)
    
    try:
        node = IKKeyboardController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
