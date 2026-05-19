"""
Humanoid Kinematics Module using Pink IK Library
Single IK solver for the entire humanoid (both arms + head)
"""

import numpy as np
import pink
import pinocchio as pin
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from typing import Optional, Dict


class HumanoidKinematics:
    """
    Humanoid kinematics solver for the robot.
    Uses a single IK solver to handle both arms and head simultaneously.
    """

    def __init__(
        self,
        urdf_path: str,
        end_effector_links: Dict[str, str],  # e.g., {'left': 'left_end_effector', 'right': 'right_end_effector'}
        joint_names: Dict[str, list[str]],  # e.g., {'left': [...], 'right': [...], 'head': [...]}
        initial_joint_positions: Dict[str, np.ndarray],
        position_cost: float = 20.0,
        orientation_cost: float = 1.0,
        regularization_cost: float = 0.0001,
        robot_joint_fq: float = 50.0,
        solver: str = "quadprog",
        position_error_tolerance: float = 0.01,
        learning_rate: float = 0.1,
        debug_mode: bool = False,
    ):
        """
        Initialize the HumanoidKinematics class.

        Args:
            urdf_path: Path to the humanoid URDF file.
            end_effector_links: Dictionary mapping limb names to end effector link names.
            joint_names: Dictionary mapping limb names to joint name lists.
            initial_joint_positions: Dictionary mapping limb names to initial joint positions.
            position_cost: Cost for position error in IK.
            orientation_cost: Cost for orientation error in IK.
            regularization_cost: Cost for regularization in IK.
            robot_joint_fq: Robot joint command frequency in Hz.
            solver: QP solver to use ('quadprog', 'osqp', etc.).
            position_error_tolerance: Tolerance for position error in IK.
            learning_rate: Learning rate for the IK solver.
            debug_mode: Enable debug printing.
        """
        self._urdf_path = urdf_path
        self._end_effector_links = end_effector_links
        self._joint_names = joint_names
        self._position_error_tolerance = position_error_tolerance
        self._solver = solver
        self._dt = learning_rate
        self.robot_joint_dt = 1.0 / robot_joint_fq
        self._debug_mode = debug_mode

        # Load the robot model from the URDF file
        # Use full model - fixed joints will have 0 DOF and won't affect IK
        self._robot_model = pin.buildModelFromUrdf(self._urdf_path)
        self._data = self._robot_model.createData()
        
        if self._debug_mode:
            print(f"[HumanoidKinematics] Loaded model: nq={self._robot_model.nq}, nv={self._robot_model.nv}, njoints={self._robot_model.njoints}")

        # Create frame tasks for each end effector
        self._frame_tasks = {}
        for limb, ee_link in end_effector_links.items():
            self._frame_tasks[limb] = FrameTask(
                ee_link,
                position_cost=position_cost,
                orientation_cost=orientation_cost,
                lm_damping=1.0,
            )

        # Joint Angles Regularization loss
        self._posture_task = PostureTask(cost=regularization_cost)

        # Build joint mapping with proper handling of continuous joints
        # Store both q-space indices (idx_q, nq) and v-space indices (idx_v, nv)
        self._q_neutral = pin.neutral(self._robot_model)
        self._joint_info = {}  # jname -> dict(jid, idx_q, nq, idx_v, nv)
        self._limb_to_joint_names = {}
        
        for limb, jnames in joint_names.items():
            self._limb_to_joint_names[limb] = list(jnames)
            for jname in jnames:
                if not self._robot_model.existJointName(jname):
                    raise ValueError(f"Joint {jname} not found in URDF")
                
                jid = self._robot_model.getJointId(jname)
                self._joint_info[jname] = {
                    "jid": jid,
                    "idx_q": int(self._robot_model.idx_qs[jid]),
                    "nq": int(self._robot_model.nqs[jid]),
                    "idx_v": int(self._robot_model.idx_vs[jid]),
                    "nv": int(self._robot_model.nvs[jid]),
                }
                if self._joint_info[jname]["nv"] != 1:
                    raise ValueError(f"Joint {jname} has nv={self._joint_info[jname]['nv']} (expected 1 DOF)")

        # Build initial tangent vector (velocity space, size nv)
        v_init = np.zeros(self._robot_model.nv)
        for limb, joint_pos in initial_joint_positions.items():
            for i, jname in enumerate(self._limb_to_joint_names[limb]):
                if i < len(joint_pos):
                    v_init[self._joint_info[jname]["idx_v"]] = float(joint_pos[i])

        # Clamp only bounded 1D joints (regular revolute with finite limits)
        for jname, info in self._joint_info.items():
            if info["nq"] == 1:  # Only for regular revolute joints
                q_idx = info["idx_q"]
                v_idx = info["idx_v"]
                lo = self._robot_model.lowerPositionLimit[q_idx]
                hi = self._robot_model.upperPositionLimit[q_idx]
                if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
                    v_init[v_idx] = float(np.clip(v_init[v_idx], lo, hi))

        # Convert tangent vector to configuration on the manifold
        q_init = pin.integrate(self._robot_model, self._q_neutral, v_init)
        
        # Normalize configuration (important for continuous joints)
        try:
            pin.normalize(self._robot_model, q_init)
        except Exception:
            pass
        
        self._configuration = pink.Configuration(self._robot_model, self._data, q_init)
        
        # Set posture task target to initial configuration (for regularization)
        self._posture_task.set_target(q_init)
        
        # Store initial v for reference
        self._v_init = v_init
        
        # Debug: Print all joints and initial end effector positions
        if self._debug_mode:
            print(f"[HumanoidKinematics] Robot model info:")
            print(f"  nq={self._robot_model.nq}, nv={self._robot_model.nv}, njoints={self._robot_model.njoints}")
            print(f"[HumanoidKinematics] Initial configuration (nq={self._robot_model.nq}):")
            print(f"  q_init: {q_init}")
            print(f"  v_init: {v_init}")
            for limb, jnames in self._limb_to_joint_names.items():
                v_values = [v_init[self._joint_info[j]["idx_v"]] for j in jnames]
                print(f"  {limb} joint names: {jnames}")
                print(f"  {limb} joint angles (v): {v_values}")
            
            # Compute forward kinematics to show initial end effector positions
            pin.forwardKinematics(self._robot_model, self._data, self._configuration.q)
            pin.updateFramePlacements(self._robot_model, self._data)
            print(f"[HumanoidKinematics] Initial end effector positions:")
            for limb, ee_link in end_effector_links.items():
                ee_transform = self._configuration.get_transform_frame_to_world(ee_link)
                print(f"  {limb}: pos={ee_transform.translation}")

    def compute_ik(
        self, 
        target_poses: Dict[str, pin.SE3],  # e.g., {'left': SE3(...), 'right': SE3(...)}
        iterations: int = 50
    ) -> Dict[str, np.ndarray]:
        """
        Compute inverse kinematics for multiple end effectors simultaneously.

        Args:
            target_poses: Dictionary mapping limb names to target SE3 poses.
            iterations: Number of IK iterations.

        Returns:
            Dictionary mapping limb names to joint position arrays.
        """
        # Validate target poses
        if self._debug_mode:
            for limb, target_se3 in target_poses.items():
                pos = target_se3.translation
                if np.any(np.isnan(pos)) or np.any(np.isinf(pos)):
                    print(f"[HumanoidKinematics] ERROR: Invalid target pose for {limb}: {pos}")
                    raise ValueError(f"Invalid target pose for {limb}: contains NaN or Inf")
        
        # Set target poses for active frame tasks
        active_tasks = []
        for limb, target_se3 in target_poses.items():
            if limb in self._frame_tasks:
                task = self._frame_tasks[limb]
                task.set_target(target_se3)
                active_tasks.append(task)
                if self._debug_mode:
                    print(f"[HumanoidKinematics] Target for {limb}: pos={target_se3.translation}")
        
        # Add posture task for regularization (now that continuous joints are fixed)
        tasks = active_tasks + [self._posture_task]
        
        # Solve IK with joint limit barriers
        last_valid_q = self._configuration.q.copy()
        for iter_num in range(iterations):
            try:
                velocity = solve_ik(
                    self._configuration,
                    tasks,
                    self._dt,
                    solver=self._solver
                )
                
                # Check for NaN in velocity before integration
                if np.any(np.isnan(velocity)) or np.any(np.isinf(velocity)):
                    if self._debug_mode:
                        print(f"[HumanoidKinematics] WARNING: NaN/Inf in velocity at iteration {iter_num}, using last valid config")
                    # Use last valid configuration and break
                    self._configuration = pink.Configuration(self._robot_model, self._data, last_valid_q)
                    break
                
                self._configuration.integrate_inplace(velocity, self._dt)
                
                # Clamp only bounded revolute joints (nq==1 with finite limits)
                q = self._configuration.q.copy()
                for info in self._joint_info.values():
                    if info["nq"] == 1:  # Only for regular revolute joints
                        q_idx = info["idx_q"]
                        lo = self._robot_model.lowerPositionLimit[q_idx]
                        hi = self._robot_model.upperPositionLimit[q_idx]
                        if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
                            q[q_idx] = np.clip(q[q_idx], lo, hi)
                
                # Re-normalize to keep continuous joints valid
                try:
                    pin.normalize(self._robot_model, q)
                except Exception:
                    pass
                
                self._configuration = pink.Configuration(self._robot_model, self._data, q)
                
                # Check for NaN in configuration
                if np.any(np.isnan(self._configuration.q)) or np.any(np.isinf(self._configuration.q)):
                    if self._debug_mode:
                        print(f"[HumanoidKinematics] WARNING: NaN/Inf detected in configuration at iteration {iter_num}, using last valid config")
                    # Restore last valid configuration and break
                    self._configuration = pink.Configuration(self._robot_model, self._data, last_valid_q)
                    break
                
                # Save valid configuration
                last_valid_q = self._configuration.q.copy()
                
            except Exception as e:
                if self._debug_mode:
                    print(f"[HumanoidKinematics] WARNING: Exception during IK iteration {iter_num}: {e}, using last valid config")
                # Restore last valid configuration and break
                self._configuration = pink.Configuration(self._robot_model, self._data, last_valid_q)
                break
        
        # Extract joint angles (NOT q coordinates!) using pin.difference
        # This correctly handles continuous joints by returning the tangent vector
        v = pin.difference(self._robot_model, self._q_neutral, self._configuration.q)
        
        result = {}
        for limb, jnames in self._limb_to_joint_names.items():
            result[limb] = np.array([v[self._joint_info[j]["idx_v"]] for j in jnames], dtype=float)
        
        # Compute errors for logging
        if self._debug_mode:
            for limb in target_poses.keys():
                if limb in self._frame_tasks:
                    current_transform = self._configuration.get_transform_frame_to_world(
                        self._end_effector_links[limb]
                    )
                    pos_error = np.linalg.norm(
                        target_poses[limb].translation - current_transform.translation
                    )
                    print(f"[{limb}] IK position error: {pos_error:.4f}m")
        
        return result

    def get_current_poses(self) -> Dict[str, pin.SE3]:
        """
        Get current poses of all end effectors.

        Returns:
            Dictionary mapping limb names to current SE3 poses.
        """
        poses = {}
        for limb, ee_link in self._end_effector_links.items():
            poses[limb] = self._configuration.get_transform_frame_to_world(ee_link)
        return poses

    def set_configuration(self, joint_positions: Dict[str, np.ndarray]):
        """
        Set the robot configuration from joint positions (angles in tangent space).

        Args:
            joint_positions: Dictionary mapping limb names to joint position arrays (angles).
        """
        # Build tangent vector from joint angles
        v = np.zeros(self._robot_model.nv)
        for limb, joint_pos in joint_positions.items():
            if limb in self._limb_to_joint_names:
                for i, jname in enumerate(self._limb_to_joint_names[limb]):
                    if i < len(joint_pos):
                        v[self._joint_info[jname]["idx_v"]] = float(joint_pos[i])
        
        # Clamp only bounded revolute joints
        for jname, info in self._joint_info.items():
            if info["nq"] == 1:
                q_idx = info["idx_q"]
                v_idx = info["idx_v"]
                lo = self._robot_model.lowerPositionLimit[q_idx]
                hi = self._robot_model.upperPositionLimit[q_idx]
                if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
                    v[v_idx] = float(np.clip(v[v_idx], lo, hi))
        
        # Convert to configuration on manifold
        q = pin.integrate(self._robot_model, self._q_neutral, v)
        try:
            pin.normalize(self._robot_model, q)
        except Exception:
            pass
        
        self._configuration = pink.Configuration(self._robot_model, self._data, q)
