"""
Kinematics utilities for the SO100 robot.
Contains forward and inverse kinematics solvers using PyBullet.
"""

import math
import numpy as np
import pybullet as p
from typing import Optional, Tuple, List
import logging
import json
import os
from pathlib import Path

from ..config import (
    NUM_JOINTS, NUM_IK_JOINTS, 
    USE_REFERENCE_POSES, REFERENCE_POSES_FILE, IK_POSITION_ERROR_THRESHOLD,
    IK_HYSTERESIS_THRESHOLD, IK_MOVEMENT_PENALTY_WEIGHT
)

logger = logging.getLogger(__name__)

class ForwardKinematics:
    """Forward kinematics solver using PyBullet."""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, end_effector_link_index: int):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
    
    def compute(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for given joint angles.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            
        Returns:
            Tuple of (position, quaternion) of end effector
        """
        if self.physics_client is None or self.robot_id is None:
            return np.array([0.2, 0.0, 0.15]), np.array([0, 0, 0, 1])
        
        # Use joint angles but keep gripper at neutral position for FK calculation
        # to ensure Wrist_Pitch_Roll position is independent of gripper state
        fk_state_angles = joint_angles_deg.copy()
        fk_state_angles[5] = 0.0  # Set gripper to neutral (closed) position for FK calculation
        
        # Set joint positions
        joint_angles_rad = np.deg2rad(fk_state_angles)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_angles_rad[i])
        
        # Get end effector position and orientation
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        position = np.array(link_state[0])
        quaternion = np.array(link_state[1])
        
        return position, quaternion


class IKSolver:
    """Inverse kinematics solver using PyBullet with multiple reference poses."""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, 
                 end_effector_link_index: int, joint_limits_min_deg: np.ndarray, 
                 joint_limits_max_deg: np.ndarray, arm_name: str = ""):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
        self.joint_limits_min_deg = joint_limits_min_deg
        self.joint_limits_max_deg = joint_limits_max_deg
        self.arm_name = arm_name
        
        # Precompute IK limits for first NUM_IK_JOINTS
        self.ik_lower_limits = np.deg2rad(joint_limits_min_deg[:NUM_IK_JOINTS])
        self.ik_upper_limits = np.deg2rad(joint_limits_max_deg[:NUM_IK_JOINTS])
        self.ik_ranges = self.ik_upper_limits - self.ik_lower_limits
        
        # Load reference poses
        self.reference_poses = self._load_reference_poses()
        
        # Create FK solver for evaluating solutions
        self.fk_solver = ForwardKinematics(physics_client, robot_id, joint_indices, end_effector_link_index)
    
    def _load_reference_poses(self) -> List[np.ndarray]:
        """Load reference poses from file for this arm."""
        reference_poses = []
        
        # Check if reference poses are enabled
        if not USE_REFERENCE_POSES:
            logger.info("Reference poses disabled in configuration")
            return reference_poses
        
        try:
            from ..utils import get_absolute_path
            cache_file = get_absolute_path(REFERENCE_POSES_FILE)
            if cache_file.exists():
                with open(cache_file, 'r') as f:
                    data = json.load(f)
                
                arm_poses = data.get(self.arm_name, [])
                if arm_poses:
                    # Convert to numpy arrays and extract only the first 3 joints for IK
                    for pose in arm_poses:
                        pose_array = np.array(pose[:NUM_IK_JOINTS])
                        pose_rad = np.deg2rad(pose_array)
                        reference_poses.append(pose_rad)
                    
                    logger.info(f"Loaded {len(reference_poses)} reference poses for {self.arm_name} arm")
                else:
                    logger.info(f"No reference poses found for {self.arm_name} arm")
            else:
                logger.info("No reference poses file found. Use read_pose.py to record reference poses.")
                
        except Exception as e:
            logger.warning(f"Failed to load reference poses: {e}")
        
        return reference_poses
    
    def _evaluate_ik_solution(self, solution: np.ndarray, target_position: np.ndarray, 
                             current_joints_rad: Optional[np.ndarray] = None, 
                             hysteresis_threshold: float = 0.05) -> float:
        """
        Evaluate the quality of an IK solution based on position error and joint movement.
        
        Args:
            solution: IK solution in radians
            target_position: Target end effector position
            current_joints_rad: Current joint angles in radians for movement penalty
            hysteresis_threshold: Minimum improvement needed to switch solutions (meters)
        """
        try:
            # Convert solution to full joint array (keep other joints at 0)
            full_angles = np.zeros(NUM_JOINTS)
            full_angles[:NUM_IK_JOINTS] = np.rad2deg(solution)
            
            # Compute forward kinematics
            achieved_position, _ = self.fk_solver.compute(full_angles)
            
            # Calculate position error
            position_error = np.linalg.norm(achieved_position - target_position)
            
            # Add joint movement penalty if current joints provided
            movement_penalty = 0.0
            if current_joints_rad is not None:
                # Calculate joint space distance (only for IK joints)
                joint_diff = solution - current_joints_rad[:NUM_IK_JOINTS]
                joint_movement = np.linalg.norm(joint_diff)
                
                # Convert joint movement to a position-equivalent penalty
                movement_penalty = joint_movement * IK_MOVEMENT_PENALTY_WEIGHT
                
            # Total cost combines position error and movement penalty
            total_cost = position_error + movement_penalty
            return total_cost
            
        except Exception as e:
            logger.warning(f"Error evaluating IK solution: {e}")
            return float('inf')
    
    def solve(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray], 
              current_angles_deg: np.ndarray) -> np.ndarray:
        """
        Solve inverse kinematics for position control using first 3 joints.
        Tries multiple reference poses and returns the best solution.
        
        Args:
            target_position: Target end effector position
            target_orientation_quat: Target orientation (optional, position-only if None)
            current_angles_deg: Current joint angles in degrees
            
        Returns:
            Joint angles for first NUM_IK_JOINTS in degrees
        """
        if self.physics_client is None or self.robot_id is None:
            return current_angles_deg[:NUM_IK_JOINTS]
        
        # Get current actual robot position and error
        current_actual_position, _ = self.fk_solver.compute(current_angles_deg)
        current_actual_error = np.linalg.norm(current_actual_position - target_position)
        
        # Convert current angles to radians and prepare for IK state
        # Keep gripper at neutral position to prevent gripper motion from affecting IK target
        ik_state_angles = current_angles_deg.copy()
        ik_state_angles[5] = 0.0  # Set gripper to neutral (closed) position for IK calculation
        current_angles_rad = np.deg2rad(ik_state_angles)
        
        # Helper functions for state management
        def set_robot_to_current_state():
            """Helper to set robot to exact current state"""
            for i in range(NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], current_angles_rad[i])
        
        def set_robot_to_reference_state(ref_pose_rad: np.ndarray):
            """Helper to set robot to reference pose state"""
            full_ref_state = current_angles_rad.copy()
            full_ref_state[:NUM_IK_JOINTS] = ref_pose_rad
            for i in range(NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], full_ref_state[i])
        
        # Prepare list of rest poses to try
        rest_poses_to_try = []
        
        # 1. Current configuration (most likely to be close to solution)
        current_rest_pose = np.deg2rad(current_angles_deg[:NUM_IK_JOINTS])
        rest_poses_to_try.append(('current', current_rest_pose))
        
        # 2. Reference poses from recorded configurations
        for i, ref_pose in enumerate(self.reference_poses):
            rest_poses_to_try.append((f'reference_{i+1}', ref_pose))
        
        best_solution = None
        best_error = float('inf')
        best_source = None
        current_solution_error = None
        current_solution_joints = None
        
        # Track best reference pose separately from overall best
        best_reference_solution = None
        best_reference_error = float('inf')
        best_reference_source = None
        best_reference_position_error = float('inf')  # Pure position error without movement penalty
        
        # Try each rest pose configuration
        for source_name, rest_pose in rest_poses_to_try:
            try:
                # Always start with a clean, known robot state before each IK attempt
                if source_name == 'current':
                    # For current pose, use exact current state
                    set_robot_to_current_state()
                else:
                    # For reference poses, set to that reference configuration
                    set_robot_to_reference_state(rest_pose)
                
                # Perform IK with the appropriate rest pose
                ik_solution = p.calculateInverseKinematics(
                    bodyUniqueId=self.robot_id,
                    endEffectorLinkIndex=self.end_effector_link_index,
                    targetPosition=target_position.tolist(),
                    lowerLimits=self.ik_lower_limits.tolist(),
                    upperLimits=self.ik_upper_limits.tolist(),
                    jointRanges=self.ik_ranges.tolist(),
                    restPoses=rest_pose.tolist() if isinstance(rest_pose, np.ndarray) else rest_pose,
                    solver=0,                                # 0 = DLS (Damped Least Squares)
                    maxNumIterations=100,
                    residualThreshold=1e-4
                )
                
                # CRITICAL: Always restore to exact current state after each IK attempt
                # This prevents state contamination between attempts
                set_robot_to_current_state()
                
                # Evaluate this solution
                solution_array = np.array(ik_solution[:NUM_IK_JOINTS])
                
                # PyBullet sometimes ignores joint limits, so clamp the solution
                # Convert limits back to degrees for comparison
                joint_limits_min_deg = np.rad2deg(self.ik_lower_limits)
                joint_limits_max_deg = np.rad2deg(self.ik_upper_limits)
                solution_degrees = np.rad2deg(solution_array)
                
                # Check and wrap shoulder_pan (first joint) if outside limits
                if solution_degrees[0] < joint_limits_min_deg[0] or solution_degrees[0] > joint_limits_max_deg[0]:
                    # Try wrapping by ±360°
                    for offset in [-360.0, 360.0]:
                        wrapped_angle = solution_degrees[0] + offset
                        if joint_limits_min_deg[0] <= wrapped_angle <= joint_limits_max_deg[0]:
                            solution_degrees[0] = wrapped_angle
                            break
                    else:
                        # If wrapping doesn't work, clamp it
                        clamped_angle = np.clip(solution_degrees[0], joint_limits_min_deg[0], joint_limits_max_deg[0])
                        solution_degrees[0] = clamped_angle
                
                # Clamp other joints normally
                solution_degrees[1:] = np.clip(solution_degrees[1:], joint_limits_min_deg[1:], joint_limits_max_deg[1:])
                
                # Convert back to radians for evaluation
                solution_array = np.deg2rad(solution_degrees)
                
                # For current pose, don't penalize movement (it's the baseline)
                if source_name == 'current':
                    error = self._evaluate_ik_solution(solution_array, target_position, None)
                    current_solution_error = error
                    current_solution_joints = solution_array.copy()
                else:
                    # Reference poses: calculate both position error and total error with movement penalty
                    position_only_error = self._evaluate_ik_solution(solution_array, target_position, None)
                    error = self._evaluate_ik_solution(solution_array, target_position, current_angles_rad)
                    
                    # Track best reference pose based on total error (for tie-breaking)
                    if error < best_reference_error:
                        best_reference_error = error
                        best_reference_solution = solution_array
                        best_reference_source = source_name
                        best_reference_position_error = position_only_error
                
                # Keep track of the overall best solution (for logging purposes)
                if error < best_error:
                    best_error = error
                    best_solution = solution_array
                    best_source = source_name
                
                # Only use early exit for extremely good current solutions
                if source_name == 'current' and error < 0.0001:  # 0.1mm - very strict threshold
                    break
                    
            except Exception as e:
                logger.debug(f"IK failed with {source_name} rest pose: {e}")
                # Always restore state even on error
                set_robot_to_current_state()
                continue
        
        # Final state restoration to ensure consistency
        set_robot_to_current_state()
        
        # Simple logic: Always prefer current pose unless reference pose is SIGNIFICANTLY better (>5cm)
        final_solution = current_solution_joints  # Default to current pose
        final_error = current_solution_error
        final_source = 'current'
        
        # Check if any reference pose is significantly better than current pose
        # Compare PURE POSITION ERRORS (no movement penalty) for the 5cm threshold
        if (best_reference_source is not None and current_actual_error is not None):
            position_improvement = current_actual_error - best_reference_position_error
            if position_improvement > IK_HYSTERESIS_THRESHOLD:  # 5cm improvement in position accuracy required
                logger.info(f"IK: Using {best_reference_source} (significant position improvement: {position_improvement:.4f}m > {IK_HYSTERESIS_THRESHOLD}m)")
                final_solution = best_reference_solution
                final_error = best_reference_error
                final_source = best_reference_source
        
        if final_solution is not None:
            final_angles = np.rad2deg(final_solution)
            return final_angles
        else:
            logger.warning("All IK attempts failed, returning current angles")
            return current_angles_deg[:NUM_IK_JOINTS]


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    Convert VR controller position to robot coordinate system.
    
    VR coordinate system: X=right, Y=up, Z=back (towards user)
    Robot coordinate system: X=forward, Y=left, Z=up
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR +Z (back) -> Robot +X (forward)
        vr_pos['z'] * scale,    # VR +X (right) -> Robot -Y (right) 
        vr_pos['y'] * scale     # VR +Y (up) -> Robot +Z (up)
    ])


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """Compute relative position from VR origin to current position."""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'], 
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr, scale) 