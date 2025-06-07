"""
Kinematics utilities for the SO100 robot.
Contains forward and inverse kinematics solvers using PyBullet.
"""

import math
import numpy as np
import pybullet as p
from typing import Optional, Tuple
import logging

from ..config import NUM_JOINTS, NUM_IK_JOINTS, ELBOW_LOWER_MARGIN_DEG

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
    """Inverse kinematics solver using PyBullet."""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, 
                 end_effector_link_index: int, joint_limits_min_deg: np.ndarray, 
                 joint_limits_max_deg: np.ndarray):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
        self.joint_limits_min_deg = joint_limits_min_deg
        self.joint_limits_max_deg = joint_limits_max_deg
        
        # Precompute IK limits for first NUM_IK_JOINTS with safety margins
        raw_lower_limits = np.deg2rad(joint_limits_min_deg[:NUM_IK_JOINTS])
        raw_upper_limits = np.deg2rad(joint_limits_max_deg[:NUM_IK_JOINTS])
        
        # Apply elbow margin to prevent hyperextension (joint index 2 only)
        self.ik_lower_limits = raw_lower_limits.copy()
        self.ik_upper_limits = raw_upper_limits.copy()
        
        # Apply elbow lower margin only
        elbow_idx = 2  # elbow_flex index
        self.ik_lower_limits[elbow_idx] = raw_lower_limits[elbow_idx] + math.radians(ELBOW_LOWER_MARGIN_DEG)
        
        # Log the elbow constraint
        original_lower = math.degrees(raw_lower_limits[elbow_idx])
        constrained_lower = math.degrees(self.ik_lower_limits[elbow_idx])
        logger.info(f"IK elbow_flex: lower limit {original_lower:.1f}° -> {constrained_lower:.1f}° (margin: +{ELBOW_LOWER_MARGIN_DEG}°)")
        
        self.ik_ranges = self.ik_upper_limits - self.ik_lower_limits
        
        # Rest poses for natural 3-DOF arm configuration
        self.rest_poses = [0.0] * NUM_IK_JOINTS
        self.rest_poses[0] = 0.0        # shoulder_pan neutral
        self.rest_poses[1] = math.pi/2  # shoulder_lift pointing forward
        self.rest_poses[2] = math.pi/4  # elbow_flex slightly bent
    
    def solve(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray], 
              current_angles_deg: np.ndarray) -> np.ndarray:
        """
        Solve inverse kinematics for position control using first 3 joints.
        
        Args:
            target_position: Target end effector position
            target_orientation_quat: Target orientation (optional, position-only if None)
            current_angles_deg: Current joint angles in degrees
            
        Returns:
            Joint angles for first NUM_IK_JOINTS in degrees
        """
        if self.physics_client is None or self.robot_id is None:
            return current_angles_deg[:NUM_IK_JOINTS]
        
        # Use current angles to set PyBullet state, but keep gripper at neutral position
        # to prevent gripper motion from affecting the Wrist_Pitch_Roll IK target
        ik_state_angles = current_angles_deg.copy()
        ik_state_angles[5] = 0.0  # Set gripper to neutral (closed) position for IK calculation
        
        current_angles_rad = np.deg2rad(ik_state_angles)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], current_angles_rad[i])
        
        try:
            # Position-only IK (no orientation control)
            ik_solution = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_link_index,
                targetPosition=target_position.tolist(),
                lowerLimits=self.ik_lower_limits.tolist(),
                upperLimits=self.ik_upper_limits.tolist(),
                jointRanges=self.ik_ranges.tolist(),
                restPoses=self.rest_poses,
                solver=0,                                # 0 = DLS (Damped Least Squares)
                maxNumIterations=100,
                residualThreshold=1e-4
            )
            
            return np.rad2deg(ik_solution[:NUM_IK_JOINTS])
            
        except Exception as e:
            logger.warning(f"IK failed: {e}")
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