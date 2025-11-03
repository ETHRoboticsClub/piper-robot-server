"""
Simulation interface for Piper robot using PyBullet or other simulators.
Provides the same API as PiperSDKInterface but uses simulation instead of hardware.
"""

import logging
import time
from typing import Any, Dict, Optional

import numpy as np

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    p = None

from .geometry import transform2pose, xyzrpy2transform

logger = logging.getLogger(__name__)

JOINT_LIMITS_RAD = {
    "min": [-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439],
    "max": [2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439],
}
GRIPPER_ANGLE_MAX = 0.07  # 70mm


class SimulationInterface:
    """Base class for simulation interfaces."""
    
    def __init__(self, urdf_path: str, use_gui: bool = False):
        self.urdf_path = urdf_path
        self.use_gui = use_gui
        self.is_connected_flag = False
        
    def connect(self):
        raise NotImplementedError
        
    def disconnect(self):
        raise NotImplementedError
        
    def set_joint_positions(self, positions):
        raise NotImplementedError
        
    def get_status(self) -> Dict[str, Any]:
        raise NotImplementedError
        
    def get_end_effector_pose(self) -> Dict[str, float]:
        raise NotImplementedError
        
    def get_connection_status(self) -> bool:
        return self.is_connected_flag


class PyBulletSimInterface(SimulationInterface):
    """PyBullet-based simulation interface for Piper robot."""
    
    def __init__(self, urdf_path: str, use_gui: bool = False, robot_base_position: Optional[list] = None):
        super().__init__(urdf_path, use_gui)
        
        if not PYBULLET_AVAILABLE:
            raise ImportError("PyBullet is not installed. Install with: pip install pybullet")
        
        self.robot_id = None
        self.joint_indices = list(range(6))  # Joints 0-5
        self.gripper_index = 6
        self.ee_link_index = 6  # End effector link
        self.robot_base_position = robot_base_position or [0, 0, 0]
        
        # Current joint states
        self.current_joint_positions = [0.0] * 7
        
    def connect(self):
        """Initialize PyBullet simulation."""
        if self.is_connected_flag:
            logger.warning("Simulation already connected")
            return
            
        # Connect to PyBullet and store client id
        if self.use_gui:
            self.client_id = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=self.client_id)
        else:
            self.client_id = p.connect(p.DIRECT)

        # Set up simulation environment (use the specific client id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load ground plane into this client
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Load robot URDF into this client
        try:
            self.robot_id = p.loadURDF(
                self.urdf_path,
                basePosition=self.robot_base_position,
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION,
                physicsClientId=self.client_id,
            )
            logger.info(f"Loaded robot URDF from {self.urdf_path} (client {self.client_id})")
        except Exception as e:
            logger.error(f"Failed to load URDF: {e}")
            raise

        # Get number of joints and discover movable joint indices dynamically
        # getNumJoints does not take physicsClientId in all pybullet versions
        try:
            self.num_joints = p.getNumJoints(self.robot_id)
        except TypeError:
            self.num_joints = p.getNumJoints(self.robot_id)
        logger.info(f"Robot has {self.num_joints} joints")

        # Inspect joints and build name->index map
        movable = []
        joint_name_map = {}
        # Prepare allowed joint types (not all pybullet versions expose JOINT_CONTINUOUS)
        allowed_types = [getattr(p, 'JOINT_REVOLUTE', 0), getattr(p, 'JOINT_PRISMATIC', 1)]
        if hasattr(p, 'JOINT_CONTINUOUS'):
            allowed_types.append(p.JOINT_CONTINUOUS)

        for i in range(self.num_joints):
            # Some pybullet versions accept physicsClientId, others ignore it; handle both
            try:
                joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            except TypeError:
                joint_info = p.getJointInfo(self.robot_id, i)

            jname = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
            jtype = joint_info[2]
            joint_name_map[jname] = i
            logger.debug(f"Joint {i}: {jname} (type: {jtype})")
            # Consider revolute, prismatic and continuous as actuated
            if jtype in allowed_types:
                movable.append((i, jname))

        # Heuristic: first six movable joints are arm joints, gripper is the one containing 'grip' or last movable
        movable_indices = [i for i, _ in movable]
        if len(movable_indices) >= 6:
            # Support dual-arm URDFs: if >=12 movable joints, split into left and right
            if len(movable_indices) >= 12:
                # Classify joints by name pattern: left=joint{N}, right=arm2_joint{N}
                left_arm_joints = []
                right_arm_joints = []
                left_grippers = []
                right_grippers = []
                
                for idx, name in movable:
                    lname = name.lower()
                    # Detect right arm joints (arm2_joint pattern)
                    if 'arm2' in lname:
                        if 'grip' in lname or 'gripper' in lname:
                            right_grippers.append((idx, name))
                        else:
                            right_arm_joints.append((idx, name))
                    # Detect left arm joints (joint pattern without arm2)
                    elif 'joint' in lname:
                        if 'grip' in lname or 'gripper' in lname:
                            left_grippers.append((idx, name))
                        else:
                            left_arm_joints.append((idx, name))
                
                # Sort by index to preserve order, then extract just indices
                left_arm_joints.sort(key=lambda x: x[0])
                right_arm_joints.sort(key=lambda x: x[0])
                left_grippers.sort(key=lambda x: x[0])
                right_grippers.sort(key=lambda x: x[0])
                
                # First 6 are the main arm joints, remaining are gripper joints
                self.left_joint_indices = [idx for idx, _ in left_arm_joints[:6]]
                self.right_joint_indices = [idx for idx, _ in right_arm_joints[:6]]
                
                logger.info(f"Detected left arm joints: {[name for _, name in left_arm_joints[:6]]}")
                logger.info(f"Detected right arm joints: {[name for _, name in right_arm_joints[:6]]}")
                
                # Fallback if detection failed
                if len(self.left_joint_indices) < 6 or len(self.right_joint_indices) < 6:
                    logger.warning("Name-based detection incomplete, falling back to index split")
                    self.left_joint_indices = movable_indices[:6]
                    self.right_joint_indices = movable_indices[6:12]
                
                # Set grippers: use joint7 (index 6 in left_arm_joints list means 7th joint)
                # The gripper is typically the 7th joint after the 6 arm joints
                left_gripper = None
                right_gripper = None
                
                # Look for explicit gripper joints or use joints after the first 6
                if left_grippers:
                    left_gripper = left_grippers[0][0]
                else:
                    # Find joint7 for left arm (first joint after the 6 arm joints)
                    for idx, name in left_arm_joints[6:]:
                        if 'joint7' in name.lower():
                            left_gripper = idx
                            break
                    if left_gripper is None and len(left_arm_joints) > 6:
                        left_gripper = left_arm_joints[6][0]
                
                if right_grippers:
                    right_gripper = right_grippers[0][0]
                else:
                    # Find arm2_joint7 for right arm
                    for idx, name in right_arm_joints[6:]:
                        if 'joint7' in name.lower():
                            right_gripper = idx
                            break
                    if right_gripper is None and len(right_arm_joints) > 6:
                        right_gripper = right_arm_joints[6][0]
                
                # Final fallback
                if left_gripper is None:
                    left_gripper = self.left_joint_indices[-1] if self.left_joint_indices else movable_indices[5]
                if right_gripper is None:
                    right_gripper = self.right_joint_indices[-1] if self.right_joint_indices else movable_indices[11]
                
                self.joint_indices = self.left_joint_indices
                self.gripper_index = left_gripper
                self.right_gripper_index = right_gripper
                
                # Log gripper info with joint names for debugging
                left_gripper_name = next((name for idx, name in movable if idx == left_gripper), "unknown")
                right_gripper_name = next((name for idx, name in movable if idx == right_gripper), "unknown")
                logger.info(f"Left gripper: index {left_gripper} ({left_gripper_name}), Right gripper: index {right_gripper} ({right_gripper_name})")
            else:
                self.joint_indices = movable_indices[:6]
                # Try to find gripper by name
                gripper_idx = None
                for i, name in movable:
                    if 'grip' in name.lower() or 'gripper' in name.lower():
                        gripper_idx = i
                        break
                if gripper_idx is None and len(movable_indices) > 6:
                    gripper_idx = movable_indices[6]
                self.gripper_index = gripper_idx if gripper_idx is not None else movable_indices[-1]
        else:
            # Fallback to defaults
            logger.warning("Could not detect 6 movable joints, using default indices")
            self.joint_indices = list(range(min(6, self.num_joints)))
            self.gripper_index = min(6, self.num_joints - 1)

        # Reset discovered joints to zero
        if hasattr(self, 'left_joint_indices') and hasattr(self, 'right_joint_indices'):
            for i in self.left_joint_indices + self.right_joint_indices:
                p.resetJointState(self.robot_id, i, 0.0, physicsClientId=self.client_id)
            p.resetJointState(self.robot_id, self.gripper_index, 0.0, physicsClientId=self.client_id)
            try:
                p.resetJointState(self.robot_id, self.right_gripper_index, 0.0, physicsClientId=self.client_id)
            except Exception:
                pass
        else:
            for i in self.joint_indices:
                p.resetJointState(self.robot_id, i, 0.0, physicsClientId=self.client_id)
            p.resetJointState(self.robot_id, self.gripper_index, 0.0, physicsClientId=self.client_id)

        # Enable force/torque sensors if needed
        for i in self.joint_indices:
            try:
                p.enableJointForceTorqueSensor(self.robot_id, i, True, physicsClientId=self.client_id)
            except Exception:
                # Not critical
                pass

        self.is_connected_flag = True
        logger.info("PyBullet simulation connected successfully")
        
    def disconnect(self):
        """Disconnect from PyBullet."""
        if self.client_id is not None:
            p.disconnect(self.client_id)
            self.is_connected_flag = False
            logger.info("PyBullet simulation disconnected")
            
    def set_joint_positions(self, positions):
        """Set target joint positions in simulation.
        
        Args:
            positions: List of 7 values [joint0, joint1, ..., joint5, gripper]
        """
        if not self.is_connected_flag:
            logger.warning("Cannot set positions: simulation not connected")
            return
            
        # Clip joint positions to limits
        clipped_positions = []
        for i, pos in enumerate(positions[:6]):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(pos, min_rad), max_rad)
            clipped_positions.append(clipped_pos)
            
        # Clip gripper position
        gripper_pos = min(max(positions[6], 0.0), GRIPPER_ANGLE_MAX)
        clipped_positions.append(gripper_pos)
        
        # Set joint positions using position control
        logger.debug(f"PyBullet client {self.client_id} - setting joints {self.joint_indices[:6]} gripper {self.gripper_index} -> {clipped_positions}")
        for i, pos in enumerate(clipped_positions[:6]):
            p.setJointMotorControl2(
                self.robot_id,
                self.joint_indices[i],
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=100.0,
                maxVelocity=2.0,
                physicsClientId=self.client_id,
            )
            
        # Set gripper positions: split the desired width between joint7 and joint8
        # Each gripper joint moves by half of the desired gripper width
        half_gripper_pos = clipped_positions[6] / 2.0
        
        # Set joint7 (primary gripper joint)
        p.setJointMotorControl2(
            self.robot_id,
            self.gripper_index,
            p.POSITION_CONTROL,
            targetPosition=half_gripper_pos,
            force=10.0,
            maxVelocity=0.5,
            physicsClientId=self.client_id,
        )
        
        # Set joint8 (secondary gripper joint) if it exists
        gripper_joint8 = self.gripper_index + 1
        if gripper_joint8 < self.num_joints:
            p.setJointMotorControl2(
                self.robot_id,
                gripper_joint8,
                p.POSITION_CONTROL,
                targetPosition=-half_gripper_pos,
                force=10.0,
                maxVelocity=0.5,
                physicsClientId=self.client_id,
            )
        # Step simulation for this client
        try:
            p.stepSimulation(physicsClientId=self.client_id)
        except TypeError:
            # Older pybullet versions may not accept physicsClientId in stepSimulation
            p.stepSimulation()

        # Store current positions
        self.current_joint_positions = clipped_positions

    def set_arm_positions(self, arm_id: str, positions):
        """Set joint positions for a specific arm in a dual-URDF robot.

        arm_id: expected to contain 'left' or 'right' (e.g., 'left_follower')
        positions: list of 7 values for that arm
        """
        if not self.is_connected_flag:
            logger.warning("Cannot set arm positions: simulation not connected")
            return

        clipped_positions = []
        for i, pos in enumerate(positions[:6]):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(pos, min_rad), max_rad)
            clipped_positions.append(clipped_pos)

        gripper_pos = min(max(positions[6], 0.0), GRIPPER_ANGLE_MAX)
        clipped_positions.append(gripper_pos)

        # Determine which indices to use
        if 'right' in arm_id.lower() and hasattr(self, 'right_joint_indices'):
            target_indices = self.right_joint_indices
            gripper_idx = getattr(self, 'right_gripper_index', self.gripper_index)
        else:
            target_indices = getattr(self, 'left_joint_indices', self.joint_indices)
            gripper_idx = self.gripper_index

        logger.debug(f"PyBullet client {self.client_id} - set_arm_positions {arm_id} indices {target_indices} gripper {gripper_idx} -> {clipped_positions}")

        for idx, pos in enumerate(clipped_positions[:6]):
            p.setJointMotorControl2(
                self.robot_id,
                target_indices[idx],
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=100.0,
                maxVelocity=2.0,
                physicsClientId=self.client_id,
            )

        # Set gripper positions: split the desired width between joint7 and joint8
        # Each gripper joint moves by half of the desired gripper width
        half_gripper_pos = clipped_positions[6] / 2.0
        
        # Set joint7 (primary gripper joint)
        p.setJointMotorControl2(
            self.robot_id,
            gripper_idx,
            p.POSITION_CONTROL,
            targetPosition=half_gripper_pos,
            force=10.0,
            maxVelocity=0.5,
            physicsClientId=self.client_id,
        )
        
        # Set joint8 (secondary gripper joint) if it exists
        gripper_joint8 = gripper_idx + 1
        if gripper_joint8 < self.num_joints:
            p.setJointMotorControl2(
                self.robot_id,
                gripper_joint8,
                p.POSITION_CONTROL,
                targetPosition=-half_gripper_pos,
                force=10.0,
                maxVelocity=0.5,
                physicsClientId=self.client_id,
            )

        try:
            p.stepSimulation(physicsClientId=self.client_id)
        except TypeError:
            p.stepSimulation()

    def get_arm_status(self, arm_id: str) -> Dict[str, Any]:
        """Return joint state dictionary for specific arm."""
        if not self.is_connected_flag:
            return {f"joint_{i}.pos": 0.0 for i in range(7)}

        obs = {}
        if 'right' in arm_id.lower() and hasattr(self, 'right_joint_indices'):
            indices = self.right_joint_indices
            gripper_idx = getattr(self, 'right_gripper_index', self.gripper_index)
        else:
            indices = getattr(self, 'left_joint_indices', self.joint_indices)
            gripper_idx = self.gripper_index

        for idx, joint_idx in enumerate(indices[:6]):
            joint_state = p.getJointState(self.robot_id, joint_idx, physicsClientId=self.client_id)
            obs[f"joint_{idx}.pos"] = joint_state[0]

        gripper_state = p.getJointState(self.robot_id, gripper_idx, physicsClientId=self.client_id)
        obs["joint_6.pos"] = gripper_state[0]

        return obs

    def get_arm_end_effector_pose(self, arm_id: str) -> Dict[str, float]:
        """Return EE pose for specific arm. Uses heuristic link selection."""
        if not self.is_connected_flag:
            return {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}

        # Heuristic: try to find link named containing 'ee' or 'ee_link' for left/right
        ee_link = None
        for i in range(self.num_joints):
            try:
                info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            except TypeError:
                info = p.getJointInfo(self.robot_id, i)
            name = info[12].decode('utf-8') if isinstance(info[12], bytes) else str(info[12])
            lname = name.lower()
            if 'ee' in lname or 'ee_link' in lname:
                if 'right' in arm_id.lower() and ('arm2' in lname or 'right' in lname):
                    ee_link = i
                    break
                if 'left' in arm_id.lower() and ('arm2' not in lname and 'right' not in lname):
                    ee_link = i
                    break

        if ee_link is None:
            # fallback: use last joint of the respective arm
            if 'right' in arm_id.lower() and hasattr(self, 'right_joint_indices'):
                ee_link = self.right_joint_indices[-1]
            else:
                ee_link = self.left_joint_indices[-1] if hasattr(self, 'left_joint_indices') else self.joint_indices[-1]

        link_state = p.getLinkState(self.robot_id, ee_link, physicsClientId=self.client_id)
        position = link_state[0]
        orientation = link_state[1]
        euler = p.getEulerFromQuaternion(orientation)
        return {"x": position[0], "y": position[1], "z": position[2], "roll": euler[0], "pitch": euler[1], "yaw": euler[2]}
        
    def get_status(self) -> Dict[str, Any]:
        """Get current joint states from simulation."""
        if not self.is_connected_flag:
            return {f"joint_{i}.pos": 0.0 for i in range(7)}
            
        obs_dict = {}
        
        # Get joint positions
        for idx, joint_idx in enumerate(self.joint_indices[:6]):
            joint_state = p.getJointState(self.robot_id, joint_idx, physicsClientId=self.client_id)
            obs_dict[f"joint_{idx}.pos"] = joint_state[0]  # position
            
        # Get gripper position
        gripper_state = p.getJointState(self.robot_id, self.gripper_index, physicsClientId=self.client_id)
        obs_dict["joint_6.pos"] = gripper_state[0]

        return obs_dict
        
    def get_end_effector_pose(self) -> Dict[str, float]:
        """Get end effector pose from simulation."""
        if not self.is_connected_flag:
            return {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
            
        # Get link state for end effector
        link_state = p.getLinkState(self.robot_id, self.ee_link_index, physicsClientId=self.client_id)
        position = link_state[0]  # World position
        orientation = link_state[1]  # World orientation (quaternion)
        
        # Convert quaternion to euler angles (roll, pitch, yaw)
        euler = p.getEulerFromQuaternion(orientation)
        
        return {
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "roll": euler[0],
            "pitch": euler[1],
            "yaw": euler[2]
        }
        
    def get_connection_status(self) -> bool:
        """Check if simulation is connected."""
        return self.is_connected_flag


class NewtonSimInterface(SimulationInterface):
    """
    Placeholder for Newton simulation interface.
    Implement this class when you have Newton simulator available.
    """
    
    def __init__(self, urdf_path: str, use_gui: bool = False):
        super().__init__(urdf_path, use_gui)
        logger.info("Newton simulation interface initialized (not implemented yet)")
        
    def connect(self):
        logger.warning("Newton simulation not yet implemented")
        self.is_connected_flag = True
        # TODO: Initialize Newton simulator
        
    def disconnect(self):
        self.is_connected_flag = False
        # TODO: Disconnect from Newton
        
    def set_joint_positions(self, positions):
        # TODO: Send positions to Newton
        pass
        
    def get_status(self) -> Dict[str, Any]:
        # TODO: Get state from Newton
        return {f"joint_{i}.pos": 0.0 for i in range(7)}
        
    def get_end_effector_pose(self) -> Dict[str, float]:
        # TODO: Get EE pose from Newton
        return {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
