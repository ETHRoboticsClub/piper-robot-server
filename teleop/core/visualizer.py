"""
PyBullet visualization module for the SO100 robot.
Handles 3D visualization, markers, and coordinate frames.
"""

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from typing import Dict, List, Optional, Tuple
import logging
from scipy.spatial.transform import Rotation as R

from ..config import (
    JOINT_NAMES, NUM_JOINTS, URDF_TO_INTERNAL_NAME_MAP, 
    END_EFFECTOR_LINK_NAME
)

logger = logging.getLogger(__name__)


class PyBulletVisualizer:
    """PyBullet visualization for robot teleoperation."""
    
    def __init__(self, urdf_path: str, use_gui: bool = True):
        self.urdf_path = urdf_path
        self.use_gui = use_gui
        
        # PyBullet state
        self.physics_client = None
        self.robot_ids = {'left': None, 'right': None}  # Two robot instances
        self.joint_indices = {'left': [None] * NUM_JOINTS, 'right': [None] * NUM_JOINTS}  # Joint indices for both arms
        self.end_effector_link_indices = {'left': -1, 'right': -1}  # End effector links for both arms
        
        # Visualization markers
        self.viz_markers = {}
        self.debug_line_ids = {}
        
        # Joint limits
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        self.is_connected = False
    
    def setup(self) -> bool:
        """Initialize PyBullet and load the robot."""
        try:
            if self.use_gui:
                self.physics_client = p.connect(p.GUI)
            else:
                self.physics_client = p.connect(p.DIRECT)
        except p.error as e:
            logger.warning(f"Could not connect to PyBullet: {e}")
            if self.use_gui:
                try:
                    self.physics_client = p.connect(p.DIRECT)
                    logger.info("Fallback to DIRECT mode")
                except p.error:
                    logger.error("Failed to connect to PyBullet")
                    return False
            else:
                return False
        
        if self.physics_client < 0:
            return False
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        # Load robot URDF
        if not os.path.exists(self.urdf_path):
            logger.error(f"URDF file not found: {self.urdf_path}")
            return False
        
        try:
            self.robot_ids['left'] = p.loadURDF(self.urdf_path, [0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
        except p.error as e:
            logger.error(f"Failed to load URDF: {e}")
            return False
        
        # Load right robot 40cm away in X direction
        try:
            self.robot_ids['right'] = p.loadURDF(self.urdf_path, [-0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
        except p.error as e:
            logger.error(f"Failed to load right robot URDF: {e}")
            return False
        
        # Map joint names to PyBullet indices
        if not self._map_joints():
            return False
        
        # Find end effector link
        if not self._find_end_effector():
            return False
        
        # Read joint limits
        self._read_joint_limits()
        
        # Create visualization markers
        self._create_markers()
        
        # Setup camera position behind the robot (negative Y)
        self._setup_camera()
        
        self.is_connected = True
        logger.info("PyBullet visualization setup complete")
        return True
    
    def _map_joints(self) -> bool:
        """Map joint names to PyBullet indices for both robots."""
        success = True
        
        for arm_name, robot_id in self.robot_ids.items():
            logger.info(f"Mapping joints for {arm_name} robot:")
            num_joints = p.getNumJoints(robot_id)
            p_name_to_index = {}
            
            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                joint_name = info[1].decode('UTF-8')
                joint_type = info[2]
                logger.info(f"  Index: {i}, Name: '{joint_name}', Type: {joint_type}")
                p_name_to_index[joint_name] = i
                if joint_type != p.JOINT_FIXED:
                    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)
            
            # Map to our joint indices
            mapped_count = 0
            for urdf_name, internal_name in URDF_TO_INTERNAL_NAME_MAP.items():
                if internal_name in JOINT_NAMES and urdf_name in p_name_to_index:
                    target_idx = JOINT_NAMES.index(internal_name)
                    self.joint_indices[arm_name][target_idx] = p_name_to_index[urdf_name]
                    mapped_count += 1
                    logger.info(f"  Mapped: '{internal_name}' -> '{urdf_name}' (Index {p_name_to_index[urdf_name]})")
            
            if mapped_count < NUM_JOINTS:
                missing = [name for i, name in enumerate(JOINT_NAMES) if self.joint_indices[arm_name][i] is None]
                logger.error(f"Could not map all joints for {arm_name} robot. Missing: {missing}")
                success = False
        
        return success
    
    def _find_end_effector(self) -> bool:
        """Find the end effector link index for both robots."""
        success = True
        
        for arm_name, robot_id in self.robot_ids.items():
            num_joints = p.getNumJoints(robot_id)
            found = False
            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                link_name = info[12].decode('UTF-8')
                if link_name == END_EFFECTOR_LINK_NAME:
                    self.end_effector_link_indices[arm_name] = i
                    logger.info(f"Found end effector link '{END_EFFECTOR_LINK_NAME}' for {arm_name} robot at index {i}")
                    found = True
                    break
            
            if not found:
                logger.error(f"Could not find end effector link '{END_EFFECTOR_LINK_NAME}' for {arm_name} robot")
                success = False
        
        return success
    
    def _read_joint_limits(self):
        """Read joint limits from URDF (using left robot as reference)."""
        logger.info("Reading URDF joint limits:")
        for i in range(NUM_JOINTS):
            pb_index = self.joint_indices['left'][i]
            joint_name = JOINT_NAMES[i]
            if pb_index is not None:
                joint_info = p.getJointInfo(self.robot_ids['left'], pb_index)
                lower, upper = joint_info[8], joint_info[9]
                if lower < upper:
                    self.joint_limits_min_deg[i] = math.degrees(lower)
                    self.joint_limits_max_deg[i] = math.degrees(upper)
                    logger.info(f"  {joint_name}: {self.joint_limits_min_deg[i]:.1f}° to {self.joint_limits_max_deg[i]:.1f}°")
                else:
                    logger.info(f"  {joint_name}: No limits found, using defaults")
    
    def _create_markers(self):
        """Create visualization markers."""
        # Target markers for both arms
        red_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
        self.viz_markers['left_target'] = p.createMultiBody(baseVisualShapeIndex=red_shape, basePosition=[0, 0, -1])
        
        blue_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
        self.viz_markers['right_target'] = p.createMultiBody(baseVisualShapeIndex=blue_shape, basePosition=[0, 0, -1])
        
        # Goal markers
        green_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[0, 1, 0, 0.9])
        self.viz_markers['left_goal'] = p.createMultiBody(baseVisualShapeIndex=green_shape, basePosition=[0, 0, -1])
        
        yellow_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[1, 1, 0, 0.9])
        self.viz_markers['right_goal'] = p.createMultiBody(baseVisualShapeIndex=yellow_shape, basePosition=[0, 0, -1])
        
        # Initialize coordinate frames
        self.viz_markers['left_target_frame'] = []
        self.viz_markers['right_target_frame'] = []
        self.viz_markers['left_goal_frame'] = []
        self.viz_markers['right_goal_frame'] = []
        
        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for XYZ
        for marker_name in ['left_target_frame', 'right_target_frame', 'left_goal_frame', 'right_goal_frame']:
            frame_lines = []
            for i in range(3):
                line_id = p.addUserDebugLine([0, 0, -1], [0, 0, -1], lineColorRGB=axis_colors[i], lineWidth=3)
                frame_lines.append(line_id)
            self.viz_markers[marker_name] = frame_lines
    
    def _setup_camera(self):
        """Setup camera position behind the robot (negative Y direction)."""
        # Position camera behind the robot in negative Y direction
        camera_distance = 0.5  # Distance from target
        camera_yaw = 160       # Look from negative Y toward positive Y (toward robot)
        camera_pitch = -30    # Slight downward angle
        camera_target = [0.0, 0.0, 0.2]  # Look at robot workspace center
        
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw, 
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target
        )
        
        logger.info(f"Camera positioned behind robot at distance={camera_distance}, yaw={camera_yaw}°, pitch={camera_pitch}°")
    
    def update_robot_pose(self, joint_angles_deg: np.ndarray, arm: str = 'left'):
        """Update robot joint positions in visualization for specified arm."""
        if not self.is_connected or arm not in self.robot_ids:
            return
        
        joint_angles_rad = np.deg2rad(joint_angles_deg)
        for i in range(NUM_JOINTS):
            if self.joint_indices[arm][i] is not None:
                p.resetJointState(self.robot_ids[arm], self.joint_indices[arm][i], joint_angles_rad[i])
    
    def update_marker_position(self, marker_name: str, position: np.ndarray, 
                              orientation: Optional[np.ndarray] = None):
        """Update position of a visualization marker."""
        if not self.is_connected or marker_name not in self.viz_markers:
            return
        
        if orientation is None:
            orientation = [0, 0, 0, 1]
        
        p.resetBasePositionAndOrientation(
            self.viz_markers[marker_name], 
            position.tolist(), 
            orientation
        )
    
    def update_coordinate_frame(self, frame_name: str, position: np.ndarray, 
                               orientation_quat: Optional[np.ndarray] = None):
        """Update coordinate frame visualization."""
        if not self.is_connected or frame_name not in self.viz_markers:
            return
        
        frame_lines = self.viz_markers[frame_name]
        if not frame_lines:
            return
        
        axis_length = 0.05
        
        # Default to identity rotation
        if orientation_quat is None:
            orientation_quat = [0, 0, 0, 1]
        
        # Convert quaternion to rotation matrix
        r = R.from_quat(orientation_quat)
        rotation_matrix = r.as_matrix()
        
        # Update each axis line (X=red, Y=green, Z=blue)
        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for i in range(3):
            if i < len(frame_lines):
                axis_vector = rotation_matrix[:, i] * axis_length
                end_point = position + axis_vector
                
                p.addUserDebugLine(
                    position.tolist(), 
                    end_point.tolist(), 
                    lineColorRGB=axis_colors[i], 
                    lineWidth=3,
                    replaceItemUniqueId=frame_lines[i]
                )
    
    def hide_marker(self, marker_name: str):
        """Hide a marker by moving it off-screen."""
        if marker_name in self.viz_markers:
            self.update_marker_position(marker_name, np.array([0, 0, -1]))
    
    def hide_frame(self, frame_name: str):
        """Hide a coordinate frame."""
        if frame_name in self.viz_markers:
            frame_lines = self.viz_markers[frame_name]
            for line_id in frame_lines:
                p.addUserDebugLine(
                    [0, 0, -1], [0, 0, -1], 
                    lineColorRGB=[0, 0, 0], 
                    lineWidth=1,
                    replaceItemUniqueId=line_id
                )
    
    def step_simulation(self):
        """Step the simulation forward."""
        if self.is_connected:
            p.stepSimulation()
    
    def disconnect(self):
        """Disconnect from PyBullet."""
        if self.is_connected and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            self.is_connected = False
            logger.info("PyBullet disconnected")
    
    @property
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint limits in degrees."""
        return self.joint_limits_min_deg.copy(), self.joint_limits_max_deg.copy() 