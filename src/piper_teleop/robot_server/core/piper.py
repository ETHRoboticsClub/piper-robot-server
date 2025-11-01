# Implementation of Piper robot
from dataclasses import dataclass
from typing import Any

import numpy as np

#
from .geometry import xyzrpy2transform
from .piper_sdk_interface import PiperSDKInterface
from .sim_interface import PyBulletSimInterface, NewtonSimInterface


@dataclass
class PiperConfig:
    port: str
    id: str
    use_sim: bool = False
    sim_type: str = "pybullet"  # "pybullet" or "newton"
    sim_gui: bool = False
    urdf_path: str = None
    sim_shared: bool = False  # If True, this config shares a single simulator instance with the other arm
    arm_side: str = "left"  # 'left' or 'right' identifier for multi-arm URDFs


class Piper:
    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig, sdk_instance: Any = None):
        self.config = config
        self.use_sim = config.use_sim
        self.arm_side = config.arm_side

        # Allow injecting a shared SDK/simulation instance (used for dual-URDFs)
        if sdk_instance is not None:
            self.sdk = sdk_instance
            return

        if self.use_sim:
            # Use simulation interface
            if config.sim_type == "newton":
                self.sdk = NewtonSimInterface(urdf_path=config.urdf_path, use_gui=config.sim_gui)
            else:  # default to pybullet
                # Set base position based on which arm this is
                base_position = [0, 0, 0] if config.id == "left_follower" else [0, -0.57, 0]
                self.sdk = PyBulletSimInterface(
                    urdf_path=config.urdf_path, 
                    use_gui=config.sim_gui,
                    robot_base_position=base_position
                )
        else:
            # Use real hardware interface
            self.sdk = PiperSDKInterface(port=config.port)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(7)}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return self.sdk.get_connection_status()

    def connect(self, calibrate: bool = True) -> None:
        # Connect to hardware or simulation
        if self.use_sim:
            self.sdk.connect()
        # For hardware, already connected in SDK init
        self.configure()

    def disconnect(self) -> None:
        self.sdk.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        # If the SDK supports per-arm status (shared dual-URDF), prefer that
        if hasattr(self.sdk, "get_arm_status"):
            try:
                return self.sdk.get_arm_status(self.arm_side)
            except Exception:
                # Fallback to generic status
                pass
        obs_dict = self.sdk.get_status()
        return obs_dict

    def get_end_effector_transform(self) -> np.ndarray:

        # If the SDK exposes per-arm EE pose, use that
        if hasattr(self.sdk, "get_arm_end_effector_pose"):
            try:
                raw_pose = self.sdk.get_arm_end_effector_pose(self.arm_side)
            except Exception:
                raw_pose = self.sdk.get_end_effector_pose()
        else:
            raw_pose = self.sdk.get_end_effector_pose()
        raw_transform = xyzrpy2transform(
            raw_pose["x"], raw_pose["y"], raw_pose["z"], raw_pose["roll"], raw_pose["pitch"], raw_pose["yaw"]
        )
        link6_to_gripper_transform = xyzrpy2transform(0.0, 0.0, 0.13, 0.0, -1.57, 0.0)
        gripper_transform = raw_transform @ link6_to_gripper_transform
        return gripper_transform

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # map the action from the leader to joints for the follower
        positions = [
            action.get("joint_0.pos"),
            action.get("joint_1.pos"),
            action.get("joint_2.pos"),
            action.get("joint_3.pos"),
            action.get("joint_4.pos"),
            action.get("joint_5.pos"),
            action.get("joint_6.pos"),
        ]

        # If the SDK supports per-arm setting (shared dual-URDF), call that
        if hasattr(self.sdk, "set_arm_positions"):
            try:
                self.sdk.set_arm_positions(self.arm_side, positions)
                return action
            except Exception:
                # fall back to generic
                pass

        self.sdk.set_joint_positions(positions)
        return action
