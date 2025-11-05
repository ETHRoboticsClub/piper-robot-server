# Piper SDK interface

import time
from typing import Any, Dict

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Is the piper_sdk installed: pip install piper_sdk")
    C_PiperInterface_V2: Any = None  # For type checking and docs

JOINT_LIMITS_RAD = {
    "min": [-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439],
    "max": [2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439],
}
DEG_TO_RAD = 0.017444
RAD_TO_DEG = 1 / DEG_TO_RAD
GRIPPER_ANGLE_MAX = 0.07  # 70mm


class PiperSDKInterface:
    def __init__(self, port: str = "can0"):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed.")
        self.piper = C_PiperInterface_V2(port)
        self.piper.ConnectPort()
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        self.min_pos = [pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [0]
        self.max_pos = [pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [
            10
        ]  # Gripper max position in mm

    def set_joint_positions(self, positions):

        joint_angles = []
        for i, pos in enumerate(positions[:6]):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(pos, min_rad), max_rad)
            pos_deg = clipped_pos * RAD_TO_DEG
            joint_angle = round(pos_deg * 1e3)  # Convert to millidegrees
            joint_angles.append(joint_angle)

        gripper_position = min(max(positions[6], 0.0), GRIPPER_ANGLE_MAX)
        gripper_position_int = round(gripper_position * 1e6)

        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(*joint_angles)
        self.piper.GripperCtrl(gripper_position_int, 1000, 0x01, 0)

    def set_joint_positions_and_ff(self, positions, ff_torques):
        velocities = [0.0] * 6
        kps = [10.0] * 6
        kds = [0.8] * 6

        kps[2] = 2.5
        kds[2] = 0.2

        kps[4] = 15.0
        kds[4] = 1.2

        ff_torques[0] = 0.0
        ff_torques[3] = 0.0
        ff_torques[5] = 0.0

        ff_torques[1] = 0.0
        #ff_torques[2] = 0.0
        ff_torques[2] = ff_torques[2]
        ff_torques[4] = 0.0

        # ctrl_mode=0x01 (CAN control), move_mode=0x04 (MOVE M), is_mit_mode=0xAD (Enable MIT)
        self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x04, is_mit_mode=0xAD)


        for i in range(6):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(positions[i], min_rad), max_rad)

            self.piper.JointMitCtrl(
                motor_num=i + 1,      # Motor numbers are 1-based
                pos_ref=clipped_pos,
                vel_ref=velocities[i],
                kp=kps[i],
                kd=kds[i],
                t_ref=ff_torques[i]
            )

        gripper_position = min(max(positions[6], 0.0), GRIPPER_ANGLE_MAX)
        gripper_position_int = round(gripper_position * 1e6)

        self.piper.GripperCtrl(gripper_position_int, 1000, 0x01, 0)

    def get_fw_version(self):
        print(self.piper.GetPiperFirmwareVersion())

    def get_status(self) -> Dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()
        gripper.gripper_state.grippers_angle

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_0.pos": (joint_state.joint_1 / 1000) * DEG_TO_RAD,
            "joint_1.pos": (joint_state.joint_2 / 1000) * DEG_TO_RAD,
            "joint_2.pos": (joint_state.joint_3 / 1000) * DEG_TO_RAD,
            "joint_3.pos": (joint_state.joint_4 / 1000) * DEG_TO_RAD,
            "joint_4.pos": (joint_state.joint_5 / 1000) * DEG_TO_RAD,
            "joint_5.pos": (joint_state.joint_6 / 1000) * DEG_TO_RAD,
        }
        obs_dict.update(
            {
                "joint_6.pos": gripper.gripper_state.grippers_angle / 1000000,
            }
        )

        return obs_dict

    def get_end_effector_pose(self) -> Dict[str, float]:
        """
        Returns the current end-effector pose as a sequence of floats.

        Returns:
        Sequence[float]: (x, y, z, roll, pitch, yaw) in meters and radians.
        """
        pose = self.piper.GetArmEndPoseMsgs()
        x = pose.end_pose.X_axis * 1e-6  # Convert from mm to m
        y = pose.end_pose.Y_axis * 1e-6
        z = pose.end_pose.Z_axis * 1e-6
        roll = pose.end_pose.RX_axis * 1e-3 * DEG_TO_RAD
        pitch = pose.end_pose.RY_axis * 1e-3 * DEG_TO_RAD
        yaw = pose.end_pose.RZ_axis * 1e-3 * DEG_TO_RAD
        return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}

    def get_connection_status(self):
        return self.piper.get_connect_status()

    def disconnect(self):
        self.piper.DisconnectPort()
