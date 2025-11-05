import logging
from typing import Optional

from piper_sdk import ArmMsgFeedbackStatusEnum

from piper_teleop.robot_server.core.robot_interface import suppress_stdout_stderr
from piper_teleop.robot_server.core.piper import Piper, PiperConfig

logger = logging.getLogger(__name__)

class PiperLeader:
    def __init__(self, assert_robot_mode=False):
        self.robot_left: Optional[Piper] = None
        self.robot_right: Optional[Piper] = None
        self.left_config = PiperConfig(port=f"leader_left_piper", id="leader_left_piper")
        self.right_config = PiperConfig(port=f"leader_right_piper", id="leader_right_piper")
        self.assert_robot_mode = assert_robot_mode
    def _connect_robot(self, config:PiperConfig) -> Piper:
        logger.info(f"Connecting leader arms {config.id}")
        with suppress_stdout_stderr():
            robot = Piper(config)
            robot.connect()
            if self.assert_robot_mode:
                status = robot.sdk.piper.GetArmStatus()
                assert status.arm_status.ctrl_mode == ArmMsgFeedbackStatusEnum.CtrlMode.TEACHING_MODE
            return robot

    def connect(self):
        """Connect to robot hardware."""
        logger.info("Connecting leader arms robot...")
        logger.info("connecting left arm")
        try:
            self.robot_left = self._connect_robot(self.left_config)
            logger.info("✅ Left arm connected successfully")
        except Exception as e:
            logger.error(f"❌ Left arm connection failed: {e}")
        try:
            self.robot_right = self._connect_robot(self.right_config)
            logger.info("✅ Right arm connected successfully")
        except Exception as e:
            logger.error(f"❌ Right arm connection failed: {e}")

    def get_zero_observation(self):
        return {'joint_0.pos': 0.,
                'joint_1.pos': 0.,
                'joint_2.pos': 0.,
                'joint_3.pos': 0.,
                'joint_4.pos': 0.,
                'joint_5.pos': 0.,
                'joint_6.pos': 0.}
    def get_observations(self):
        left_obs = self.robot_left.get_observation() if self.robot_left else self.get_zero_observation()
        right_obs = self.robot_right.get_observation() if self.robot_right else self.get_zero_observation()
        return {'left': left_obs, 'right': right_obs}
