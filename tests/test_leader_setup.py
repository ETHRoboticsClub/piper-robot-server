import pprint
import time

import pytest
from piper_sdk import ArmMsgFeedbackStatusEnum

from piper_teleop.robot_server.core.piper import Piper, PiperConfig
from piper_teleop.robot_server.robot_leader import PiperLeader


# @pytest.mark.skip(reason='only for manual testing')
def test_read_robot_positions():
    left_config = PiperConfig(port="left_piper", id="left_piper")
    robot = Piper(left_config)
    while True:
        obs = robot.get_observation()
        pprint.pprint(obs)
        status = robot.sdk.piper.GetArmStatus()

        # assert status.arm_status.ctrl_mode == ArmMsgFeedbackStatusEnum.CtrlMode.TEACHING_MODE
        time.sleep(1.0)


def test_leader():
    leader = PiperLeader()
    print("leader created")
    leader.connect()
    print("leader connected")
    obs = leader.get_observations()
    pprint.pprint(obs)
    assert "left" in obs
    assert "right" in obs
