from pathlib import Path

import numpy as np
import pytest

from piper_teleop.robot_server.lerobot_policy import LerobotPolicy


# @pytest.mark.skip()
def test_predict_action():
    policy_path = Path(__file__).parent.parent / "policy_checkpoint" / "checkpoints" / "last" / "pretrained_model"
    repo_id = "ETHRC/pick_and_place"

    policy = LerobotPolicy(policy_path, repo_id)

    # Create dummy observations (similar to Recorder format)
    left_joints = {f"joint_{i}.pos": np.random.randn() for i in range(7)}
    right_joints = {f"joint_{i}.pos": np.random.randn() for i in range(7)}

    # Create dummy camera images (H, W, C format with uint8 values)
    cams = {
        "observation.images.wrist1": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.wrist2": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.stereo": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
    }

    # Predict action
    action = policy.predict(left_joints, right_joints, cams)

    # Verify output shape
    assert action.shape == (14,)
    print(f"Predicted action shape: {action.shape}")
    print(f"Action values: {action}")
