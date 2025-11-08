import logging
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy

logger = logging.getLogger(__name__)


class LerobotPolicy:
    """Slim wrapper for LeRobot policy inference."""

    def __init__(self, policy_path:str, repo_id: str, device='cpu'):
        self.policy_path = str(policy_path)
        self.repo_id = repo_id
        self.device = torch.device(device)

        dataset_meta = LeRobotDatasetMetadata(repo_id)
        policy_config = PreTrainedConfig.from_pretrained(self.policy_path)

        # Override device in config to match desired device
        policy_config.device = str(self.device)

        self.policy = make_policy(policy_config, ds_meta=dataset_meta)

    def predict(
        self,
        left_joints: dict,
        right_joints: dict,
        cams: dict[str, np.ndarray],
        dof: int = 7,
    ) -> np.ndarray:
        state = np.array(
            [left_joints[f"joint_{i}.pos"] for i in range(dof)]
            + [right_joints[f"joint_{i}.pos"] for i in range(dof)],
            dtype=np.float32,
        )
        batch = {
            "observation.state": torch.from_numpy(state).unsqueeze(0),  # Add batch dimension
        }

        for cam_key, img in cams.items():
            img_tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
            batch[cam_key] = img_tensor.unsqueeze(0)  # Add batch dimension
        batch = {k: v.to(self.device) for k, v in batch.items()}

        with torch.no_grad():
            action = self.policy.select_action(batch)

        return action.cpu().numpy().squeeze(0)
