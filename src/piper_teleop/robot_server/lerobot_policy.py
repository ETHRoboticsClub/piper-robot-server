import logging
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import predict_action, rename_stats

logger = logging.getLogger(__name__)


class LerobotPolicy:
    """Slim wrapper for LeRobot policy inference."""

    def __init__(self, policy_path: str, repo_id: str, device="cpu"):
        self.policy_path = str(policy_path)
        self.repo_id = repo_id
        self.device = torch.device(device)

        self.dataset = LeRobotDataset(repo_id)
        self.dataset_meta = self.dataset.meta
        policy_config = PreTrainedConfig.from_pretrained(self.policy_path)

        # Override device in config to match desired device
        policy_config.device = str(self.device)

        self.policy = make_policy(policy_config, ds_meta=self.dataset_meta)

        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=policy_config.pretrained_path,
            dataset_stats=rename_stats(self.dataset_meta.stats, dict()),
            preprocessor_overrides={
                "device_processor": {"device": self.device},
                "rename_observations_processor": {"rename_map": dict()},
            },
        )

        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()

        self.teleop_action_processor, self.robot_action_processor, self.robot_observation_processor = (
            make_default_processors()
        )

    def convert_actions_to_dict(self, actions: np.ndarray) -> tuple[dict, dict]:
        dict_left = dict()
        dict_right = dict()
        for i, name in enumerate(self.dataset_meta.features["action"]["names"]):
            action = float(actions[i])
            if name[:2] == "L.":
                dict_left[name.replace("L.", "") + ".pos"] = action
            elif name[:2] == "R.":
                dict_right[name.replace("R.", "") + ".pos"] = action
            else:
                raise Exception("wrong name")
        return dict_left, dict_right

    def predict(
        self,
        left_joints: dict,
        right_joints: dict,
        cams: dict[str, np.ndarray],
        dof: int = 7,
    ) -> np.ndarray:
        state = np.array(
            [left_joints[f"joint_{i}.pos"] for i in range(dof)] + [right_joints[f"joint_{i}.pos"] for i in range(dof)],
            dtype=np.float32,
        )
        batch = {
            "observation.state": state,
            **cams,
        }

        action_values = predict_action(
            observation=batch,
            policy=self.policy,
            device=self.device,
            preprocessor=self.preprocessor,
            postprocessor=self.postprocessor,
            use_amp=self.policy.config.use_amp,
            task="pick and place",
            robot_type=self.dataset_meta.robot_type,
        )

        # act_processed = make_robot_action(action_values, self.dataset_meta.features)
        # act_processed = self.robot_action_processor((act_processed, batch))

        action_values = action_values.cpu().numpy().squeeze(0)

        return self.convert_actions_to_dict(action_values)
