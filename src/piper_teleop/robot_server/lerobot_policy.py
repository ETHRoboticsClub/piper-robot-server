import logging
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import OBS_STR, build_dataset_frame, predict_action, rename_stats

logger = logging.getLogger(__name__)


class LerobotPolicy:
    """Slim wrapper for LeRobot policy inference."""

    def __init__(self, policy_path: str, repo_id: str, device="cpu"):
        self.policy_path = str(policy_path)
        self.repo_id = repo_id
        self.device = torch.device(device)

        self.dataset = LeRobotDataset(repo_id, batch_encoding_size=1)
        self.dataset_meta = self.dataset.meta
        policy_config = PreTrainedConfig.from_pretrained(self.policy_path)
        policy_config.pretrained_path = self.policy_path

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

    def convert_actions_to_dict(self, actions: dict) -> tuple[dict, dict]:
        dict_left = dict()
        dict_right = dict()
        for k, v in actions.items():
            if k.startswith("L."):
                dict_left[k.replace("L.", "") + ".pos"] = v
            elif k.startswith("R."):
                dict_right[k.replace("R.", "") + ".pos"] = v
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
        obs = {k.replace("observation.images.", ""): v for k, v in cams.items()}

        for left_joint in left_joints:
            obs["L." + left_joint.replace(".pos", "")] = left_joints[left_joint]
        for right_joint in right_joints:
            obs["R." + right_joint.replace(".pos", "")] = right_joints[right_joint]

        batch = self.robot_observation_processor(obs)

        observation_frame = build_dataset_frame(self.dataset.features, batch, prefix=OBS_STR)

        action_values = predict_action(
            observation=observation_frame,
            policy=self.policy,
            device=self.device,
            preprocessor=self.preprocessor,
            postprocessor=self.postprocessor,
            use_amp=self.policy.config.use_amp,
            task="pick and place",
            robot_type=self.dataset_meta.robot_type,
        )

        act_processed = make_robot_action(action_values, self.dataset_meta.features)
        act_processed = self.robot_action_processor((act_processed, obs))

        return self.convert_actions_to_dict(act_processed)
        