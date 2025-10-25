import argparse

import logging
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import ACTION


logger = logging.getLogger(__name__)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Replay an episode from a dataset")
    parser.add_argument("--repo-id", type=str, help="The repo ID of the dataset")
    parser.add_argument("--episode-id", type=int, help="The episode ID to replay")
    parser.add_argument("--root", type=str, help="The root directory for the dataset")

    args = parser.parse_args()

    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.root, episodes=[args.episode_id])

    episode_frames = dataset.hf_dataset.filter(lambda x: x["episode_index"] == args.episode_id)
    actions = episode_frames.select_columns(ACTION)

    logger.info(f"Replaying episode {args.episode_id} with {len(actions)} frames")
