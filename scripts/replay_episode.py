import argparse
import logging
import time
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import busy_wait
from piper_teleop.robot_server.core import RobotInterface
from piper_teleop.config import config

logger = logging.getLogger(__name__)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Replay an episode from a dataset")
    parser.add_argument("--repo-id", type=str, help="The repo ID of the dataset")
    parser.add_argument("--episode-id", type=int, help="The episode ID to replay")
    parser.add_argument("--root", type=str, help="The root directory for the dataset")

    args = parser.parse_args()

    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.root, episodes=[args.episode_id])

    episode_frames = dataset.hf_dataset.filter(lambda x: x["episode_index"] == args.episode_id)
    actions = episode_frames.select_columns('action')
    logger.info(f"Replaying episode {args.episode_id} with {len(actions)} frames")
    robot = RobotInterface(config, )
    robot.setup_kinematics()
    robot.connect()
    for idx in range(dataset.num_frames):
        start_episode_t = time.perf_counter()

        action_array = actions[idx]["action"]
        dict_left = dict()
        dict_right = dict()
        for i, name in enumerate(dataset.features["action"]["names"]):
            action = float(action_array[i])
            if name[:2] == 'L.':
                dict_left[name.replace('L.','') + '.pos'] = action
            elif name[:2] == 'R.':
                dict_right[name.replace('R.', '') + '.pos'] = action
            else:
                raise Exception('wrong name')

        robot.left_robot.send_action(dict_left)
        robot.right_robot.send_action(dict_right)
        q_1 = [dict_left[k] for k in sorted(dict_left)[:6]]
        q_2 = [dict_right[k] for k in sorted(dict_right)[:6]]
        robot.ik_solver.vis.display(np.array(q_1 + q_2))
        dt_s = time.perf_counter() - start_episode_t
        busy_wait(1 / dataset.fps - dt_s)
