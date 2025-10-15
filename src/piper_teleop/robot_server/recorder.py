import atexit
import logging
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robot_devices.control_utils import init_keyboard_listener
from lerobot.common.utils.utils import say

logger = logging.getLogger(__name__)

from enum import Enum, auto

class RecState(Enum):
    INIT = auto()
    RESET_ENV = auto()
    RECORDING = auto()


class Recorder:
    def __init__(self,
                 repo_id:str,
                 task: str,
                 resume=False,
                 root=Path(__file__).parents[3] / 'data',
                 single_arm=False,
                 cams: Optional[Dict[str, Any]] = None,
                 dof=7,
                 fps=30,
                 robot_type='piper',
                 play_sound=True,
                 ):
        # Feutures
        self.single_arm = single_arm
        if self.single_arm:
            self.joints = [f'joint_{i}' for i in range(dof)]
        else:
            self.joints = [f'L.joint_{i}' for i in range(dof)] + [f'R.joint_{i}' for i in range(dof)]

        if cams is not None:
            for cam in cams.values():
                assert isinstance(cam, tuple), 'cam values need to be tuples (H, W, C)'
                assert len(cam) == 3, 'cam values need to be tuples (H, W, C)'
        self.cams = cams
        self.dof = dof

        # lerobot dataset
        self.repo_id = repo_id
        self.task=task
        self.root = str(root)
        self.resume = resume
        self.fps = fps
        self.robot_type=robot_type
        self.dataset: Optional[LeRobotDataset] = None

        listener, events = init_keyboard_listener()
        self.events = events
        self.state = RecState.INIT
        self.play_sound = play_sound

        atexit.register(self.exit)

    def __exit__(self, exc_type, exc, tb):
        self.exit()
        return False

    def exit(self):
        if self.dataset is not None:
            self.dataset.stop_image_writer()
            logger.info("Cleaning up recorder resources")

    @property
    def features(self):
        feutures = dict()
        feutures['action'] = {"dtype": "float32",
                              "shape": (len(self.joints),),
                              "names": self.joints,
                               }
        feutures['observation.state'] = {"dtype": "float32",
                                         "shape": (len(self.joints),),
                                         "names": self.joints,
                                          }
        if self.cams is not None:
            for cam, hwc in self.cams.items():
                feutures[f'observation.images.{cam}']  = {"dtype": "video" ,
                                                        "shape": hwc,
                                                        "names": ["height", "width", "channels"],
                                                        }
        return feutures

    def _create_dataset(self):
        dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            root=self.root,
            features=self.features,
            fps=self.fps,
            robot_type=self.robot_type,
            use_videos=True,
            image_writer_processes=0,
            image_writer_threads=4,
        )
        self.dataset = dataset
    
    def start_recording(self):
        logger.info('start recording')
        self._create_dataset()
    
    def add_observation(self, left_joints, right_joints, left_joints_target, right_joints_target, cams):
        if self.state == RecState.RECORDING:
            state = np.array(
                [left_joints[f'joint_{i}.pos'] for i in range(self.dof)] +
                [right_joints[f'joint_{i}.pos'] for i in range(self.dof)],
                dtype=np.float32
            )
            target = np.array(
                [left_joints_target[f'joint_{i}.pos'] for i in range(self.dof)] +
                [right_joints_target[f'joint_{i}.pos'] for i in range(self.dof)],
                dtype=np.float32
            )
            frame = {'observation.state': state,
                     'action': target,
                     'task': self.task,
                     **cams}
            self.dataset.add_frame(frame)


    def _transition(self, next_state, message=None, action=None, message_post=None):
        if message and self.play_sound:
            say(message)
        if action:
            action()
        if message_post:
            say(message_post)
        self.state = next_state

    def _save_episodes(self):
        self.dataset.save_episode()

    def _delete_episodes(self):
        self.dataset.clear_episode_buffer()

    def handle_keyboard_event(self):
        """
        Keyboard controls:
            →  (Right Arrow): Exit early / advance to next stage.
                              In RECORDING: save current episode and reset.
            ←  (Left Arrow) : Re-record current episode (discard + reset env).
            Esc             : Stop recording and return to INIT (from any state).

        """
        if self.events['rerecord_episode']:
            # From anywhere, go reset env to rerecord
            self._transition(RecState.RESET_ENV, "rerecording, reset environment", self._delete_episodes)
            self.events['rerecord_episode'] = False

        if self.events['exit_early']:
            if self.state == RecState.INIT:
                self._transition(RecState.RESET_ENV, "reset environment")
            elif self.state == RecState.RESET_ENV:
                self._transition(RecState.RECORDING, "starting recording")
            elif self.state == RecState.RECORDING:
                self._transition(RecState.RESET_ENV, "wait saving episode", self._save_episodes, 'episode saved. Resetting Env')
            self.events['exit_early'] = False