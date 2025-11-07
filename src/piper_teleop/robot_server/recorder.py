import atexit
import logging
import time
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import say
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun

from piper_teleop.robot_server.camera.camera_config import CameraConfig
from piper_teleop.robot_server.recorder_utils import convert_image_dataset_to_video

logger = logging.getLogger(__name__)

class RecState(Enum):
    INIT = auto()
    RESET_ENV = auto()
    RECORDING = auto()
    FINISHED = auto()


class Recorder:

    def __init__(
        self,
        repo_id: str,
        task: str,
        resume=False,
        root=Path(__file__).parents[3] / "data",
        single_arm=False,
        cameras: list[CameraConfig] = None,
        dof=7,
        fps=30,
        robot_type="piper",
        play_sound=True,
        use_video=False,
        image_writer_processes=0,
        image_writer_threads=12,
        display_data=False,
        convert_images_to_video=False
    ):
        if use_video is False:
            logger.info("Init recorder in fast mode. Images will be converted to video at the end of recording")
            self.convert_images_to_video = convert_images_to_video
        else:
            self.convert_images_to_video = False
        # Features
        self.single_arm = single_arm
        if self.single_arm:
            self.joints = [f"joint_{i}" for i in range(dof)]
        else:
            self.joints = [f"L.joint_{i}" for i in range(dof)] + [f"R.joint_{i}" for i in range(dof)]

        # Build camera feature shapes from configs if provided
        self.cams = None
        if cameras is not None:
            assert isinstance(cameras, list), "cameras must be a list of CameraConfig"
            cams_dict = {}
            for cam_cfg in cameras:
                assert isinstance(cam_cfg, CameraConfig), "each camera must be a CameraConfig"
                cams_dict[cam_cfg.name] = (cam_cfg.frame_height, cam_cfg.frame_width, 3)
            self.cams = cams_dict
        self.dof = dof

        # lerobot dataset
        self.repo_id = repo_id
        self.task = task
        self.root = str(root)
        self.resume = resume
        self.fps = fps
        self.robot_type = robot_type
        self.dataset: Optional[LeRobotDataset] = None
        self.dataset_manager: Optional[VideoEncodingManager] = None
        self.image_writer_processes = image_writer_processes
        self.image_writer_threads = image_writer_threads
        self.use_video = use_video

        listener, events = init_keyboard_listener()
        self.events = events
        self.state = RecState.INIT
        self.play_sound = play_sound
        self.display_data = display_data

        atexit.register(self.exit)

    def __exit__(self, exc_type, exc, tb):
        self.exit()
        return False

    def exit(self):
        if self.dataset is not None:
            logger.info("Cleaning up recorder resources")
            self.dataset.stop_image_writer()
            logger.info("Ending up recorder resources")
            self._end_recording()

    @property
    def features(self):
        feutures = dict()
        feutures["action"] = {
            "dtype": "float32",
            "shape": (len(self.joints),),
            "names": self.joints,
        }
        feutures["observation.state"] = {
            "dtype": "float32",
            "shape": (len(self.joints),),
            "names": self.joints,
        }
        if self.cams is not None:
            for cam, hwc in self.cams.items():
                feutures[f"observation.images.{cam}"] = {
                    "dtype": "video" if self.use_video else "image",
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
            use_videos=self.use_video,
            image_writer_processes=self.image_writer_processes,
            image_writer_threads=self.image_writer_threads,
        )
        if self.display_data:
            init_rerun(session_name="recording")

        self.dataset = dataset
        self.dataset_manager = VideoEncodingManager(dataset)

    def start_recording(self):
        logger.info("start recording")
        self._create_dataset()

    def add_observation(
        self,
        left_joints,
        right_joints,
        left_joints_target,
        right_joints_target,
        cams: dict[str, np.ndarray],
    ) -> None:
        if self.state == RecState.RECORDING:

            state = np.array(
                [left_joints[f"joint_{i}.pos"] for i in range(self.dof)]
                + [right_joints[f"joint_{i}.pos"] for i in range(self.dof)],
                dtype=np.float32,
            )
            target = np.array(
                [left_joints_target[f"joint_{i}.pos"] for i in range(self.dof)]
                + [right_joints_target[f"joint_{i}.pos"] for i in range(self.dof)],
                dtype=np.float32,
            )
            frame = {"observation.state": state,
                     "action": target,
                     "task": self.task,
                     **cams}
            self.dataset.add_frame(frame)  # type: ignore

    def show_data(self,
                     left_joints,
                     right_joints,
                     left_joints_target,
                     right_joints_target,
                     cams: dict[str, np.ndarray],
                     ):
        left_joints = {"L" + k: v for k,v in left_joints.items()}
        right_joints = {"R" + k: v for k,v in right_joints.items()}
        obs = {**left_joints, **right_joints, **cams}
        left_joints_action = {"L" + k: v for k,v in left_joints_target.items()}
        right_joints_action = {"R" + k: v for k,v in right_joints_target.items()}
        action = {**left_joints_action, **right_joints_action}
        log_rerun_data(observation=obs, action=action)

    def _transition(self, next_state, message=None, action=None, message_post=None):
        if message and self.play_sound:
            logger.info(message)
            say(message)
        if action:
            action()
        if message_post and self.play_sound:
            logger.info(message_post)
            say(message_post)
        self.state = next_state

    def _end_recording(self):
        self._delete_episodes()
        self.dataset_manager.__exit__('cleanup', None, None)
        if self.convert_images_to_video:
            convert_image_dataset_to_video(self.dataset.root)
            # avoid twice conversion
            self.convert_images_to_video = False
        self.events["stop_recording"] = False

    def _save_episodes(self):
        start_save = time.perf_counter()
        self.dataset.save_episode()
        save_time = time.perf_counter() - start_save
        logger.info(f"Save episode time {save_time:.2f}s frames {self.dataset.num_frames}")

    def _delete_episodes(self):
        # Gives lerobot dataset time to catch up before deleting
        time.sleep(0.2)
        self.dataset.clear_episode_buffer()

    def handle_keyboard_event(self):
        """
        Keyboard controls:
            →  (Right Arrow): Exit early / advance to next stage.
                              In RECORDING: save current episode and reset.
            ←  (Left Arrow) : Re-record current episode (discard + reset env).
            Esc             : Stop recording and return to INIT (from any state).

        """
        if self.events["rerecord_episode"]:
            # From anywhere, go reset env to rerecord
            self._transition(RecState.RESET_ENV, "Deleted episode", self._delete_episodes)
            self.events["rerecord_episode"] = False
            self.events["exit_early"] = False

        if self.events["exit_early"]:
            if self.state == RecState.INIT:
                self._transition(RecState.RESET_ENV, "reset environment")
            elif self.state == RecState.RESET_ENV:
                self._transition(RecState.RECORDING, "starting recording")
            elif self.state == RecState.RECORDING:
                self._transition(RecState.RESET_ENV, None, self._save_episodes, "Episode saved")
            self.events["exit_early"] = False

        if self.events["stop_recording"]:
            self._transition(RecState.FINISHED, "Stop Recording", self._end_recording, None)
