import time
import atexit
import logging
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import write_info, DEFAULT_VIDEO_PATH
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import say
import pyarrow.parquet as pq

logger = logging.getLogger(__name__)

from enum import Enum, auto

def convert_image_dataset_to_video(dataset: LeRobotDataset):
    """Convert a dataset recorded with image frames into the canonical video layout."""
    logger.info("converting images to video")
    import shutil

    image_keys = list(dataset.meta.image_keys)
    if not image_keys:
        return

    # Step 1: Update metadata to video format
    for key in image_keys:
        dataset.meta.info["features"][key]["dtype"] = "video"
    dataset.meta.info["video_path"] = DEFAULT_VIDEO_PATH
    write_info(dataset.meta.info, dataset.meta.root)
    dataset.meta.load_metadata()

    # Step 2: Encode videos for each episode
    for episode_index in range(dataset.num_episodes):
        dataset.encode_episode_videos(episode_index)

    # Step 3: Regenerate parquet files without image columns
    # (Video datasets don't store image data in parquet)
    for episode_index in range(dataset.num_episodes):
        parquet_path = dataset.root / dataset.meta.get_data_file_path(episode_index)

        # Load existing parquet data
        table = pq.read_table(parquet_path)

        # Remove image columns (they're now in videos)
        columns_to_keep = [col for col in table.schema.names if not col.startswith('observation.images.')]
        table_without_images = table.select(columns_to_keep)

        # Save back to parquet
        pq.write_table(table_without_images, parquet_path)

    dataset.meta.info["total_videos"] = dataset.num_episodes * len(dataset.meta.video_keys)
    write_info(dataset.meta.info, dataset.meta.root)
    dataset.meta.load_metadata()

    # Step 4: Clean up images directory
    images_dir = dataset.root / "images"
    if images_dir.exists():
        shutil.rmtree(images_dir)

class RecState(Enum):
    INIT = auto()
    RESET_ENV = auto()
    RECORDING = auto()
    FINISHED = auto()


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
                 use_video=False,
                 image_writer_processes=4,
                 image_writer_threads=16,
                 ):
        # Feutures
        if use_video is False:
            logger.info('Init recorder in fast mode. Images will be converted to video at the end of recording')
            self.convert_images_to_video = True
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
        self.image_writer_processes = image_writer_processes
        self.image_writer_threads = image_writer_threads
        self.use_video = use_video

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
            logger.info("Cleaning up recorder resources")
            self.dataset.stop_image_writer()
            logger.info("Ending up recorder resources")
            self._end_recording()

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
                feutures[f'observation.images.{cam}']  = {"dtype": "video" if self.use_video else "image",
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
        self.dataset = dataset

    def convert_images_to_video(self):

        pass

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
                     **cams}
            self.dataset.add_frame(frame, self.task)


    def _transition(self, next_state, message=None, action=None, message_post=None):
        if message and self.play_sound:
            logger.info(message)
            say(message)
        if action:
            action()
        if message_post:
            logger.info(message_post)
            say(message_post)
        self.state = next_state

    def _end_recording(self):
        self._delete_episodes()
        if self.convert_images_to_video:
            convert_image_dataset_to_video(self.dataset)
            # avoid twice conversion
            self.convert_images_to_video = False


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
        if self.events['rerecord_episode']:
            # From anywhere, go reset env to rerecord
            self._transition(RecState.RESET_ENV, "rerecording, reset environment", self._delete_episodes)
            self.events['rerecord_episode'] = False
            self.events['exit_early'] = False

        if self.events['exit_early']:
            if self.state == RecState.INIT:
                self._transition(RecState.RESET_ENV, "reset environment")
            elif self.state == RecState.RESET_ENV:
                self._transition(RecState.RECORDING, "starting recording")
            elif self.state == RecState.RECORDING:
                self._transition(RecState.RESET_ENV, None, self._save_episodes, 'Episode saved')
            self.events['exit_early'] = False

        if self.events['stop_recording']:
            self._transition(RecState.FINISHED, "Stop Recording", self._end_recording, None)