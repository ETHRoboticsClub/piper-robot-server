import atexit
import logging
import shutil
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import pyarrow.dataset as ds
import pyarrow.parquet as pq
import tqdm
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import DEFAULT_VIDEO_PATH, write_info
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import say
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun
from lerobot.datasets.video_utils import VideoEncodingManager

from piper_teleop.robot_server.camera.camera_config import CameraConfig

logger = logging.getLogger(__name__)

from enum import Enum, auto


def convert_image_dataset_to_video(dataset: LeRobotDataset):
    """Convert a Lerobot v3 dataset recorded with PNGs embedded in parquet to canonical video layout."""
    logger.info('Converting images to video')
    if dataset.num_episodes == 0:
        logger.info('No episodes detected')
        return

    # Snapshot image keys before mutating metadata
    image_keys = [key for key, ft in dataset.meta.info["features"].items() if ft["dtype"] == "image"]
    if not image_keys:
        logger.info('No images keys detected')
        return

    logger.info("Converting dataset at %s from image -> video for keys %s", dataset.root, image_keys)

    # Ensure metadata declares video storage going forward
    for key in image_keys:
        feature = dataset.meta.info["features"][key]
        feature["dtype"] = "video"
        feature.pop("info", None)  # will be regenerated from encoded videos
    dataset.meta.info["video_path"] = dataset.meta.info.get("video_path") or DEFAULT_VIDEO_PATH
    write_info(dataset.meta.info, dataset.meta.root)

    episodes_root = dataset.meta.root / "meta" / "episodes"
    episode_files = sorted(episodes_root.glob("chunk-*/*.parquet"))
    if not episode_files:
        logger.warning("No episode metadata files found under %s; aborting conversion.", episodes_root)
        return

    # Reset latest episode pointer so chunking logic appends deterministically
    dataset.meta.latest_episode = None
    dataset.meta.episodes = None

    # Track which parquet files need their visual columns dropped afterwards
    touched_data_files: set[Path] = set()
    dataset_cache: dict[Path, ds.Dataset] = {}

    def _get_dataset(path: Path) -> ds.Dataset:
        ds_obj = dataset_cache.get(path)
        if ds_obj is None:
            ds_obj = ds.dataset(path, format="parquet")
            dataset_cache[path] = ds_obj
        return ds_obj

    def _load_episode_frames(
        chunk_idx: int, file_idx: int, start_idx: int, end_idx: int
    ) -> dict[str, list[dict]]:
        data_path = dataset.meta.root / dataset.meta.data_path.format(
            chunk_index=chunk_idx, file_index=file_idx
        )
        if not data_path.exists():
            raise FileNotFoundError(f"Missing data parquet file needed for conversion: {data_path}")

        touched_data_files.add(data_path)
        ds_obj = _get_dataset(data_path)
        requested_cols = list(dict.fromkeys([*image_keys, "index"]))
        filter_expr = (ds.field("index") >= start_idx) & (ds.field("index") < end_idx)
        table = ds_obj.to_table(columns=requested_cols, filter=filter_expr, use_threads=True).combine_chunks()

        expected_rows = end_idx - start_idx
        if table.num_rows != expected_rows:
            raise ValueError(
                f"Expected {expected_rows} frames between indices {start_idx}:{end_idx} "
                f"but loaded {table.num_rows} rows from {data_path}"
            )

        frames_by_key = {}
        for key in image_keys:
            if key not in table.column_names:
                raise KeyError(f"Column {key} missing from data parquet during conversion.")
            frames_by_key[key] = table[key].to_pylist()
        return frames_by_key

    for ep_file in tqdm.tqdm(episode_files):
        ep_df = pd.read_parquet(ep_file)
        if ep_df.empty:
            continue

        # Pre-create columns for video metadata to keep schema consistent
        for key in image_keys:
            base = f"videos/{key}/"
            if base + "chunk_index" not in ep_df.columns:
                ep_df[base + "chunk_index"] = pd.Series([pd.NA] * len(ep_df), dtype="Int64")
            if base + "file_index" not in ep_df.columns:
                ep_df[base + "file_index"] = pd.Series([pd.NA] * len(ep_df), dtype="Int64")
            if base + "from_timestamp" not in ep_df.columns:
                ep_df[base + "from_timestamp"] = pd.Series([np.nan] * len(ep_df), dtype="float64")
            if base + "to_timestamp" not in ep_df.columns:
                ep_df[base + "to_timestamp"] = pd.Series([np.nan] * len(ep_df), dtype="float64")

        for row_idx, row in ep_df.iterrows():
            episode_index = int(row["episode_index"])
            data_chunk = int(row["data/chunk_index"])
            data_file = int(row["data/file_index"])
            start_idx = int(row["dataset_from_index"])
            end_idx = int(row["dataset_to_index"])
            frames_by_key = _load_episode_frames(data_chunk, data_file, start_idx, end_idx)

            latest_entry = {"episode_index": [episode_index]}
            for key in image_keys:
                frames = frames_by_key[key]
                if len(frames) == 0:
                    raise ValueError(f"Episode {episode_index} has no frames for key {key}")

                img_dir = dataset._get_image_file_dir(episode_index, key)
                img_dir.mkdir(parents=True, exist_ok=True)
                for local_idx, frame in enumerate(frames):
                    if not isinstance(frame, dict) or "bytes" not in frame:
                        raise ValueError(f"Unexpected frame format for {key}: {frame}")
                    fpath = dataset._get_image_file_path(episode_index, key, local_idx)
                    with open(fpath, "wb") as handle:
                        handle.write(frame["bytes"])

                metadata = dataset._save_episode_video(key, episode_index)
                base = f"videos/{key}/"
                ep_df.at[row_idx, base + "chunk_index"] = metadata[base + "chunk_index"]
                ep_df.at[row_idx, base + "file_index"] = metadata[base + "file_index"]
                ep_df.at[row_idx, base + "from_timestamp"] = metadata[base + "from_timestamp"]
                ep_df.at[row_idx, base + "to_timestamp"] = metadata[base + "to_timestamp"]

                # Update latest episode pointer so subsequent encodes append correctly
                for suffix in ("chunk_index", "file_index", "from_timestamp", "to_timestamp"):
                    latest_entry[f"{base}{suffix}"] = [metadata[f"{base}{suffix}"]]

            dataset.meta.latest_episode = latest_entry

        ep_df.to_parquet(ep_file, index=False)

    # Drop embedded image columns from parquet data now that videos are encoded
    for data_path in touched_data_files:
        schema = pq.read_schema(data_path)
        keep_cols = [name for name in schema.names if name not in image_keys]
        if len(keep_cols) == len(schema.names):
            continue
        table = pq.read_table(data_path, columns=keep_cols)
        pq.write_table(table, data_path)

    write_info(dataset.meta.info, dataset.meta.root)
    dataset.meta.episodes = None  # mark cached metadata as stale

    images_dir = dataset.root / "images"
    if images_dir.exists():
        shutil.rmtree(images_dir)


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
            convert_image_dataset_to_video(self.dataset)
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
