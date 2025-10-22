import hashlib
import time
from unittest.mock import MagicMock

import numpy as np
import pyarrow.parquet as pq
import pytest
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.datasets.utils import load_info

from piper_teleop.robot_server.camera.camera_config import CameraConfig
from piper_teleop.robot_server.recorder import Recorder, RecState, convert_image_dataset_to_video


def snapshot_files(root, exclude_patterns=None):
    """Create SHA256 checksums for all files, optionally excluding patterns."""
    exclude_patterns = exclude_patterns or []
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file() and not any(pattern in str(path) for pattern in exclude_patterns)
    }


def compare_parquet_files(path1, path2):
    """
    Compare two parquet files to ensure they're structurally identical.

    This includes checking that image/video columns have the same format:
    - Video datasets should have {path, timestamp} structure
    - Both datasets should have the same column structure
    """
    table1 = pq.read_table(path1)
    table2 = pq.read_table(path2)

    # Compare number of rows
    if len(table1) != len(table2):
        print(f"Row count mismatch: {len(table1)} vs {len(table2)}")
        return False

    # Compare all columns (including visual data)
    if set(table1.schema.names) != set(table2.schema.names):
        print(f"Column names mismatch: {table1.schema.names} vs {table2.schema.names}")
        return False

    # Compare data in each column
    for col_name in table1.schema.names:
        col1 = table1.column(col_name).to_pylist()
        col2 = table2.column(col_name).to_pylist()

        # For image/video columns, check the structure
        if col_name.startswith("observation.images."):
            # Both should have the same structure
            if len(col1) != len(col2):
                print(f"Column {col_name}: different lengths")
                return False

            # Check first element structure
            if len(col1) > 0:
                elem1 = col1[0]
                elem2 = col2[0]

                # Check if both have the same keys (should be {path, timestamp} for video)
                if isinstance(elem1, dict) and isinstance(elem2, dict):
                    if set(elem1.keys()) != set(elem2.keys()):
                        print(f"Column {col_name}: structure mismatch")
                        print(f"  Dataset 1 keys: {elem1.keys()}")
                        print(f"  Dataset 2 keys: {elem2.keys()}")
                        return False

                    # For video format, should have 'path' and 'timestamp' keys
                    if "path" not in elem1 or "timestamp" not in elem1:
                        print(f"Column {col_name}: not in video format (missing path/timestamp)")
                        print(f"  Keys: {elem1.keys()}")
                        return False
        else:
            # For non-image columns, data should match exactly
            if col1 != col2:
                print(f"Column {col_name}: data mismatch")
                return False

    return True


def test_create_temp_dataset_and_read(tmp_path):
    repo_id = "test"
    root = tmp_path / "lerobot"
    features = dict()
    features["action"] = {
        "dtype": "float32",
        "shape": (7,),
        "names": [f"joint_{i}" for i in range(7)],
    }
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        features=features,
        fps=30,
        robot_type="piper",
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=4,
    )
    frame = {
        "action": np.random.random(7).astype(np.float32),
    }
    dataset.add_frame(frame, task="pick and place")
    dataset.save_episode()
    # Reading
    ds_meta = LeRobotDatasetMetadata(repo_id=repo_id, root=root)
    delta_timestamps = {"action": [t / ds_meta.fps for t in range(5)]}

    dataset = LeRobotDataset(repo_id=repo_id, root=root, delta_timestamps=delta_timestamps)
    data = dataset[-1]
    assert torch.equal(data["action"][-1], torch.from_numpy(frame["action"]))


def test_init_recorder():
    rec = Recorder(repo_id="test", task="test", use_video=True, play_sound=False)
    assert rec.joints == [f"L.joint_{i}" for i in range(7)] + [f"R.joint_{i}" for i in range(7)]
    expected_dict = {
        "dtype": "float32",
        "shape": (14,),
        "names": [f"L.joint_{i}" for i in range(7)] + [f"R.joint_{i}" for i in range(7)],
    }

    assert rec.features["action"] == expected_dict
    assert rec.features["observation.state"] == expected_dict

    rec = Recorder(repo_id="test", single_arm=True, task="test", play_sound=False)
    expected_dict = {
        "dtype": "float32",
        "shape": (7,),
        "names": [f"joint_{i}" for i in range(7)],
    }
    assert rec.joints == [f"joint_{i}" for i in range(7)]
    assert rec.features["action"] == expected_dict
    assert rec.features["observation.state"] == expected_dict

    cameras = [CameraConfig(name="front", frame_width=740, frame_height=480)]
    rec = Recorder(repo_id="test", single_arm=True, cameras=cameras, task="test", use_video=False, play_sound=False)
    assert rec.features["observation.images.front"] == {
        "dtype": "image",
        "shape": (480, 740, 3),
        "names": ["height", "width", "channels"],
    }


def test_create_dataset(tmp_path):
    root = tmp_path / "lerobot"
    rec = Recorder(repo_id="test", root=root, single_arm=True, task="test", play_sound=False)
    assert root.exists() == False
    rec._create_dataset()
    frame = {
        "action": np.random.random(7).astype(np.float32),
        "observation.state": np.random.random(7).astype(np.float32),
    }
    rec.dataset.add_frame(frame, task="pick and place")
    assert root.exists() == True


def test_rerecord(tmp_path):
    root = tmp_path / "lerobot"
    cameras = [CameraConfig(name="left", frame_width=640, frame_height=480)]
    rec = Recorder(repo_id="test", root=root, single_arm=False, play_sound=False, task="test", cameras=cameras)
    rec.start_recording()
    rec.state = RecState.RECORDING
    for i in range(20):
        time.sleep(1 / 30)
        img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
        left_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        left_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        rec.add_observation(
            left_joints=left_joints,
            right_joints=right_joints,
            left_joints_target=left_joints_target,
            right_joints_target=right_joints_target,
            cams={"observation.images.left": img},
        )

    rec.events["rerecord_episode"] = True
    rec.events["exit_early"] = True
    rec.handle_keyboard_event()
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events["rerecord_episode"] == False
    assert rec.events["exit_early"] == False


def test_recorder_state(tmp_path):
    root = tmp_path / "lerobot"
    rec = Recorder(repo_id="test", root=root, single_arm=True, play_sound=False, task="test")
    rec.dataset = MagicMock()
    assert rec.events == {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    assert rec.state == RecState.INIT
    rec.events["exit_early"] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events["exit_early"] == False

    rec.events["exit_early"] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RECORDING
    assert rec.events["exit_early"] == False

    rec.events["exit_early"] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events["exit_early"] == False

    rec.events["rerecord_episode"] = True
    rec.state = RecState.RECORDING
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events["exit_early"] == False

    rec.events["stop_recording"] = True
    rec.state = RecState.RECORDING
    rec.handle_keyboard_event()
    assert rec.state == RecState.FINISHED


def test_converting_to_video(tmp_path):
    root = tmp_path / "lerobot"
    cameras = [CameraConfig(name="left", frame_width=640, frame_height=480)]
    rec = Recorder(repo_id="test", root=root, single_arm=False, play_sound=False, task="test", cameras=cameras)
    root_2 = tmp_path / "lerobot_2"
    rec_vid = Recorder(
        repo_id="test",
        root=root_2,
        single_arm=False,
        play_sound=False,
        task="test",
        cameras=cameras,
        use_video=True,
    )
    rec.start_recording()
    rec_vid.start_recording()
    NUM_EP = 2
    NUM_FRAMES = 5

    np.random.seed(42)

    for _ in range(NUM_EP):
        rec.state = RecState.RECORDING
        rec_vid.state = RecState.RECORDING
        for _ in range(NUM_FRAMES):
            img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
            left_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
            right_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
            left_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
            right_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
            rec.add_observation(
                left_joints=left_joints,
                right_joints=right_joints,
                left_joints_target=left_joints_target,
                right_joints_target=right_joints_target,
                cams={"observation.images.left": img},
            )
            rec_vid.add_observation(
                left_joints=left_joints,
                right_joints=right_joints,
                left_joints_target=left_joints_target,
                right_joints_target=right_joints_target,
                cams={"observation.images.left": img},
            )
        rec.events["exit_early"] = True
        rec.handle_keyboard_event()
        rec_vid.events["exit_early"] = True
        rec_vid.handle_keyboard_event()

    # Verify initial state
    assert len(list((root / "images" / "observation.images.left" / "episode_000000").iterdir())) == NUM_FRAMES
    assert len(list((root / "images" / "observation.images.left").iterdir())) == NUM_EP
    # Video dataset may have images directory but no PNG files
    assert len(list(root_2.rglob("*.png"))) == 0
    assert rec.dataset.num_episodes == NUM_EP
    assert rec_vid.dataset.num_episodes == NUM_EP

    convert_image_dataset_to_video(rec.dataset)

    # Both should have videos, data, and meta directories
    # images directory may or may not exist (if it exists, it should be empty)
    assert {f.name for f in root.iterdir() if f.name != "images"} == {
        f.name for f in root_2.iterdir() if f.name != "images"
    }
    assert len(list(root.rglob("*.png"))) == 0
    assert len(list(root.rglob("*.mp4"))) == len(list(root_2.rglob("*.mp4")))
    assert rec.dataset.meta.video_keys == rec_vid.dataset.meta.video_keys
    assert rec.dataset.meta.video_path == rec_vid.dataset.meta.video_path
    assert rec.dataset.meta.info["total_videos"] == rec_vid.dataset.meta.info["total_videos"]

    converted_info = load_info(root)
    reference_info = load_info(root_2)
    assert converted_info == reference_info

    # Compare non-parquet files with checksums (they should be byte-identical)
    files_rec = snapshot_files(root, exclude_patterns=[".parquet"])
    files_rec_vid = snapshot_files(root_2, exclude_patterns=[".parquet"])
    assert files_rec == files_rec_vid, "Non-parquet files don't match!"

    # Compare parquet files by data content (checksums may differ due to metadata)
    parquet_rec = sorted(root.rglob("*.parquet"))
    parquet_rec_vid = sorted(root_2.rglob("*.parquet"))
    assert len(parquet_rec) == len(
        parquet_rec_vid
    ), f"Different number of parquet files: {len(parquet_rec)} vs {len(parquet_rec_vid)}"

    for pf_rec in parquet_rec:
        rel_path = pf_rec.relative_to(root)
        pf_rec_vid = root_2 / rel_path
        assert pf_rec_vid.exists(), f"Missing parquet file: {rel_path}"
        assert compare_parquet_files(pf_rec, pf_rec_vid), f"Parquet data mismatch in {rel_path}"


@pytest.mark.skip  # manual test
def test_benchmark_lerobot(tmp_path):
    """Benchmark test for LeRobot dataset operations."""
    root = tmp_path / "lerobot"
    cameras = [CameraConfig(name="left", frame_width=640, frame_height=480)]
    rec = Recorder(
        repo_id="test",
        root=root,
        single_arm=False,
        play_sound=False,
        task="test",
        image_writer_processes=8,
        image_writer_threads=16,
        cameras=cameras,
    )
    rec.state = RecState.RECORDING
    rec.start_recording()

    start_frame = time.perf_counter()
    for i in range(500):
        img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
        left_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        left_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints_target = {f"joint_{j}.pos": np.random.uniform(-np.pi, np.pi) for j in range(7)}
        rec.add_observation(
            left_joints=left_joints,
            right_joints=right_joints,
            left_joints_target=left_joints_target,
            right_joints_target=right_joints_target,
            cams={"observation.images.left": img},
        )
    end_time = time.perf_counter() - start_frame

    start_save = time.perf_counter()
    rec.dataset.save_episode()
    save_time = time.perf_counter() - start_save

    print(f"  Mean time per frame: {end_time*1000:.2f} ms")
    print(f"  Save episode time:   {save_time:.2f} s")
