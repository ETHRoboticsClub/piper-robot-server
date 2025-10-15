import time
from unittest.mock import MagicMock

import torch
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata

from piper_teleop.robot_server.recorder import Recorder, RecState


def test_create_temp_dataset_and_read(tmp_path):
    repo_id = 'test'
    root =  tmp_path / 'lerobot'
    features = dict()
    features['action'] = {
        "dtype": "float32",
        "shape": (7,),
        "names": [f"joint_{i}" for i in range(7)],
    }
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        features=features,
        fps=30,
        robot_type='piper',
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=4,
    )
    frame  = {'action': np.random.random(7).astype(np.float32),
              }
    dataset.add_frame(frame, task='pick and place')
    dataset.save_episode()
    # Reading
    ds_meta = LeRobotDatasetMetadata(repo_id=repo_id, root=root)
    delta_timestamps = {
        'action': [t / ds_meta.fps for t in range(5)]
    }

    dataset = LeRobotDataset(
        repo_id=repo_id,
        root=root,
        delta_timestamps=delta_timestamps
    )
    data = dataset[-1]
    assert torch.equal(data['action'][-1], torch.from_numpy(frame['action']))

def test_init_recorder():
    rec = Recorder(repo_id='test', task='test')
    assert rec.joints ==[f"L.joint_{i}" for i in range(7)] + [f"R.joint_{i}" for i in range(7)]
    expected_dict  = {"dtype": "float32",
                    "shape": (14,),
                    "names": [f"L.joint_{i}" for i in range(7)] + [f"R.joint_{i}" for i in range(7)]}

    assert rec.features['action'] == expected_dict
    assert rec.features['observation.state'] == expected_dict

    rec = Recorder(repo_id='test', single_arm=True, task='test')
    expected_dict = {"dtype": "float32",
                     "shape": (7,),
                     "names": [f"joint_{i}" for i in range(7)],
                     }
    assert rec.joints ==[f"joint_{i}" for i in range(7)]
    assert rec.features['action'] == expected_dict
    assert rec.features['observation.state'] == expected_dict

    rec = Recorder(repo_id='test', single_arm=True, cams={'front': (480, 740, 3)}, task='test')
    assert rec.features['observation.images.front'] == {"dtype": "video",
                                                        "shape": (480,740,3),
                                                        "names": ["height", "width", "channels"],
                                                }

def test_create_dataset(tmp_path):
    root = tmp_path / 'lerobot'
    rec = Recorder(repo_id='test',root=root, single_arm=True, task='test')
    assert root.exists() == False
    rec._create_dataset()
    frame  = {'action': np.random.random(7).astype(np.float32),
              'observation.state': np.random.random(7).astype(np.float32),
              }
    rec.dataset.add_frame(frame, task='pick and place')
    assert root.exists() == True


def test_recorder_state(tmp_path):
    root = tmp_path / 'lerobot'
    rec = Recorder(repo_id='test', root=root, single_arm=True, play_sound=False, task='test')
    rec.dataset = MagicMock()
    assert rec.events == {'exit_early': False,
                          'rerecord_episode': False,
                          'stop_recording': False}

    assert rec.state == RecState.INIT
    rec.events['exit_early'] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events['exit_early'] == False

    rec.events['exit_early'] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RECORDING
    assert rec.events['exit_early'] == False

    rec.events['exit_early'] = True
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events['exit_early'] == False

    rec.events['rerecord_episode'] = True
    rec.state = RecState.RECORDING
    rec.handle_keyboard_event()
    assert rec.state == RecState.RESET_ENV
    assert rec.events['exit_early'] == False


@pytest.mark.skip # manual test
def test_benchmark_lerobot(tmp_path):
    """Benchmark test for LeRobot dataset operations."""
    root = tmp_path / 'lerobot'
    rec = Recorder(repo_id='test', root=root,
                   single_arm=False,
                   play_sound=False, task='test',
                   image_writer_processes=8,
                   image_writer_threads=16,
                   cams={'left': (480, 640, 3),
                         }
                   )
    rec.state = RecState.RECORDING
    rec.start_recording()

    start_frame = time.perf_counter()
    for i in range(500):
        img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
        left_joints = {f'joint_{j}.pos': np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints = {f'joint_{j}.pos': np.random.uniform(-np.pi, np.pi) for j in range(7)}
        left_joints_target = {f'joint_{j}.pos': np.random.uniform(-np.pi, np.pi) for j in range(7)}
        right_joints_target = {f'joint_{j}.pos': np.random.uniform(-np.pi, np.pi) for j in range(7)}
        rec.add_observation(left_joints=left_joints,
                            right_joints=right_joints,
                            left_joints_target=left_joints_target,
                            right_joints_target=right_joints_target,
                            cams={'observation.images.left': img})
    end_time = time.perf_counter() - start_frame

    start_save = time.perf_counter()
    rec.dataset.save_episode()
    save_time = time.perf_counter() - start_save


    print(f"  Mean time per frame: {end_time*1000:.2f} ms")
    print(f"  Save episode time:   {save_time:.2f} s")





