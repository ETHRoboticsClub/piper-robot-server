import logging
import time
from io import BytesIO
from pathlib import Path

import PIL
import numpy as np
import pyarrow.dataset as ds
import pyarrow.parquet as pq
import tqdm
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata

logger = logging.getLogger(__name__)
def _replace_dict_str(data, a, b):
    if isinstance(data, str):
        return data.replace(a, b)
    elif isinstance(data, dict):
        return {k: _replace_dict_str(v, a, b) for k, v in data.items()}
    elif isinstance(data, list):
        return [_replace_dict_str(v, a, b) for v in data]
    else:
        # nothing to do?
        return data
def convert_image_dataset_to_video(root: Path | str, batch_parquet=50, image_writer_processes=0, image_writer_threads=12):
    path = Path(root)
    ds_meta = LeRobotDatasetMetadata(repo_id='', root=path)
    features_new = _replace_dict_str(ds_meta.features, 'image', 'video')
    new_dataset = LeRobotDataset.create(
        repo_id=ds_meta.repo_id,
        root=path.with_name(path.name + '_video'),
        features=features_new,
        fps=ds_meta.fps,
        robot_type=ds_meta.robot_type,
        use_videos=True,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
    )
    ds_dataset = ds.dataset(path / 'data')
    parquet_files = sorted(ds_dataset.files)
    ep_index = 0
    glob_index = 0
    with tqdm.tqdm(total=ds_meta.total_episodes, desc='converting episodes') as pbar:
        for parquet_file in parquet_files:
            parquet_file_reader = pq.ParquetFile(parquet_file)
            for batch in parquet_file_reader.iter_batches(batch_size=batch_parquet):
                for i in range(len(batch)):
                    row = {col: batch[col][i].as_py() for col in batch.column_names}
                    if ep_index < row['episode_index']:
                        ep_index = row['episode_index']
                        new_dataset.save_episode()
                        frame_count_in_episode = 0
                        pbar.update(1)
                    task = ds_meta.tasks.index[row['task_index']]
                    obs_action = {'action': np.array(row['action'], dtype=np.float32),
                                  'observation.state': np.array(row['observation.state'], dtype=np.float32),
                                  'task': task}
                    cam = {k: np.array(PIL.Image.open(BytesIO(v['bytes']))) for k, v in row.items() if 'observation.images' in k}
                    frame = {**obs_action, **cam}
                    new_dataset.add_frame(frame)
                    assert glob_index == row['index'], 'indexing error, this should not happen we iterate sequentially'
                    glob_index += 1
    new_dataset.save_episode()
    new_dataset.finalize()

    logger.info(f"Converting image dataset to video format: {path}")
    logger.info("Video conversion completed successfully!")