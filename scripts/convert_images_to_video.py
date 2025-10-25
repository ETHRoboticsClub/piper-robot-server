#!/usr/bin/env python3
"""
Script to convert LeRobot datasets from image format to video format.

This script takes a dataset directory where images were recorded as individual frames
and converts them to the canonical video format used by LeRobot, following the same
process as in recorder.py.

Usage:
    python convert_images_to_video.py <dataset_path>

Example:
    python convert_images_to_video.py /path/to/dataset
"""

import argparse
import logging
import shutil
import sys
from pathlib import Path
from typing import Optional

import pyarrow.parquet as pq
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.datasets.utils import DEFAULT_VIDEO_PATH, load_info, write_info
from lerobot.datasets.video_utils import encode_video_frames

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def convert_image_dataset_to_video(dataset_path: Path) -> None:
    """
    Convert a dataset recorded with image frames into the canonical video layout.

    This function follows the same process as in recorder.py:
    1. Update metadata to video format
    2. Encode videos for each episode
    3. Regenerate parquet files without image columns
    4. Clean up images directory

    Args:
        dataset_path: Path to the dataset directory
    """
    # Load dataset info directly
    info = load_info(dataset_path)

    if info.get("total_episodes", 0) == 0:
        logger.warning("Dataset has no episodes to convert")
        return

    logger.info("Converting images to video format...")

    # Find image keys from features (either dtype="image" or images still exist)
    image_keys = []
    for key, feature_info in info.get("features", {}).items():
        if key.startswith("observation.images.") and feature_info.get("dtype") == "image":
            image_keys.append(key)

    # Also check for video keys that might need encoding (metadata says video but no actual videos exist)
    video_keys_needing_encoding = []
    for key, feature_info in info.get("features", {}).items():
        if key.startswith("observation.images.") and feature_info.get("dtype") == "video":
            # Check if images still exist for this key
            key_img_dir = dataset_path / "images" / key
            if key_img_dir.exists() and key_img_dir.is_dir():
                # Check if there are any episode directories with images
                has_images = False
                for episode_dir in key_img_dir.iterdir():
                    if episode_dir.is_dir() and episode_dir.name.startswith("episode_"):
                        if list(episode_dir.glob("*.png")):
                            has_images = True
                            break
                if has_images:
                    video_keys_needing_encoding.append(key)

    # Combine both types of keys that need processing
    keys_to_process = image_keys + video_keys_needing_encoding

    if not keys_to_process:
        logger.warning("No image keys found in dataset metadata and no images to convert")
        return

    # Use the keys that need processing
    image_keys = keys_to_process

    logger.info(f"Found image keys: {image_keys}")

    # Step 1: Update metadata to video format
    logger.info("Step 1: Updating metadata to video format...")
    for key in image_keys:
        info["features"][key]["dtype"] = "video"
    info["video_path"] = DEFAULT_VIDEO_PATH
    write_info(info, dataset_path)

    # Step 2: Encode videos for each episode
    logger.info(f"Step 2: Encoding videos for {info['total_episodes']} episodes...")

    # Get FPS from dataset info
    fps = info.get("fps", 30)

    # Create videos directory structure
    videos_dir = dataset_path / "videos"
    videos_dir.mkdir(exist_ok=True)

    for episode_index in range(info["total_episodes"]):
        logger.info(f"Encoding episode {episode_index + 1}/{info['total_episodes']}")

        for image_key in image_keys:
            # Create video path
            chunk_index = episode_index // info.get("chunks_size", 1000)
            chunk_dir = videos_dir / f"chunk-{chunk_index:03d}"
            chunk_dir.mkdir(exist_ok=True)

            video_key_dir = chunk_dir / image_key
            video_key_dir.mkdir(exist_ok=True)

            video_path = video_key_dir / f"episode_{episode_index:06d}.mp4"

            # Skip if video already exists
            if video_path.exists():
                logger.info(f"Video already exists: {video_path}")
                continue

            # Find the image directory for this episode and image key
            images_dir = dataset_path / "images"
            if not images_dir.exists():
                logger.warning(f"Images directory not found: {images_dir}")
                continue

            # Look for the image directory structure
            img_dir = None

            # First try the direct structure (image_key/episode_XXXXXX/)
            key_img_dir = images_dir / image_key
            if key_img_dir.exists() and key_img_dir.is_dir():
                episode_dir = key_img_dir / f"episode_{episode_index:06d}"
                if episode_dir.exists() and episode_dir.is_dir():
                    # Check if this directory contains images
                    episode_files = list(episode_dir.glob("*.png"))
                    if episode_files:
                        img_dir = episode_dir
                        logger.info(f"Found images in direct structure: {img_dir}")

            # If not found, try the chunk structure
            if not img_dir:
                for chunk_img_dir in images_dir.iterdir():
                    if chunk_img_dir.is_dir() and chunk_img_dir.name.startswith("chunk-"):
                        for key_img_dir in chunk_img_dir.iterdir():
                            if key_img_dir.is_dir() and key_img_dir.name == image_key:
                                # Check if this directory contains images for our episode
                                episode_files = list(key_img_dir.glob(f"episode_{episode_index:06d}_*.png"))
                                if episode_files:
                                    img_dir = key_img_dir
                                    logger.info(f"Found images in chunk structure: {img_dir}")
                                    break
                        if img_dir:
                            break

            if not img_dir:
                logger.warning(f"No images found for episode {episode_index} and key {image_key}")
                continue

            # Encode video from images
            try:
                logger.info(f"Encoding video: {video_path}")
                encode_video_frames(img_dir, video_path, fps, overwrite=True)
                logger.info(f"Successfully encoded video: {video_path}")
            except Exception as e:
                logger.error(f"Failed to encode video {video_path}: {e}")
                continue

    # Step 3: Regenerate parquet files without image columns
    logger.info("Step 3: Regenerating parquet files without image columns...")

    # Find parquet files
    data_dir = dataset_path / "data"
    if data_dir.exists():
        for chunk_dir in data_dir.iterdir():
            if chunk_dir.is_dir() and chunk_dir.name.startswith("chunk-"):
                for parquet_file in chunk_dir.glob("*.parquet"):
                    logger.info(f"Processing parquet file: {parquet_file}")

                    # Load existing parquet data
                    table = pq.read_table(parquet_file)

                    # Remove image columns (they're now in videos)
                    columns_to_keep = [col for col in table.schema.names if not col.startswith("observation.images.")]
                    table_without_images = table.select(columns_to_keep)

                    # Save back to parquet
                    pq.write_table(table_without_images, parquet_file)
                    logger.info(f"Updated parquet file: {parquet_file}")

    # Update total videos count
    info["total_videos"] = info["total_episodes"] * len(image_keys)
    write_info(info, dataset_path)

    # Step 4: Clean up encoded image directories
    logger.info("Step 4: Cleaning up encoded image directories...")
    images_dir = dataset_path / "images"
    if images_dir.exists():
        # Remove image directories for keys that were encoded
        # Handle both direct structure (images/key/) and chunk structure (images/chunk-XXX/key/)
        for item in images_dir.iterdir():
            if item.is_dir():
                if item.name.startswith("chunk-"):
                    # Chunk-based structure
                    for key_img_dir in item.iterdir():
                        if key_img_dir.is_dir() and key_img_dir.name in image_keys:
                            logger.info(f"Removing encoded image directory: {key_img_dir}")
                            shutil.rmtree(key_img_dir)
                elif item.name in image_keys:
                    # Direct structure
                    logger.info(f"Removing encoded image directory: {item}")
                    shutil.rmtree(item)

        # Check if images directory is now empty and remove it
        if images_dir.exists() and not any(images_dir.iterdir()):
            logger.info(f"Removing empty images directory: {images_dir}")
            shutil.rmtree(images_dir)
    else:
        logger.info("No images directory found to clean up")

    logger.info("Conversion completed successfully!")


def main():
    """Main function to handle command line arguments and run conversion."""
    parser = argparse.ArgumentParser(
        description="Convert LeRobot dataset from image format to video format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python convert_images_to_video.py /path/to/dataset
    python convert_images_to_video.py ./data/my_dataset
        """,
    )

    parser.add_argument("dataset_path", type=str, help="Path to the dataset directory to convert")

    parser.add_argument("--dry-run", action="store_true", help="Show what would be converted without actually doing it")

    args = parser.parse_args()

    # Convert to Path object
    dataset_path = Path(args.dataset_path).resolve()

    # Validate dataset path
    if not dataset_path.exists():
        logger.error(f"Dataset path does not exist: {dataset_path}")
        sys.exit(1)

    if not dataset_path.is_dir():
        logger.error(f"Dataset path is not a directory: {dataset_path}")
        sys.exit(1)

    # Check for required files
    info_file = dataset_path / "meta" / "info.json"
    if not info_file.exists():
        logger.error(f"Dataset info file not found: {info_file}")
        logger.error("This doesn't appear to be a valid LeRobot dataset")
        sys.exit(1)

    if args.dry_run:
        logger.info("DRY RUN MODE - No changes will be made")
        logger.info(f"Would convert dataset at: {dataset_path}")
        return

    # Load dataset info to check if conversion is needed
    try:
        info = load_info(dataset_path)
        logger.info(f"Successfully loaded dataset info from: {dataset_path}")
        logger.info(f"Dataset has {info.get('total_episodes', 0)} episodes and {info.get('total_frames', 0)} frames")
    except Exception as e:
        logger.error(f"Failed to load dataset info from {dataset_path}: {e}")
        sys.exit(1)

    # Check if dataset has image keys (either dtype="image" or images still exist)
    image_keys = []
    for key, feature_info in info.get("features", {}).items():
        if key.startswith("observation.images.") and feature_info.get("dtype") == "image":
            image_keys.append(key)

    # Also check for video keys that might need encoding (metadata says video but no actual videos exist)
    video_keys_needing_encoding = []
    for key, feature_info in info.get("features", {}).items():
        if key.startswith("observation.images.") and feature_info.get("dtype") == "video":
            # Check if images still exist for this key
            key_img_dir = dataset_path / "images" / key
            if key_img_dir.exists() and key_img_dir.is_dir():
                # Check if there are any episode directories with images
                has_images = False
                for episode_dir in key_img_dir.iterdir():
                    if episode_dir.is_dir() and episode_dir.name.startswith("episode_"):
                        if list(episode_dir.glob("*.png")):
                            has_images = True
                            break
                if has_images:
                    video_keys_needing_encoding.append(key)

    # Combine both types of keys that need processing
    keys_to_process = image_keys + video_keys_needing_encoding

    if not keys_to_process:
        logger.warning("Dataset has no image keys and no images to convert. Nothing to convert.")
        return

    if video_keys_needing_encoding:
        logger.info(f"Found video keys with existing images that need encoding: {video_keys_needing_encoding}")

    if image_keys:
        logger.info(f"Found image keys that need conversion: {image_keys}")

    # Use the keys that need processing
    image_keys = keys_to_process

    # Perform conversion
    try:
        convert_image_dataset_to_video(dataset_path)
        logger.info("Dataset conversion completed successfully!")
    except Exception as e:
        logger.error(f"Conversion failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
