#!/usr/bin/env python3
"""
Script to upload LeRobot datasets to Hugging Face Hub.

This script takes a converted dataset (with videos) and uploads it to Hugging Face Hub,
making it available for sharing and use by others.

Usage:
    python upload_to_huggingface.py <dataset_path> <repo_id> [options]

Example:
    python upload_to_huggingface.py /path/to/dataset my-username/my-dataset-name
    python upload_to_huggingface.py ./data/my_dataset my-org/piper-teleop-data --private
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv
from huggingface_hub import HfApi, login
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import load_info

load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def check_hf_authentication() -> bool:
    """
    Check if user is authenticated with Hugging Face Hub.

    Returns:
        True if authenticated, False otherwise
    """
    try:
        api = HfApi()
        user = api.whoami()
        logger.info(f"Authenticated as: {user['name']}")
        return True
    except Exception as e:
        logger.error(f"Not authenticated with Hugging Face Hub: {e}")
        return False


def authenticate_hf(token: Optional[str] = None) -> bool:
    """
    Authenticate with Hugging Face Hub.

    Args:
        token: Hugging Face token. If None, will prompt user to login.

    Returns:
        True if authentication successful, False otherwise
    """
    try:
        if token:
            login(token=token)
        else:
            login()
        return check_hf_authentication()
    except Exception as e:
        logger.error(f"Authentication failed: {e}")
        return False


def validate_dataset(dataset_path: Path) -> bool:
    """
    Validate that the dataset is ready for upload.

    Args:
        dataset_path: Path to the dataset directory

    Returns:
        True if dataset is valid and ready for upload, False otherwise
    """
    try:
        # Load dataset info directly
        info = load_info(dataset_path)
        logger.info(f"Successfully loaded dataset info from: {dataset_path}")
        logger.info(f"Dataset has {info.get('total_episodes', 0)} episodes and {info.get('total_frames', 0)} frames")

        # Check if dataset has videos (should be converted)
        video_keys = []
        for key, feature_info in info.get("features", {}).items():
            if key.startswith("observation.images.") and feature_info.get("dtype") == "video":
                video_keys.append(key)

        if not video_keys:
            logger.error(
                "Dataset has no video keys. Please convert images to video first using convert_images_to_video.py"
            )
            return False

        logger.info(f"Dataset has video keys: {video_keys}")

        # Check if dataset still has image keys (shouldn't after conversion)
        image_keys = []
        for key, feature_info in info.get("features", {}).items():
            if key.startswith("observation.images.") and feature_info.get("dtype") == "image":
                image_keys.append(key)

        if image_keys:
            logger.warning(f"Dataset still has image keys: {image_keys}")
            logger.warning("This suggests the dataset may not be fully converted to video format")

        # Check if videos directory exists and has videos
        videos_dir = dataset_path / "videos"
        if not videos_dir.exists():
            logger.error("Videos directory not found. Dataset may not be properly converted.")
            return False

        # Check if there are actual video files
        video_files = list(videos_dir.rglob("*.mp4"))
        if not video_files:
            logger.error("No video files found in videos directory. Dataset may not be properly converted.")
            return False

        logger.info(f"Found {len(video_files)} video files in dataset")
        return True

    except Exception as e:
        logger.error(f"Failed to validate dataset at {dataset_path}: {e}")
        return False


def create_repo_if_not_exists(api: HfApi, repo_id: str, private: bool = False) -> bool:
    """
    Create a repository on Hugging Face Hub if it doesn't exist.

    Args:
        api: Hugging Face API instance
        repo_id: Repository ID (e.g., "username/dataset-name")
        private: Whether to create a private repository

    Returns:
        True if repository exists or was created successfully, False otherwise
    """
    try:
        # Check if repo exists
        try:
            api.repo_info(repo_id, repo_type="dataset")
            logger.info(f"Repository {repo_id} already exists")
            return True
        except Exception as e:
            # Repository doesn't exist, create it
            logger.info(f"Creating repository {repo_id}...")
            try:
                api.create_repo(repo_id=repo_id, repo_type="dataset", private=private)
                logger.info(f"Successfully created repository {repo_id}")
                return True
            except Exception as create_error:
                if "409" in str(create_error) and "Conflict" in str(create_error):
                    # Repository already exists (race condition)
                    logger.info(f"Repository {repo_id} already exists (created by another process)")
                    return True
                else:
                    raise create_error

    except Exception as e:
        logger.error(f"Failed to create repository {repo_id}: {e}")
        return False


def upload_dataset(
    dataset_path: Path, repo_id: str, private: bool = False, commit_message: Optional[str] = None
) -> bool:
    """
    Upload dataset to Hugging Face Hub.

    Args:
        dataset_path: Path to the dataset directory
        repo_id: Repository ID on Hugging Face Hub
        private: Whether the repository should be private
        commit_message: Custom commit message for the upload

    Returns:
        True if upload successful, False otherwise
    """
    try:
        api = HfApi()

        # Create repository if it doesn't exist
        if not create_repo_if_not_exists(api, repo_id, private):
            return False

        # Prepare commit message
        if not commit_message:
            commit_message = f"Upload LeRobot dataset: {dataset_path.name}"

        logger.info(f"Starting upload of {dataset_path} to {repo_id}...")
        logger.info("This may take a while depending on dataset size...")

        # Upload the entire dataset directory
        api.upload_folder(
            folder_path=dataset_path,
            repo_id=repo_id,
            repo_type="dataset",
            commit_message=commit_message,
        )

        logger.info(f"Successfully uploaded dataset to: https://huggingface.co/datasets/{repo_id}")
        return True

    except Exception as e:
        logger.error(f"Upload failed: {e}")
        return False


def main():
    """Main function to handle command line arguments and run upload."""
    parser = argparse.ArgumentParser(
        description="Upload LeRobot dataset to Hugging Face Hub",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python upload_to_huggingface.py /path/to/dataset my-username/my-dataset-name
    python upload_to_huggingface.py ./data/my_dataset my-org/piper-teleop-data --private
    python upload_to_huggingface.py ./data/my_dataset my-username/dataset --token hf_xxxxx
        """,
    )

    parser.add_argument("dataset_path", type=str, help="Path to the dataset directory to upload")

    parser.add_argument("repo_id", type=str, help="Repository ID on Hugging Face Hub (e.g., 'username/dataset-name')")

    parser.add_argument("--private", action="store_true", help="Create a private repository (default: public)")

    parser.add_argument(
        "--token", type=str, help="Hugging Face token for authentication (optional, will prompt if not provided)"
    )

    parser.add_argument("--commit-message", type=str, help="Custom commit message for the upload")

    parser.add_argument("--dry-run", action="store_true", help="Show what would be uploaded without actually doing it")

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

    # Validate repository ID format
    if "/" not in args.repo_id:
        logger.error("Repository ID must be in format 'username/dataset-name'")
        sys.exit(1)

    if args.dry_run:
        logger.info("DRY RUN MODE - No changes will be made")
        logger.info(f"Would upload dataset from: {dataset_path}")
        logger.info(f"Would upload to repository: {args.repo_id}")
        logger.info(f"Repository type: {'private' if args.private else 'public'}")
        return

    # Validate dataset
    if not validate_dataset(dataset_path):
        sys.exit(1)

    # Check authentication
    if not check_hf_authentication():
        logger.info("Authentication required for Hugging Face Hub")
        if not authenticate_hf(args.token):
            logger.error("Authentication failed. Please check your token or run 'huggingface-cli login'")
            sys.exit(1)

    # Perform upload
    try:
        success = upload_dataset(
            dataset_path=dataset_path, repo_id=args.repo_id, private=args.private, commit_message=args.commit_message
        )

        if success:
            logger.info("Dataset upload completed successfully!")
            logger.info(f"View your dataset at: https://huggingface.co/datasets/{args.repo_id}")
        else:
            logger.error("Dataset upload failed")
            sys.exit(1)

    except KeyboardInterrupt:
        logger.info("Upload cancelled by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error during upload: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
