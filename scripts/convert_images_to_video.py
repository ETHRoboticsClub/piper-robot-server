import argparse
import logging
from pathlib import Path

from piper_teleop.robot_server.recorder_utils import convert_image_dataset_to_video

# Force logging configuration after imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    force=True  # Python 3.8+ - forces reconfiguration
)


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

    dataset_path = Path(args.dataset_path).expanduser().resolve()

    if args.dry_run:
        print("DRY RUN MODE - No changes will be made")
        print(f"Would convert dataset at: {dataset_path}")
        return
    convert_image_dataset_to_video(dataset_path)



if __name__ == "__main__":
    logging.getLogger('piper_teleop.robot_server.recorder').setLevel(logging.INFO)
    main()
