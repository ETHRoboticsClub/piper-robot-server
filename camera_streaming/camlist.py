# Note: works only on Linux
import cv2
from cv2_enumerate_cameras import enumerate_cameras

def list_cameras():
    """List all the available cameras on the system for debugging purposes"""
    print("\n=== CAMERA DEVICES DEBUG ===")
    for camera_info in enumerate_cameras(apiPreference=cv2.CAP_V4L2):
        print(f"Camera {camera_info.index}: {camera_info.name}")
    print("===========================\n")


if __name__ == "__main__":
    list_cameras()