import cv2
from cv2_enumerate_cameras import enumerate_cameras

def list_cameras():
    """
    List all the available cameras on the system for debugging purposes.
    
    Note: This function works only on Linux, as it relies on the V4L2 (Video for Linux 2) API.
    """
    print("\n=== CAMERA DEVICES DEBUG ===")
    for camera_info in enumerate_cameras(apiPreference=cv2.CAP_V4L2):
        print(f"Camera {camera_info.index}: {camera_info.name}")
    print("===========================\n")


if __name__ == "__main__":
    list_cameras()