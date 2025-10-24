import numpy as np

from piper_teleop.robot_server.camera import CameraConfig, CameraMode
from piper_teleop.robot_server.camera.camera_shared_data import SingleCameraSharedData, SharedCameraData


def test_single_camera_data():
    mem = SingleCameraSharedData(name='wrist', frame_shape=(480, 640, 3), capacity=2)
    img_got = mem.read()
    assert img_got.mean() == 0.0
    assert mem.write_counter.value == 0

    img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
    mem.write(img)
    img_got = mem.read()
    assert img_got.mean() > 0
    assert mem.write_counter.value == 1
    np.testing.assert_equal(img, img_got)

    img = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
    mem.write(img)
    img_got = mem.read()
    assert img_got.mean() > 0
    assert mem.write_counter.value == 0
    np.testing.assert_equal(img, img_got)

def test_shared_camera_data():
    cfg_1 = CameraConfig('cam_1', mode=CameraMode.RECORDING)
    cfg_2 = CameraConfig('cam_2', mode=CameraMode.HYBRID)
    cfg_3 = CameraConfig('cam_3', mode=CameraMode.STREAMING)
    shared = SharedCameraData([cfg_1, cfg_2, cfg_3],)
    img_1 = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
    img_2 = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
    shared.copy('cam_1', img_1,)
    shared.copy('cam_2', img_2,)
    cam_dict = shared.get_camera_dict()
    np.testing.assert_equal(cam_dict['observation.images.cam_1'], img_1)
    np.testing.assert_equal(cam_dict['observation.images.cam_2'], img_2)
    assert len(cam_dict) == 2



