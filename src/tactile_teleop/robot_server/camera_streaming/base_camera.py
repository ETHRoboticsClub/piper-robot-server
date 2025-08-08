from abc import ABC, abstractmethod


class BaseCamera(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def init_camera(self):
        raise NotImplementedError

    @abstractmethod
    async def capture_frame(self):
        raise NotImplementedError

    @abstractmethod
    def stop_camera(self):
        raise NotImplementedError
