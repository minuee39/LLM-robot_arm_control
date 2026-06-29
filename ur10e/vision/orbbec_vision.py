from .base import VisionProvider


class OrbbecVision(VisionProvider):
    def detect_objects(self):
        raise NotImplementedError("Orbbec camera is not connected yet.")