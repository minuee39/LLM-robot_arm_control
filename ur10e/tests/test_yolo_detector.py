from types import SimpleNamespace

import numpy as np
import pytest

from vision.yolo_detector import YoloDetector


class FakeTensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def cpu(self):
        return self

    def numpy(self):
        return self.value

    def __getitem__(self, index):
        return FakeTensor(self.value[index])


class FakeModel:
    def __call__(self, image, conf, verbose):
        assert image.shape == (20, 30, 3)
        assert conf == 0.4
        assert verbose is False
        box = SimpleNamespace(
            conf=FakeTensor([0.91]),
            xyxy=FakeTensor([[5.0, 4.0, 15.0, 14.0]]),
        )
        return [SimpleNamespace(boxes=[box])]


def test_yolo_detector_combines_box_with_color_name():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[4:14, 5:15] = [255, 0, 0]
    detector = YoloDetector(model=FakeModel(), confidence_threshold=0.4)

    detections = detector.detect(image)

    assert len(detections) == 1
    assert detections[0].name == "red_block"
    assert detections[0].confidence == pytest.approx(0.91)
    assert detections[0].bbox == (5, 4, 15, 14)
    assert detections[0].center_pixel == (10, 9)


def test_yolo_detector_rejects_non_rgb_input():
    detector = YoloDetector(model=FakeModel())

    with pytest.raises(ValueError, match="shape"):
        detector.detect(np.zeros((20, 30), dtype=np.uint8))
