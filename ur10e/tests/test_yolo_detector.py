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
    names = {41: "cup", 42: "fork"}

    def __init__(self, *, class_id=None):
        self.class_id = class_id

    def __call__(self, image, conf, verbose):
        assert image.shape == (20, 30, 3)
        assert conf == 0.4
        assert verbose is False
        box = SimpleNamespace(
            conf=FakeTensor([0.91]),
            xyxy=FakeTensor([[5.0, 4.0, 15.0, 14.0]]),
        )
        if self.class_id is not None:
            box.cls = FakeTensor([self.class_id])
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
    assert detections[0].color == "red"
    assert detections[0].class_name == "unknown"


def test_yolo_detector_model_mode_uses_model_class():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[4:14, 5:15] = [255, 0, 0]
    detector = YoloDetector(model=FakeModel(class_id=41), label_mode="model")

    detection = detector.detect(image)[0]

    assert detection.name == "cup"
    assert detection.class_name == "cup"
    assert detection.color == "red"


def test_yolo_detector_hybrid_mode_combines_color_and_model_class():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[4:14, 5:15] = [255, 0, 0]
    detector = YoloDetector(model=FakeModel(class_id=41), label_mode="hybrid")

    assert detector.detect(image)[0].name == "red_cup"


def test_yolo_detector_filters_model_classes_before_labeling():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    detector = YoloDetector(
        model=FakeModel(class_id=42),
        label_mode="model",
        classes=("cup",),
    )

    assert detector.detect(image) == []


def test_yolo_detector_rejects_unknown_label_mode():
    with pytest.raises(ValueError, match="label_mode"):
        YoloDetector(model=FakeModel(), label_mode="shape")


def test_yolo_detector_rejects_non_rgb_input():
    detector = YoloDetector(model=FakeModel())

    with pytest.raises(ValueError, match="shape"):
        detector.detect(np.zeros((20, 30), dtype=np.uint8))
