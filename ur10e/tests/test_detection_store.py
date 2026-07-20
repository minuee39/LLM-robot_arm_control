import pytest
import json

from vision.scene_objects import StableDetectionStore, TimedDetectionStore, write_vision_scene


def test_detection_store_returns_all_blocks_in_stable_order():
    store = TimedDetectionStore(ttl_seconds=2.0)
    store.update("green_block", 0.81, [0.0, 0.45, 0.05], timestamp=1.0)
    store.update("red_block", 0.72, [-0.3, 0.3, 0.05], timestamp=1.1)
    store.update("blue_block", 0.83, [0.3, 0.3, 0.05], timestamp=1.2)

    snapshot = store.snapshot(timestamp=2.0)

    assert list(snapshot) == ["red_block", "green_block", "blue_block"]
    assert snapshot["blue_block"] == {
        "confidence": 0.83,
        "position": [0.3, 0.3, 0.05],
    }


def test_detection_store_expires_stale_blocks():
    store = TimedDetectionStore(ttl_seconds=1.0)
    store.update("red_block", 0.7, [0.0, 0.0, 0.05], timestamp=1.0)
    store.update("blue_block", 0.8, [0.3, 0.3, 0.05], timestamp=2.5)

    snapshot = store.snapshot(timestamp=3.0)

    assert list(snapshot) == ["blue_block"]
    assert store.missing_names(timestamp=3.0) == ["red_block", "green_block"]


def test_detection_store_rejects_invalid_position():
    store = TimedDetectionStore()

    with pytest.raises(ValueError, match="three-dimensional"):
        store.update("red_block", 0.7, [0.0, 0.0])


def test_stable_detection_store_uses_median_after_minimum_samples():
    store = StableDetectionStore(
        window_size=5,
        min_samples=3,
        ttl_seconds=2.0,
        max_position_std=0.02,
    )
    store.update("red_block", 0.8, [0.100, 0.200, 0.050], timestamp=1.0)
    store.update("red_block", 0.9, [0.102, 0.198, 0.051], timestamp=1.1)
    assert store.snapshot(timestamp=1.15) == {}
    store.update("red_block", 0.7, [0.098, 0.201, 0.049], timestamp=1.2)

    snapshot = store.snapshot(timestamp=1.3)

    assert snapshot["red_block"]["position"] == [0.1, 0.2, 0.05]
    assert snapshot["red_block"]["confidence"] == 0.8
    assert snapshot["red_block"]["sample_count"] == 3


def test_stable_detection_store_rejects_low_confidence_and_outlier():
    store = StableDetectionStore(
        window_size=5,
        min_samples=2,
        min_confidence=0.6,
        outlier_distance=0.05,
    )
    assert store.update("blue_block", 0.5, [0.3, 0.3, 0.05], timestamp=1.0) is False
    assert store.update("blue_block", 0.8, [0.3, 0.3, 0.05], timestamp=1.1) is True
    assert store.update("blue_block", 0.8, [0.301, 0.3, 0.05], timestamp=1.2) is True
    assert store.update("blue_block", 0.8, [0.5, 0.3, 0.05], timestamp=1.3) is False

    assert store.snapshot(timestamp=1.4)["blue_block"]["sample_count"] == 2


def test_stable_detection_store_hides_noisy_position():
    store = StableDetectionStore(
        window_size=3,
        min_samples=3,
        max_position_std=0.01,
        outlier_distance=1.0,
    )
    store.update("green_block", 0.8, [0.0, 0.45, 0.05], timestamp=1.0)
    store.update("green_block", 0.8, [0.04, 0.45, 0.05], timestamp=1.1)
    store.update("green_block", 0.8, [-0.04, 0.45, 0.05], timestamp=1.2)

    assert store.snapshot(timestamp=1.3) == {}


def test_write_vision_scene_writes_complete_atomic_payload(tmp_path):
    objects = {
        name: {
            "confidence": 0.8,
            "position": [0.0, 0.0, 0.05],
            "sample_count": 5,
            "position_std": [0.001, 0.001, 0.001],
        }
        for name in ("red_block", "green_block", "blue_block")
    }
    path = tmp_path / "vision_scene.json"

    write_vision_scene(path, objects, updated_at=123.5)

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["updated_at"] == 123.5
    assert payload["frame"] == "world"
    assert list(payload["objects"]) == ["red_block", "green_block", "blue_block"]
    assert not (tmp_path / "vision_scene.json.tmp").exists()


def test_write_vision_scene_writes_partial_detection_payload(tmp_path):
    path = tmp_path / "vision_scene.json"
    objects = {
        "red_block": {
            "confidence": 0.8,
            "position": [0.1, 0.2, 0.05],
        },
        "blue_block": {
            "confidence": 0.85,
            "position": [0.3, 0.4, 0.05],
        },
    }

    write_vision_scene(path, objects, updated_at=123.5)

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert list(payload["objects"]) == ["red_block", "blue_block"]
