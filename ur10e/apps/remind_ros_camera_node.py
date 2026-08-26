#!/usr/bin/env python3
"""Run REMIND on a ROS 2 image topic without depending on cv_bridge."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Convert common 8-bit ROS image encodings to a contiguous BGR image."""
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f"unsupported image encoding: {message.encoding}")

    channels = channels_by_encoding[encoding]
    packed_width = int(message.width) * channels
    if int(message.step) < packed_width:
        raise ValueError(
            f"invalid image step {message.step}; expected at least {packed_width}"
        )

    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.height) * int(message.step)
    if raw.size < expected_size:
        raise ValueError(
            f"image data is truncated: received {raw.size} bytes, expected {expected_size}"
        )

    rows = raw[:expected_size].reshape(int(message.height), int(message.step))
    pixels = rows[:, :packed_width]
    if channels == 1:
        mono = pixels.reshape(int(message.height), int(message.width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    image = pixels.reshape(int(message.height), int(message.width), channels)
    if encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return np.ascontiguousarray(image)


def bgr_to_image_message(image: np.ndarray, source: Image) -> Image:
    """Create a bgr8 ROS image while preserving the source header."""
    output = Image()
    output.header = source.header
    output.height = int(image.shape[0])
    output.width = int(image.shape[1])
    output.encoding = "bgr8"
    output.is_bigendian = False
    output.step = int(image.shape[1] * 3)
    output.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return output


def parse_classes(raw: str) -> list[int | str] | None:
    values: list[int | str] = []
    for token in raw.replace(";", ",").split(","):
        token = token.strip()
        if not token:
            continue
        try:
            values.append(int(token))
        except ValueError:
            values.append(token)
    return values or None


def load_remind(args: argparse.Namespace):
    remind_root = args.remind_root.expanduser().resolve()
    if not (remind_root / "pipeline" / "reid_pipeline.py").is_file():
        raise FileNotFoundError(f"REMIND repository not found: {remind_root}")
    if not args.model.expanduser().is_file():
        raise FileNotFoundError(f"REMIND YOLO model not found: {args.model}")
    if str(remind_root) not in sys.path:
        sys.path.insert(0, str(remind_root))

    from config.config_loader import Config
    from pipeline.initialization import initialize_system
    from pipeline.reid_pipeline import ReIDPipeline
    from scripts.run_video_tracking import render_frame

    config_path = args.config or remind_root / "config" / "default_config.yaml"
    config = Config(config_path, args.override_config).to_dict()
    config.setdefault("paths", {})["output_dir"] = str(args.output_dir)
    config.setdefault("detector", {})["backend"] = "yolo"
    config.setdefault("runtime", {})["device"] = args.device
    config.setdefault("system", {})["input_width_size"] = int(args.yolo_imgsz)
    config.setdefault("timing", {})["enabled"] = bool(args.verbose_timing)
    config.setdefault("timing", {})["table"] = False

    yolo = config.setdefault("yolo", {})
    yolo["model_label"] = "ROS_CAMERA"
    yolo["models"] = {"ROS_CAMERA": str(args.model.expanduser().resolve())}
    yolo["conf_th"] = float(args.confidence)
    yolo["iou_th"] = float(args.iou)
    yolo["max_det"] = int(args.max_detections)
    yolo["classes"] = parse_classes(args.classes)

    if args.dino_model:
        config.setdefault("dino", {})["model_label"] = args.dino_model

    context = initialize_system(config)
    return ReIDPipeline(context), render_frame, context.device


class RemindCameraNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("remind_camera_node")
        self.args = args
        self.pipeline, self.render_frame, device = load_remind(args)
        self.frame_count = 0
        self.processed_count = 0
        self.image_publisher = self.create_publisher(
            Image, args.output_topic, qos_profile_sensor_data
        )
        self.detections_publisher = self.create_publisher(
            String, args.detections_topic, 10
        )
        self.subscription = self.create_subscription(
            Image, args.rgb_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"REMIND ready: input={args.rgb_topic}, output={args.output_topic}, "
            f"device={device}, stride={args.stride}"
        )

    def on_image(self, message: Image) -> None:
        frame_number = self.frame_count
        self.frame_count += 1
        if frame_number % self.args.stride:
            return

        try:
            frame = image_message_to_bgr(message)
            stamp = message.header.stamp
            timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            started_at = perf_counter()
            perception, _association, update = self.pipeline.process_frame(
                frame=frame,
                frame_id=frame_number,
                timestamp=timestamp,
            )
            elapsed = perf_counter() - started_at
            summary = getattr(update, "summary", {}) or {}
            render_base = (
                (getattr(perception, "debug", {}) or {}).get("frame_aligned_bgr")
            )
            if render_base is None:
                render_base = frame
            header = (
                f"REMIND | frame={frame_number} | "
                f"det={len(perception.detections or [])} | "
                f"visible={summary.get('n_visible', 0)} | "
                f"new={summary.get('n_created', 0)} | "
                f"amb={summary.get('n_ambiguous', 0)} | "
                f"{(1.0 / elapsed if elapsed > 0 else 0.0):.2f} FPS"
            )
            rendered, details = self.render_frame(
                render_base,
                perception.detections or [],
                update,
                header=header,
                alpha=float(self.args.mask_alpha),
            )
            self.image_publisher.publish(bgr_to_image_message(rendered, message))

            detections = String()
            detections.data = json.dumps(
                {
                    "frame_id": frame_number,
                    "timestamp": timestamp,
                    "elapsed_seconds": elapsed,
                    "summary": summary,
                    "detections": details,
                },
                ensure_ascii=True,
                default=lambda value: value.item()
                if isinstance(value, np.generic)
                else str(value),
            )
            self.detections_publisher.publish(detections)
            self.processed_count += 1
        except Exception as error:  # Keep later camera frames usable after a bad frame.
            self.get_logger().error(f"REMIND frame {frame_number} failed: {error}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remind-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--override-config", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/remind_ros"))
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--output-topic", default="/remind/annotated")
    parser.add_argument("--detections-topic", default="/remind/detections")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument("--classes", default="")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--dino-model", choices=("S", "B", "L", "7B"))
    parser.add_argument("--mask-alpha", type=float, default=0.42)
    parser.add_argument("--verbose-timing", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.stride < 1:
        raise SystemExit("error: --stride must be at least 1")
    rclpy.init()
    node = None
    try:
        node = RemindCameraNode(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
