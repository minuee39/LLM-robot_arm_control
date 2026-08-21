#!/usr/bin/env python3
"""Measure timestamp differences between Astra RGB and depth image streams."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/minwoo/Desktop/LLM/rgb_depth_sync_result.txt"),
    )
    return parser.parse_args()


def stamp_seconds(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def nearest_deltas(color_stamps: list[float], depth_stamps: list[float]) -> np.ndarray:
    color = np.asarray(color_stamps, dtype=np.float64)
    depth = np.asarray(depth_stamps, dtype=np.float64)
    deltas: list[float] = []
    depth_index = 0
    for color_stamp in color:
        while (
            depth_index + 1 < depth.size
            and abs(depth[depth_index + 1] - color_stamp)
            <= abs(depth[depth_index] - color_stamp)
        ):
            depth_index += 1
        deltas.append(depth[depth_index] - color_stamp)
    return np.asarray(deltas, dtype=np.float64)


def main() -> int:
    args = parse_args()
    if args.samples < 2 or args.timeout <= 0:
        raise ValueError("samples must be at least 2 and timeout must be positive")

    rclpy.init()
    node = rclpy.create_node("astra_rgb_depth_sync_test")
    color_stamps: list[float] = []
    depth_stamps: list[float] = []
    color_frame_id = ""
    depth_frame_id = ""

    def color_callback(msg: Image) -> None:
        nonlocal color_frame_id
        color_frame_id = msg.header.frame_id
        if len(color_stamps) < args.samples:
            color_stamps.append(stamp_seconds(msg))

    def depth_callback(msg: Image) -> None:
        nonlocal depth_frame_id
        depth_frame_id = msg.header.frame_id
        if len(depth_stamps) < args.samples:
            depth_stamps.append(stamp_seconds(msg))

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscriptions = [
        node.create_subscription(Image, args.color_topic, color_callback, qos),
        node.create_subscription(Image, args.depth_topic, depth_callback, qos),
    ]

    print("RGB와 Depth timestamp를 수집하는 중...", flush=True)
    started = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(color_stamps) >= args.samples and len(depth_stamps) >= args.samples:
            break
        if time.monotonic() - started >= args.timeout:
            break

    lines = [
        f"RGB 수집 프레임: {len(color_stamps)}",
        f"Depth 수집 프레임: {len(depth_stamps)}",
        f"RGB frame_id: {color_frame_id or '(없음)'}",
        f"Depth frame_id: {depth_frame_id or '(없음)'}",
    ]
    exit_code = 0

    if len(color_stamps) >= 2 and len(depth_stamps) >= 2:
        signed_ms = nearest_deltas(color_stamps, depth_stamps) * 1000.0
        absolute_ms = np.abs(signed_ms)
        color_duration = color_stamps[-1] - color_stamps[0]
        depth_duration = depth_stamps[-1] - depth_stamps[0]
        color_hz = (len(color_stamps) - 1) / color_duration if color_duration > 0 else 0.0
        depth_hz = (len(depth_stamps) - 1) / depth_duration if depth_duration > 0 else 0.0
        lines.extend(
            [
                f"RGB 실측 주파수: {color_hz:.3f} Hz",
                f"Depth 실측 주파수: {depth_hz:.3f} Hz",
                f"최근접 timestamp 평균 절대차: {np.mean(absolute_ms):.3f} ms",
                f"최근접 timestamp 중앙 절대차: {np.median(absolute_ms):.3f} ms",
                f"최근접 timestamp 95백분위: {np.percentile(absolute_ms, 95):.3f} ms",
                f"최근접 timestamp 최대 절대차: {np.max(absolute_ms):.3f} ms",
                f"Depth-RGB signed 평균차: {np.mean(signed_ms):+.3f} ms",
            ]
        )
    else:
        lines.append("두 영상 스트림에서 충분한 timestamp를 받지 못했습니다.")
        exit_code = 1

    result = "\n".join(lines) + "\n"
    print(result, end="")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(result, encoding="utf-8")
    print(f"결과 저장: {args.output}")

    subscriptions.clear()
    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
