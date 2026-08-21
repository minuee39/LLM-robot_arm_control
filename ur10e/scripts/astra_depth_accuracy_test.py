#!/usr/bin/env python3
"""Measure the Astra depth value in a small region at the image center."""

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
    parser.add_argument("--topic", default="/camera/depth/image_raw")
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--roi-half-size", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--reference-mm",
        type=float,
        default=600.0,
        help="Physical reference distance in millimetres (default: 600).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/minwoo/Desktop/LLM/depth_accuracy_result.txt"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.samples < 1
        or args.roi_half_size < 1
        or args.timeout <= 0
        or args.reference_mm <= 0
    ):
        raise ValueError(
            "samples, roi-half-size, timeout, and reference-mm must be positive"
        )

    rclpy.init()
    node = rclpy.create_node("astra_depth_accuracy_test")
    received_frames = 0
    valid_samples: list[float] = []

    def callback(msg: Image) -> None:
        nonlocal received_frames
        received_frames += 1

        dtype = ">u2" if msg.is_bigendian else "<u2"
        row_width = msg.step // np.dtype(dtype).itemsize
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)
        depth = depth[:, : msg.width]

        cy = msg.height // 2
        cx = msg.width // 2
        half = args.roi_half_size
        roi = depth[max(0, cy - half) : cy + half, max(0, cx - half) : cx + half]
        valid = roi[roi > 0]
        if valid.size:
            valid_samples.append(float(np.median(valid)))

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(Image, args.topic, callback, qos)
    del subscription  # The node retains the subscription.

    print(f"Depth 메시지를 기다리는 중: {args.topic}", flush=True)
    started = time.monotonic()
    while rclpy.ok() and len(valid_samples) < args.samples:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.monotonic() - started >= args.timeout:
            break

    lines = [
        f"수신한 전체 프레임: {received_frames}",
        f"유효한 중앙 Depth 프레임: {len(valid_samples)}",
    ]
    exit_code = 0

    if valid_samples:
        values = np.asarray(valid_samples, dtype=np.float64)
        median = float(np.median(values))
        lines.extend(
            [
                f"평균 raw depth: {np.mean(values):.2f}",
                f"중앙값 raw depth: {median:.2f}",
                f"표준편차: {np.std(values):.2f}",
                f"최솟값: {np.min(values):.2f}",
                f"최댓값: {np.max(values):.2f}",
                f"mm 단위 가정 거리: {median:.2f} mm",
                f"m 단위 변환: {median / 1000.0:.4f} m",
                f"기준 거리: {args.reference_mm:.2f} mm",
                f"기준 대비 오차: {median - args.reference_mm:+.2f} mm",
                f"절대 오차: {abs(median - args.reference_mm):.2f} mm",
            ]
        )
    elif received_frames == 0:
        lines.append("Depth 메시지를 받지 못했습니다. 토픽과 카메라 실행 상태를 확인하세요.")
        exit_code = 1
    else:
        lines.append("중앙 영역의 Depth 값이 모두 0입니다. 대상 위치와 측정 거리를 확인하세요.")
        exit_code = 2

    result = "\n".join(lines) + "\n"
    print(result, end="")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(result, encoding="utf-8")
    print(f"결과 저장: {args.output}")

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
