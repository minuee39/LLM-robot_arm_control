#!/usr/bin/env python3
"""Read-only ROS camera snapshot; no object truth or robot commands.

Preserve raw sensor inputs and calibration for offline pose diagnostics.
Capturing matching images alone does not certify registration/depth semantics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision.calibration import validate_rigid_transform


def camera_calibration_only(path):
    """Do not export object truth that may share the source JSON."""
    state = json.loads(Path(path).read_text(encoding="utf-8"))
    transform = validate_rigid_transform(state["camera"]["optical_to_base_footprint"])
    camera_only = {"optical_to_base_footprint": transform.tolist()}
    revision = hashlib.sha256(json.dumps(camera_only, sort_keys=True).encode()).hexdigest()
    return transform, revision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=20.)
    parser.add_argument("--sync-slop", type=float, default=.01)
    args = parser.parse_args()
    if not np.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error("timeout must be finite in (0,60]")
    if not np.isfinite(args.sync_slop) or not 0 <= args.sync_slop <= .1:
        parser.error("sync slop must be finite in [0,.1]")
    if args.output_dir.exists():
        parser.error("output directory already exists; refusing to overwrite")
    # Deliberately retain camera only: scene files can also contain object truth.
    transform, revision = camera_calibration_only(args.calibration)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from cv_bridge import CvBridge

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    rclpy.init()
    node = Node("pallet_rgbd_diagnostic_capture")
    from collections import deque
    info = deque(maxlen=1)
    pairs = []
    errors = []
    bridge = CvBridge()

    def remember_camera_info(message):
        # deque.append lacks an inspectable signature required by rclpy.
        info.append(message)

    info_sub = node.create_subscription(CameraInfo, "/sim_camera/camera_info", remember_camera_info, 10)
    rgb_sub = Subscriber(node, Image, "/sim_camera/rgb", qos_profile=10)
    depth_sub = Subscriber(node, Image, "/sim_camera/depth", qos_profile=10)
    sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], 5, args.sync_slop)

    def capture(rgb, depth):
        if not info or pairs:
            return
        try:
            rgb_array = bridge.imgmsg_to_cv2(rgb, desired_encoding="rgb8").copy()
            depth_array = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough").copy()
            if rgb_array.shape[:2] != depth_array.shape or depth_array.ndim != 2:
                raise ValueError("RGB and depth shapes differ")
            ci = info[-1]
            if (ci.width, ci.height) != (rgb.width, rgb.height):
                raise ValueError("CameraInfo dimensions differ from RGB")
            pairs.append((rgb_array, depth_array, dict(
                captured_at_unix=time.time(), rgb_stamp=stamp(rgb), depth_stamp=stamp(depth),
                camera_info_stamp=stamp(ci), stamp_clock="ros_header",
                rgb_frame=rgb.header.frame_id, depth_frame=depth.header.frame_id,
                camera_info_frame=ci.header.frame_id, depth_encoding=depth.encoding,
                k=list(ci.k), d=list(ci.d), r=list(ci.r), p=list(ci.p),
                distortion_model=ci.distortion_model, width=ci.width, height=ci.height,
                sync_delta_s=abs(stamp(rgb)-stamp(depth)),
                optical_to_base_footprint=transform.tolist(), calibration_revision=revision,
                registered_rectified="unverified", depth_semantics="unverified",
                purpose="diagnostic_only_no_object_truth",
            )))
        except Exception as error:
            errors.append(str(error))

    sync.registerCallback(capture)
    try:
        deadline = time.monotonic() + args.timeout
        while not pairs and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not pairs:
            raise RuntimeError(f"no synchronized RGB-D capture; recent errors={errors[-3:]}")
        rgb, depth, metadata = pairs[0]
        args.output_dir.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(args.output_dir / "rgbd.npz", rgb=rgb, depth=depth)
        (args.output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8")
        print(json.dumps({"output_dir": str(args.output_dir.resolve()),
                          "shape": list(depth.shape), "sync_delta_s": metadata["sync_delta_s"],
                          "depth_encoding": metadata["depth_encoding"]}))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
