#!/usr/bin/env python3
"""Read-only live joint-pose diagnostics; no control or planning scene writes."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision.box_pose_store import StableBoxPoseStore
from vision.scene_objects import EXPECTED_BLOCK_NAMES
from analyze_pallet_rgbd import analyze_arrays
from capture_pallet_rgbd import camera_calibration_only


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=20.)
    parser.add_argument("--ransac-trials", type=int, default=2048)
    parser.add_argument('--plane-distance-limit-m', type=float, default=.0005)
    parser.add_argument('--opening-kernel-size', type=int, choices=(1, 3, 5, 7, 9), default=3)
    parser.add_argument("--confirmed-sim-render-contract", action="store_true")
    args = parser.parse_args()
    if not args.confirmed_sim_render_contract:
        parser.error("simulator RenderProduct contract confirmation required")
    if not 0 < args.seconds <= 600:
        parser.error("seconds must be in (0,600]")
    if not 1 <= args.ransac_trials <= 10000:
        parser.error("RANSAC trials must be in [1,10000]")
    if not math.isfinite(args.plane_distance_limit_m) or not 0 < args.plane_distance_limit_m <= .0005:
        parser.error('plane distance limit must be finite within (0, 0.0005] metres')
    if args.output_dir.exists():
        parser.error("output already exists; refusing overwrite")
    transform, revision = camera_calibration_only(args.calibration)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from cv_bridge import CvBridge

    args.output_dir.mkdir(parents=True, exist_ok=False)
    store = StableBoxPoseStore(window_size=5, min_samples=3, ttl_s=1.,
                              max_position_std_m=.001, max_yaw_spread_rad=.05,
                              max_normal_spread_rad=.05)
    rclpy.init()
    node = Node("pallet_live_pose_diagnostic")
    bridge = CvBridge()
    last_processed = [-float("inf")]
    events = []

    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def pair(rgb, depth, ci):
        started = time.monotonic()
        if started-last_processed[0] < .25:
            return
        last_processed[0] = started
        event = {"observed_at_unix": time.time(), "exposure_stamp": stamp(rgb)}
        try:
            if any(msg.header.frame_id != "pallet_camera_optical_frame" for msg in (rgb, depth, ci)):
                raise ValueError("calibrated Pallet optical frames required")
            now = time.time()
            if any(not 0 <= now-stamp(msg) <= 1. for msg in (rgb, depth, ci)):
                raise ValueError("fresh Unix system-clock exposures required; simulation epoch rejected")
            if max(stamp(msg) for msg in (rgb, depth, ci))-min(stamp(msg) for msg in (rgb, depth, ci)) > .02:
                raise ValueError("CameraInfo/RGB/depth exposure skew exceeds diagnostic limit")
            transform, revision = camera_calibration_only(args.calibration)
            metadata = dict(width=ci.width, height=ci.height, d=list(ci.d), r=list(ci.r), k=list(ci.k),
                            rgb_frame=rgb.header.frame_id, depth_frame=depth.header.frame_id,
                            camera_info_frame=ci.header.frame_id, rgb_stamp=stamp(rgb),
                            depth_stamp=stamp(depth), stamp_clock="unix_system",
                            depth_encoding=depth.encoding, optical_to_base_footprint=transform.tolist(),
                            calibration_revision=revision)
            report = analyze_arrays(bridge.imgmsg_to_cv2(rgb, desired_encoding="rgb8"),
                                    bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough"),
                                    metadata, saturation=50, sim_render_contract_confirmed=True,
                                    ransac_trials=args.ransac_trials,
                                    plane_distance_limit_m=args.plane_distance_limit_m,
                                    opening_kernel_size=args.opening_kernel_size)
            revision = report['calibration_revision']
            finished = time.monotonic()
            age = time.time()-min(stamp(rgb), stamp(depth))
            accepted = {}
            for name in EXPECTED_BLOCK_NAMES:
                candidates = [d for d in report["detections"]
                              if d["object_id"] == name and d["diagnostic_candidate"]]
                if len(candidates) != 1:
                    store.invalidate(name, exposure_stamp=stamp(rgb))
                    accepted[name] = False
                    continue
                accepted[name] = store.update(
                    name, candidates[0], exposure_stamp=stamp(rgb), clock="unix_system",
                    calibration_revision=revision, received_at_monotonic=finished,
                    exposure_age_s=age, frame=report["frame"], unit=report["unit"])
            snapshot = store.snapshot(now_monotonic=finished)
            event.update(processing_seconds=finished-started, exposure_age_s=age, candidates=accepted,
                         stable_objects=list(snapshot), stable_observations=snapshot,
                         candidate_observations=report["detections"], calibration_revision=revision)
        except (ValueError, KeyError, OSError, TypeError) as error:
            for name in EXPECTED_BLOCK_NAMES:
                store.invalidate(name)
            event.update(rejected=str(error), processing_seconds=time.monotonic()-started)
        events.append(event)

    info_sub = Subscriber(node, CameraInfo, "/sim_camera/camera_info", qos_profile=10)
    rgb_sub = Subscriber(node, Image, "/sim_camera/rgb", qos_profile=10)
    depth_sub = Subscriber(node, Image, "/sim_camera/depth", qos_profile=10)
    sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, .01)
    sync.registerCallback(pair)
    try:
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            latest = dict(diagnostic_only=True, collision_model_approved=False,
                          updated_at_unix=time.time(), objects=store.snapshot(now_monotonic=time.monotonic()))
            tmp = args.output_dir/"latest.tmp"
            tmp.write_text(json.dumps(latest, indent=2))
            tmp.replace(args.output_dir/"latest.json")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (args.output_dir/"events.json").write_text(json.dumps(events, indent=2))
    print(json.dumps({"event_count": len(events), "rejected_count": sum("rejected" in e for e in events),
                      "three_stable_event_count": sum(len(e.get("stable_objects", [])) == 3 for e in events),
                      "output_dir": str(args.output_dir.resolve())}))
    if not events:
        raise RuntimeError("no synchronized camera events received")
    if not any(len(event.get("stable_objects", [])) == 3 for event in events):
        raise RuntimeError("no jointly stable three-block diagnostic observation")


if __name__ == "__main__":
    main()
