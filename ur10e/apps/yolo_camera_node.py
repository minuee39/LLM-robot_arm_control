import argparse
import json
from pathlib import Path
import sys
import time

PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from scene_config import BLOCK_SIZE
from vision.calibration import validate_rigid_transform
from vision.depth_utils import median_depth_in_bbox, pixel_to_camera_point, surface_point_to_box_center
from vision.depth_utils import transform_point
from vision.scene_objects import (
    EXPECTED_BLOCK_NAMES,
    StableDetectionStore,
    write_vision_scene,
)
from vision.yolo_detector import YoloDetector


DEFAULT_MODEL_PATH = PROJECT_DIR.parent / "runs" / "detect" / "train" / "weights" / "best.pt"
DEFAULT_SCENE_STATE_FILE = Path("/tmp/ur10e_isaac_scene_objects.json")
DEFAULT_VISION_SCENE_FILE = Path("/tmp/ur10e_vision_scene.json")


class YoloCameraNode(Node):
    def __init__(
        self,
        model_path: str | Path,
        *,
        confidence_threshold: float = 0.4,
        show_window: bool = False,
        rgb_topic: str = "/sim_camera/rgb",
        depth_topic: str = "/sim_camera/depth",
        camera_info_topic: str = "/sim_camera/camera_info",
        scene_state_file: str | Path = DEFAULT_SCENE_STATE_FILE,
        vision_scene_file: str | Path = DEFAULT_VISION_SCENE_FILE,
        sync_queue_size: int = 10,
        sync_slop: float = 0.05,
        stability_window: int = 10,
        stability_min_samples: int = 5,
        detection_ttl: float = 1.0,
        min_stable_confidence: float = 0.6,
        max_position_std: float = 0.02,
        outlier_distance: float = 0.05,
        publish_period: float = 0.5,
    ) -> None:
        super().__init__("yolo_camera_node")
        if sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")
        if sync_slop < 0.0:
            raise ValueError("sync_slop must be non-negative")
        if publish_period <= 0.0:
            raise ValueError("publish_period must be positive")
        self.bridge = CvBridge()
        self.detector = YoloDetector(
            model_path,
            confidence_threshold=confidence_threshold,
        )
        self.show_window = show_window
        self.camera_info = None
        self.depth_image = None
        self.has_rgb = False
        self.rgb_count = 0
        self.depth_count = 0
        self.camera_info_count = 0
        self.last_detection_count = 0
        self.last_valid_depth_count = 0
        self.scene_state_file = Path(scene_state_file).expanduser()
        self.camera_to_world = None
        self.ground_truth_positions = {}
        self.camera_transform_mtime_ns = None
        self.vision_scene_file = Path(vision_scene_file).expanduser()
        self.detection_store = StableDetectionStore(
            window_size=stability_window,
            min_samples=stability_min_samples,
            ttl_seconds=detection_ttl,
            min_confidence=min_stable_confidence,
            max_position_std=max_position_std,
            outlier_distance=outlier_distance,
        )
        self.last_sync_delta = None
        self.last_processing_ms = None
        self.pose_publish_counts = {name: 0 for name in EXPECTED_BLOCK_NAMES}
        self.pose_first_publish_time = {name: None for name in EXPECTED_BLOCK_NAMES}
        self.latest_ground_truth_errors = {}

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.rgb_subscriber = Subscriber(self, Image, rgb_topic, qos_profile=10)
        self.depth_subscriber = Subscriber(self, Image, depth_topic, qos_profile=10)
        self.rgb_depth_sync = ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )
        self.rgb_depth_sync.registerCallback(self.rgb_depth_callback)
        self.annotated_pub = self.create_publisher(Image, "/yolo/annotated", 10)
        self.detections_pub = self.create_publisher(String, "/yolo/detections", 10)
        self.pose_publishers = {
            name: self.create_publisher(PoseStamped, f"/vision/{name}/pose", 10)
            for name in EXPECTED_BLOCK_NAMES
        }
        self.create_timer(publish_period, self.publish_compact_detections)
        self.create_timer(2.0, self.status_callback)

        self.input_topics = (rgb_topic, depth_topic, camera_info_topic)
        self.get_logger().info(
            "YOLO camera node started. Waiting for " + ", ".join(self.input_topics)
        )
        self.get_logger().info(f"Using YOLO model: {Path(model_path).expanduser().resolve()}")
        self.get_logger().info(f"Using camera transform from: {self.scene_state_file}")
        self.get_logger().info(
            f"Writing stable vision scene to: {self.vision_scene_file} "
            f"(sync_slop={sync_slop:.3f}s)"
        )
        self.get_logger().info(
            "Publishing real-time world poses: "
            + ", ".join(f"/vision/{name}/pose" for name in EXPECTED_BLOCK_NAMES)
        )

    def publish_compact_detections(self) -> None:
        stable_snapshot = self.detection_store.snapshot()
        if not stable_snapshot:
            return
        snapshot = {
            name: {
                "confidence": info["confidence"],
                "position": info["position"],
            }
            for name, info in stable_snapshot.items()
        }

        detection_msg = String()
        detection_msg.data = json.dumps(snapshot, ensure_ascii=False, indent=2)
        self.detections_pub.publish(detection_msg)
        try:
            write_vision_scene(self.vision_scene_file, stable_snapshot)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Vision scene write failed: {error}")
        lines = ["Detected scene:"]
        lines.extend(
            f"  {name}: confidence={info['confidence']:.3f}, "
            f"position=({', '.join(f'{value:.3f}' for value in info['position'])})"
            for name, info in snapshot.items()
        )
        self.get_logger().info("\n".join(lines))

    def update_camera_transform(self) -> None:
        try:
            mtime_ns = self.scene_state_file.stat().st_mtime_ns
            if mtime_ns == self.camera_transform_mtime_ns:
                return
            state = json.loads(self.scene_state_file.read_text(encoding="utf-8"))
            camera = state.get("camera")
            if not isinstance(camera, dict) or "optical_to_world" not in camera:
                self.camera_to_world = None
                return
            self.camera_to_world = validate_rigid_transform(camera["optical_to_world"])
            self.ground_truth_positions = {
                str(item["name"]): np.asarray(item["position"], dtype=float)
                for item in state.get("objects", [])
                if isinstance(item, dict) and "name" in item and "position" in item
            }
            self.camera_transform_mtime_ns = mtime_ns
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.camera_to_world = None
            self.get_logger().warning(f"Camera transform unavailable: {error}")

    def status_callback(self) -> None:
        missing = []
        if not self.has_rgb:
            missing.append(self.input_topics[0])
        if self.depth_image is None:
            missing.append(self.input_topics[1])
        if self.camera_info is None:
            missing.append(self.input_topics[2])

        if missing:
            self.get_logger().info(f"Waiting for topics: {', '.join(missing)}")
            return

        elapsed_rates = {}
        now = time.monotonic()
        for name in EXPECTED_BLOCK_NAMES:
            started_at = self.pose_first_publish_time[name]
            elapsed_rates[name] = (
                0.0
                if started_at is None or now <= started_at
                else self.pose_publish_counts[name] / (now - started_at)
            )
        rates = ", ".join(f"{name}={elapsed_rates[name]:.1f}" for name in EXPECTED_BLOCK_NAMES)
        errors = ", ".join(
            f"{name}={self.latest_ground_truth_errors[name]:.4f}"
            for name in EXPECTED_BLOCK_NAMES
            if name in self.latest_ground_truth_errors
        )
        self.get_logger().info(
            "Receiving topics: "
            f"rgb={self.rgb_count}, depth={self.depth_count}, "
            f"camera_info={self.camera_info_count}, "
            f"last_yolo_boxes={self.last_detection_count}, "
            f"last_valid_depth_boxes={self.last_valid_depth_count}, "
            f"last_sync_delta={self.last_sync_delta}, "
            f"processing_ms={self.last_processing_ms}, "
            f"pose_hz=({rates}), "
            f"gt_error_m=({errors or 'unavailable'}), "
            f"missing_blocks={self.detection_store.missing_names()}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.camera_info_count += 1

    @staticmethod
    def message_stamp_seconds(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def rgb_depth_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        callback_started_at = time.perf_counter()
        self.has_rgb = True
        self.rgb_count += 1
        self.depth_count += 1
        self.last_sync_delta = abs(
            self.message_stamp_seconds(rgb_msg) - self.message_stamp_seconds(depth_msg)
        )
        if self.camera_info is None:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            self.depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
            detections = self.detector.detect(rgb)
        except Exception as error:
            self.get_logger().error(f"Synchronized RGB-D processing failed: {error}")
            return

        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self.update_camera_transform()
        self.last_detection_count = len(detections)
        self.last_valid_depth_count = 0
        intrinsics = self.camera_info.k

        best_by_name = {}
        for detection in detections:
            if detection.name not in EXPECTED_BLOCK_NAMES:
                continue
            previous = best_by_name.get(detection.name)
            if previous is None or detection.confidence > previous.confidence:
                best_by_name[detection.name] = detection

        for detection in best_by_name.values():
            u, v = detection.center_pixel
            depth = median_depth_in_bbox(self.depth_image, detection.bbox)
            camera_position = None
            world_position = None
            if depth is not None:
                try:
                    camera_position = pixel_to_camera_point(
                        u,
                        v,
                        depth,
                        intrinsics[0],
                        intrinsics[4],
                        intrinsics[2],
                        intrinsics[5],
                    )
                    if self.camera_to_world is not None:
                        surface_world_position = transform_point(camera_position, self.camera_to_world)
                        world_position = surface_point_to_box_center(
                            surface_world_position,
                            self.camera_to_world[:3, 3],
                            BLOCK_SIZE,
                        )
                    self.last_valid_depth_count += 1
                except ValueError as error:
                    self.get_logger().warning(f"Invalid camera intrinsics/depth: {error}")

            label = f"{detection.name} {detection.confidence:.2f} center=({u},{v})"
            if camera_position is None:
                label += " depth=invalid"
            elif world_position is None:
                label += " world=unavailable"
            else:
                label += " world_center=({:.2f},{:.2f},{:.2f})".format(*world_position)
                self.publish_realtime_pose(detection.name, world_position, rgb_msg)
                self.detection_store.update(
                    detection.name,
                    detection.confidence,
                    world_position,
                )

            self.draw_detection(annotated, detection.bbox, detection.center_pixel, label)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = rgb_msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as error:
            self.get_logger().error(f"Annotated image publish failed: {error}")

        if self.show_window:
            cv2.imshow("YOLO Sim Camera", annotated)
            cv2.waitKey(1)
        self.last_processing_ms = round((time.perf_counter() - callback_started_at) * 1000.0, 2)

    def publish_realtime_pose(self, name: str, world_position, rgb_msg: Image) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rgb_msg.header.stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(world_position[0])
        pose_msg.pose.position.y = float(world_position[1])
        pose_msg.pose.position.z = float(world_position[2])
        pose_msg.pose.orientation.w = 1.0
        self.pose_publishers[name].publish(pose_msg)

        now = time.monotonic()
        if self.pose_first_publish_time[name] is None:
            self.pose_first_publish_time[name] = now
        self.pose_publish_counts[name] += 1
        ground_truth = self.ground_truth_positions.get(name)
        if ground_truth is not None and ground_truth.shape == (3,):
            self.latest_ground_truth_errors[name] = float(
                np.linalg.norm(np.asarray(world_position, dtype=float) - ground_truth)
            )

    @staticmethod
    def draw_detection(image, bbox, center_pixel, label) -> None:
        x1, y1, x2, y2 = bbox
        center_u, center_v = center_pixel
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(image, (center_u, center_v), 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            label,
            (x1, max(y1 - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            2,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect blocks from ROS 2 RGB-D camera topics.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL_PATH), help="YOLO weights path.")
    parser.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold.")
    parser.add_argument("--show", action="store_true", help="Show the annotated OpenCV window.")
    parser.add_argument("--rgb-topic", default="/sim_camera/rgb")
    parser.add_argument("--depth-topic", default="/sim_camera/depth")
    parser.add_argument("--camera-info-topic", default="/sim_camera/camera_info")
    parser.add_argument("--scene-state-file", default=str(DEFAULT_SCENE_STATE_FILE))
    parser.add_argument("--vision-scene-file", default=str(DEFAULT_VISION_SCENE_FILE))
    parser.add_argument("--sync-queue-size", type=int, default=10)
    parser.add_argument("--sync-slop", type=float, default=0.05)
    parser.add_argument("--stability-window", type=int, default=10)
    parser.add_argument("--stability-min-samples", type=int, default=5)
    parser.add_argument("--detection-ttl", type=float, default=1.0)
    parser.add_argument("--min-stable-confidence", type=float, default=0.6)
    parser.add_argument("--max-position-std", type=float, default=0.02)
    parser.add_argument("--outlier-distance", type=float, default=0.05)
    parser.add_argument("--publish-period", type=float, default=0.5)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = YoloCameraNode(
        model_path=args.model,
        confidence_threshold=args.conf,
        show_window=args.show,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        scene_state_file=args.scene_state_file,
        vision_scene_file=args.vision_scene_file,
        sync_queue_size=args.sync_queue_size,
        sync_slop=args.sync_slop,
        stability_window=args.stability_window,
        stability_min_samples=args.stability_min_samples,
        detection_ttl=args.detection_ttl,
        min_stable_confidence=args.min_stable_confidence,
        max_position_std=args.max_position_std,
        outlier_distance=args.outlier_distance,
        publish_period=args.publish_period,
    )
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
