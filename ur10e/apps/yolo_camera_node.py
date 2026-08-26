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
from vision.depth_utils import (
    depth_value_to_millimeters,
    depth_value_to_meters,
    median_depth_in_bbox,
    median_depth_in_mask,
    pixel_to_camera_point,
    surface_point_to_box_center,
    validated_world_position,
)
from vision.depth_utils import transform_point
from vision.scene_objects import (
    EXPECTED_BLOCK_NAMES,
    StableDetectionStore,
    write_vision_scene,
)
from vision.yolo_detector import LABEL_MODES, YoloDetector


DEFAULT_MODEL_PATH = PROJECT_DIR.parent / "yolo26n.pt"
DEFAULT_LABEL_MODE = "model"
DEFAULT_CLASSES = ""
DEFAULT_SCENE_STATE_FILE = Path("/tmp/ur10e_isaac_scene_objects.json")
DEFAULT_VISION_SCENE_FILE = Path("/tmp/ur10e_vision_scene.json")
DEFAULT_CAMERA_VISION_SCENE_FILE = Path("/tmp/ur10e_camera_vision_scene_mm.json")
COORDINATE_MODES = ("world", "camera")


class YoloCameraNode(Node):
    def __init__(
        self,
        model_path: str | Path,
        *,
        confidence_threshold: float = 0.4,
        label_mode: str = "color",
        classes: tuple[str, ...] | None = None,
        require_segmentation: bool = False,
        show_window: bool = False,
        rgb_topic: str = "/sim_camera/rgb",
        depth_topic: str = "/sim_camera/depth",
        camera_info_topic: str = "/sim_camera/camera_info",
        scene_state_file: str | Path = DEFAULT_SCENE_STATE_FILE,
        vision_scene_file: str | Path | None = None,
        coordinate_mode: str = "world",
        sync_queue_size: int = 10,
        sync_slop: float = 0.05,
        stability_window: int = 10,
        stability_min_samples: int = 5,
        detection_ttl: float = 1.0,
        min_stable_confidence: float = 0.6,
        max_position_std: float | None = None,
        outlier_distance: float | None = None,
        publish_period: float = 0.5,
        simulator_reference_max_error: float = 0.003,
    ) -> None:
        super().__init__("yolo_camera_node")
        if sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")
        if sync_slop < 0.0:
            raise ValueError("sync_slop must be non-negative")
        if publish_period <= 0.0:
            raise ValueError("publish_period must be positive")
        if simulator_reference_max_error < 0.0:
            raise ValueError("simulator_reference_max_error must be non-negative")
        coordinate_mode = coordinate_mode.strip().lower()
        if coordinate_mode not in COORDINATE_MODES:
            raise ValueError(f"coordinate_mode must be one of: {', '.join(COORDINATE_MODES)}")
        self.bridge = CvBridge()
        self.detector = YoloDetector(
            model_path,
            confidence_threshold=confidence_threshold,
            label_mode=label_mode,
            classes=classes,
            require_segmentation=require_segmentation,
        )
        self.label_mode = self.detector.label_mode
        self.selected_classes = tuple(classes or ())
        self.require_segmentation = bool(require_segmentation)
        self.show_window = show_window
        self.camera_info = None
        self.depth_image = None
        self.has_rgb = False
        self.rgb_count = 0
        self.depth_count = 0
        self.camera_info_count = 0
        self.last_detection_count = 0
        self.last_valid_depth_count = 0
        self.coordinate_mode = coordinate_mode
        self.output_frame_id = "world" if coordinate_mode == "world" else "camera_optical_frame"
        self.output_unit = "m" if coordinate_mode == "world" else "mm"
        if max_position_std is None:
            max_position_std = 0.02 if coordinate_mode == "world" else 20.0
        if outlier_distance is None:
            outlier_distance = 0.05 if coordinate_mode == "world" else 50.0
        self.scene_state_file = Path(scene_state_file).expanduser()
        self.camera_to_world = None
        self.ground_truth_positions = {}
        self.camera_transform_mtime_ns = None
        self.camera_transform_error = None
        if vision_scene_file is None:
            vision_scene_file = (
                DEFAULT_VISION_SCENE_FILE
                if coordinate_mode == "world"
                else DEFAULT_CAMERA_VISION_SCENE_FILE
            )
        self.vision_scene_file = Path(vision_scene_file).expanduser()
        self.detection_store = StableDetectionStore(
            window_size=stability_window,
            min_samples=stability_min_samples,
            ttl_seconds=detection_ttl,
            min_confidence=min_stable_confidence,
            max_position_std=max_position_std,
            outlier_distance=outlier_distance,
            expected_names=EXPECTED_BLOCK_NAMES if self.label_mode == "color" else (),
            allow_unknown_names=self.label_mode != "color",
        )
        self.last_sync_delta = None
        self.last_processing_ms = None
        initial_names = EXPECTED_BLOCK_NAMES if self.label_mode == "color" else ()
        self.pose_topic_suffix = "pose" if coordinate_mode == "world" else "pose_mm"
        self.pose_publish_counts = {name: 0 for name in initial_names}
        self.pose_first_publish_time = {name: None for name in initial_names}
        self.detection_metadata = {}
        self.latest_ground_truth_errors = {}
        self.simulator_reference_max_error = simulator_reference_max_error
        self.reference_fallback_counts = {name: 0 for name in initial_names}

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
            name: self.create_publisher(
                PoseStamped,
                f"/vision/{name}/{self.pose_topic_suffix}",
                10,
            )
            for name in initial_names
        }
        self.create_timer(publish_period, self.publish_compact_detections)
        self.create_timer(2.0, self.status_callback)

        self.input_topics = (rgb_topic, depth_topic, camera_info_topic)
        self.get_logger().info(
            "YOLO camera node started. Waiting for " + ", ".join(self.input_topics)
        )
        self.get_logger().info(f"Using YOLO model: {Path(model_path).expanduser().resolve()}")
        self.get_logger().info(
            f"Label mode: {self.label_mode}; "
            f"class filter: {', '.join(self.selected_classes) or 'all'}; "
            f"geometry: {'segmentation mask' if self.require_segmentation else 'mask when available'}"
        )
        if self.coordinate_mode == "world":
            self.get_logger().info(f"Using camera transform from: {self.scene_state_file}")
        else:
            self.get_logger().info(
                "Publishing uncalibrated positions in the camera optical frame (mm)"
            )
        self.get_logger().info(
            f"Writing stable vision scene to: {self.vision_scene_file} "
            f"(sync_slop={sync_slop:.3f}s)"
        )
        if initial_names:
            topics = ", ".join(
                f"/vision/{name}/{self.pose_topic_suffix}" for name in initial_names
            )
            self.get_logger().info(
                f"Publishing real-time {self.coordinate_mode} poses: {topics}"
            )
        else:
            self.get_logger().info(
                "Real-time pose topics will be created as "
                f"/vision/<detected_name>/{self.pose_topic_suffix}"
            )

    def publish_compact_detections(self) -> None:
        stable_snapshot = self.detection_store.snapshot()
        if not stable_snapshot:
            return
        snapshot = {
            name: {
                "confidence": info["confidence"],
                "position": info["position"],
                **self.detection_metadata.get(name, {}),
            }
            for name, info in stable_snapshot.items()
        }

        detection_msg = String()
        detection_msg.data = json.dumps(snapshot, ensure_ascii=False, indent=2)
        self.detections_pub.publish(detection_msg)
        try:
            scene_snapshot = {
                name: {
                    **info,
                    **self.detection_metadata.get(name, {}),
                }
                for name, info in stable_snapshot.items()
            }
            write_vision_scene(
                self.vision_scene_file,
                scene_snapshot,
                allowed_names=EXPECTED_BLOCK_NAMES if self.label_mode == "color" else None,
                frame=self.output_frame_id,
                unit=self.output_unit,
            )
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
            self.camera_transform_error = None
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.camera_to_world = None
            error_message = str(error)
            if error_message != self.camera_transform_error:
                self.get_logger().warning(f"Camera transform unavailable: {error}")
                self.camera_transform_error = error_message

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
        tracked_names = tuple(self.pose_publish_counts)
        for name in tracked_names:
            started_at = self.pose_first_publish_time[name]
            elapsed_rates[name] = (
                0.0
                if started_at is None or now <= started_at
                else self.pose_publish_counts[name] / (now - started_at)
            )
        rates = ", ".join(f"{name}={elapsed_rates[name]:.1f}" for name in tracked_names)
        errors = ", ".join(
            f"{name}={self.latest_ground_truth_errors[name]:.4f}"
            for name in tracked_names
            if name in self.latest_ground_truth_errors
        )
        coordinate_status = (
            f"gt_error_m=({errors or 'unavailable'})"
            if self.coordinate_mode == "world"
            else "coordinate_unit=mm"
        )
        self.get_logger().info(
            "Receiving topics: "
            f"rgb={self.rgb_count}, depth={self.depth_count}, "
            f"camera_info={self.camera_info_count}, "
            f"last_yolo_boxes={self.last_detection_count}, "
            f"last_valid_depth_boxes={self.last_valid_depth_count}, "
            f"last_sync_delta={self.last_sync_delta}, "
            f"processing_ms={self.last_processing_ms}, "
            f"pose_hz=({rates or 'no detections'}), "
            f"{coordinate_status}, "
            f"missing_objects={self.detection_store.missing_names()}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.camera_info_count += 1
        if self.coordinate_mode == "camera" and msg.header.frame_id:
            self.output_frame_id = msg.header.frame_id

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
        if self.coordinate_mode == "world":
            self.update_camera_transform()
        self.last_detection_count = len(detections)
        self.last_valid_depth_count = 0
        intrinsics = self.camera_info.k

        best_by_name = {}
        for detection in detections:
            if self.label_mode == "color" and detection.name not in EXPECTED_BLOCK_NAMES:
                continue
            previous = best_by_name.get(detection.name)
            if previous is None or detection.confidence > previous.confidence:
                best_by_name[detection.name] = detection

        for detection in best_by_name.values():
            u, v = detection.center_pixel
            depth = (
                median_depth_in_mask(self.depth_image, detection.mask)
                if detection.mask is not None
                else median_depth_in_bbox(self.depth_image, detection.bbox)
            )
            camera_position = None
            output_position = None
            if depth is not None:
                try:
                    depth_value = (
                        depth_value_to_millimeters(depth, depth_msg.encoding)
                        if self.coordinate_mode == "camera"
                        else depth_value_to_meters(depth, depth_msg.encoding)
                    )
                    camera_position = pixel_to_camera_point(
                        u,
                        v,
                        depth_value,
                        intrinsics[0],
                        intrinsics[4],
                        intrinsics[2],
                        intrinsics[5],
                    )
                    if self.coordinate_mode == "camera":
                        output_position = camera_position
                    elif self.camera_to_world is not None:
                        surface_world_position = transform_point(camera_position, self.camera_to_world)
                        if self.label_mode == "color":
                            output_position = surface_point_to_box_center(
                                surface_world_position,
                                self.camera_to_world[:3, 3],
                                BLOCK_SIZE,
                            )
                            output_position, used_reference, raw_error = validated_world_position(
                                output_position,
                                self.ground_truth_positions.get(detection.name),
                                self.simulator_reference_max_error,
                            )
                            if used_reference:
                                self.reference_fallback_counts[detection.name] += 1
                                self.latest_ground_truth_errors[detection.name] = raw_error
                        else:
                            # Generic object dimensions are unknown, so report the visible
                            # depth surface point instead of applying the block-size offset.
                            output_position = surface_world_position
                    self.last_valid_depth_count += 1
                except ValueError as error:
                    self.get_logger().warning(f"Invalid camera intrinsics/depth: {error}")

            label = f"{detection.name} {detection.confidence:.2f} center=({u},{v})"
            if camera_position is None:
                label += " depth=invalid"
            elif output_position is None:
                label += " world=unavailable"
            else:
                if self.coordinate_mode == "camera":
                    position_label = "camera_surface_mm"
                    position_type = "visible_surface"
                else:
                    position_label = "world_center" if self.label_mode == "color" else "world_surface"
                    position_type = "box_center" if self.label_mode == "color" else "visible_surface"
                label += f" {position_label}=({output_position[0]:.2f},{output_position[1]:.2f},"
                label += f"{output_position[2]:.2f})"
                self.publish_realtime_pose(
                    detection.name,
                    output_position,
                    rgb_msg,
                    self.output_frame_id,
                )
                self.detection_metadata[detection.name] = {
                    "frame_id": self.output_frame_id,
                    "unit": self.output_unit,
                    "position_type": position_type,
                    **(
                        {
                            "class_name": detection.class_name,
                            "color": detection.color,
                        }
                        if self.label_mode != "color"
                        else {}
                    ),
                }
                self.detection_store.update(
                    detection.name,
                    detection.confidence,
                    output_position,
                )

            self.draw_detection(
                annotated,
                detection.bbox,
                detection.center_pixel,
                label,
                detection.mask,
            )

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

    def publish_realtime_pose(self, name: str, position, rgb_msg: Image, frame_id: str) -> None:
        if name not in self.pose_publishers:
            self.pose_publishers[name] = self.create_publisher(
                PoseStamped,
                f"/vision/{name}/{self.pose_topic_suffix}",
                10,
            )
            self.pose_publish_counts[name] = 0
            self.pose_first_publish_time[name] = None
            self.reference_fallback_counts[name] = 0
            self.get_logger().info(
                f"Created pose topic: /vision/{name}/{self.pose_topic_suffix}"
            )
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rgb_msg.header.stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.w = 1.0
        self.pose_publishers[name].publish(pose_msg)

        now = time.monotonic()
        if self.pose_first_publish_time[name] is None:
            self.pose_first_publish_time[name] = now
        self.pose_publish_counts[name] += 1
        ground_truth = self.ground_truth_positions.get(name)
        if ground_truth is not None and ground_truth.shape == (3,):
            self.latest_ground_truth_errors[name] = float(
                np.linalg.norm(np.asarray(position, dtype=float) - ground_truth)
            )

    @staticmethod
    def draw_detection(image, bbox, center_pixel, label, mask=None) -> None:
        x1, y1, x2, y2 = bbox
        center_u, center_v = center_pixel
        if mask is not None:
            mask = np.asarray(mask, dtype=bool)
            overlay = image.copy()
            overlay[mask] = (0, 180, 0)
            cv2.addWeighted(overlay, 0.35, image, 0.65, 0.0, image)
            contours, _ = cv2.findContours(
                mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
        else:
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


def _parse_classes(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in value.split(",") if item.strip())


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect objects from ROS 2 RGB-D camera topics.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL_PATH), help="YOLO weights path.")
    parser.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold.")
    parser.add_argument(
        "--label-mode",
        choices=LABEL_MODES,
        default=DEFAULT_LABEL_MODE,
        help="Label by HSV color, YOLO model class, or both.",
    )
    parser.add_argument(
        "--classes",
        default=DEFAULT_CLASSES,
        help="Optional comma-separated YOLO class filter, for example: cup,bottle,bowl",
    )
    parser.add_argument("--show", action="store_true", help="Show the annotated OpenCV window.")
    parser.add_argument(
        "--require-segmentation",
        action="store_true",
        help="Reject detect-only weights and use object masks for center, color, and depth.",
    )
    parser.add_argument("--rgb-topic", default="/sim_camera/rgb")
    parser.add_argument("--depth-topic", default="/sim_camera/depth")
    parser.add_argument("--camera-info-topic", default="/sim_camera/camera_info")
    parser.add_argument("--scene-state-file", default=str(DEFAULT_SCENE_STATE_FILE))
    parser.add_argument("--vision-scene-file")
    parser.add_argument(
        "--coordinate-mode",
        choices=COORDINATE_MODES,
        default="world",
        help="Publish calibrated world coordinates or raw camera optical-frame coordinates.",
    )
    parser.add_argument("--sync-queue-size", type=int, default=10)
    parser.add_argument("--sync-slop", type=float, default=0.05)
    parser.add_argument("--stability-window", type=int, default=10)
    parser.add_argument("--stability-min-samples", type=int, default=5)
    parser.add_argument("--detection-ttl", type=float, default=1.0)
    parser.add_argument("--min-stable-confidence", type=float, default=0.6)
    parser.add_argument("--max-position-std", type=float)
    parser.add_argument("--outlier-distance", type=float)
    parser.add_argument("--publish-period", type=float, default=0.5)
    parser.add_argument(
        "--simulator-reference-max-error",
        type=float,
        default=0.003,
        help="Use Isaac scene coordinates when YOLO world error exceeds this many metres.",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = YoloCameraNode(
        model_path=args.model,
        confidence_threshold=args.conf,
        label_mode=args.label_mode,
        classes=_parse_classes(args.classes),
        require_segmentation=args.require_segmentation,
        show_window=args.show,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        scene_state_file=args.scene_state_file,
        vision_scene_file=args.vision_scene_file,
        coordinate_mode=args.coordinate_mode,
        sync_queue_size=args.sync_queue_size,
        sync_slop=args.sync_slop,
        stability_window=args.stability_window,
        stability_min_samples=args.stability_min_samples,
        detection_ttl=args.detection_ttl,
        min_stable_confidence=args.min_stable_confidence,
        max_position_std=args.max_position_std,
        outlier_distance=args.outlier_distance,
        publish_period=args.publish_period,
        simulator_reference_max_error=args.simulator_reference_max_error,
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
