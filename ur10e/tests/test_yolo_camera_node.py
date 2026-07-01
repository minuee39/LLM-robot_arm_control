import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO


class YoloCameraNode(Node):
    def __init__(self, show_window=False):
        super().__init__("yolo_camera_node")

        self.bridge = CvBridge()
        model_path = Path(__file__).resolve().parents[1] / "models" / "yolov8n.pt"
        self.model = YOLO(str(model_path))
        self.show_window = show_window

        self.camera_info = None
        self.depth_image = None
        self.has_rgb = False
        self.rgb_count = 0
        self.depth_count = 0
        self.camera_info_count = 0
        self.last_detection_count = 0
        self.last_valid_depth_count = 0

        self.create_subscription(
            CameraInfo,
            "/sim_camera/camera_info",
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Image,
            "/sim_camera/depth",
            self.depth_callback,
            10,
        )

        self.create_subscription(
            Image,
            "/sim_camera/rgb",
            self.rgb_callback,
            10,
        )

        self.annotated_pub = self.create_publisher(Image, "/yolo/annotated", 10)
        self.create_timer(2.0, self.status_callback)
        self.get_logger().info(
            "YOLO camera node started. Waiting for /sim_camera/rgb, "
            "/sim_camera/depth, and /sim_camera/camera_info"
        )

    def status_callback(self):
        missing = []
        if not self.has_rgb:
            missing.append("/sim_camera/rgb")
        if self.depth_image is None:
            missing.append("/sim_camera/depth")
        if self.camera_info is None:
            missing.append("/sim_camera/camera_info")

        if missing:
            self.get_logger().info(f"Waiting for topics: {', '.join(missing)}")
            camera_topics = [
                name
                for name, _types in self.get_topic_names_and_types()
                if "camera" in name.lower()
                or "image" in name.lower()
                or "rgb" in name.lower()
                or "depth" in name.lower()
            ]
            if camera_topics:
                self.get_logger().info(
                    "Available camera-like topics: " + ", ".join(sorted(camera_topics))
                )
        else:
            self.get_logger().info(
                "Receiving topics: "
                f"rgb={self.rgb_count}, depth={self.depth_count}, "
                f"camera_info={self.camera_info_count}, "
                f"last_yolo_boxes={self.last_detection_count}, "
                f"last_valid_depth_boxes={self.last_valid_depth_count}"
            )

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_info_count += 1

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            self.depth_count += 1
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def rgb_callback(self, msg):
        self.has_rgb = True
        self.rgb_count += 1

        if self.camera_info is None or self.depth_image is None:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return

        results = self.model(rgb, verbose=False)
        self.last_detection_count = sum(len(result.boxes) for result in results)
        self.last_valid_depth_count = 0
        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        for result in results:
            boxes = result.boxes

            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                x1_i, y1_i, x2_i, y2_i = map(int, (x1, y1, x2, y2))

                if v < 0 or v >= self.depth_image.shape[0]:
                    continue
                if u < 0 or u >= self.depth_image.shape[1]:
                    continue

                Z = float(self.depth_image[v, u])
                label = f"{name} {conf:.2f} center=({u},{v})"

                if not np.isfinite(Z) or Z <= 0:
                    label += " depth=invalid"
                    self.draw_detection(annotated, x1_i, y1_i, x2_i, y2_i, u, v, label)
                    continue
                self.last_valid_depth_count += 1

                K = self.camera_info.k
                fx = K[0]
                fy = K[4]
                cx = K[2]
                cy = K[5]

                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                label += f" xyz=({X:.2f},{Y:.2f},{Z:.2f})"
                self.draw_detection(annotated, x1_i, y1_i, x2_i, y2_i, u, v, label)

                self.get_logger().info(
                    f"{name} conf={conf:.2f} pixel=({u},{v}) "
                    f"camera_xyz=({X:.3f}, {Y:.3f}, {Z:.3f})"
                )

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Annotated image publish failed: {e}")

        if self.show_window:
            cv2.imshow("YOLO Sim Camera", annotated)
            cv2.waitKey(1)

    @staticmethod
    def draw_detection(image, x1, y1, x2, y2, cx, cy, label):
        height, width = image.shape[:2]
        x1 = int(np.clip(x1, 0, width - 1))
        x2 = int(np.clip(x2, 0, width - 1))
        y1 = int(np.clip(y1, 0, height - 1))
        y2 = int(np.clip(y2, 0, height - 1))
        cx = int(np.clip(cx, 0, width - 1))
        cy = int(np.clip(cy, 0, height - 1))

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            label,
            (x1, max(y1 - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            2,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show an OpenCV window with YOLO boxes and center coordinates.",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = YoloCameraNode(show_window=args.show)
    try:
        rclpy.spin(node)
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
