#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from person_pose.inference_device import resolve_device
from person_pose_msgs.msg import PersonKeypoints
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class PosePublisher(Node):
    KEYPOINT_WEIGHTS = np.asarray(
        [
            0.05,
            0.05, 0.05,
            0.05, 0.05,
            0.15, 0.15,
            0.08, 0.08,
            0.05, 0.05,
            0.15, 0.15,
            0.07, 0.07,
            0.05, 0.05,
        ],
        dtype=np.float32,
    )

    def __init__(self) -> None:
        super().__init__("person_pose_publisher")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("keypoints_topic", "/person/keypoints")
        self.declare_parameter("device_topic", "/person_pose/device")
        self.declare_parameter("diagnostics_topic", "/person_pose/diagnostics")
        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("image_size", 256)
        self.declare_parameter("process_fps", 0.0)
        self.declare_parameter("gpu_process_fps", 15.0)
        self.declare_parameter("cpu_process_fps", 3.0)
        self.declare_parameter("detection_confidence", 0.45)
        self.declare_parameter("selection_keypoint_confidence", 0.25)
        self.declare_parameter("device", "auto")
        self.declare_parameter("allow_cpu_fallback", True)

        color_topic = str(self.get_parameter("color_topic").value)
        keypoints_topic = str(self.get_parameter("keypoints_topic").value)
        device_topic = str(self.get_parameter("device_topic").value)
        diagnostics_topic = str(
            self.get_parameter("diagnostics_topic").value
        )
        self.model_source = str(self.get_parameter("model_path").value)
        self.image_size = int(self.get_parameter("image_size").value)
        requested_fps = float(self.get_parameter("process_fps").value)
        self.requested_fps = requested_fps
        self.cpu_process_fps = float(
            self.get_parameter("cpu_process_fps").value
        )
        self.det_conf = float(
            self.get_parameter("detection_confidence").value
        )
        self.selection_kp_conf = float(
            self.get_parameter("selection_keypoint_confidence").value
        )
        self.allow_cpu_fallback = bool(
            self.get_parameter("allow_cpu_fallback").value
        )
        requested_device = str(self.get_parameter("device").value)
        self.device, selection_reason = resolve_device(
            requested_device, torch, self.allow_cpu_fallback
        )
        automatic_fps = float(
            self.get_parameter(
                "cpu_process_fps"
                if self.device == "cpu"
                else "gpu_process_fps"
            ).value
        )
        self.process_fps = max(
            0.1, requested_fps if requested_fps > 0.0 else automatic_fps
        )

        self.model = YOLO(self.model_source)
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_header = None
        self.image_generation = 0
        self.processed_generation = 0
        self.last_time = 0.0
        self.fallback_used = self.device == "cpu" and requested_device != "cpu"
        self.last_error = ""

        self.pub = self.create_publisher(PersonKeypoints, keypoints_topic, 10)
        self.device_pub = self.create_publisher(String, device_topic, 10)
        self.diagnostics_pub = self.create_publisher(
            String, diagnostics_topic, 10
        )
        self.create_subscription(
            Image, color_topic, self.color_cb, qos_profile_sensor_data
        )
        self.create_timer(0.01, self.process)
        self.create_timer(1.0, self.publish_runtime_status)

        if selection_reason:
            self.get_logger().warning(selection_reason)
        self.get_logger().info(
            f"Pose node ready: device={self.device}, "
            f"process_fps={self.process_fps:.1f}, model={self.model_source}"
        )
        self.publish_runtime_status()

    def color_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, "bgr8"
            ).copy()
            self.latest_header = msg.header
            self.image_generation += 1
        except Exception as error:
            self.last_error = f"COLOR_CONVERSION: {error}"
            self.get_logger().error(self.last_error)

    def publish_runtime_status(self) -> None:
        device = String()
        device.data = self.device
        self.device_pub.publish(device)

        diagnostics = String()
        state = "OK" if not self.last_error else self.last_error
        diagnostics.data = (
            f"{state}|device={self.device}|"
            f"cpu_fallback={'true' if self.fallback_used else 'false'}"
        )
        self.diagnostics_pub.publish(diagnostics)

    def publish_empty(self, width: int, height: int) -> None:
        msg = PersonKeypoints()
        if self.latest_header is not None:
            msg.header = self.latest_header
        msg.image_width = width
        msg.image_height = height
        msg.detected = False
        self.pub.publish(msg)

    def selection_zone(self, xy, conf, image_width, box) -> int:
        """Return LEFT/CENTER/RIGHT priority (0 is most dangerous)."""
        weighted = np.zeros(3, dtype=np.float32)
        count = min(len(xy), len(conf), len(self.KEYPOINT_WEIGHTS))
        for index in range(count):
            if conf[index] < self.selection_kp_conf:
                continue
            x, y = xy[index]
            if not np.isfinite(x) or not np.isfinite(y):
                continue
            zone = min(int(3.0 * float(x) / max(image_width, 1)), 2)
            weighted[max(zone, 0)] += self.KEYPOINT_WEIGHTS[index]
        if np.any(weighted):
            return int(np.argmax(weighted))
        center_x = 0.5 * (float(box[0]) + float(box[2]))
        return max(0, min(int(3.0 * center_x / max(image_width, 1)), 2))

    def predict(self, frame):
        try:
            return self._predict_once(frame)
        except Exception as error:
            if self.device == "cpu" or not self.allow_cpu_fallback:
                raise
            previous_device = self.device
            self.device = "cpu"
            if self.requested_fps <= 0.0:
                self.process_fps = max(0.1, self.cpu_process_fps)
            self.fallback_used = True
            self.last_error = (
                f"GPU_INFERENCE_FAILED: {type(error).__name__}: {error}"
            )
            self.get_logger().error(
                f"Inference on {previous_device} failed; retrying on CPU: "
                f"{error}. New process_fps={self.process_fps:.1f}"
            )
            try:
                torch.cuda.empty_cache()
            except Exception:
                pass
            self.model = YOLO(self.model_source)
            return self._predict_once(frame)

    def _predict_once(self, frame):
        return self.model.predict(
            frame,
            imgsz=self.image_size,
            conf=self.det_conf,
            classes=[0],
            verbose=False,
            device=self.device,
        )[0]

    def process(self) -> None:
        if (
            self.latest_image is None
            or self.image_generation == self.processed_generation
        ):
            return
        now = time.monotonic()
        if now - self.last_time < 1.0 / self.process_fps:
            return
        self.last_time = now
        self.processed_generation = self.image_generation

        frame = self.latest_image.copy()
        height, width = frame.shape[:2]
        try:
            result = self.predict(frame)
        except Exception as error:
            self.last_error = (
                f"INFERENCE_FAILED: {type(error).__name__}: {error}"
            )
            self.get_logger().error(self.last_error)
            self.publish_empty(width, height)
            return

        if self.last_error.startswith("INFERENCE_FAILED"):
            self.last_error = ""
        if (
            result.boxes is None
            or result.keypoints is None
            or len(result.boxes) == 0
        ):
            self.publish_empty(width, height)
            return

        boxes = result.boxes.xyxy.cpu().numpy()
        areas = (boxes[:, 2] - boxes[:, 0]) * (
            boxes[:, 3] - boxes[:, 1]
        )
        all_xy = result.keypoints.xy.cpu().numpy()
        keypoint_conf = result.keypoints.conf
        all_conf = (
            np.ones((len(boxes), 17), np.float32)
            if keypoint_conf is None
            else keypoint_conf.cpu().numpy()
        )
        priorities = [
            self.selection_zone(all_xy[index], all_conf[index], width, box)
            for index, box in enumerate(boxes)
        ]
        selected = min(
            range(len(boxes)),
            key=lambda index: (priorities[index], -areas[index]),
        )

        msg = PersonKeypoints()
        if self.latest_header is not None:
            msg.header = self.latest_header
        msg.image_width = width
        msg.image_height = height
        msg.detected = True
        msg.x = all_xy[selected][:, 0].astype(np.float32).tolist()
        msg.y = all_xy[selected][:, 1].astype(np.float32).tolist()
        msg.confidence = all_conf[selected].astype(np.float32).tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
