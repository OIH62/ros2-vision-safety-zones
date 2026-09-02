#!/usr/bin/env python3
from __future__ import annotations

import time
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from person_pose_msgs.msg import PersonKeypoints
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


# COCO keypoint order. Keep this table aligned with person_zone_cpp.
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


class PersonZoneDebugNode(Node):
    """Render person-zone diagnostics without performing pose inference."""

    def __init__(self) -> None:
        super().__init__("person_zone_debug_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("keypoints_topic", "/person/keypoints")
        self.declare_parameter("state_topic", "/person_zone_state")
        self.declare_parameter("position_topic", "/person_position")
        self.declare_parameter("exit_direction_topic", "/person_exit_direction")
        self.declare_parameter("warning_topic", "/person_emergency")
        self.declare_parameter("edge_warning_topic", "/person_edge_warning")
        self.declare_parameter("diagnostics_topic", "/person_zone/diagnostics")
        self.declare_parameter("pose_device_topic", "/person_pose/device")
        self.declare_parameter("debug_image_topic", "/person_zone/debug_image")
        self.declare_parameter("keypoint_confidence", 0.25)
        self.declare_parameter("boundary_margin_px", 10)
        self.declare_parameter("lost_person_frames", 20)
        self.declare_parameter("debug_fps", 15.0)

        color_topic = str(self.get_parameter("color_topic").value)
        keypoints_topic = str(self.get_parameter("keypoints_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        position_topic = str(self.get_parameter("position_topic").value)
        exit_direction_topic = str(
            self.get_parameter("exit_direction_topic").value
        )
        warning_topic = str(self.get_parameter("warning_topic").value)
        edge_warning_topic = str(
            self.get_parameter("edge_warning_topic").value
        )
        diagnostics_topic = str(
            self.get_parameter("diagnostics_topic").value
        )
        pose_device_topic = str(
            self.get_parameter("pose_device_topic").value
        )
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self.keypoint_confidence = float(
            self.get_parameter("keypoint_confidence").value
        )
        self.boundary_margin_px = int(
            self.get_parameter("boundary_margin_px").value
        )
        self.lost_person_frames = int(
            self.get_parameter("lost_person_frames").value
        )
        self.debug_fps = max(
            1.0, min(25.0, float(self.get_parameter("debug_fps").value))
        )

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_header = None
        self.image_generation = 0
        self.processed_generation = 0
        self.latest_keypoints: Optional[PersonKeypoints] = None
        self.latest_position: Optional[PointStamped] = None
        self.current_state = "NONE"
        self.exit_direction = "NONE"
        self.current_warning = False
        self.edge_warning = "NONE"
        self.diagnostics = "STARTING"
        self.pose_device = "starting"
        self.visible_keypoint_count = 0
        self.lost_count = 0
        self.color_times: deque[float] = deque()
        self.pose_times: deque[float] = deque()
        self.debug_times: deque[float] = deque()

        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.debug_pub = self.create_publisher(Image, debug_topic, debug_qos)

        self.color_sub = self.create_subscription(
            Image, color_topic, self.color_callback, qos_profile_sensor_data
        )
        self.keypoints_sub = self.create_subscription(
            PersonKeypoints,
            keypoints_topic,
            self.keypoints_callback,
            qos_profile_sensor_data,
        )
        self.state_sub = self.create_subscription(
            String, state_topic, self.state_callback, 10
        )
        self.position_sub = self.create_subscription(
            PointStamped, position_topic, self.position_callback, 10
        )
        self.exit_direction_sub = self.create_subscription(
            String,
            exit_direction_topic,
            self.exit_direction_callback,
            10,
        )
        self.warning_sub = self.create_subscription(
            Bool, warning_topic, self.warning_callback, 10
        )
        self.edge_warning_sub = self.create_subscription(
            String, edge_warning_topic, self.edge_warning_callback, 10
        )
        self.diagnostics_sub = self.create_subscription(
            String, diagnostics_topic, self.diagnostics_callback, 10
        )
        self.pose_device_sub = self.create_subscription(
            String, pose_device_topic, self.pose_device_callback, 10
        )

        self.timer = self.create_timer(1.0 / self.debug_fps, self.process)
        self.get_logger().info(
            f"Debug renderer started (visualization only, {self.debug_fps:.1f} Hz)"
        )

    def color_callback(self, msg: Image) -> None:
        self.record_rate(self.color_times)
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            ).copy()
            self.latest_image_header = msg.header
            self.image_generation += 1
        except Exception as error:
            self.get_logger().error(f"Color conversion failed: {error}")

    def keypoints_callback(self, msg: PersonKeypoints) -> None:
        self.record_rate(self.pose_times)
        self.latest_keypoints = msg

    def state_callback(self, msg: String) -> None:
        self.current_state = msg.data

    def position_callback(self, msg: PointStamped) -> None:
        self.latest_position = msg

    def exit_direction_callback(self, msg: String) -> None:
        self.exit_direction = msg.data

    def warning_callback(self, msg: Bool) -> None:
        self.current_warning = msg.data

    def edge_warning_callback(self, msg: String) -> None:
        self.edge_warning = msg.data

    def diagnostics_callback(self, msg: String) -> None:
        self.diagnostics = msg.data

    def pose_device_callback(self, msg: String) -> None:
        self.pose_device = msg.data

    @staticmethod
    def record_rate(samples: deque[float]) -> None:
        now = time.monotonic()
        samples.append(now)
        while samples and now - samples[0] > 2.0:
            samples.popleft()

    @staticmethod
    def measured_rate(samples: deque[float]) -> float:
        if len(samples) < 2:
            return 0.0
        duration = samples[-1] - samples[0]
        return (len(samples) - 1) / duration if duration > 0.0 else 0.0

    def keypoint_arrays(
        self, msg: Optional[PersonKeypoints], width: int, height: int
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if msg is None or not msg.detected:
            return None

        x = np.asarray(msg.x, dtype=np.float32)
        y = np.asarray(msg.y, dtype=np.float32)
        conf = np.asarray(msg.confidence, dtype=np.float32)
        if x.size == 0 or x.size != y.size or x.size != conf.size:
            return None

        xy = np.column_stack((x, y))
        valid = (
            (conf >= self.keypoint_confidence)
            & np.isfinite(xy).all(axis=1)
            & (xy[:, 0] >= 0.0)
            & (xy[:, 0] < width)
            & (xy[:, 1] >= 0.0)
            & (xy[:, 1] < height)
        )
        return xy, valid

    @staticmethod
    def representative_point(
        xy: np.ndarray, valid: np.ndarray
    ) -> Optional[Tuple[float, float]]:
        torso_indices = [index for index in (5, 6, 11, 12) if index < len(xy)]
        torso = xy[[index for index in torso_indices if valid[index]]]
        points = torso if len(torso) >= 2 else xy[valid]
        if len(points) == 0:
            return None
        return float(np.median(points[:, 0])), float(np.median(points[:, 1]))

    @staticmethod
    def human_ratios(
        xy: np.ndarray, valid: np.ndarray, width: int
    ) -> dict[str, float]:
        count = min(len(xy), len(KEYPOINT_WEIGHTS))
        if count == 0:
            return {"LEFT": 0.0, "CENTER": 0.0, "RIGHT": 0.0}

        weighted = {"LEFT": 0.0, "CENTER": 0.0, "RIGHT": 0.0}
        total_weight = 0.0
        boundary_1 = width / 3.0
        boundary_2 = 2.0 * width / 3.0

        for index in range(count):
            if not valid[index]:
                continue
            weight = float(KEYPOINT_WEIGHTS[index])
            x = float(xy[index, 0])
            if x < boundary_1:
                zone = "LEFT"
            elif x < boundary_2:
                zone = "CENTER"
            else:
                zone = "RIGHT"
            weighted[zone] += weight
            total_weight += weight

        if total_weight > 0.0:
            return {
                zone: weight / total_weight
                for zone, weight in weighted.items()
            }
        return {"LEFT": 0.0, "CENTER": 0.0, "RIGHT": 0.0}

    def draw_boundaries(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        boundary_1 = width // 3
        boundary_2 = 2 * width // 3
        margin = self.boundary_margin_px

        for boundary in (boundary_1, boundary_2):
            cv2.line(frame, (boundary, 0), (boundary, height), (255, 255, 255), 2)
            cv2.line(
                frame,
                (boundary - margin, 0),
                (boundary - margin, height),
                (150, 150, 150),
                1,
            )
            cv2.line(
                frame,
                (boundary + margin, 0),
                (boundary + margin, height),
                (150, 150, 150),
                1,
            )

        cv2.putText(
            frame, "LEFT", (35, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (255, 255, 255), 2
        )
        cv2.putText(
            frame, "CENTER", (boundary_1 + 35, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )
        cv2.putText(
            frame, "RIGHT", (boundary_2 + 35, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

    def draw_status(
        self,
        frame: np.ndarray,
        person_visible: bool,
        ratios: dict[str, float],
    ) -> None:
        height, width = frame.shape[:2]

        # Compact bottom status bar: status | human ratio | XYZ.
        bar_top = max(55, height - 100)
        ratio_x = int(width * 0.32)
        xyz_x = int(width * 0.70)
        cv2.rectangle(
            frame, (5, bar_top), (width - 5, height - 5), (0, 0, 0), -1
        )
        diagnostic_color = (
            (180, 180, 180)
            if self.diagnostics == "OK"
            else (0, 0, 255)
        )
        diagnostic_text = (
            f"RGB:{self.measured_rate(self.color_times):.0f} "
            f"Pose:{self.measured_rate(self.pose_times):.0f} "
            f"Dbg:{self.measured_rate(self.debug_times):.0f}Hz "
            f"KP:{self.visible_keypoint_count}/17 "
            f"Lost:{self.lost_count}/{self.lost_person_frames} "
            f"Edge:{self.edge_warning} Device:{self.pose_device} "
            f"{self.diagnostics}"
        )
        cv2.rectangle(
            frame, (5, bar_top - 23), (width - 5, bar_top - 2),
            (0, 0, 0), -1
        )
        cv2.putText(
            frame, diagnostic_text, (10, bar_top - 7),
            cv2.FONT_HERSHEY_SIMPLEX, 0.34, diagnostic_color, 1
        )
        cv2.line(
            frame, (ratio_x, bar_top + 7), (ratio_x, height - 12),
            (90, 90, 90), 1
        )
        cv2.line(
            frame, (xyz_x, bar_top + 7), (xyz_x, height - 12),
            (90, 90, 90), 1
        )

        status_lines = (
            f"ZONE: {self.current_state}",
            f"Warning: {'TRUE' if self.current_warning else 'FALSE'}",
            f"Exit: {self.exit_direction}",
        )
        for row, text in enumerate(status_lines):
            if row == 0:
                color = (0, 255, 255)
            elif row == 1 and self.current_warning:
                color = (0, 0, 255)
            else:
                color = (255, 255, 255)
            cv2.putText(
                frame, text, (14, bar_top + 25 + row * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if row == 0 else 0.45,
                color,
                2 if row == 0 else 1,
            )

        cv2.putText(
            frame, "HUMAN RATIO", (ratio_x + 10, bar_top + 21),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1
        )
        for row, zone in enumerate(("LEFT", "CENTER", "RIGHT")):
            cv2.putText(
                frame,
                f"{zone:<6}: {ratios[zone] * 100:5.1f}%",
                (ratio_x + 10, bar_top + 43 + row * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (255, 255, 255),
                1,
            )

        xyz_lines = ["XYZ (m)", "X: --", "Y: --", "Z: --"]
        if person_visible and self.latest_position is not None:
            point = self.latest_position.point
            xyz_lines = [
                "XYZ (m)",
                f"X: {point.x:.2f}",
                f"Y: {point.y:.2f}",
                f"Z: {point.z:.2f}",
            ]
        for row, text in enumerate(xyz_lines):
            cv2.putText(
                frame, text, (xyz_x + 10, bar_top + 20 + row * 21),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (200, 200, 200),
                1,
            )

        if not person_visible:
            text_size, _ = cv2.getTextSize(
                "NO PERSON", cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3
            )
            x = max(0, (width - text_size[0]) // 2)
            cv2.putText(
                frame, "NO PERSON", (x, max(70, height // 2)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3
            )

        if self.exit_direction != "NONE":
            warning = f"WARNING: {self.exit_direction} EXIT"
            text_size, _ = cv2.getTextSize(
                warning, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3
            )
            warning_x = max(0, (width - text_size[0]) // 2)
            cv2.rectangle(
                frame,
                (max(0, warning_x - 15), 50),
                (min(width - 1, warning_x + text_size[0] + 15), 105),
                (0, 0, 180),
                -1,
            )
            cv2.putText(
                frame, warning, (warning_x, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3
            )

    def process(self) -> None:
        if (
            self.latest_image is None
            or self.image_generation == self.processed_generation
        ):
            return

        self.processed_generation = self.image_generation
        frame = self.latest_image.copy()
        height, width = frame.shape[:2]
        self.draw_boundaries(frame)

        parsed = self.keypoint_arrays(self.latest_keypoints, width, height)
        representative = None
        visible = np.empty((0, 2), dtype=np.float32)
        ratios = {"LEFT": 0.0, "CENTER": 0.0, "RIGHT": 0.0}
        if parsed is not None:
            xy, valid = parsed
            visible = xy[valid]
            representative = self.representative_point(xy, valid)
            ratios = self.human_ratios(xy, valid, width)
            self.visible_keypoint_count = int(np.count_nonzero(valid))
        else:
            self.visible_keypoint_count = 0

        for x, y in visible:
            cv2.circle(frame, (int(x), int(y)), 4, (0, 255, 255), -1)
        if representative is not None:
            cv2.circle(
                frame,
                (int(representative[0]), int(representative[1])),
                8,
                (0, 0, 255),
                -1,
            )

        person_visible = representative is not None and len(visible) > 0
        self.lost_count = (
            0 if person_visible
            else min(self.lost_count + 1, self.lost_person_frames)
        )
        self.draw_status(frame, person_visible, ratios)

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        debug_msg = Image()
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = frame.shape[1] * 3
        debug_msg.data = frame.tobytes()
        if self.latest_image_header is not None:
            debug_msg.header = self.latest_image_header
        self.debug_pub.publish(debug_msg)
        self.record_rate(self.debug_times)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonZoneDebugNode()
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
