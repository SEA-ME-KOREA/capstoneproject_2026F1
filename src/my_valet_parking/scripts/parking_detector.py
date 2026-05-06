#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


EMPTY = 1
OCCUPIED = 2
PASSAGE = 3
UNKNOWN = -1


@dataclass(frozen=True)
class ParkingSlot:
    slot_id: int
    row_name: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    yaw: float = 0.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class ParkingDetector(Node):
    def __init__(self) -> None:
        super().__init__("parking_detector")

        self.marker_pub = self.create_publisher(
            MarkerArray, "/parking/slot_markers", 10
        )
        self.state_pub = self.create_publisher(
            Int32MultiArray, "/parking/slot_states", 10
        )
        self.debug_pub = self.create_publisher(Image, "/debug/image_raw", 10)

        self.create_subscription(Image, "/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.marker_timer = self.create_timer(0.2, self.publish_markers)

        self.slots = self.build_slots()
        self.slot_states = [UNKNOWN for _ in self.slots]
        self.confirmed_slots: Dict[int, ParkingSlot] = {}
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.latest_passage_marker: Optional[Marker] = None

        self.get_logger().info(
            "Parking detector ready: Canny/Hough parking-line skeleton "
            "detection + passage visualization."
        )

    def build_slots(self) -> List[ParkingSlot]:
        divider_xs = [
            -1.875,
            -1.50,
            -1.125,
            -0.75,
            -0.375,
            0.0,
            0.375,
            0.75,
            1.125,
            1.50,
            1.875,
        ]
        slot_width = 0.375 - 0.018
        slot_depth = 0.75 - 0.018
        row_centers = [
            ("upper", 0.75),
            ("lower_inner", -0.75),
            ("lower_outer", -1.65),
        ]

        slots: List[ParkingSlot] = []
        slot_id = 0
        for row_name, center_y in row_centers:
            for left_x, right_x in zip(divider_xs[:-1], divider_xs[1:]):
                slots.append(
                    ParkingSlot(
                        slot_id=slot_id,
                        row_name=row_name,
                        center_x=(left_x + right_x) * 0.5,
                        center_y=center_y,
                        size_x=slot_width,
                        size_y=slot_depth,
                        yaw=0.0,
                    )
                )
                slot_id += 1
        return slots

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose = (
            position.x,
            position.y,
            yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )

    def image_callback(self, msg: Image) -> None:
        frame = self.decode_image(msg)
        if frame is None or self.current_pose is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi_top = int(frame.shape[0] * 0.40)
        white_mask = self.white_mask_from_hsv(hsv, roi_top)
        edges = cv2.Canny(gray, 35, 110)
        debug_frame = frame.copy()
        debug_frame[white_mask > 0] = (0, 0, 255)

        self.slot_states = [UNKNOWN for _ in self.slots]
        self.confirmed_slots = {}

        for slot in self.slots:
            roi = self.project_slot_roi(slot, frame.shape[1], frame.shape[0])
            if roi is None:
                continue

            x0, y0, x1, y1 = roi
            roi_hsv = hsv[y0:y1, x0:x1]
            roi_white = white_mask[y0:y1, x0:x1]
            roi_edges = edges[y0:y1, x0:x1]
            if roi_hsv.size == 0:
                continue

            line_segments = self.detect_slot_line_segments(roi_edges)
            if not self.slot_directly_confirmed(line_segments):
                continue

            state = self.classify_slot_roi(
                roi_hsv,
                roi_white,
                roi_edges,
                line_segments,
            )
            self.slot_states[slot.slot_id] = state
            self.confirmed_slots[slot.slot_id] = slot

        self.latest_passage_marker = self.build_passage_marker(frame, white_mask)

        state_msg = Int32MultiArray()
        state_msg.data = list(self.slot_states)
        self.state_pub.publish(state_msg)
        self.debug_pub.publish(self.encode_debug_image(debug_frame, msg))

    def decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        channels = max(1, msg.step // msg.width)
        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warn("Failed to reshape camera frame in parking detector.")
            return None

        encoding = msg.encoding.lower()
        if encoding.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if encoding.startswith("bgr"):
            return frame[:, :, :3].copy()
        if encoding.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def white_mask_from_hsv(self, hsv: np.ndarray, roi_top: int) -> np.ndarray:
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        clipped_mask = np.zeros_like(mask)
        clipped_mask[roi_top:, :] = mask[roi_top:, :]
        return clipped_mask

    def encode_debug_image(self, frame: np.ndarray, source_msg: Image) -> Image:
        debug_msg = Image()
        debug_msg.header = source_msg.header
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = frame.shape[1] * 3
        debug_msg.data = frame.tobytes()
        return debug_msg

    def project_slot_roi(
        self,
        slot: ParkingSlot,
        image_width: int,
        image_height: int,
    ) -> Optional[Tuple[int, int, int, int]]:
        if self.current_pose is None:
            return None

        search_top = int(image_height * 0.40)
        search_bottom = image_height
        search_height = max(1, search_bottom - search_top)
        slot_index_in_row = slot.slot_id % 10
        row_index = slot.slot_id // 10

        # Search across the entire bottom 60% of the image to catch scaled lines.
        x_center = image_width * (0.10 + 0.088 * slot_index_in_row)
        y_center = search_top + search_height * (0.18 + 0.20 * row_index)
        roi_width = image_width * 0.16
        roi_height = image_height * 0.18

        x0 = int(clamp(x_center - roi_width * 0.5, 0, image_width - 1))
        x1 = int(clamp(x_center + roi_width * 0.5, x0 + 1, image_width))
        y0 = int(clamp(y_center - roi_height * 0.5, search_top, image_height - 1))
        y1 = int(clamp(y_center + roi_height * 0.5, y0 + 1, image_height))
        return (x0, y0, x1, y1)

    def detect_slot_line_segments(
        self,
        roi_edges: np.ndarray,
    ) -> List[Tuple[int, int, int, int]]:
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=10,
            minLineLength=max(6, roi_edges.shape[1] // 8),
            maxLineGap=16,
        )
        if lines is None:
            return []
        return [tuple(map(int, line[0])) for line in lines]

    def slot_directly_confirmed(
        self,
        line_segments: List[Tuple[int, int, int, int]],
    ) -> bool:
        if len(line_segments) < 3:
            return False

        vertical_segments = 0
        horizontal_segments = 0
        for x0, y0, x1, y1 in line_segments:
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            if dy > dx * 1.5:
                vertical_segments += 1
            elif dx > dy * 1.5:
                horizontal_segments += 1
        return vertical_segments >= 2 and horizontal_segments >= 1

    def classify_slot_roi(
        self,
        roi_hsv: np.ndarray,
        roi_white: np.ndarray,
        roi_edges: np.ndarray,
        line_segments: List[Tuple[int, int, int, int]],
    ) -> int:
        saturation = roi_hsv[:, :, 1]
        value = roi_hsv[:, :, 2]

        asphalt_mask = (saturation < 70) & (value > 35) & (value < 190)
        asphalt_ratio = float(np.count_nonzero(asphalt_mask)) / float(asphalt_mask.size)
        non_asphalt_ratio = 1.0 - asphalt_ratio
        edge_ratio = float(np.count_nonzero(roi_edges)) / float(roi_edges.size)
        white_ratio = float(np.count_nonzero(roi_white)) / float(roi_white.size)
        vertical_segments = 0
        horizontal_segments = 0
        for x0, y0, x1, y1 in line_segments:
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            if dy > dx * 1.5:
                vertical_segments += 1
            elif dx > dy * 1.5:
                horizontal_segments += 1

        # Relaxed logic: as soon as a U-shape skeleton is visible, publish Empty.
        if vertical_segments >= 2 and horizontal_segments >= 1:
            return EMPTY
        if white_ratio > 0.12 and edge_ratio < 0.05 and non_asphalt_ratio < 0.18:
            return PASSAGE
        return OCCUPIED

    def build_passage_marker(
        self,
        frame: np.ndarray,
        white_mask: np.ndarray,
    ) -> Optional[Marker]:
        if self.current_pose is None:
            return None

        lane_center = self.detect_corridor_center(white_mask)
        if lane_center is None:
            return None

        pose_x, pose_y, pose_yaw = self.current_pose
        forward_distance = 0.9
        world_x = pose_x + math.cos(pose_yaw) * forward_distance
        world_y = pose_y + math.sin(pose_yaw) * forward_distance

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "parking_passage"
        marker.id = 1000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = world_x
        marker.pose.position.y = world_y
        marker.pose.position.z = 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.2
        marker.scale.y = 0.70
        marker.scale.z = 0.02
        marker.color.a = 0.30
        marker.color.r = 0.20
        marker.color.g = 0.60
        marker.color.b = 1.00
        return marker

    def detect_corridor_center(self, white_mask: np.ndarray) -> Optional[float]:
        height, width = white_mask.shape[:2]
        roi = white_mask[
            int(height * 0.55): int(height * 0.90),
            int(width * 0.10): int(width * 0.90),
        ]
        if roi.size == 0:
            return None

        column_energy = np.sum(roi > 0, axis=0)
        half = column_energy.shape[0] // 2
        left_peak = int(np.argmax(column_energy[:half]))
        right_peak = int(np.argmax(column_energy[half:])) + half
        if column_energy[left_peak] < 10 or column_energy[right_peak] < 10:
            return None
        return float(int(width * 0.10) + (left_peak + right_peak) * 0.5)

    def publish_markers(self) -> None:
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        for slot_id, slot in self.confirmed_slots.items():
            state = self.slot_states[slot_id]
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "parking_slots"
            marker.id = slot.slot_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = slot.center_x
            marker.pose.position.y = slot.center_y
            marker.pose.position.z = 0.05
            marker.pose.orientation.z = math.sin(slot.yaw * 0.5)
            marker.pose.orientation.w = math.cos(slot.yaw * 0.5)
            marker.scale.x = slot.size_x
            marker.scale.y = slot.size_y
            marker.scale.z = 0.02
            marker.color.a = 0.28 if state == UNKNOWN else 0.45

            if state == EMPTY:
                marker.color.r = 0.1
                marker.color.g = 0.9
                marker.color.b = 0.2
            elif state == PASSAGE:
                marker.color.r = 0.2
                marker.color.g = 0.6
                marker.color.b = 1.0
            elif state == UNKNOWN:
                marker.color.r = 0.8
                marker.color.g = 0.8
                marker.color.b = 0.8
            else:
                marker.color.r = 0.9
                marker.color.g = 0.1
                marker.color.b = 0.1

            markers.markers.append(marker)

        for slot in self.slots:
            if slot.slot_id in self.confirmed_slots:
                continue
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "parking_slots_unknown"
            marker.id = 10000 + slot.slot_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = slot.center_x
            marker.pose.position.y = slot.center_y
            marker.pose.position.z = 0.03
            marker.pose.orientation.z = math.sin(slot.yaw * 0.5)
            marker.pose.orientation.w = math.cos(slot.yaw * 0.5)
            marker.scale.x = slot.size_x
            marker.scale.y = slot.size_y
            marker.scale.z = 0.01
            marker.color.a = 0.12
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
            markers.markers.append(marker)

        if self.latest_passage_marker is not None:
            markers.markers.append(self.latest_passage_marker)

        self.marker_pub.publish(markers)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = ParkingDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
