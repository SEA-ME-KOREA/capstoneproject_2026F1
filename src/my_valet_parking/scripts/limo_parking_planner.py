#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionStep(Enum):
    CAPTURE_INITIAL = "capture_initial"
    DRIVE_FORWARD = "drive_forward"
    SETTLE_AT_GOAL = "settle_at_goal"
    RETURN_BY_ODOM = "return_by_odom"
    CORRECTION = "correction"
    COMPLETE = "complete"


class LimoParkingPlanner(Node):
    def __init__(self) -> None:
        super().__init__("limo_parking_planner")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Image, "/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(
            Int32MultiArray,
            "/parking/slot_states",
            self.slot_state_callback,
            10,
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.camera_watchdog_timer = self.create_timer(1.0, self.check_camera_timeout)

        self.step = MissionStep.CAPTURE_INITIAL
        self.logged_steps = set()

        self.forward_target_m = 1.0
        self.odom_goal_tolerance_m = 0.08
        self.pixel_tolerance = 5.0
        self.max_drive_speed = 0.20
        self.max_reverse_speed = 0.12
        self.max_correction_speed = 0.03
        self.max_yaw_rate = 0.24
        self.goal_settle_cycles = 15

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: float = 0.0
        self.start_position: Optional[Tuple[float, float]] = None
        self.start_yaw: Optional[float] = None

        self.latest_frame: Optional[np.ndarray] = None
        self.reference_lane_center_px: Optional[float] = None
        self.current_lane_center_px: Optional[float] = None
        self.current_lane_offset_px: Optional[float] = None
        self.current_corridor_center_px: Optional[float] = None
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False
        self.last_capture_wait_log_time = self.get_clock().now()
        self.capture_start_time = self.get_clock().now()
        self.current_white_pixel_count = 0

        self.forward_path: List[Tuple[float, float, float]] = []
        self.reverse_target_index: int = 0
        self.stop_cycles_remaining = 0
        self.latest_slot_states: List[int] = []
        self.logged_empty_slot: Optional[int] = None
        self.empty_slot_confirmed = False
        self.empty_slot_started_drive = False
        self.target_stop_pose: Optional[Tuple[float, float, float]] = None

        self.get_logger().info(
            "Parking planner ready: 1.0 m forward exploration, reverse to "
            "recorded empty-slot pose, final vision correction."
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        self.current_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        if self.start_position is None:
            self.start_position = self.current_position
            self.start_yaw = self.current_yaw

        if self.step == MissionStep.DRIVE_FORWARD:
            self.record_forward_path()

    def image_callback(self, msg: Image) -> None:
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False
        frame = self.decode_image(msg)
        if frame is None:
            return

        self.latest_frame = frame
        self.current_white_pixel_count = self.count_white_pixels(frame)
        lane_features = self.detect_lane_features(frame)
        self.current_corridor_center_px = lane_features[2] if lane_features is not None else None
        self.current_lane_center_px = self.detect_white_lane_center(frame)
        if (
            self.reference_lane_center_px is not None
            and self.current_lane_center_px is not None
        ):
            self.current_lane_offset_px = (
                self.current_lane_center_px - self.reference_lane_center_px
            )
        else:
            self.current_lane_offset_px = None

        if self.step == MissionStep.CAPTURE_INITIAL and self.capture_initial_frame():
            self.step = MissionStep.DRIVE_FORWARD

    def slot_state_callback(self, msg: Int32MultiArray) -> None:
        self.latest_slot_states = list(msg.data)
        if self.logged_empty_slot is None:
            for slot_index, state in enumerate(self.latest_slot_states):
                if state == 1:
                    self.logged_empty_slot = slot_index
                    self.empty_slot_confirmed = True
                    self.get_logger().info(
                        f"Perception reported first empty slot index: {slot_index}"
                    )
                    if (
                        self.step == MissionStep.DRIVE_FORWARD
                        and self.current_position is not None
                        and self.target_stop_pose is None
                    ):
                        self.target_stop_pose = (
                            self.current_position[0],
                            self.current_position[1],
                            self.current_yaw,
                        )
                        self.get_logger().info(
                            "Recorded target_stop_pose from empty-slot detection at "
                            f"({self.target_stop_pose[0]:.2f}, "
                            f"{self.target_stop_pose[1]:.2f}, "
                            f"{math.degrees(self.target_stop_pose[2]):.1f} deg)."
                        )
                    break

    def decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        channels = max(1, msg.step // msg.width)
        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warn("Failed to reshape camera frame in parking planner.")
            return None

        encoding = msg.encoding.lower()
        if encoding.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if encoding.startswith("bgr"):
            return frame[:, :, :3].copy()
        if encoding.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def detect_white_lane_center(
        self,
        frame: np.ndarray,
        top_ratio: float = 0.40,
        bottom_ratio: float = 1.0,
    ) -> Optional[float]:
        height, width = frame.shape[:2]
        roi = frame[
            int(height * top_ratio): int(height * bottom_ratio),
            int(width * 0.10): int(width * 0.90),
        ]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 45, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        moments = cv2.moments(mask)
        if moments["m00"] <= 0.0:
            return None

        center_x = float(moments["m10"] / moments["m00"])
        roi_left = float(int(width * 0.10))
        return roi_left + center_x

    def detect_lane_features(
        self,
        frame: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        height, width = frame.shape[:2]
        roi = frame[
            int(height * 0.40): int(height * 1.00),
            int(width * 0.08): int(width * 0.92),
        ]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 45, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        histogram = np.sum(mask > 0, axis=0)
        if histogram.size < 4:
            return None

        half = histogram.shape[0] // 2
        left_index = int(np.argmax(histogram[:half]))
        right_index = int(np.argmax(histogram[half:])) + half
        if histogram[left_index] < 10 or histogram[right_index] < 10:
            return None

        roi_left = float(int(width * 0.08))
        left_px = roi_left + left_index
        right_px = roi_left + right_index
        center_px = (left_px + right_px) * 0.5
        return (left_px, right_px, center_px)

    def count_white_pixels(self, frame: np.ndarray) -> int:
        height, width = frame.shape[:2]
        roi = frame[int(height * 0.40):, :]
        if roi.size == 0:
            return 0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return int(np.count_nonzero(mask))

    def signed_progress_from_start(self) -> Optional[float]:
        if (
            self.current_position is None
            or self.start_position is None
            or self.start_yaw is None
        ):
            return None

        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        heading_x = math.cos(self.start_yaw)
        heading_y = math.sin(self.start_yaw)
        return dx * heading_x + dy * heading_y

    def capture_initial_frame(self) -> bool:
        if self.latest_frame is None:
            self.log_capture_wait("Waiting for the first camera frame.")
            return False

        reference_center = self.detect_white_lane_center(
            self.latest_frame,
            top_ratio=0.40,
            bottom_ratio=1.0,
        )
        if reference_center is None:
            self.log_capture_wait(
                "CAPTURE_INITIAL waiting: initial parking line not detected yet."
            )
            return False

        self.reference_lane_center_px = reference_center
        self.current_lane_center_px = reference_center
        self.current_lane_offset_px = 0.0
        self.forward_path.clear()
        self.record_forward_path(force=True)
        self.get_logger().info(
            "Captured initial lane center reference at "
            f"{self.reference_lane_center_px:.1f} px."
        )
        return True

    def log_capture_wait(self, message: str) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_capture_wait_log_time).nanoseconds / 1e9
        if elapsed >= 2.0:
            self.get_logger().info(message)
            self.last_capture_wait_log_time = now

    def compare_with_initial_frame(self) -> Optional[float]:
        if self.reference_lane_center_px is None or self.latest_frame is None:
            return None

        current_center = self.detect_white_lane_center(self.latest_frame)
        if current_center is None:
            return None

        self.current_lane_center_px = current_center
        self.current_lane_offset_px = current_center - self.reference_lane_center_px
        return self.current_lane_offset_px

    def check_camera_timeout(self) -> None:
        elapsed = (
            self.get_clock().now() - self.last_image_received_time
        ).nanoseconds / 1e9
        if elapsed > 10.0 and not self.camera_timeout_logged:
            self.get_logger().error(
                "No camera data received for over 10 seconds. Restart the "
                "simulation environment."
            )
            self.camera_timeout_logged = True

    def record_forward_path(self, force: bool = False) -> None:
        if self.current_position is None:
            return

        pose = (self.current_position[0], self.current_position[1], self.current_yaw)
        if not self.forward_path:
            self.forward_path.append(pose)
            return

        last_x, last_y, _last_yaw = self.forward_path[-1]
        distance = math.hypot(pose[0] - last_x, pose[1] - last_y)
        if force or distance >= 0.05:
            self.forward_path.append(pose)

    def build_forward_command(self, remaining_distance: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = clamp(abs(remaining_distance) * 0.35, 0.05, self.max_drive_speed)
        if self.current_corridor_center_px is not None and self.latest_frame is not None:
            image_center = self.latest_frame.shape[1] * 0.5
            corridor_error = self.current_corridor_center_px - image_center
            cmd.angular.z = clamp(
                -0.005 * corridor_error,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )
        elif self.start_yaw is not None:
            # If lane features disappear, keep the vehicle on a straight odom heading.
            yaw_error = math.atan2(
                math.sin(self.start_yaw - self.current_yaw),
                math.cos(self.start_yaw - self.current_yaw),
            )
            cmd.angular.z = clamp(
                yaw_error * 1.2,
                -0.12,
                0.12,
            )
        return cmd

    def drive_forward(self) -> bool:
        progress = self.signed_progress_from_start()
        if progress is None:
            return False

        remaining = self.forward_target_m - progress
        if (
            progress >= self.forward_target_m
            or abs(remaining) <= self.odom_goal_tolerance_m
        ):
            self.publish_stop()
            return True

        cmd = self.build_forward_command(remaining)
        self.cmd_pub.publish(cmd)
        return False

    def run_reverse_tracking(self) -> bool:
        if self.current_position is None:
            return False

        if self.target_stop_pose is None:
            if not self.forward_path:
                return False
            if self.reverse_target_index <= 0:
                self.publish_stop()
                return True
            target_x, target_y, _target_yaw = self.forward_path[self.reverse_target_index]
        else:
            target_x, target_y, _target_yaw = self.target_stop_pose
        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - self.current_yaw),
            math.cos(target_heading - self.current_yaw),
        )

        if distance < self.odom_goal_tolerance_m:
            if self.target_stop_pose is not None:
                self.publish_stop()
                return True
            self.reverse_target_index = max(0, self.reverse_target_index - 1)
            return False

        cmd = Twist()
        cmd.linear.x = -clamp(distance * 0.6, 0.04, self.max_reverse_speed)
        cmd.angular.z = clamp(
            heading_error * 1.2,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        self.cmd_pub.publish(cmd)
        return False

    def run_correction_step(self) -> bool:
        offset_px = self.compare_with_initial_frame()
        if offset_px is None:
            self.publish_stop()
            self.get_logger().warn(
                "Correction paused: white parking line is not visible."
            )
            return False

        progress = self.signed_progress_from_start()
        distance_to_target: Optional[float] = None
        longitudinal_error = 0.0
        if self.current_position is not None:
            if self.target_stop_pose is not None:
                target_x, target_y, target_yaw = self.target_stop_pose
            elif self.start_position is not None and self.start_yaw is not None:
                target_x, target_y = self.start_position
                target_yaw = self.start_yaw
            else:
                target_x = target_y = target_yaw = None
            if target_x is not None:
                dx = self.current_position[0] - target_x
                dy = self.current_position[1] - target_y
                distance_to_target = math.hypot(dx, dy)
                longitudinal_error = (
                    dx * math.cos(target_yaw) + dy * math.sin(target_yaw)
                )
        if (
            abs(offset_px) <= self.pixel_tolerance
            and (distance_to_target is None or distance_to_target <= 0.05)
        ):
            self.publish_stop()
            self.get_logger().info(
                f"Correction complete. Lane offset {offset_px:.2f} px within tolerance."
            )
            return True

        cmd = Twist()
        if distance_to_target is not None:
            cmd.linear.x = clamp(
                -0.5 * longitudinal_error,
                -self.max_correction_speed,
                self.max_correction_speed,
            )
        elif progress is not None:
            cmd.linear.x = clamp(
                -0.5 * progress,
                -self.max_correction_speed,
                self.max_correction_speed,
            )
        cmd.angular.z = clamp(-0.003 * offset_px, -self.max_yaw_rate, self.max_yaw_rate)
        self.cmd_pub.publish(cmd)
        return False

    def control_loop(self) -> None:
        if self.step not in self.logged_steps:
            self.get_logger().info(f"Mission step -> {self.step.value}")
            self.logged_steps.add(self.step)

        if self.step == MissionStep.CAPTURE_INITIAL:
            capture_elapsed = (
                self.get_clock().now() - self.capture_start_time
            ).nanoseconds / 1e9
            if not self.empty_slot_confirmed:
                self.log_capture_wait(
                    "CAPTURE_INITIAL waiting: no camera-confirmed Empty slot yet."
                )
            if (
                capture_elapsed >= 3.0
                and self.latest_frame is not None
                and self.current_white_pixel_count >= 250
            ):
                if self.reference_lane_center_px is None:
                    if self.current_corridor_center_px is not None:
                        self.reference_lane_center_px = self.current_corridor_center_px
                    else:
                        self.reference_lane_center_px = self.latest_frame.shape[1] * 0.5
                self.current_lane_center_px = self.reference_lane_center_px
                self.current_lane_offset_px = 0.0
                self.forward_path.clear()
                self.record_forward_path(force=True)
                self.get_logger().info(
                    "CAPTURE_INITIAL timeout reached. White pixels detected, "
                    "forcing DRIVE_FORWARD."
                )
                self.step = MissionStep.DRIVE_FORWARD
                return
            if self.empty_slot_confirmed and not self.empty_slot_started_drive:
                self.get_logger().info(
                    "Empty slot signal received. Forcing mission start into DRIVE_FORWARD."
                )
                self.empty_slot_started_drive = True
                self.forward_path.clear()
                self.record_forward_path(force=True)
                self.step = MissionStep.DRIVE_FORWARD
                return
            if self.capture_initial_frame():
                self.step = MissionStep.DRIVE_FORWARD
            return

        if self.step == MissionStep.DRIVE_FORWARD:
            if self.drive_forward():
                self.stop_cycles_remaining = self.goal_settle_cycles
                self.reverse_target_index = max(0, len(self.forward_path) - 1)
                if self.target_stop_pose is None and self.current_position is not None:
                    self.target_stop_pose = (
                        self.current_position[0],
                        self.current_position[1],
                        self.current_yaw,
                    )
                    self.get_logger().warn(
                        "No empty-slot pose was recorded during forward drive. "
                        "Using the 1.0 m goal pose as target_stop_pose fallback."
                    )
                self.get_logger().info(
                    "Reached 1.0 m goal. Stopping before reverse return to "
                    "target_stop_pose."
                )
                self.step = MissionStep.SETTLE_AT_GOAL
            return

        if self.step == MissionStep.SETTLE_AT_GOAL:
            self.publish_stop()
            self.stop_cycles_remaining -= 1
            if self.stop_cycles_remaining <= 0:
                self.step = MissionStep.RETURN_BY_ODOM
            return

        if self.step == MissionStep.RETURN_BY_ODOM:
            if self.compare_with_initial_frame() is not None:
                self.get_logger().info(
                    "Initial parking line re-detected during reverse. "
                    "Switching to vision correction immediately."
                )
                self.step = MissionStep.CORRECTION
                return
            if self.run_reverse_tracking():
                self.get_logger().info(
                    "Reverse return to target_stop_pose completed. Switching to "
                    "vision correction."
                )
                self.step = MissionStep.CORRECTION
            return

        if self.step == MissionStep.CORRECTION:
            if self.run_correction_step():
                self.step = MissionStep.COMPLETE
            return

        self.publish_stop()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = LimoParkingPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
