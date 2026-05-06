#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionState(Enum):
    CAPTURE_INITIAL = "capture_initial"
    DRIVE_FORWARD = "drive_forward"
    STOP_BEFORE_REVERSE = "stop_before_reverse"
    DRIVE_REVERSE = "drive_reverse"
    VISION_CORRECTION = "vision_correction"
    COMPLETE = "complete"


class SimpleStraightMission(Node):
    def __init__(self) -> None:
        super().__init__("simple_straight_mission")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_image_pub = self.create_publisher(Image, "/debug/image_raw", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Image, "/rgb/image_raw", self.image_callback, 10)

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.camera_watchdog_timer = self.create_timer(1.0, self.check_camera_timeout)

        self.state = MissionState.CAPTURE_INITIAL
        self.logged_states = set()

        self.drive_speed_mps = 0.08
        self.correction_speed_mps = 0.02
        self.forward_target_m = 1.0
        self.reverse_target_progress_m = 0.0
        self.stop_before_reverse_sec = 0.5
        self.final_distance_tolerance_m = 0.05
        self.pixel_tolerance_px = 3.0
        self.angle_tolerance_deg = 4.0
        self.max_heading_rate = 0.18
        self.expected_drive_time_sec = self.forward_target_m / self.drive_speed_mps
        self.vision_takeover_distance_m = 0.60
        self.forward_pixel_kp = 0.0020
        self.forward_pixel_kd = 0.0008
        self.forward_angle_kp = 0.90
        self.correction_pixel_gain = 0.0060
        self.correction_angle_gain = 1.40

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw = 0.0
        self.start_position: Optional[Tuple[float, float]] = None
        self.start_yaw: Optional[float] = None

        self.latest_frame: Optional[np.ndarray] = None
        self.initial_lane_center_px: Optional[float] = None
        self.initial_lane_angle_rad: Optional[float] = None
        self.current_lane_center_px: Optional[float] = None
        self.current_lane_angle_rad: Optional[float] = None
        self.current_lane_error_px: Optional[float] = None
        self.current_lane_angle_error_rad: Optional[float] = None
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False
        self.last_status_log_time = self.get_clock().now()
        self.capture_start_time = self.get_clock().now()
        self.current_white_pixel_count = 0
        self.stop_until_time = None
        self.last_pixel_error_for_pd = 0.0
        self.last_pd_time_sec: Optional[float] = None

        self.get_logger().info(
            "[INFO] Simple straight mission ready: capture initial line, "
            "drive forward 1.0 m, reverse back by odometry, then final vision correction."
        )
        self.get_logger().info(
            "[INFO] Parameters: forward_target_m=1.0 m, drive_speed=0.08 m/s, "
            f"expected_pure_drive_time={self.expected_drive_time_sec:.1f} s."
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

    def image_callback(self, msg: Image) -> None:
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False

        frame = self.decode_image(msg)
        if frame is None:
            return

        self.latest_frame = frame
        self.current_white_pixel_count = self.count_white_pixels_near_bottom(frame)

        if self.state in (
            MissionState.CAPTURE_INITIAL,
            MissionState.DRIVE_FORWARD,
            MissionState.DRIVE_REVERSE,
            MissionState.VISION_CORRECTION,
        ):
            line_pose = self.detect_parking_line_pose(frame)
            if line_pose is not None:
                self.current_lane_center_px, self.current_lane_angle_rad = line_pose
            else:
                self.current_lane_center_px = None
                self.current_lane_angle_rad = None
            if (
                self.initial_lane_center_px is not None
                and self.current_lane_center_px is not None
            ):
                self.current_lane_error_px = (
                    self.current_lane_center_px - self.initial_lane_center_px
                )
            else:
                self.current_lane_error_px = None
            if (
                self.initial_lane_angle_rad is not None
                and self.current_lane_angle_rad is not None
            ):
                self.current_lane_angle_error_rad = math.atan2(
                    math.sin(self.current_lane_angle_rad - self.initial_lane_angle_rad),
                    math.cos(self.current_lane_angle_rad - self.initial_lane_angle_rad),
                )
            else:
                self.current_lane_angle_error_rad = None

        self.publish_debug_image(frame)

    def decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        channels = max(1, msg.step // msg.width)

        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warn("[INFO] Camera frame reshape failed.")
            return None

        encoding = msg.encoding.lower()
        if encoding.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if encoding.startswith("bgr"):
            return frame[:, :, :3].copy()
        if encoding.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def detect_white_mask(self, frame: np.ndarray) -> Tuple[np.ndarray, int, int]:
        height, width = frame.shape[:2]
        roi = frame[
            int(height * 0.40): height,
            int(width * 0.10): int(width * 0.90),
        ]
        if roi.size == 0:
            return np.zeros((1, 1), dtype=np.uint8), int(height * 0.40), int(width * 0.10)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask, int(height * 0.40), int(width * 0.10)

    def detect_parking_line_pose(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        mask, roi_top, roi_left = self.detect_white_mask(frame)
        ys, xs = np.where(mask > 0)
        if xs.size < 20:
            return None

        moments = cv2.moments(mask)
        if moments["m00"] <= 0.0:
            return None

        center_x = float(moments["m10"] / moments["m00"])
        points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = math.atan2(float(vy), float(vx))
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if angle > math.pi / 2.0:
            angle -= math.pi
        if angle < -math.pi / 2.0:
            angle += math.pi

        return float(roi_left) + center_x, angle

    def detect_parking_line_center(self, frame: np.ndarray) -> Optional[float]:
        line_pose = self.detect_parking_line_pose(frame)
        if line_pose is None:
            return None
        return line_pose[0]

    def count_white_pixels_near_bottom(self, frame: np.ndarray) -> int:
        height = frame.shape[0]
        roi = frame[max(0, height - 100): height, :]
        if roi.size == 0:
            return 0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([179, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return int(np.count_nonzero(mask))

    def publish_debug_image(self, frame: np.ndarray) -> None:
        debug_frame = frame.copy()
        mask, roi_top, roi_left = self.detect_white_mask(frame)
        ys, xs = np.where(mask > 0)

        if xs.size > 0:
            sample_step = max(1, xs.size // 800)
            for idx in range(0, xs.size, sample_step):
                px = roi_left + int(xs[idx])
                py = roi_top + int(ys[idx])
                cv2.circle(debug_frame, (px, py), 1, (0, 0, 255), -1)

        if self.current_lane_center_px is not None:
            cv2.circle(
                debug_frame,
                (int(self.current_lane_center_px), debug_frame.shape[0] - 30),
                6,
                (0, 255, 0),
                -1,
            )
        if self.current_lane_center_px is not None and self.current_lane_angle_rad is not None:
            cx = int(self.current_lane_center_px)
            cy = int(roi_top + max(10, mask.shape[0] * 0.4))
            dx = int(80 * math.cos(self.current_lane_angle_rad))
            dy = int(80 * math.sin(self.current_lane_angle_rad))
            cv2.line(debug_frame, (cx - dx, cy - dy), (cx + dx, cy + dy), (255, 0, 0), 2)

        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_debug"
        msg.height = debug_frame.shape[0]
        msg.width = debug_frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = debug_frame.shape[1] * 3
        msg.data = debug_frame.tobytes()
        self.debug_image_pub.publish(msg)

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

    def distance_from_start(self) -> Optional[float]:
        if self.current_position is None or self.start_position is None:
            return None

        return math.hypot(
            self.current_position[0] - self.start_position[0],
            self.current_position[1] - self.start_position[1],
        )

    def heading_error_to_start(self) -> float:
        if self.current_position is None or self.start_position is None:
            return 0.0

        target_heading = math.atan2(
            self.start_position[1] - self.current_position[1],
            self.start_position[0] - self.current_position[0],
        )
        return math.atan2(
            math.sin(target_heading - self.current_yaw),
            math.cos(target_heading - self.current_yaw),
        )

    def capture_initial_reference(self) -> bool:
        if self.latest_frame is None:
            self.get_logger().info("[INFO] CAPTURE_INITIAL: waiting for camera frame.")
            return False

        line_pose = self.detect_parking_line_pose(self.latest_frame)
        if line_pose is None:
            self.get_logger().info(
                "[INFO] CAPTURE_INITIAL: waiting for visible parking line."
            )
            return False

        lane_center, lane_angle = line_pose
        self.initial_lane_center_px = lane_center
        self.initial_lane_angle_rad = lane_angle
        self.current_lane_center_px = lane_center
        self.current_lane_angle_rad = lane_angle
        self.current_lane_error_px = 0.0
        self.current_lane_angle_error_rad = 0.0
        self.get_logger().info(
            "[INFO] CAPTURE_INITIAL: recorded reference line center at "
            f"{lane_center:.2f}px and angle {math.degrees(lane_angle):.2f} deg."
        )
        return True

    def should_force_capture_initial(self) -> bool:
        elapsed = (
            self.get_clock().now() - self.capture_start_time
        ).nanoseconds / 1e9
        return elapsed >= 5.0 and self.current_white_pixel_count >= 100

    def drive_forward(self) -> bool:
        progress = self.signed_progress_from_start()
        if progress is None:
            return False

        if progress >= self.forward_target_m:
            self.publish_stop()
            return True

        cmd = Twist()
        cmd.linear.x = self.drive_speed_mps
        if (
            self.current_lane_error_px is not None
            and self.current_lane_angle_error_rad is not None
        ):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self.last_pd_time_sec is None:
                dt = 0.05
            else:
                dt = max(1e-3, now_sec - self.last_pd_time_sec)
            derivative = (self.current_lane_error_px - self.last_pixel_error_for_pd) / dt
            cmd.angular.z = clamp(
                -self.forward_pixel_kp * self.current_lane_error_px
                - self.forward_pixel_kd * derivative
                - self.forward_angle_kp * self.current_lane_angle_error_rad,
                -self.max_heading_rate,
                self.max_heading_rate,
            )
            self.last_pixel_error_for_pd = self.current_lane_error_px
            self.last_pd_time_sec = now_sec
        elif self.start_yaw is not None:
            yaw_error = math.atan2(
                math.sin(self.start_yaw - self.current_yaw),
                math.cos(self.start_yaw - self.current_yaw),
            )
            cmd.angular.z = clamp(yaw_error * 1.2, -0.12, 0.12)
        self.cmd_pub.publish(cmd)
        return False

    def drive_reverse(self) -> bool:
        progress = self.signed_progress_from_start()
        if progress is None:
            return False

        if progress <= self.reverse_target_progress_m:
            self.publish_stop()
            return True

        cmd = Twist()
        cmd.linear.x = -self.drive_speed_mps
        if (
            self.current_lane_error_px is not None
            and self.current_lane_angle_error_rad is not None
        ):
            cmd.angular.z = clamp(
                -self.correction_pixel_gain * self.current_lane_error_px
                - self.correction_angle_gain * self.current_lane_angle_error_rad,
                -self.max_heading_rate,
                self.max_heading_rate,
            )
        else:
            cmd.angular.z = clamp(self.heading_error_to_start() * 1.2, -0.12, 0.12)
        self.cmd_pub.publish(cmd)
        return False

    def run_vision_correction(self) -> bool:
        distance = self.distance_from_start()
        progress = self.signed_progress_from_start()

        if distance is None or progress is None:
            return False

        if (
            self.current_lane_center_px is None
            or self.current_lane_error_px is None
            or self.current_lane_angle_error_rad is None
        ):
            self.publish_stop()
            self.get_logger().info(
                "[INFO] VISION_CORRECTION: parking line not visible, pausing."
            )
            return False

        pixel_error = self.current_lane_error_px
        angle_error_deg = math.degrees(self.current_lane_angle_error_rad)
        if (
            abs(pixel_error) <= self.pixel_tolerance_px
            and abs(angle_error_deg) <= self.angle_tolerance_deg
            and distance <= self.final_distance_tolerance_m
        ):
            self.publish_stop()
            self.get_logger().info(
                "[INFO] VISION_CORRECTION: completed with "
                f"pixel error {pixel_error:.2f}px, angle error {angle_error_deg:.2f} deg "
                f"and distance {distance:.3f}m."
            )
            return True

        cmd = Twist()
        cmd.linear.x = clamp(
            -0.5 * progress,
            -self.correction_speed_mps,
            self.correction_speed_mps,
        )
        cmd.angular.z = clamp(
            -self.correction_pixel_gain * pixel_error
            - self.correction_angle_gain * self.current_lane_angle_error_rad,
            -self.max_heading_rate,
            self.max_heading_rate,
        )
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            "[INFO] VISION_CORRECTION: correcting with "
            f"distance={distance:.3f}m, pixel_error={pixel_error:.2f}px, "
            f"angle_error={angle_error_deg:.2f} deg."
        )
        return False

    def check_camera_timeout(self) -> None:
        elapsed = (
            self.get_clock().now() - self.last_image_received_time
        ).nanoseconds / 1e9
        if elapsed > 10.0 and not self.camera_timeout_logged:
            self.get_logger().error(
                "[INFO] No camera data received for over 10 seconds."
            )
            self.camera_timeout_logged = True

    def log_status(self, message: str, interval_sec: float = 1.0) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_status_log_time).nanoseconds / 1e9
        if elapsed >= interval_sec:
            self.get_logger().info(message)
            self.last_status_log_time = now

    def control_loop(self) -> None:
        if self.state not in self.logged_states:
            self.get_logger().info(f"[INFO] State -> {self.state.value}")
            self.logged_states.add(self.state)

        if self.state == MissionState.CAPTURE_INITIAL:
            if self.capture_initial_reference():
                self.last_pd_time_sec = None
                self.last_pixel_error_for_pd = 0.0
                self.state = MissionState.DRIVE_FORWARD
                return
            if self.should_force_capture_initial():
                if self.latest_frame is not None:
                    fallback_pose = self.detect_parking_line_pose(self.latest_frame)
                    if fallback_pose is None:
                        fallback_center = self.latest_frame.shape[1] * 0.5
                        fallback_angle = 0.0
                    else:
                        fallback_center, fallback_angle = fallback_pose
                    self.initial_lane_angle_rad = fallback_angle
                    self.current_lane_angle_rad = fallback_angle
                    self.current_lane_angle_error_rad = 0.0
                    if fallback_pose is None:
                        fallback_center = self.latest_frame.shape[1] * 0.5
                    self.initial_lane_center_px = fallback_center
                    self.current_lane_center_px = fallback_center
                    self.current_lane_error_px = 0.0
                self.get_logger().info(
                    "[INFO] CAPTURE_INITIAL: 5 s timeout reached with bottom white "
                    f"pixels={self.current_white_pixel_count}. Forcing DRIVE_FORWARD."
                )
                self.last_pd_time_sec = None
                self.last_pixel_error_for_pd = 0.0
                self.state = MissionState.DRIVE_FORWARD
            return

        if self.state == MissionState.DRIVE_FORWARD:
            self.log_status(
                "[INFO] DRIVE_FORWARD: driving 1.0 m by odometry at 0.08 m/s "
                f"(expected {self.expected_drive_time_sec:.1f} s)."
            )
            if self.drive_forward():
                self.get_logger().info(
                    "[INFO] DRIVE_FORWARD: reached 1.0 m, stopping for 0.5 s "
                    "before reverse."
                )
                self.stop_until_time = (
                    self.get_clock().now().nanoseconds / 1e9
                    + self.stop_before_reverse_sec
                )
                self.state = MissionState.STOP_BEFORE_REVERSE
            return

        if self.state == MissionState.STOP_BEFORE_REVERSE:
            self.publish_stop()
            self.log_status(
                "[INFO] STOP_BEFORE_REVERSE: holding stop for 0.5 s to reduce overshoot."
            )
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self.stop_until_time is not None and now_sec >= self.stop_until_time:
                self.get_logger().info(
                    "[INFO] STOP_BEFORE_REVERSE: stop hold complete, switching to reverse."
                )
                self.state = MissionState.DRIVE_REVERSE
            return

        if self.state == MissionState.DRIVE_REVERSE:
            distance = self.distance_from_start()
            if (
                distance is not None
                and distance <= self.vision_takeover_distance_m
                and self.current_lane_error_px is not None
                and self.current_lane_angle_error_rad is not None
            ):
                self.get_logger().info(
                    "[INFO] DRIVE_REVERSE: line visible within 0.6 m, prioritizing "
                    "vision correction."
                )
                self.state = MissionState.VISION_CORRECTION
                return
            self.log_status(
                "[INFO] DRIVE_REVERSE: reversing 1.0 m back toward the start pose "
                "by odometry."
            )
            if self.drive_reverse():
                self.get_logger().info(
                    "[INFO] DRIVE_REVERSE: returned to the start progress target, "
                    "switching to vision correction."
                )
                self.state = MissionState.VISION_CORRECTION
            return

        if self.state == MissionState.VISION_CORRECTION:
            if self.run_vision_correction():
                self.state = MissionState.COMPLETE
            return

        self.publish_stop()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimpleStraightMission()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
