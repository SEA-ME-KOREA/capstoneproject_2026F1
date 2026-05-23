import math
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MissionStep(Enum):
    CAPTURE_INITIAL = "capture_initial"
    DRIVE_FORWARD = "drive_forward"
    DRIVE_BACKWARD = "drive_backward"
    CORRECTION = "correction"
    COMPLETE = "complete"


class LimoLaneMission(Node):
    def __init__(self) -> None:
        super().__init__("limo_lane_mission")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Image, "/rgb/image_raw", self.image_callback, 10)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.camera_watchdog_timer = self.create_timer(1.0, self.check_camera_timeout)

        self.step = MissionStep.CAPTURE_INITIAL
        self.logged_steps = set()

        self.forward_target_m = 0.30
        self.backward_target_from_start_m = 0.10
        self.odom_goal_tolerance_m = 0.10
        self.pixel_tolerance = 6.0
        self.max_drive_speed = 0.08
        self.max_correction_speed = 0.02
        self.max_correction_yaw_rate = 0.12

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: float = 0.0
        self.start_position: Optional[Tuple[float, float]] = None
        self.start_yaw: Optional[float] = None

        self.latest_frame: Optional[np.ndarray] = None
        self.reference_frame: Optional[np.ndarray] = None
        self.reference_lane_center_px: Optional[float] = None
        self.current_lane_center_px: Optional[float] = None
        self.current_lane_offset_px: Optional[float] = None
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False

        self.get_logger().info(
            "Lane mission initialized: capture, forward 0.30 m, backward "
            "to 0.10 m, then vision correction."
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        if self.start_position is None:
            self.start_position = self.current_position
            self.start_yaw = self.current_yaw

    def image_callback(self, msg: Image) -> None:
        self.get_logger().info("Raw Image Data Detected")
        self.last_image_received_time = self.get_clock().now()
        self.camera_timeout_logged = False
        frame = self.decode_image(msg)
        if frame is None:
            return

        self.latest_frame = frame
        self.current_lane_center_px = self.detect_white_lane_center(frame)
        if self.reference_lane_center_px is not None and self.current_lane_center_px is not None:
            self.current_lane_offset_px = (
                self.current_lane_center_px - self.reference_lane_center_px
            )
        else:
            self.current_lane_offset_px = None

        if self.step == MissionStep.CAPTURE_INITIAL and self.capture_initial_frame():
            self.step = MissionStep.DRIVE_FORWARD
            if self.step not in self.logged_steps:
                self.get_logger().info(f"Mission step -> {self.step.value}")
                self.logged_steps.add(self.step)

    def decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0:
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        channels = max(1, msg.step // msg.width)
        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warn("Failed to reshape camera frame.")
            return None

        encoding = msg.encoding.lower()
        if encoding.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if encoding.startswith("bgr"):
            return frame[:, :, :3].copy()
        if encoding.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def detect_white_lane_center(self, frame: np.ndarray) -> Optional[float]:
        height, width = frame.shape[:2]
        roi = frame[int(height * 0.55): int(height * 0.92), int(width * 0.10): int(width * 0.90)]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
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

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def signed_progress_from_start(self) -> Optional[float]:
        if self.current_position is None or self.start_position is None or self.start_yaw is None:
            return None

        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        heading_x = math.cos(self.start_yaw)
        heading_y = math.sin(self.start_yaw)
        return dx * heading_x + dy * heading_y

    def capture_initial_frame(self) -> bool:
        if self.latest_frame is None:
            return False

        reference_center = self.detect_white_lane_center(self.latest_frame)
        if reference_center is None:
            self.get_logger().warn(
                "Initial frame received, but white parking line was not "
                "detected yet."
            )
            return False

        self.reference_frame = self.latest_frame.copy()
        self.reference_lane_center_px = reference_center
        self.current_lane_center_px = reference_center
        self.current_lane_offset_px = 0.0
        self.get_logger().info(
            "Captured initial frame. Reference lane center: "
            f"{self.reference_lane_center_px:.1f} px"
        )
        return True

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
                "No camera data received for over 10 seconds. Restart the simulation environment."
            )
            self.camera_timeout_logged = True

    def run_correction_step(self) -> bool:
        offset_px = self.compare_with_initial_frame()
        if offset_px is None:
            self.publish_stop()
            self.get_logger().warn("Correction paused: white parking line is not visible.")
            return False

        progress = self.signed_progress_from_start()
        if abs(offset_px) <= self.pixel_tolerance:
            if progress is None or abs(progress) <= 0.03:
                self.publish_stop()
                self.get_logger().info(
                    f"Correction complete. Lane offset {offset_px:.2f} px within tolerance."
                )
                return True

        cmd = Twist()
        if progress is not None:
            cmd.linear.x = clamp(
                -0.5 * progress,
                -self.max_correction_speed,
                self.max_correction_speed,
            )
        cmd.angular.z = clamp(
            -0.003 * offset_px,
            -self.max_correction_yaw_rate,
            self.max_correction_yaw_rate,
        )
        self.cmd_pub.publish(cmd)
        return False

    def drive_with_target(self, target_progress_m: float, reverse: bool = False) -> bool:
        progress = self.signed_progress_from_start()
        if progress is None:
            return False

        remaining = target_progress_m - progress
        if reverse:
            reached = progress <= target_progress_m
        else:
            reached = progress >= target_progress_m

        if reached or abs(remaining) <= self.odom_goal_tolerance_m:
            self.publish_stop()
            return True

        cmd = Twist()
        direction = -1.0 if reverse else 1.0
        speed = clamp(abs(remaining) * 0.5, 0.03, self.max_drive_speed)
        cmd.linear.x = direction * speed
        self.cmd_pub.publish(cmd)
        return False

    def control_loop(self) -> None:
        if self.step not in self.logged_steps:
            self.get_logger().info(f"Mission step -> {self.step.value}")
            self.logged_steps.add(self.step)

        if self.step == MissionStep.CAPTURE_INITIAL:
            if self.capture_initial_frame():
                self.step = MissionStep.DRIVE_FORWARD
            return

        if self.step == MissionStep.DRIVE_FORWARD:
            if self.drive_with_target(self.forward_target_m, reverse=False):
                self.get_logger().info("Forward drive completed using odom target 0.30 m.")
                self.step = MissionStep.DRIVE_BACKWARD
            return

        if self.step == MissionStep.DRIVE_BACKWARD:
            if self.drive_with_target(self.backward_target_from_start_m, reverse=True):
                self.get_logger().info(
                    "Backward drive completed near the initial point. "
                    "Switching to vision correction."
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoLaneMission()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
