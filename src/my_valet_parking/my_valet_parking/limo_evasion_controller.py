#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


CONTROL_PERIOD_S = 0.05
IDLE_WARN_PERIOD_S = 5.0
FORWARD_TARGET_DISTANCE_M = 1.0
FRONT_OBSTACLE_STOP_M = 0.5
LEFT_FRONT_START_DEG = 20.0
LEFT_FRONT_END_DEG = 70.0
RIGHT_FRONT_START_DEG = -70.0
RIGHT_FRONT_END_DEG = -20.0
WAIT_REAR_DURATION_S = 1.0
PATH_SAMPLE_DISTANCE_M = 0.05
RETURN_WAYPOINT_TOLERANCE_M = 0.08
HOME_POSITION_TOLERANCE_M = 0.05
HOME_YAW_TOLERANCE_RAD = math.radians(5.0)
VISION_PIXEL_TOLERANCE_PX = 5.0
VISION_MATCH_THRESHOLD = 0.8
MAX_LINEAR_SPEED_MPS = 0.1
MAX_ANGULAR_SPEED_RADPS = 0.3
LINEAR_ACCEL_LIMIT_MPS2 = 0.12
ANGULAR_ACCEL_LIMIT_RADPS2 = 0.25
TEMPLATE_WIDTH_PX = 200
TEMPLATE_HEIGHT_PX = 200
VISION_LINEAR_GAIN = 0.0015
VISION_ANGULAR_GAIN = 0.004
VISION_LINEAR_MAX_MPS = 0.05


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def shortest_angular_distance(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class MissionState(Enum):
    IDLE = "IDLE"
    SCAN_INITIAL = "SCAN_INITIAL"
    EVADE_FORWARD = "EVADE_FORWARD"
    WAIT_REAR = "WAIT_REAR"
    RETURN_HOME = "RETURN_HOME"
    VISION_ALIGN = "VISION_ALIGN"


class LimoEvasionController(Node):
    def __init__(self) -> None:
        super().__init__("limo_evasion_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_image_pub = self.create_publisher(Image, "/debug/image_raw", 10)
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/rgb/image_raw", self.image_callback, qos_profile_sensor_data
        )
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.state = MissionState.IDLE
        self.state_logged = set()
        self.idle_enter_time = self.get_clock().now()
        self.last_idle_warn_time = self.idle_enter_time

        self.odom_received = False
        self.scan_received = False
        self.image_received = False
        self.idle_auto_started = False

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.home_position: Optional[Tuple[float, float]] = None
        self.home_yaw: Optional[float] = None
        self.local_position: Optional[Tuple[float, float]] = None

        self.forward_start_distance_m: Optional[float] = None
        self.wait_rear_start_time = None
        self.path_back: List[Tuple[float, float]] = []
        self.return_path: List[Tuple[float, float]] = []
        self.last_path_point: Optional[Tuple[float, float]] = None

        self.latest_frame: Optional[np.ndarray] = None
        self.latest_image_header = None
        self.reference_template: Optional[np.ndarray] = None
        self.reference_template_origin: Optional[Tuple[int, int]] = None

        self.front_obstacle_distance_m = math.inf
        self.left_front_distance_m = math.inf
        self.right_front_distance_m = math.inf
        self.last_avoid_direction: Optional[str] = None
        self.current_linear_cmd = 0.0
        self.current_angular_cmd = 0.0

        self.get_logger().info(
            "limo_evasion_controller ready: IDLE -> SCAN_INITIAL -> EVADE_FORWARD -> WAIT_REAR -> RETURN_HOME -> VISION_ALIGN"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_received = True

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        if self.home_position is None:
            self.home_position = self.current_position
            self.home_yaw = self.current_yaw
            self.local_position = (0.0, 0.0)
            self.path_back = [(0.0, 0.0)]
            self.last_path_point = (0.0, 0.0)
        elif self.current_position is not None:
            self.local_position = (
                self.current_position[0] - self.home_position[0],
                self.current_position[1] - self.home_position[1],
            )

        self.maybe_append_path_point()
        self.try_start_from_idle()

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_received = True

        front_ranges = self.extract_sector_ranges(msg, -20.0, 20.0)
        left_front_ranges = self.extract_sector_ranges(
            msg, LEFT_FRONT_START_DEG, LEFT_FRONT_END_DEG
        )
        right_front_ranges = self.extract_sector_ranges(
            msg, RIGHT_FRONT_START_DEG, RIGHT_FRONT_END_DEG
        )
        self.front_obstacle_distance_m = min(front_ranges) if front_ranges else math.inf
        self.left_front_distance_m = (
            min(left_front_ranges) if left_front_ranges else math.inf
        )
        self.right_front_distance_m = (
            min(right_front_ranges) if right_front_ranges else math.inf
        )
        self.try_start_from_idle()

    def image_callback(self, msg: Image) -> None:
        frame = self.decode_image(msg)
        if frame is None:
            return

        self.image_received = True
        self.latest_frame = frame
        self.latest_image_header = msg.header
        self.try_start_from_idle()

    def try_start_from_idle(self) -> None:
        if self.state != MissionState.IDLE or self.idle_auto_started:
            return

        if self.odom_received and self.scan_received and self.image_received:
            self.idle_auto_started = True
            self.transition_to(MissionState.SCAN_INITIAL)
            self.get_logger().info(
                "All required sensor data received once. Leaving IDLE and entering SCAN_INITIAL."
            )

    def control_loop(self) -> None:
        self.log_state_once()

        if self.state == MissionState.IDLE:
            self.handle_idle()
            return

        if self.state == MissionState.SCAN_INITIAL:
            if self.capture_initial_reference():
                self.forward_start_distance_m = self.project_along_home_heading()
                self.path_back = [(0.0, 0.0)]
                self.last_path_point = (0.0, 0.0)
                self.last_avoid_direction = None
                self.transition_to(MissionState.EVADE_FORWARD)
            else:
                self.publish_smoothed_command(0.0, 0.0)
            return

        if self.state == MissionState.EVADE_FORWARD:
            if self.run_evade_forward():
                self.wait_rear_start_time = self.get_clock().now()
                self.return_path = list(reversed(self.path_back))
                self.transition_to(MissionState.WAIT_REAR)
            return

        if self.state == MissionState.WAIT_REAR:
            if self.run_wait_rear():
                self.transition_to(MissionState.RETURN_HOME)
            return

        if self.state == MissionState.RETURN_HOME:
            if self.run_return_home():
                self.transition_to(MissionState.VISION_ALIGN)
            return

        if self.state == MissionState.VISION_ALIGN:
            if self.run_vision_align():
                self.publish_smoothed_command(0.0, 0.0)
                self.get_logger().info("Mission complete. Returning to IDLE.")
                self.idle_auto_started = False
                self.transition_to(MissionState.IDLE)
            return

    def handle_idle(self) -> None:
        now = self.get_clock().now()
        if now - self.last_idle_warn_time >= Duration(seconds=IDLE_WARN_PERIOD_S):
            missing = []
            if not self.odom_received:
                missing.append("Odometry(/odom)")
            if not self.scan_received:
                missing.append("LiDAR(/scan)")
            if not self.image_received:
                missing.append("Image(/rgb/image_raw)")

            if missing:
                self.get_logger().warn(
                    "[WARN] IDLE timeout over 5s. Waiting for: " + ", ".join(missing)
                )
            self.last_idle_warn_time = now

        self.publish_smoothed_command(0.0, 0.0)

    def capture_initial_reference(self) -> bool:
        if self.latest_frame is None:
            return False

        template, origin = self.extract_center_template(self.latest_frame)
        if template is None or origin is None:
            self.get_logger().warn("SCAN_INITIAL: failed to extract center ROI template.")
            return False

        self.reference_template = template
        self.reference_template_origin = origin
        self.get_logger().info(
            f"SCAN_INITIAL: stored reference_template at x={origin[0]}, y={origin[1]}, size={template.shape[1]}x{template.shape[0]}."
        )
        return True

    def run_evade_forward(self) -> bool:
        if self.forward_start_distance_m is None:
            self.forward_start_distance_m = self.project_along_home_heading()

        traveled = self.project_along_home_heading() - self.forward_start_distance_m
        remaining = FORWARD_TARGET_DISTANCE_M - traveled

        if remaining <= 0.02:
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().info("EVADE_FORWARD: reached forward target of 1.0 m.")
            return True

        if self.front_obstacle_distance_m <= FRONT_OBSTACLE_STOP_M:
            if self.left_front_distance_m > self.right_front_distance_m:
                linear_cmd = 0.05
                angular_cmd = -0.3
                self.log_avoid_direction("RIGHT")
            else:
                linear_cmd = 0.05
                angular_cmd = 0.3
                self.log_avoid_direction("LEFT")
        else:
            linear_cmd = 0.1
            angular_cmd = 0.0
            self.last_avoid_direction = None

        self.publish_smoothed_command(linear_cmd, angular_cmd)
        return False

    def run_wait_rear(self) -> bool:
        self.publish_smoothed_command(0.0, 0.0)
        if self.wait_rear_start_time is None:
            return False

        elapsed = (self.get_clock().now() - self.wait_rear_start_time).nanoseconds / 1e9
        return elapsed >= WAIT_REAR_DURATION_S

    def run_return_home(self) -> bool:
        if self.local_position is None or self.current_yaw is None or self.home_yaw is None:
            self.publish_smoothed_command(0.0, 0.0)
            return False

        while self.return_path and self.distance_to_point(self.return_path[0]) <= RETURN_WAYPOINT_TOLERANCE_M:
            self.return_path.pop(0)

        distance_to_home = self.distance_to_point((0.0, 0.0))
        yaw_error = self.heading_error_to_home_yaw()

        if (
            not self.return_path
            and distance_to_home <= HOME_POSITION_TOLERANCE_M
            and abs(yaw_error) <= HOME_YAW_TOLERANCE_RAD
        ):
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().info("RETURN_HOME: returned to local start point (0, 0).")
            return True

        target_point = self.return_path[0] if self.return_path else (0.0, 0.0)
        reverse_heading = math.atan2(
            self.local_position[1] - target_point[1],
            self.local_position[0] - target_point[0],
        )
        heading_error = shortest_angular_distance(reverse_heading, self.current_yaw)
        linear_cmd = -clamp(
            self.distance_to_point(target_point) * 0.5,
            0.03,
            MAX_LINEAR_SPEED_MPS,
        )
        angular_cmd = clamp(
            heading_error * 1.2, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS
        )
        self.publish_smoothed_command(linear_cmd, angular_cmd)
        return False

    def run_vision_align(self) -> bool:
        if (
            self.reference_template is None
            or self.reference_template_origin is None
            or self.latest_frame is None
        ):
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().warn("VISION_ALIGN: reference template unavailable.")
            return False

        match = self.match_reference_template(self.latest_frame)
        if match is None:
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().warn("VISION_ALIGN: template matching failed.")
            return False

        match_top_left, max_val = match
        x_offset = float(match_top_left[0] - self.reference_template_origin[0])
        y_offset = float(match_top_left[1] - self.reference_template_origin[1])
        distance_to_home = self.distance_to_point((0.0, 0.0))
        yaw_error = self.heading_error_to_home_yaw()
        self.publish_debug_match_image(self.latest_frame, match_top_left, max_val)

        if max_val < VISION_MATCH_THRESHOLD:
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().warn(
                f"VISION_ALIGN: low template match confidence {max_val:.3f} < {VISION_MATCH_THRESHOLD:.2f}."
            )
            return False

        if (
            abs(x_offset) <= VISION_PIXEL_TOLERANCE_PX
            and abs(y_offset) <= VISION_PIXEL_TOLERANCE_PX
            and distance_to_home <= HOME_POSITION_TOLERANCE_M
            and abs(yaw_error) <= HOME_YAW_TOLERANCE_RAD
        ):
            self.publish_smoothed_command(0.0, 0.0)
            self.get_logger().info(
                f"VISION_ALIGN: matched offsets x={x_offset:.2f}px, y={y_offset:.2f}px, confidence={max_val:.3f}."
            )
            return True

        linear_cmd = clamp(
            -(y_offset * VISION_LINEAR_GAIN) - (distance_to_home * 0.4),
            -VISION_LINEAR_MAX_MPS,
            VISION_LINEAR_MAX_MPS,
        )
        angular_cmd = clamp(
            -(x_offset * VISION_ANGULAR_GAIN) - (yaw_error * 0.8),
            -MAX_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.get_logger().info(
            f"VISION_ALIGN: confidence={max_val:.3f}, x_offset={x_offset:.1f}px, y_offset={y_offset:.1f}px."
        )
        self.publish_smoothed_command(linear_cmd, angular_cmd)
        return False

    def publish_smoothed_command(self, target_linear: float, target_angular: float) -> None:
        target_linear = clamp(target_linear, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS)
        target_angular = clamp(target_angular, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS)

        linear_step = LINEAR_ACCEL_LIMIT_MPS2 * CONTROL_PERIOD_S
        angular_step = ANGULAR_ACCEL_LIMIT_RADPS2 * CONTROL_PERIOD_S

        self.current_linear_cmd = self.approach(
            self.current_linear_cmd, target_linear, linear_step
        )
        self.current_angular_cmd = self.approach(
            self.current_angular_cmd, target_angular, angular_step
        )

        cmd = Twist()
        cmd.linear.x = self.current_linear_cmd
        cmd.angular.z = self.current_angular_cmd
        self.cmd_pub.publish(cmd)

    def extract_sector_ranges(
        self, msg: LaserScan, start_deg: float, end_deg: float
    ) -> List[float]:
        sector_ranges: List[float] = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + (index * msg.angle_increment)
            angle_deg = math.degrees(angle)
            if start_deg <= angle_deg <= end_deg:
                sector_ranges.append(distance)
        return sector_ranges

    def decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None

        channels = max(1, msg.step // msg.width)
        frame = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warn("Image reshape failed.")
            return None

        encoding = msg.encoding.lower()
        if encoding.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if encoding.startswith("bgr"):
            return frame[:, :, :3].copy()
        if encoding.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def extract_center_template(
        self, frame: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int]]]:
        height, width = frame.shape[:2]
        roi_width = min(TEMPLATE_WIDTH_PX, width)
        roi_height = min(TEMPLATE_HEIGHT_PX, height)
        x0 = max(0, (width - roi_width) // 2)
        y0 = max(0, (height - roi_height) // 2)
        roi = frame[y0 : y0 + roi_height, x0 : x0 + roi_width]
        if roi.size == 0:
            return None, None
        return roi.copy(), (x0, y0)

    def match_reference_template(
        self, frame: np.ndarray
    ) -> Optional[Tuple[Tuple[int, int], float]]:
        if self.reference_template is None:
            return None

        if (
            frame.shape[0] < self.reference_template.shape[0]
            or frame.shape[1] < self.reference_template.shape[1]
        ):
            return None

        result = cv2.matchTemplate(
            frame, self.reference_template, cv2.TM_CCOEFF_NORMED
        )
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        return (max_loc[0], max_loc[1]), float(max_val)

    def publish_debug_match_image(
        self, frame: np.ndarray, match_top_left: Tuple[int, int], confidence: float
    ) -> None:
        if self.reference_template is None:
            return

        debug_frame = frame.copy()
        top_left = match_top_left
        bottom_right = (
            top_left[0] + self.reference_template.shape[1],
            top_left[1] + self.reference_template.shape[0],
        )
        cv2.rectangle(debug_frame, top_left, bottom_right, (0, 255, 0), 2)
        cv2.putText(
            debug_frame,
            f"match={confidence:.2f}",
            (top_left[0], max(20, top_left[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        msg = Image()
        if self.latest_image_header is not None:
            msg.header = self.latest_image_header
        msg.height = debug_frame.shape[0]
        msg.width = debug_frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = debug_frame.shape[1] * debug_frame.shape[2]
        msg.data = debug_frame.tobytes()
        self.debug_image_pub.publish(msg)

    def maybe_append_path_point(self) -> None:
        if self.local_position is None:
            return

        if self.state != MissionState.EVADE_FORWARD:
            return

        if self.last_path_point is None:
            self.path_back.append(self.local_position)
            self.last_path_point = self.local_position
            return

        if self.distance_between(self.local_position, self.last_path_point) >= PATH_SAMPLE_DISTANCE_M:
            self.path_back.append(self.local_position)
            self.last_path_point = self.local_position

    def project_along_home_heading(self) -> float:
        if self.local_position is None or self.home_yaw is None:
            return 0.0

        heading_x = math.cos(self.home_yaw)
        heading_y = math.sin(self.home_yaw)
        return (self.local_position[0] * heading_x) + (self.local_position[1] * heading_y)

    def distance_to_point(self, point: Tuple[float, float]) -> float:
        if self.local_position is None:
            return math.inf
        return self.distance_between(self.local_position, point)

    def distance_between(
        self, point_a: Tuple[float, float], point_b: Tuple[float, float]
    ) -> float:
        return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])

    def heading_error_to_home_yaw(self) -> float:
        if self.current_yaw is None or self.home_yaw is None:
            return 0.0
        return shortest_angular_distance(self.home_yaw, self.current_yaw)

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def approach(self, current: float, target: float, delta: float) -> float:
        if current < target:
            return min(current + delta, target)
        if current > target:
            return max(current - delta, target)
        return current

    def transition_to(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return

        self.state = new_state
        self.state_logged.discard(new_state)
        if new_state == MissionState.IDLE:
            now = self.get_clock().now()
            self.idle_enter_time = now
            self.last_idle_warn_time = now

    def log_state_once(self) -> None:
        if self.state in self.state_logged:
            return
        self.get_logger().info(f"State -> {self.state.value}")
        self.state_logged.add(self.state)

    def log_avoid_direction(self, direction: str) -> None:
        if self.last_avoid_direction == direction:
            return
        self.last_avoid_direction = direction
        self.get_logger().info(f"[INFO] Avoiding obstacle: Steering {direction}")

    def stop(self) -> None:
        self.current_linear_cmd = 0.0
        self.current_angular_cmd = 0.0
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoEvasionController()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
