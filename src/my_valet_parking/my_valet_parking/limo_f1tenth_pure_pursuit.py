#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


CONTROL_PERIOD_S = 0.05
IDLE_WARN_PERIOD_S = 5.0
STATUS_LOG_PERIOD_S = 0.5
LIDAR_LOG_PERIOD_S = 0.5
SCAN_DEBUG_PERIOD_S = 1.0
ALIGN_LOG_PERIOD_S = 0.5
FORWARD_TARGET_DISTANCE_M = 1.0
PATH_SAMPLE_DISTANCE_M = 0.05
CENTER_OBSTACLE_TRIGGER_M = 0.35
ESTOP_DISTANCE_M = 0.25
ALIGN_ESTOP_DISTANCE_M = 0.12
LEFT_START_DEG = 30.0
LEFT_END_DEG = 90.0
CENTER_START_DEG = -30.0
CENTER_END_DEG = 30.0
ALIGN_CENTER_START_DEG = -15.0
ALIGN_CENTER_END_DEG = 15.0
RIGHT_START_DEG = -90.0
RIGHT_END_DEG = -30.0
RETURN_POINT_TOLERANCE_M = 0.15
HOME_POSITION_TOLERANCE_M = 0.25
HOME_YAW_TOLERANCE_RAD = math.radians(5.0)
VISION_PIXEL_TOLERANCE_PX = 5.0
VISION_MATCH_THRESHOLD = 0.8
TEMPLATE_WIDTH_PX = 200
TEMPLATE_HEIGHT_PX = 200
ALIGN_ERROR_RATIO_LIMIT = 0.10
EVADE_LINEAR_SPEED_MPS = 0.08
AVOID_LINEAR_SPEED_MPS = 0.04
RETURN_LINEAR_SPEED_MPS = -0.08
MAX_RETURN_ANGULAR_SPEED_RADPS = 0.30
ALIGN_LINEAR_SPEED_MPS = 0.015
ALIGN_MAX_ANGULAR_SPEED_RADPS = 0.16


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def shortest_angular_distance(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class MissionState(Enum):
    IDLE = "IDLE"
    SCAN_INITIAL = "SCAN_INITIAL"
    EVADE_FORWARD = "EVADE_FORWARD"
    PURE_PURSUIT_RETURN = "PURE_PURSUIT_RETURN"
    VISION_ALIGN = "VISION_ALIGN"
    FINISH = "FINISH"


class LimoF1TenthPurePursuit(Node):
    def __init__(self) -> None:
        super().__init__("limo_f1tenth_pure_pursuit")

        self.declare_parameter("WHEELBASE", 0.2)
        self.declare_parameter("LOOKAHEAD_DISTANCE", 0.30)
        self.declare_parameter("MAX_STEERING_ANGLE", 0.523)

        self.wheelbase = float(self.get_parameter("WHEELBASE").value)
        self.lookahead_distance = float(
            self.get_parameter("LOOKAHEAD_DISTANCE").value
        )
        self.max_steering_angle = float(
            self.get_parameter("MAX_STEERING_ANGLE").value
        )
        self.scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_image_pub = self.create_publisher(Image, "/debug/image_raw", 10)
        self.lookahead_marker_pub = self.create_publisher(Marker, "/lookahead_marker", 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, self.scan_qos
        )
        self.create_subscription(
            Image, "/rgb/image_raw", self.image_callback, qos_profile_sensor_data
        )
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.state = MissionState.IDLE
        self.state_logged = set()
        self.idle_enter_time = self.get_clock().now()
        self.last_idle_warn_time = self.idle_enter_time
        self.last_status_log_time = self.idle_enter_time
        self.last_lidar_log_time = self.idle_enter_time
        self.last_scan_debug_log_time = self.idle_enter_time
        self.last_steering_log_time = self.idle_enter_time
        self.last_align_log_time = self.idle_enter_time
        self.last_reverse_debug_log_time = self.idle_enter_time
        self.idle_auto_started = False

        self.odom_received = False
        self.scan_received = False
        self.image_received = False

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.home_position: Optional[Tuple[float, float]] = None
        self.home_yaw: Optional[float] = None

        self.forward_start_distance_m: Optional[float] = None
        self.logged_path: List[Tuple[float, float, float]] = []
        self.path_history = self.logged_path
        self.return_path: List[Tuple[float, float, float]] = []
        self.last_logged_pose: Optional[Tuple[float, float, float]] = None

        self.front_obstacle_distance_m = math.inf
        self.left_distance_m = math.inf
        self.center_distance_m = math.inf
        self.right_distance_m = math.inf
        self.align_scan_distance_m = math.inf
        self.initial_scan_dist: Optional[float] = None
        self.reference_template_width_px: Optional[float] = None
        self.last_avoid_direction: Optional[str] = None

        self.latest_frame: Optional[np.ndarray] = None
        self.latest_image_header = None
        self.reference_template: Optional[np.ndarray] = None
        self.reference_template_origin: Optional[Tuple[int, int]] = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.lookahead_pose: Optional[Tuple[float, float, float]] = None
        self.sensor_frame_id = "laser_link"
        self.static_tf_sent = False

        self.get_logger().info(
            "limo_f1tenth_pure_pursuit ready: IDLE -> SCAN_INITIAL -> "
            "EVADE_FORWARD -> PURE_PURSUIT_RETURN -> VISION_ALIGN"
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
            initial_pose = (self.current_position[0], self.current_position[1], self.current_yaw)
            self.logged_path = [initial_pose]
            self.path_history = self.logged_path
            self.last_logged_pose = initial_pose

        self.maybe_log_path_pose()
        self.try_start_from_idle()

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_received = True
        if msg.header.frame_id:
            self.sensor_frame_id = msg.header.frame_id
        self.publish_static_sensor_tf()

        front_ranges = self.extract_sector_ranges(msg, CENTER_START_DEG, CENTER_END_DEG)
        left_ranges = self.extract_sector_ranges(msg, LEFT_START_DEG, LEFT_END_DEG)
        center_ranges = self.extract_sector_ranges(msg, CENTER_START_DEG, CENTER_END_DEG)
        right_ranges = self.extract_sector_ranges(msg, RIGHT_START_DEG, RIGHT_END_DEG)
        align_center_ranges = self.extract_sector_ranges(
            msg, ALIGN_CENTER_START_DEG, ALIGN_CENTER_END_DEG
        )
        self.front_obstacle_distance_m = min(front_ranges) if front_ranges else math.inf
        self.left_distance_m = min(left_ranges) if left_ranges else math.inf
        self.center_distance_m = min(center_ranges) if center_ranges else math.inf
        self.right_distance_m = min(right_ranges) if right_ranges else math.inf
        self.align_scan_distance_m = (
            sum(align_center_ranges) / len(align_center_ranges)
            if align_center_ranges
            else math.inf
        )
        self.maybe_log_lidar(
            front_count=len(front_ranges),
            left_count=len(left_ranges),
            center_count=len(center_ranges),
            right_count=len(right_ranges),
        )
        self.maybe_log_scan_debug(
            msg,
            front_count=len(front_ranges),
            center_count=len(center_ranges),
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
            self.publish_lookahead_marker()
            self.maybe_log_status()
            return

        if self.state == MissionState.SCAN_INITIAL:
            if self.capture_initial_reference():
                self.forward_start_distance_m = self.project_along_home_heading()
                if self.current_position is not None and self.current_yaw is not None:
                    start_pose = (
                        self.current_position[0],
                        self.current_position[1],
                        self.current_yaw,
                    )
                    self.logged_path = [start_pose]
                    self.path_history = self.logged_path
                    self.last_logged_pose = start_pose
                self.transition_to(MissionState.EVADE_FORWARD)
            else:
                self.publish_cmd(0.0, 0.0)
            self.publish_lookahead_marker()
            self.maybe_log_status()
            return

        if self.state == MissionState.EVADE_FORWARD:
            if self.run_evade_forward():
                self.append_current_pose_if_needed(force=True)
                self.publish_cmd(0.0, 0.0)
                self.return_path = list(reversed(self.logged_path))
                self.transition_to(MissionState.PURE_PURSUIT_RETURN)
            self.publish_lookahead_marker()
            self.maybe_log_status()
            return

        if self.state == MissionState.PURE_PURSUIT_RETURN:
            if self.run_pure_pursuit_return():
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    "Reached Return Target. Switching to VISION_ALIGN mode."
                )
                self.transition_to(MissionState.VISION_ALIGN)
            self.publish_lookahead_marker()
            self.maybe_log_status()
            return

        if self.state == MissionState.VISION_ALIGN:
            if self.run_vision_align():
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info("Mission complete. Entering FINISH.")
                self.transition_to(MissionState.FINISH)
            self.publish_lookahead_marker()
            self.maybe_log_status()
            return

        if self.state == MissionState.FINISH:
            self.publish_cmd(0.0, 0.0)
            self.publish_lookahead_marker()
            self.maybe_log_status()
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
        self.publish_cmd(0.0, 0.0)

    def capture_initial_reference(self) -> bool:
        if self.latest_frame is None:
            return False

        template, origin = self.extract_center_template(self.latest_frame)
        if template is None or origin is None:
            self.get_logger().warn("SCAN_INITIAL: failed to extract center ROI template.")
            return False

        self.reference_template = template
        self.reference_template_origin = origin
        self.reference_template_width_px = float(template.shape[1])
        self.initial_scan_dist = (
            self.align_scan_distance_m if math.isfinite(self.align_scan_distance_m) else None
        )
        if self.initial_scan_dist is None:
            self.get_logger().warn(
                "SCAN_INITIAL: failed to store initial center LiDAR mean distance."
            )
            return False
        self.get_logger().info(
            f"SCAN_INITIAL: stored reference template at x={origin[0]}, y={origin[1]}, "
            f"size={template.shape[1]}x{template.shape[0]}, "
            f"initial_scan_dist={self.initial_scan_dist:.3f} m."
        )
        return True

    def run_evade_forward(self) -> bool:
        if self.forward_start_distance_m is None:
            self.forward_start_distance_m = self.project_along_home_heading()

        traveled = self.project_along_home_heading() - self.forward_start_distance_m
        remaining = FORWARD_TARGET_DISTANCE_M - traveled
        if remaining <= 0.02:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("EVADE_FORWARD: reached forward target of 1.0 m.")
            return True

        if self.center_distance_m <= ESTOP_DISTANCE_M:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"[E-STOP] Obstacle too close: center distance {self.center_distance_m:.2f} m"
            )
            return False

        if self.center_distance_m <= CENTER_OBSTACLE_TRIGGER_M:
            if self.left_distance_m <= self.right_distance_m:
                linear_cmd = AVOID_LINEAR_SPEED_MPS
                angular_cmd = -0.45
                self.log_avoid_direction("RIGHT")
            else:
                linear_cmd = AVOID_LINEAR_SPEED_MPS
                angular_cmd = 0.45
                self.log_avoid_direction("LEFT")
        else:
            linear_cmd = EVADE_LINEAR_SPEED_MPS
            angular_cmd = 0.0
            self.last_avoid_direction = None

        self.publish_cmd(linear_cmd, angular_cmd)
        return False

    def run_pure_pursuit_return(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        self.prune_passed_return_points()
        distance_to_home = self.distance_to_xy((self.home_position[0], self.home_position[1])) if self.home_position else math.inf
        if distance_to_home <= HOME_POSITION_TOLERANCE_M:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f"PURE_PURSUIT_RETURN: reached return threshold at {distance_to_home:.3f} m."
            )
            return True

        if len(self.return_path) <= 1:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                "PURE_PURSUIT_RETURN: consumed return path. Forcing transition to alignment."
            )
            return True

        lookahead_pose = self.find_lookahead_pose()
        if lookahead_pose is None:
            target_pose = self.return_path[-1] if self.return_path else self.logged_path[0]
            lookahead_pose = target_pose
        self.lookahead_pose = lookahead_pose

        local_x, local_y = self.transform_to_local_frame(lookahead_pose)
        lookahead_dist = math.hypot(local_x, local_y)
        if lookahead_dist < 1e-6:
            self.publish_cmd(0.0, 0.0)
            return False

        # For reverse tracking, evaluate the lookahead point in the rear-driving frame.
        alpha = math.atan2(-local_y, -local_x)
        delta = math.atan(
            2.0 * self.wheelbase * math.sin(alpha) / lookahead_dist
        )
        delta = clamp(delta, -self.max_steering_angle, self.max_steering_angle)
        self.maybe_log_reverse_debug(alpha, delta, distance_to_home)

        linear_cmd = RETURN_LINEAR_SPEED_MPS
        angular_cmd = linear_cmd * math.tan(delta) / self.wheelbase
        angular_cmd = clamp(
            angular_cmd, -MAX_RETURN_ANGULAR_SPEED_RADPS, MAX_RETURN_ANGULAR_SPEED_RADPS
        )
        self.publish_cmd(linear_cmd, angular_cmd)
        return False

    def run_vision_align(self) -> bool:
        if (
            self.reference_template is None
            or self.reference_template_origin is None
            or self.latest_frame is None
            or self.initial_scan_dist is None
            or self.reference_template_width_px is None
        ):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("VISION_ALIGN: reference template or initial scan unavailable.")
            return False

        if self.center_distance_m <= ALIGN_ESTOP_DISTANCE_M:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"[E-STOP] VISION_ALIGN obstacle too close: center distance {self.center_distance_m:.2f} m"
            )
            return False

        match = self.match_reference_template(self.latest_frame)
        if match is None:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("VISION_ALIGN: template matching failed.")
            return False

        match_top_left, confidence = match
        x_offset = float(match_top_left[0] - self.reference_template_origin[0])
        self.publish_debug_match_image(self.latest_frame, match_top_left, confidence)

        if confidence < VISION_MATCH_THRESHOLD:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"VISION_ALIGN: low template match confidence {confidence:.3f} < "
                f"{VISION_MATCH_THRESHOLD:.2f}."
            )
            return False

        if not math.isfinite(self.align_scan_distance_m):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("VISION_ALIGN: current center LiDAR mean distance unavailable.")
            return False

        vision_reference_px = max(self.reference_template_width_px, 1.0)
        vision_error_px = abs(x_offset)
        vision_error_pct = vision_error_px / vision_reference_px
        lidar_distance_error = self.align_scan_distance_m - self.initial_scan_dist
        lidar_error_pct = (
            abs(lidar_distance_error) / self.initial_scan_dist
            if self.initial_scan_dist > 1e-6
            else math.inf
        )
        vision_pass = vision_error_pct <= ALIGN_ERROR_RATIO_LIMIT
        lidar_pass = lidar_error_pct <= ALIGN_ERROR_RATIO_LIMIT
        self.maybe_log_align_error(
            lidar_error_pct,
            vision_error_pct,
            lidar_distance_error,
            vision_error_px,
            lidar_pass,
            vision_pass,
        )

        if lidar_pass and vision_pass:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                "VISION_ALIGN 10% tolerance PASS | "
                f"LiDAR: {lidar_error_pct * 100.0:.2f}% "
                f"({self.align_scan_distance_m:.3f} m vs {self.initial_scan_dist:.3f} m) | "
                f"Vision: {vision_error_pct * 100.0:.2f}% "
                f"({vision_error_px:.1f} px / ref {vision_reference_px:.1f} px)."
            )
            return True

        angular_cmd = clamp(
            -0.006 * x_offset,
            -ALIGN_MAX_ANGULAR_SPEED_RADPS,
            ALIGN_MAX_ANGULAR_SPEED_RADPS,
        )
        if not vision_pass:
            linear_cmd = 0.0
        elif lidar_distance_error > 0.0 and not lidar_pass:
            linear_cmd = ALIGN_LINEAR_SPEED_MPS
        elif lidar_distance_error < 0.0 and not lidar_pass:
            linear_cmd = -ALIGN_LINEAR_SPEED_MPS
        else:
            linear_cmd = 0.0
        self.publish_cmd(linear_cmd, angular_cmd)
        return False

    def maybe_log_align_error(
        self,
        lidar_error_pct: float,
        vision_error_pct: float,
        lidar_distance_error: float,
        vision_error_px: float,
        lidar_pass: bool,
        vision_pass: bool,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_align_log_time).nanoseconds / 1e9 < ALIGN_LOG_PERIOD_S:
            return

        self.get_logger().info(
            "VISION_ALIGN 10% tolerance "
            f"{'PASS' if lidar_pass and vision_pass else 'FAIL'} | "
            f"LiDAR: {lidar_error_pct * 100.0:.2f}% "
            f"(delta {lidar_distance_error:+.3f} m, limit {ALIGN_ERROR_RATIO_LIMIT * 100.0:.1f}%) "
            f"[{'OK' if lidar_pass else 'OUT'}] | "
            f"Vision: {vision_error_pct * 100.0:.2f}% "
            f"(offset {vision_error_px:.1f} px, limit {ALIGN_ERROR_RATIO_LIMIT * 100.0:.1f}%) "
            f"[{'OK' if vision_pass else 'OUT'}]"
        )
        self.last_align_log_time = now

    def maybe_log_reverse_debug(
        self, alpha: float, delta: float, distance_to_home: float
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_reverse_debug_log_time).nanoseconds / 1e9 < STATUS_LOG_PERIOD_S:
            return

        self.get_logger().info(
            f"Reverse Alpha: {alpha:.3f} | Delta: {delta:.3f} | Target Dist: {distance_to_home:.3f}"
        )
        self.last_reverse_debug_log_time = now

    def maybe_log_path_pose(self) -> None:
        if (
            self.state != MissionState.EVADE_FORWARD
            or self.current_position is None
            or self.current_yaw is None
        ):
            return

        self.append_current_pose_if_needed(force=False)

    def append_current_pose_if_needed(self, force: bool) -> None:
        if self.current_position is None or self.current_yaw is None:
            return

        current_pose = (
            self.current_position[0],
            self.current_position[1],
            self.current_yaw,
        )
        if self.last_logged_pose is None:
            self.logged_path.append(current_pose)
            self.last_logged_pose = current_pose
            return

        if force or self.distance_between_xy(current_pose, self.last_logged_pose) >= PATH_SAMPLE_DISTANCE_M:
            self.logged_path.append(current_pose)
            self.last_logged_pose = current_pose

    def prune_passed_return_points(self) -> None:
        while len(self.return_path) > 1:
            current_distance = self.distance_to_xy(
                (self.return_path[0][0], self.return_path[0][1])
            )
            next_distance = self.distance_to_xy(
                (self.return_path[1][0], self.return_path[1][1])
            )
            if (
                current_distance <= RETURN_POINT_TOLERANCE_M
                or next_distance < current_distance
            ):
                self.return_path.pop(0)
            else:
                break

    def find_lookahead_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.current_position is None:
            return None

        if not self.return_path:
            return None

        nearest_index = min(
            range(len(self.return_path)),
            key=lambda idx: self.distance_to_xy(
                (self.return_path[idx][0], self.return_path[idx][1])
            ),
        )

        for idx in range(nearest_index, -1, -1):
            pose = self.return_path[idx]
            if self.distance_to_xy((pose[0], pose[1])) > self.lookahead_distance:
                return pose

        return self.return_path[0]

    def transform_to_local_frame(
        self, target_pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        dx = target_pose[0] - self.current_position[0]
        dy = target_pose[1] - self.current_position[1]
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return local_x, local_y

    def publish_cmd(self, linear: float, angular: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.last_cmd_linear = linear
        self.last_cmd_angular = angular
        self.cmd_pub.publish(cmd)
        self.maybe_log_steering_command(angular)

    def maybe_log_steering_command(self, angular: float) -> None:
        if abs(angular) < 1e-6:
            return

        now = self.get_clock().now()
        if (now - self.last_steering_log_time).nanoseconds / 1e9 < STATUS_LOG_PERIOD_S:
            return

        direction = "LEFT" if angular > 0.0 else "RIGHT"
        self.get_logger().info(
            f"Steering: {direction} | Value: {angular:.3f}"
        )
        self.last_steering_log_time = now

    def maybe_log_status(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_status_log_time).nanoseconds / 1e9 < STATUS_LOG_PERIOD_S:
            return

        if self.state == MissionState.EVADE_FORWARD and self.forward_start_distance_m is not None:
            traveled = self.project_along_home_heading() - self.forward_start_distance_m
            target_dist = max(0.0, FORWARD_TARGET_DISTANCE_M - traveled)
        elif self.state == MissionState.PURE_PURSUIT_RETURN and self.home_position is not None:
            target_dist = self.distance_to_xy(self.home_position)
        else:
            target_dist = 0.0

        self.get_logger().info(
            f"Current State: {self.state.value} | Target Dist: {target_dist:.3f} | "
            f"Cmd: v={self.last_cmd_linear:.3f}, w={self.last_cmd_angular:.3f}"
        )
        self.last_status_log_time = now

    def maybe_log_lidar(
        self,
        front_count: int,
        left_count: int,
        center_count: int,
        right_count: int,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_lidar_log_time).nanoseconds / 1e9 < LIDAR_LOG_PERIOD_S:
            return

        trigger_margin = self.center_distance_m - CENTER_OBSTACLE_TRIGGER_M
        estop_margin = self.center_distance_m - ESTOP_DISTANCE_M
        if math.isfinite(self.front_obstacle_distance_m):
            self.get_logger().info(
                f"[LiDAR] Front/Center/Left/Right Min Dist: "
                f"{self.front_obstacle_distance_m:.2f}/"
                f"{self.center_distance_m:.2f}/"
                f"{self.left_distance_m:.2f}/"
                f"{self.right_distance_m:.2f}m | "
                f"Hits F/C/L/R: {front_count}/{center_count}/{left_count}/{right_count} | "
                f"SUV trigger@{CENTER_OBSTACLE_TRIGGER_M:.2f}m margin {trigger_margin:+.2f}m | "
                f"E-stop@{ESTOP_DISTANCE_M:.2f}m margin {estop_margin:+.2f}m"
            )
        else:
            self.get_logger().info(
                f"[LiDAR] Front/Center/Left/Right Min Dist: inf/"
                f"{self.center_distance_m:.2f}/"
                f"{self.left_distance_m:.2f}/"
                f"{self.right_distance_m:.2f}m | "
                f"Hits F/C/L/R: {front_count}/{center_count}/{left_count}/{right_count}"
            )
        self.last_lidar_log_time = now

    def maybe_log_scan_debug(
        self,
        msg: LaserScan,
        front_count: int,
        center_count: int,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_scan_debug_log_time).nanoseconds / 1e9 < SCAN_DEBUG_PERIOD_S:
            return

        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
        ]
        mean_distance = (
            sum(valid_ranges) / len(valid_ranges) if valid_ranges else math.inf
        )
        if math.isfinite(mean_distance):
            self.get_logger().info(
                f"[LiDAR] frame={msg.header.frame_id or 'unknown'} | "
                f"Scan Count: {len(msg.ranges)} | Mean Dist: {mean_distance:.2f}m | "
                f"Center Hits: {center_count} | Front Hits: {front_count} | "
                f"Obstacle Seen: {'YES' if center_count > 0 and math.isfinite(self.center_distance_m) else 'NO'}"
            )
        else:
            self.get_logger().info(
                f"[LiDAR] frame={msg.header.frame_id or 'unknown'} | "
                f"Scan Count: {len(msg.ranges)} | Mean Dist: inf | "
                f"Center Hits: {center_count} | Front Hits: {front_count} | Obstacle Seen: NO"
            )
        self.last_scan_debug_log_time = now

    def publish_static_sensor_tf(self) -> None:
        if self.static_tf_sent:
            return
        if self.sensor_frame_id == "base_link":
            self.get_logger().info(
                "Skipping static TF broadcast because /scan already uses frame_id=base_link."
            )
            self.static_tf_sent = True
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = self.sensor_frame_id
        transform.transform.translation.x = 0.12
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.10
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)
        self.static_tf_sent = True
        self.get_logger().info(
            f"Published static TF: base_link -> {self.sensor_frame_id}"
        )

    def publish_lookahead_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pure_pursuit"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1

        if self.lookahead_pose is None:
            marker.action = Marker.DELETE
        else:
            marker.pose.position.x = self.lookahead_pose[0]
            marker.pose.position.y = self.lookahead_pose[1]
            marker.pose.position.z = 0.05

        self.lookahead_marker_pub.publish(marker)

    def extract_sector_ranges(
        self, msg: LaserScan, start_deg: float, end_deg: float
    ) -> List[float]:
        sector_ranges: List[float] = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
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
        result = cv2.matchTemplate(frame, self.reference_template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        return (max_loc[0], max_loc[1]), float(max_val)

    def publish_debug_match_image(
        self, frame: np.ndarray, match_top_left: Tuple[int, int], confidence: float
    ) -> None:
        if self.reference_template is None:
            return

        debug_frame = frame.copy()
        bottom_right = (
            match_top_left[0] + self.reference_template.shape[1],
            match_top_left[1] + self.reference_template.shape[0],
        )
        cv2.rectangle(debug_frame, match_top_left, bottom_right, (0, 255, 0), 2)
        cv2.putText(
            debug_frame,
            f"match={confidence:.2f}",
            (match_top_left[0], max(20, match_top_left[1] - 10)),
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

    def project_along_home_heading(self) -> float:
        if (
            self.current_position is None
            or self.home_position is None
            or self.home_yaw is None
        ):
            return 0.0
        dx = self.current_position[0] - self.home_position[0]
        dy = self.current_position[1] - self.home_position[1]
        heading_x = math.cos(self.home_yaw)
        heading_y = math.sin(self.home_yaw)
        return dx * heading_x + dy * heading_y

    def distance_to_xy(self, point: Optional[Tuple[float, float]]) -> float:
        if self.current_position is None or point is None:
            return math.inf
        return math.hypot(self.current_position[0] - point[0], self.current_position[1] - point[1])

    def distance_between_xy(
        self, pose_a: Tuple[float, float, float], pose_b: Tuple[float, float, float]
    ) -> float:
        return math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])

    def heading_error_to_home_yaw(self) -> float:
        if self.current_yaw is None or self.home_yaw is None:
            return 0.0
        return shortest_angular_distance(self.home_yaw, self.current_yaw)

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

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
        self.publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoF1TenthPurePursuit()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
