#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


CONTROL_PERIOD_S = 0.05
STATUS_LOG_PERIOD_S = 0.5
WAIT_FOR_EXIT_DEBUG_LOG_PERIOD_S = 1.0
IDLE_WARN_PERIOD_S = 5.0
ABORT_ALERT_PERIOD_S = 2.0

START_X_M = 0.0215
START_Y_M = -0.1030
START_YAW_RAD = math.pi

MAX_LINEAR_SPEED_MPS = 0.05
MAX_ANGULAR_SPEED_RADPS = 0.30
REVERSE_APPROACH_SPEED_MPS = 0.035
FORWARD_APPROACH_SPEED_MPS = 0.030
TARGET_TOLERANCE_M = 0.025
YAW_TOLERANCE_RAD = math.radians(2.5)

# Target parking slot pose in world frame (R2C6: (0, -0.75) per project memory).
SLOT_WORLD_X_M = 0.0
SLOT_WORLD_Y_M = -0.75

# Stage trajectory parameters.
EVADE_REVERSE_DISTANCE_M = 0.60
PARK_STAGE_1_TARGET = (0.45, -0.10)
PARK_STAGE_1_FORWARD_SETUP_M = 0.12
PARK_STAGE_2_RADIUS_M = 0.45
PARK_STAGE_2_MIN_RADIUS_M = 0.18
PARK_STAGE_2_MAX_RADIUS_M = 0.45
PARK_STAGE_2_TARGET_YAW_RAD = -math.pi / 2.0
PARK_STAGE_2_LINEAR_SPEED_MPS = 0.035
# Stage 2 only: lift the angular limit so the robot can actually steer hard
# enough to follow the planned arc into the slot. The global cap stays at
# MAX_ANGULAR_SPEED_RADPS for the other stages (precision alignment, E-stop
# recovery, etc.) where aggressive yaw rate would overshoot.
PARK_STAGE_2_MAX_ANGULAR_SPEED_RADPS = 1.2
# Floor that guarantees adequate steering authority even when the kinematic
# arc speed (v/r) is small because linear.x has been throttled down.
PARK_STAGE_2_MIN_ANGULAR_SPEED_RADPS = 0.35
# Proportional gain on yaw error -- adds a feedback term on top of the
# kinematic arc so angular.z scales up when the robot is far from the target
# heading, instead of staying pinned to v/r.
PARK_STAGE_2_KP_YAW = 3.5
PARK_STAGE_3_SLOT_X_M = SLOT_WORLD_X_M
PARK_STAGE_3_DISTANCE_M = 0.20

DEPTH_INVALID_REPLACEMENT_M = 10.0      # Gazebo depth NaN/Inf/no-return => far/clear

PULL_FORWARD_DISTANCE_M = 0.17
PULL_FORWARD_SPEED_MPS = 0.035
PULL_FORWARD_TOLERANCE_M = 0.015
TARGET_SLOT_FRESHNESS_S = 0.5

# --- Requirement 2: Precision Alignment (Stage 3) ---
YAW_PRECISION_TOLERANCE_RAD = math.radians(1.0)   # final yaw error <= 1 deg
X_PRECISION_TOLERANCE_M = 0.008                   # final lateral error <= 8 mm
PARK_STAGE_3_FORWARD_SPEED_MPS = 0.022
PARK_STAGE_3_CREEP_SPEED_MPS = 0.010
PARK_STAGE_3_KP_LATERAL = 1.4
PARK_STAGE_3_KP_YAW = 2.2
PARK_STAGE_3_LATERAL_OFFSET_CLAMP_RAD = 0.18
PARK_STAGE_3_FINAL_STRAIGHTEN_S = 0.6              # hold straight pose before declaring success
PARK_STAGE_3_FINE_REMAINING_M = 0.04               # threshold to enter precision phase

# --- Requirement 3: Spot Validation (LiDAR + Depth fusion) ---
SLOT_DEPTH_REQUIRED_M = 0.30        # required free depth into the slot before committing
SLOT_DEPTH_ABORT_M = 0.18           # in-flight abort threshold during Stage 3
SPOT_VALIDATION_BLOCK_HOLD_S = 0.4  # debounce before declaring slot blocked
SPOT_VALIDATION_PASS_HOLD_S = 0.3   # debounce before declaring slot clear
SPOT_LIDAR_BODY_HALF_DEG = 12.0     # +/- 12 deg around body-forward for slot scan
SPOT_DEPTH_ROI = (0.30, 0.70, 0.30, 0.85)  # (x0, x1, y0, y1) image fractions
SPOT_DEPTH_PERCENTILE = 15.0
SPOT_DEPTH_MIN_PIXELS = 20

ESTOP_FRONT_START_DEG = -30.0
ESTOP_FRONT_END_DEG = 30.0
ESTOP_DISTANCE_M = 0.30
RECOVERY_SPEED_MPS = -0.03
RECOVERY_SECONDS = 0.8


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_angular_distance(target: float, current: float) -> float:
    return normalize_angle(target - current)


class MissionState(Enum):
    EVADE = "EVADE"
    WAIT_FOR_EXIT = "WAIT_FOR_EXIT"
    PULL_FORWARD = "PULL_FORWARD"
    PARK_STAGE_1 = "PARK_STAGE_1"
    PARK_STAGE_2 = "PARK_STAGE_2"
    SPOT_VALIDATION = "SPOT_VALIDATION"
    PARK_STAGE_3 = "PARK_STAGE_3"
    FINISH = "FINISH"
    ABORT = "ABORT"


class LimoValetParkingNode(Node):
    def __init__(self) -> None:
        super().__init__("limo_valet_parking_node")

        self.scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.depth_qos = qos_profile_sensor_data

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.debug_image_pub = self.create_publisher(Image, "debug/valet_exit_gate", 10)
        self.create_subscription(
            Odometry, "odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "scan", self.scan_callback, self.scan_qos
        )
        self.create_subscription(
            Image, "rgb/image_raw", self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "depth/image_raw", self.depth_callback, self.depth_qos
        )
        self.create_subscription(
            PoseStamped, "target_slot", self.target_slot_callback, 10
        )
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.state = MissionState.EVADE
        self.state_logged = set()
        now = self.get_clock().now()
        self.last_status_log_time = now
        self.last_wait_for_exit_debug_log_time: Optional[rclpy.time.Time] = None
        self.last_idle_warn_time = now
        self.last_abort_alert_time: Optional[rclpy.time.Time] = None
        self.abort_reason: str = ""

        self.odom_received = False
        self.scan_received = False
        self.image_received = False
        self.depth_received = False
        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_image_header = None
        self.latest_depth: Optional[np.ndarray] = None  # meters, float32
        self.target_slot_received = False
        self.target_slot_position: Optional[Tuple[float, float]] = None
        self.target_slot_yaw: Optional[float] = None
        self.latest_target_slot: Optional[PoseStamped] = None
        self.target_received_time = None

        self.front_obstacle_distance_m = math.inf
        self.evade_target: Optional[Tuple[float, float]] = None
        self.pull_forward_start_position: Optional[Tuple[float, float]] = None
        self.pull_forward_start_yaw: Optional[float] = None
        self.park_stage_1_target: Optional[Tuple[float, float]] = None
        self.stage_2_radius_m: Optional[float] = None
        self.stage_3_start_y: Optional[float] = None
        self.stage_3_goal_y: Optional[float] = None
        self.stage_3_final_align_start = None

        # Spot validation state
        self.spot_block_since = None
        self.spot_pass_since = None
        self.last_spot_lidar_clear_m = math.inf
        self.last_spot_depth_clear_m = math.inf

        self.recovery_active = False
        self.recovery_start_time = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        self.get_logger().info(
            "limo_valet_parking_node ready: "
            "EVADE -> WAIT_FOR_EXIT -> PULL_FORWARD -> SPOT_VALIDATION -> "
            "PARK_STAGE_1 -> PARK_STAGE_2 -> PARK_STAGE_3 -> FINISH (or ABORT)"
        )
        self.get_logger().info(
            f"Configured start pose: x={START_X_M:.4f}, y={START_Y_M:.4f}, "
            f"yaw={START_YAW_RAD:.4f} rad."
        )

    # ----- ROS subscription callbacks -----

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_received = True
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_received = True
        self.latest_scan = msg
        front_ranges = self.extract_sector_ranges(
            msg, ESTOP_FRONT_START_DEG, ESTOP_FRONT_END_DEG
        )
        self.front_obstacle_distance_m = min(front_ranges) if front_ranges else math.inf

    def image_callback(self, msg: Image) -> None:
        frame = self.decode_image(msg)
        if frame is None:
            return
        self.image_received = True
        self.latest_frame = frame
        self.latest_image_header = msg.header

    def depth_callback(self, msg: Image) -> None:
        depth = self.decode_depth(msg)
        if depth is None:
            return
        self.depth_received = True
        self.latest_depth = depth

    def target_slot_callback(self, msg: PoseStamped) -> None:
        orientation = msg.pose.orientation
        yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.latest_target_slot = msg
        self.target_received_time = self.get_clock().now()
        self.target_slot_position = (msg.pose.position.x, msg.pose.position.y)
        self.target_slot_yaw = yaw
        first_target = not self.target_slot_received
        self.target_slot_received = True
        if first_target:
            self.get_logger().info(
                "[TARGET] target_slot received: "
                f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, "
                f"yaw={yaw:.3f} rad."
            )

    # ----- main control loop -----

    def control_loop(self) -> None:
        self.log_state_once()

        if self.state == MissionState.ABORT:
            self.run_abort()
            self.maybe_log_status()
            return

        if not self.required_inputs_ready():
            self.handle_waiting_for_inputs()
            return

        # EVADE must always complete its full reverse (0.6 m) before checking
        # for target_slot. Previously this gate included EVADE, which let the
        # FSM jump straight to PULL_FORWARD the instant the tracker published
        # a (potentially false-positive) slot lock -- skipping the reverse
        # maneuver that is supposed to clear LIMO2's exit path.
        if self.state == MissionState.WAIT_FOR_EXIT:
            if self.target_slot_is_fresh():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.PULL_FORWARD)
                self.maybe_log_status()
                return

        # The legacy reverse-recovery E-stop is only safe before we commit to the
        # slot. Once we are validating / entering the slot, reversing would back
        # out of the parking spot, so the spot-validation logic supersedes it.
        if self.state in {
            MissionState.EVADE,
            MissionState.PULL_FORWARD,
            MissionState.PARK_STAGE_1,
            MissionState.PARK_STAGE_2,
        }:
            if self.handle_estop_or_recovery():
                self.maybe_log_status()
                return

        if self.state == MissionState.EVADE:
            if self.run_evade():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.WAIT_FOR_EXIT)
            self.maybe_log_status()
            return

        if self.state == MissionState.WAIT_FOR_EXIT:
            if self.run_wait_for_exit():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.PULL_FORWARD)
            self.maybe_log_status()
            return

        if self.state == MissionState.PULL_FORWARD:
            if self.run_pull_forward():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.SPOT_VALIDATION)
            self.maybe_log_status()
            return

        if self.state == MissionState.PARK_STAGE_1:
            if self.run_park_stage_1():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.PARK_STAGE_2)
            self.maybe_log_status()
            return

        if self.state == MissionState.PARK_STAGE_2:
            if self.run_park_stage_2():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.PARK_STAGE_3)
            self.maybe_log_status()
            return

        if self.state == MissionState.SPOT_VALIDATION:
            verdict = self.run_spot_validation()
            if verdict == "PASS":
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.PARK_STAGE_1)
            elif verdict == "ABORT":
                self.publish_cmd(0.0, 0.0)
                self.trigger_abort(
                    f"사전 스캔 단계 — lidar={self.last_spot_lidar_clear_m:.2f} m, "
                    f"depth={self.last_spot_depth_clear_m:.2f} m"
                )
            self.maybe_log_status()
            return

        if self.state == MissionState.PARK_STAGE_3:
            if self.run_park_stage_3():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(MissionState.FINISH)
            self.maybe_log_status()
            return

        if self.state == MissionState.FINISH:
            self.publish_cmd(0.0, 0.0)
            self.maybe_log_status()

    def required_inputs_ready(self) -> bool:
        return (
            self.odom_received
            and self.scan_received
            and self.image_received
            and self.depth_received
        )

    def handle_waiting_for_inputs(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if now - self.last_idle_warn_time < Duration(seconds=IDLE_WARN_PERIOD_S):
            return

        missing = []
        if not self.odom_received:
            missing.append("Odometry(odom)")
        if not self.scan_received:
            missing.append("LiDAR(scan)")
        if not self.image_received:
            missing.append("RGB(rgb/image_raw)")
        if not self.depth_received:
            missing.append("Depth(depth/image_raw)")
        self.get_logger().warn("Waiting for required inputs: " + ", ".join(missing))
        self.last_idle_warn_time = now

    def handle_estop_or_recovery(self) -> bool:
        now = self.get_clock().now()
        if self.recovery_active:
            elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
            if elapsed < RECOVERY_SECONDS:
                self.publish_cmd(RECOVERY_SPEED_MPS, 0.0)
                return True

            if self.front_obstacle_distance_m <= ESTOP_DISTANCE_M:
                self.publish_cmd(0.0, 0.0)
                return True

            self.recovery_active = False
            self.recovery_start_time = None
            self.publish_cmd(0.0, 0.0)
            return True

        if self.front_obstacle_distance_m <= ESTOP_DISTANCE_M:
            self.recovery_active = True
            self.recovery_start_time = now
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"[E-STOP] Front obstacle {self.front_obstacle_distance_m:.3f} m "
                f"<= {ESTOP_DISTANCE_M:.3f} m. Starting recovery reverse."
            )
            return True

        return False

    # ----- behaviors -----

    def run_evade(self) -> bool:
        if self.current_position is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.evade_target is None:
            self.evade_target = (
                self.current_position[0] + EVADE_REVERSE_DISTANCE_M,
                self.current_position[1],
            )
            self.get_logger().info(
                f"EVADE target set to x={self.evade_target[0]:.3f}, "
                f"y={self.evade_target[1]:.3f}."
            )

        return self.drive_to_xy(self.evade_target, reverse=True)

    def run_wait_for_exit(self) -> bool:
        """Wait for dynamic_tracker_node to publish /limo1/target_slot."""
        self.maybe_log_wait_for_exit_debug()
        self.publish_cmd(0.0, 0.0)
        return self.target_slot_is_fresh()

    def run_pull_forward(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.pull_forward_start_position is None:
            self.pull_forward_start_position = self.current_position
            self.pull_forward_start_yaw = self.current_yaw
            self.get_logger().info(
                "PULL_FORWARD start: "
                f"target_distance={PULL_FORWARD_DISTANCE_M:.3f} m, "
                "angular=0.000 rad/s."
            )

        dx = self.current_position[0] - self.pull_forward_start_position[0]
        dy = self.current_position[1] - self.pull_forward_start_position[1]
        start_yaw = self.pull_forward_start_yaw
        if start_yaw is None:
            start_yaw = self.current_yaw
        traveled = max(0.0, dx * math.cos(start_yaw) + dy * math.sin(start_yaw))
        remaining = PULL_FORWARD_DISTANCE_M - traveled
        if remaining <= PULL_FORWARD_TOLERANCE_M:
            self.get_logger().info(
                f"PULL_FORWARD complete: traveled={traveled:.3f} m."
            )
            return True

        linear = min(PULL_FORWARD_SPEED_MPS, max(0.012, remaining * 0.5))
        self.publish_cmd(linear, 0.0)
        return False

    def run_park_stage_1(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.park_stage_1_target is None:
            self.park_stage_1_target = self.compute_park_stage_1_target()
            self.get_logger().info(
                f"PARK_STAGE_1 target set to x={self.park_stage_1_target[0]:.3f}, "
                f"y={self.park_stage_1_target[1]:.3f}."
            )

        return self.drive_to_xy(self.park_stage_1_target, reverse=False)

    def compute_park_stage_1_target(self) -> Tuple[float, float]:
        if self.current_position is None or self.current_yaw is None:
            return PARK_STAGE_1_TARGET
        if self.target_slot_position is None or self.target_slot_yaw is None:
            return PARK_STAGE_1_TARGET

        slot_x, slot_y = self.target_slot_position
        slot_yaw = self.target_slot_yaw
        final_heading = (math.cos(slot_yaw), math.sin(slot_yaw))
        left_normal = (-math.sin(slot_yaw), math.cos(slot_yaw))
        approach_distance = PARK_STAGE_3_DISTANCE_M + PARK_STAGE_2_RADIUS_M
        approach = (
            slot_x - final_heading[0] * approach_distance
            + left_normal[0] * PARK_STAGE_2_RADIUS_M,
            slot_y - final_heading[1] * approach_distance
            + left_normal[1] * PARK_STAGE_2_RADIUS_M,
        )

        x, y = self.current_position
        dx = approach[0] - x
        dy = approach[1] - y
        forward_projection = dx * math.cos(self.current_yaw) + dy * math.sin(
            self.current_yaw
        )
        if forward_projection > TARGET_TOLERANCE_M:
            return approach

        return (
            x + math.cos(self.current_yaw) * PARK_STAGE_1_FORWARD_SETUP_M,
            y + math.sin(self.current_yaw) * PARK_STAGE_1_FORWARD_SETUP_M,
        )

    def run_park_stage_2(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.stage_2_radius_m is None:
            self.stage_2_radius_m = self.compute_stage_2_radius()
            self.get_logger().info(
                f"PARK_STAGE_2 dynamic radius={self.stage_2_radius_m:.3f} m."
            )

        target_yaw = self.target_slot_yaw
        if target_yaw is None:
            target_yaw = PARK_STAGE_2_TARGET_YAW_RAD
        yaw_error = shortest_angular_distance(target_yaw, self.current_yaw)
        if abs(yaw_error) <= YAW_TOLERANCE_RAD:
            self.get_logger().info("PARK_STAGE_2: reached -pi/2 yaw, handing off to PARK_STAGE_3.")
            return True

        # Kinematic arc speed for v = omega * r. At v=0.035, r=0.45 this is only
        # ~0.078 rad/s, which is well below what the LIMO chassis can deliver
        # and was the root cause of the wide turn that overshot the slot.
        arc_angular_speed = abs(PARK_STAGE_2_LINEAR_SPEED_MPS) / self.stage_2_radius_m
        # Yaw-error feedback term -- this is the low-speed compensation: it
        # boosts angular.z based on how far the heading is from the target,
        # independent of linear.x, so steering authority does not collapse when
        # we slow down or come to a stop for in-place rotation.
        yaw_feedback_mag = abs(yaw_error) * PARK_STAGE_2_KP_YAW

        if yaw_error > 0.0:
            linear = PARK_STAGE_2_LINEAR_SPEED_MPS
            angular_mag = arc_angular_speed + yaw_feedback_mag
            angular_mag = max(angular_mag, PARK_STAGE_2_MIN_ANGULAR_SPEED_RADPS)
            angular_mag = clamp(
                angular_mag, 0.0, PARK_STAGE_2_MAX_ANGULAR_SPEED_RADPS
            )
            angular = angular_mag
        else:
            # In-place rotation toward the slot heading. linear=0 means v/r is
            # not usable, so rely entirely on the yaw-feedback term plus a
            # minimum floor to keep the wheels actively turning.
            linear = 0.0
            angular_mag = max(yaw_feedback_mag, PARK_STAGE_2_MIN_ANGULAR_SPEED_RADPS)
            angular_mag = clamp(
                angular_mag, 0.0, PARK_STAGE_2_MAX_ANGULAR_SPEED_RADPS
            )
            angular = -angular_mag

        self.publish_cmd(linear, angular, angular_limit=PARK_STAGE_2_MAX_ANGULAR_SPEED_RADPS)
        return False

    def compute_stage_2_radius(self) -> float:
        if self.current_position is None or self.target_slot_position is None:
            return PARK_STAGE_2_RADIUS_M
        x_offset = math.hypot(
            self.current_position[0] - self.target_slot_position[0],
            self.current_position[1] - self.target_slot_position[1],
        ) - PARK_STAGE_3_DISTANCE_M
        return clamp(
            x_offset,
            PARK_STAGE_2_MIN_RADIUS_M,
            PARK_STAGE_2_MAX_RADIUS_M,
        )

    def run_spot_validation(self) -> str:
        """CHECK_SPOT: accept the target slot produced by dynamic_tracker_node."""
        self.publish_cmd(0.0, 0.0)

        self.spot_pass_since, cleared_held = self.update_hold_gate(
            self.target_slot_received,
            self.spot_pass_since,
            SPOT_VALIDATION_PASS_HOLD_S,
        )

        if cleared_held:
            target_x, target_y = self.target_slot_position
            self.get_logger().info(
                f"[CHECK_SPOT] PASS — target_slot=({target_x:.3f}, {target_y:.3f}), "
                f"yaw={self.target_slot_yaw:.3f} rad."
            )
            return "PASS"

        return "WAIT"

    def measure_spot_depth_lidar(self) -> float:
        if self.latest_scan is None:
            return math.inf
        ranges = self.extract_sector_ranges(
            self.latest_scan,
            -SPOT_LIDAR_BODY_HALF_DEG,
            SPOT_LIDAR_BODY_HALF_DEG,
        )
        if not ranges:
            return math.inf
        ranges.sort()
        cutoff = max(1, len(ranges) // 5)  # 20th percentile to reject noise
        return ranges[cutoff - 1]

    def measure_spot_depth_camera(self) -> float:
        if self.latest_depth is None:
            return math.inf
        depth = self.latest_depth
        h, w = depth.shape[:2]
        x0 = int(w * SPOT_DEPTH_ROI[0])
        x1 = int(w * SPOT_DEPTH_ROI[1])
        y0 = int(h * SPOT_DEPTH_ROI[2])
        y1 = int(h * SPOT_DEPTH_ROI[3])
        return self._depth_percentile(depth[y0:y1, x0:x1], SPOT_DEPTH_PERCENTILE)

    @staticmethod
    def _depth_percentile(roi: np.ndarray, percentile: float) -> float:
        if roi.size == 0:
            return math.inf
        finite = roi[
            np.isfinite(roi)
            & (roi > 0.05)
            & (roi <= DEPTH_INVALID_REPLACEMENT_M)
        ]
        if finite.size < SPOT_DEPTH_MIN_PIXELS:
            return math.inf
        return float(np.percentile(finite, percentile))

    def run_park_stage_3(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False
        if self.target_slot_position is None or self.target_slot_yaw is None:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn("PARK_STAGE_3 waiting for target_slot.")
            return False

        if self.stage_3_start_y is None:
            self.stage_3_start_y = self.current_position[1]
            self.stage_3_goal_y = self.target_slot_position[1]
            self.get_logger().info(
                f"PARK_STAGE_3 target: x={self.target_slot_position[0]:.3f}, "
                f"y={self.target_slot_position[1]:.3f}, "
                f"yaw={self.target_slot_yaw:.3f}."
            )

        # Continuous slot re-validation. Abort hard if depth collapses during entry.
        lidar_clear = self.measure_spot_depth_lidar()
        depth_clear = self.measure_spot_depth_camera()
        self.last_spot_lidar_clear_m = lidar_clear
        self.last_spot_depth_clear_m = depth_clear
        if min(lidar_clear, depth_clear) < SLOT_DEPTH_ABORT_M:
            self.publish_cmd(0.0, 0.0)
            self.trigger_abort(
                f"진입 중 검출 — lidar={lidar_clear:.2f} m, depth={depth_clear:.2f} m"
            )
            return False

        target_x, target_y = self.target_slot_position
        target_yaw = self.target_slot_yaw
        final_heading = (math.cos(target_yaw), math.sin(target_yaw))
        left_normal = (-math.sin(target_yaw), math.cos(target_yaw))
        to_target = (
            target_x - self.current_position[0],
            target_y - self.current_position[1],
        )
        remaining = max(
            0.0,
            to_target[0] * final_heading[0] + to_target[1] * final_heading[1],
        )
        x_error = (
            (self.current_position[0] - target_x) * left_normal[0]
            + (self.current_position[1] - target_y) * left_normal[1]
        )
        yaw_error_to_target = shortest_angular_distance(target_yaw, self.current_yaw)

        # ---- Coarse forward phase ----
        if remaining > PARK_STAGE_3_FINE_REMAINING_M:
            desired_yaw_offset = clamp(
                x_error * PARK_STAGE_3_KP_LATERAL,
                -PARK_STAGE_3_LATERAL_OFFSET_CLAMP_RAD,
                PARK_STAGE_3_LATERAL_OFFSET_CLAMP_RAD,
            )
            desired_yaw = target_yaw - desired_yaw_offset
            yaw_err = shortest_angular_distance(desired_yaw, self.current_yaw)
            linear = clamp(remaining * 0.4, 0.015, PARK_STAGE_3_FORWARD_SPEED_MPS)
            angular = clamp(
                yaw_err * PARK_STAGE_3_KP_YAW,
                -MAX_ANGULAR_SPEED_RADPS,
                MAX_ANGULAR_SPEED_RADPS,
            )
            self.publish_cmd(linear, angular)
            self.stage_3_final_align_start = None
            return False

        # ---- Precision phase ----
        yaw_ok = abs(yaw_error_to_target) <= YAW_PRECISION_TOLERANCE_RAD
        x_ok = abs(x_error) <= X_PRECISION_TOLERANCE_M
        depth_ok = remaining <= 0.005

        if yaw_ok and x_ok and depth_ok:
            now = self.get_clock().now()
            if self.stage_3_final_align_start is None:
                self.stage_3_final_align_start = now
            elapsed = (now - self.stage_3_final_align_start).nanoseconds / 1e9
            self.publish_cmd(0.0, 0.0)
            if elapsed >= PARK_STAGE_3_FINAL_STRAIGHTEN_S:
                self.get_logger().info(
                    "PARK_STAGE_3 precision OK: "
                    f"x_err={x_error * 1000.0:+.2f} mm, "
                    f"yaw_err={math.degrees(yaw_error_to_target):+.2f} deg, "
                    f"remaining={remaining * 1000.0:+.2f} mm."
                )
                return True
            return False

        self.stage_3_final_align_start = None

        # In the fine zone, prefer rotating in place when yaw is still off so the
        # final pose can be straightened without nudging y further.
        if abs(yaw_error_to_target) > math.radians(1.5):
            angular = clamp(
                yaw_error_to_target * PARK_STAGE_3_KP_YAW,
                -MAX_ANGULAR_SPEED_RADPS * 0.7,
                MAX_ANGULAR_SPEED_RADPS * 0.7,
            )
            self.publish_cmd(0.0, angular)
            return False

        # Small lateral correction left over: curve gently while creeping forward.
        desired_yaw_offset = clamp(
            x_error * PARK_STAGE_3_KP_LATERAL * 0.8,
            -PARK_STAGE_3_LATERAL_OFFSET_CLAMP_RAD * 0.5,
            PARK_STAGE_3_LATERAL_OFFSET_CLAMP_RAD * 0.5,
        )
        desired_yaw = target_yaw - desired_yaw_offset
        yaw_err = shortest_angular_distance(desired_yaw, self.current_yaw)
        creep = 0.0 if depth_ok else PARK_STAGE_3_CREEP_SPEED_MPS
        angular = clamp(
            yaw_err * PARK_STAGE_3_KP_YAW,
            -MAX_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.publish_cmd(creep, angular)
        return False

    def trigger_abort(self, reason: str) -> None:
        if self.state == MissionState.ABORT:
            return
        self.abort_reason = reason
        self.get_logger().error(
            f"[ALERT] 주차 불가: 주차 공간 미확보 (사용자 휴대폰 알림 연동 예정) — {reason}"
        )
        self.transition_to(MissionState.ABORT)

    def run_abort(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if (
            self.last_abort_alert_time is None
            or now - self.last_abort_alert_time >= Duration(seconds=ABORT_ALERT_PERIOD_S)
        ):
            self.get_logger().error(
                "[ALERT] 주차 불가: 주차 공간 미확보 "
                f"(사용자 휴대폰 알림 연동 예정) — {self.abort_reason}"
            )
            self.last_abort_alert_time = now

    def drive_to_xy(self, target: Tuple[float, float], reverse: bool) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        if distance <= TARGET_TOLERANCE_M:
            return True

        desired_motion_yaw = math.atan2(dy, dx)
        control_heading = (
            normalize_angle(self.current_yaw + math.pi)
            if reverse
            else self.current_yaw
        )
        heading_error = shortest_angular_distance(desired_motion_yaw, control_heading)
        speed = clamp(distance * 0.35, 0.015, REVERSE_APPROACH_SPEED_MPS)
        linear = -speed if reverse else speed
        if abs(heading_error) > math.radians(35.0):
            linear *= 0.45
        angular = clamp(
            heading_error * 1.4,
            -MAX_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.publish_cmd(linear, angular)
        return False

    # ----- helpers -----

    def update_hold_gate(self, raw_pass: bool, since_time, hold_seconds: float):
        now = self.get_clock().now()
        if not raw_pass:
            return None, False
        if since_time is None:
            since_time = now
        held = now - since_time >= Duration(seconds=hold_seconds)
        return since_time, held

    def target_slot_is_fresh(self) -> bool:
        if not self.target_slot_received or self.target_received_time is None:
            return False
        return (
            self.get_clock().now() - self.target_received_time
            <= Duration(seconds=TARGET_SLOT_FRESHNESS_S)
        )

    def target_slot_age_s(self) -> float:
        if self.target_received_time is None:
            return math.inf
        return (self.get_clock().now() - self.target_received_time).nanoseconds / 1e9

    @staticmethod
    def angle_in_sector(angle: float, low: float, high: float) -> bool:
        low_n = normalize_angle(low)
        high_n = normalize_angle(high)
        if low_n <= high_n:
            return low_n <= angle <= high_n
        return angle >= low_n or angle <= high_n

    def extract_sector_ranges(
        self, msg: LaserScan, start_deg: float, end_deg: float
    ) -> List[float]:
        sector_ranges: List[float] = []
        for index, distance in enumerate(msg.ranges):
            usable = self.usable_scan_range(distance, msg)
            if usable is None:
                continue
            angle = msg.angle_min + (index * msg.angle_increment)
            angle_deg = math.degrees(angle)
            if start_deg <= angle_deg <= end_deg:
                sector_ranges.append(usable)
        return sector_ranges

    def usable_scan_range(self, distance: float, msg: LaserScan) -> Optional[float]:
        if math.isnan(distance):
            return None
        if math.isinf(distance):
            return msg.range_max
        if distance < msg.range_min:
            return None
        return min(distance, msg.range_max)

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

    def decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None
        encoding = msg.encoding.lower()
        try:
            if encoding in ("32fc1", "32fc"):
                buf = np.frombuffer(msg.data, dtype=np.float32)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32)
            elif encoding in ("16uc1", "mono16"):
                buf = np.frombuffer(msg.data, dtype=np.uint16)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
            else:
                # Fall back to float32 assumption (gazebo_ros_openni_kinect default).
                buf = np.frombuffer(msg.data, dtype=np.float32)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f"Depth reshape failed for encoding '{encoding}': {exc}")
            return None
        depth = np.nan_to_num(
            depth,
            nan=DEPTH_INVALID_REPLACEMENT_M,
            posinf=DEPTH_INVALID_REPLACEMENT_M,
            neginf=DEPTH_INVALID_REPLACEMENT_M,
        )
        depth = np.where(depth > 0.05, depth, DEPTH_INVALID_REPLACEMENT_M)
        depth = np.minimum(depth, DEPTH_INVALID_REPLACEMENT_M).astype(np.float32)
        return depth

    def publish_cmd(
        self,
        linear: float,
        angular: float,
        angular_limit: Optional[float] = None,
    ) -> None:
        # angular_limit lets a specific stage (currently PARK_STAGE_2) request a
        # higher angular cap than the global default without leaking that cap
        # into precision alignment or recovery, where MAX_ANGULAR_SPEED_RADPS
        # is intentionally conservative.
        max_angular = (
            angular_limit if angular_limit is not None else MAX_ANGULAR_SPEED_RADPS
        )
        linear = clamp(linear, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS)
        angular = clamp(angular, -max_angular, max_angular)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.last_cmd_linear = linear
        self.last_cmd_angular = angular
        self.cmd_pub.publish(cmd)

    def maybe_log_status(self) -> None:
        now = self.get_clock().now()
        if now - self.last_status_log_time < Duration(seconds=STATUS_LOG_PERIOD_S):
            return

        x = self.current_position[0] if self.current_position else math.nan
        y = self.current_position[1] if self.current_position else math.nan
        yaw = self.current_yaw if self.current_yaw is not None else math.nan
        target_x = self.target_slot_position[0] if self.target_slot_position else math.nan
        target_y = self.target_slot_position[1] if self.target_slot_position else math.nan
        target_yaw = self.target_slot_yaw if self.target_slot_yaw is not None else math.nan
        target_age = self.target_slot_age_s()
        self.get_logger().info(
            f"State={self.state.value} pose=({x:.3f}, {y:.3f}, {yaw:.3f}) "
            f"cmd=({self.last_cmd_linear:.3f}, {self.last_cmd_angular:.3f}) "
            f"front={self.front_obstacle_distance_m:.3f} "
            f"target=({target_x:.3f}, {target_y:.3f}, {target_yaw:.3f}, "
            f"received={self.target_slot_received}, fresh={self.target_slot_is_fresh()}, "
            f"age={target_age:.2f}s) "
            f"slot=(lidar={self.last_spot_lidar_clear_m:.2f} "
            f"depth={self.last_spot_depth_clear_m:.2f})"
        )
        self.last_status_log_time = now

    def maybe_log_wait_for_exit_debug(self) -> None:
        now = self.get_clock().now()
        if (
            self.last_wait_for_exit_debug_log_time is not None
            and now - self.last_wait_for_exit_debug_log_time
            < Duration(seconds=WAIT_FOR_EXIT_DEBUG_LOG_PERIOD_S)
        ):
            return

        self.get_logger().info(
            "[DEBUG] WAIT_FOR_EXIT target wait: "
            f"target_received={self.target_slot_received}, "
            f"target_fresh={self.target_slot_is_fresh()}, "
            f"target_age={self.target_slot_age_s():.2f}s, "
            f"topic=target_slot"
        )
        self.last_wait_for_exit_debug_log_time = now

    def transition_to(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return

        previous_state = self.state
        self.get_logger().info(f"Transition: {previous_state.value} -> {new_state.value}")
        self.state = new_state
        self.state_logged.discard(new_state)

        if new_state == MissionState.WAIT_FOR_EXIT:
            self.last_wait_for_exit_debug_log_time = None
        elif new_state == MissionState.PULL_FORWARD:
            self.pull_forward_start_position = None
            self.pull_forward_start_yaw = None
        elif new_state == MissionState.SPOT_VALIDATION:
            self.spot_block_since = None
            self.spot_pass_since = None
        elif new_state == MissionState.PARK_STAGE_1:
            self.park_stage_1_target = None
        elif new_state == MissionState.PARK_STAGE_2:
            self.stage_2_radius_m = None
        elif new_state == MissionState.PARK_STAGE_3:
            self.stage_3_start_y = None
            self.stage_3_goal_y = None
            self.stage_3_final_align_start = None
            self.spot_block_since = None
            self.spot_pass_since = None
        elif new_state == MissionState.FINISH:
            self.get_logger().info("Mission complete. FINISH state reached.")
        elif new_state == MissionState.ABORT:
            self.last_abort_alert_time = None

    def log_state_once(self) -> None:
        if self.state in self.state_logged:
            return
        self.get_logger().info(f"State -> {self.state.value}")
        self.state_logged.add(self.state)

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop(self) -> None:
        self.publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoValetParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
