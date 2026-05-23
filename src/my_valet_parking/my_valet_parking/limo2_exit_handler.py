#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan


CONTROL_PERIOD_S = 0.05
STATUS_LOG_PERIOD_S = 1.0
IDLE_WARN_PERIOD_S = 5.0

WAIT_SECONDS = 10.0
SPEED_MPS = 0.05
DRIVE_FREE_SPEED_MPS = 0.30
MAX_LINEAR_SPEED_MPS = 0.30
MAX_ANGULAR_SPEED_RADPS = 0.30
TARGET_TOLERANCE_M = 0.025
YAW_TOLERANCE_RAD = math.radians(2.5)

SLOT_X_M = 0.0
SLOT_Y_M = -0.75
AISLE_Y_M = 0.0

ESTOP_FRONT_START_DEG = -30.0
ESTOP_FRONT_END_DEG = 30.0
ESTOP_DISTANCE_M = 0.30

EXIT_ALIGN_YAW_RAD = math.pi / 2.0
EXIT_ALIGN_TOLERANCE_RAD = math.radians(10.0)
EXIT_FORWARD_SECTOR_HALF_DEG = 45.0
EXIT_FORWARD_CAUTION_DISTANCE_M = 0.60
EXIT_FORWARD_HARD_STOP_M = 0.40

FREE_SCAN_BIN_DEG = 5.0
FREE_SCAN_WINDOW_HALF_DEG = 30.0
FREE_DRIVE_DISTANCE_M = 3.0
FREE_OBSTACLE_REDIRECT_M = 0.60

# LIMO uses libgazebo_ros_ackermann_drive: linear=0 freezes the wheels even
# if angular!=0. To rotate, we must drive a forward arc (linear nonzero).
TURN_ARC_SPEED_MPS = 0.10


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_angular_distance(target: float, current: float) -> float:
    return normalize_angle(target - current)


class ExitState(Enum):
    WAIT = "WAIT"
    EXIT_SLOT = "EXIT_SLOT"
    DECIDE_FREE = "DECIDE_FREE"
    TURN_TO_FREE = "TURN_TO_FREE"
    DRIVE_FREE = "DRIVE_FREE"
    FINISH = "FINISH"


class Limo2ExitHandler(Node):
    def __init__(self) -> None:
        super().__init__("limo2_exit_handler")

        self.scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.cmd_pub = self.create_publisher(Twist, "/limo2/cmd_vel", 10)
        self.create_subscription(Odometry, "/limo2/odom", self.odom_callback, 10)
        self.create_subscription(
            LaserScan,
            "/limo2/scan",
            self.scan_callback,
            self.scan_qos,
        )
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        now = self.get_clock().now()
        self.state = ExitState.WAIT
        self.state_logged = set()
        self.start_time = now
        self.last_status_log_time = now
        self.last_idle_warn_time = now
        self.last_estop_log_time = now

        self.odom_received = False
        self.scan_received = False
        self.latest_scan: Optional[LaserScan] = None
        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.front_obstacle_distance_m = math.inf
        self.front_wide_min_m = math.inf
        self.free_target_yaw: Optional[float] = None
        self.free_drive_traveled = 0.0
        self.free_drive_last_position: Optional[Tuple[float, float]] = None
        self.last_chosen_angle_deg = 0.0
        self.last_chosen_score = 0.0
        self.free_redirect_request = False

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        self.get_logger().info(
            "limo2_exit_handler ready: WAIT -> EXIT_SLOT -> "
            "DECIDE_FREE -> TURN_TO_FREE -> DRIVE_FREE -> FINISH"
        )
        self.get_logger().info(
            "Topics: /limo2/cmd_vel, /limo2/odom, /limo2/scan. "
            f"wait={WAIT_SECONDS:.1f}s, speed={SPEED_MPS:.3f} m/s."
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_received = True
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_received = True
        self.latest_scan = msg
        front_ranges = self.extract_sector_ranges(
            msg,
            ESTOP_FRONT_START_DEG,
            ESTOP_FRONT_END_DEG,
        )
        self.front_obstacle_distance_m = min(front_ranges) if front_ranges else math.inf
        wide_ranges = self.extract_sector_ranges(
            msg,
            -EXIT_FORWARD_SECTOR_HALF_DEG,
            EXIT_FORWARD_SECTOR_HALF_DEG,
        )
        self.front_wide_min_m = min(wide_ranges) if wide_ranges else math.inf

    def control_loop(self) -> None:
        self.log_state_once()

        if self.state == ExitState.FINISH:
            self.publish_cmd(0.0, 0.0)
            self.maybe_log_status("exit complete")
            return

        if not self.required_inputs_ready():
            self.handle_waiting_for_inputs()
            return

        if self.front_obstacle_distance_m <= ESTOP_DISTANCE_M:
            self.handle_front_estop()
            return

        if self.state == ExitState.WAIT:
            if self.run_wait():
                self.transition_to(ExitState.EXIT_SLOT)
            self.maybe_log_status()
            return

        if self.state == ExitState.EXIT_SLOT:
            if self.run_exit_slot():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(ExitState.DECIDE_FREE)
            self.maybe_log_status()
            return

        if self.state == ExitState.DECIDE_FREE:
            if self.run_decide_free():
                self.transition_to(ExitState.TURN_TO_FREE)
            self.maybe_log_status()
            return

        if self.state == ExitState.TURN_TO_FREE:
            if self.run_turn_to_free():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(ExitState.DRIVE_FREE)
            self.maybe_log_status()
            return

        if self.state == ExitState.DRIVE_FREE:
            if self.run_drive_free():
                self.publish_cmd(0.0, 0.0)
                self.transition_to(ExitState.FINISH)
            elif self.free_redirect_request:
                self.free_redirect_request = False
                self.transition_to(ExitState.DECIDE_FREE)
            self.maybe_log_status()

    def required_inputs_ready(self) -> bool:
        return self.odom_received and self.scan_received

    def handle_waiting_for_inputs(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if now - self.last_idle_warn_time < Duration(seconds=IDLE_WARN_PERIOD_S):
            return

        missing = []
        if not self.odom_received:
            missing.append("Odometry(/limo2/odom)")
        if not self.scan_received:
            missing.append("LiDAR(/limo2/scan)")
        self.get_logger().warn("Waiting for required inputs: " + ", ".join(missing))
        self.last_idle_warn_time = now

    def handle_front_estop(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if now - self.last_estop_log_time < Duration(seconds=STATUS_LOG_PERIOD_S):
            return

        self.get_logger().warn(
            f"[E-STOP] Front obstacle {self.front_obstacle_distance_m:.3f} m "
            f"<= {ESTOP_DISTANCE_M:.3f} m. Holding stop."
        )
        self.last_estop_log_time = now

    def run_wait(self) -> bool:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        return elapsed >= WAIT_SECONDS

    def run_exit_slot(self) -> bool:
        if self.current_yaw is None or self.current_position is None:
            self.publish_cmd(0.0, 0.0)
            return False

        # Stage A: align yaw to face aisle (+y) before forward motion.
        # Handles spawn poses where the robot is facing the wrong way.
        yaw_err = shortest_angular_distance(EXIT_ALIGN_YAW_RAD, self.current_yaw)
        if abs(yaw_err) > EXIT_ALIGN_TOLERANCE_RAD:
            angular = clamp(
                yaw_err * 1.2,
                -MAX_ANGULAR_SPEED_RADPS,
                MAX_ANGULAR_SPEED_RADPS,
            )
            self.publish_cmd(0.0, angular)
            return False

        # Stage B: forward-motion-specific hard stop on wider front sector.
        if self.front_wide_min_m <= EXIT_FORWARD_HARD_STOP_M:
            self.publish_cmd(0.0, 0.0)
            now = self.get_clock().now()
            if now - self.last_estop_log_time >= Duration(seconds=STATUS_LOG_PERIOD_S):
                self.get_logger().warn(
                    f"[EXIT_SLOT HOLD] front(wide ±{EXIT_FORWARD_SECTOR_HALF_DEG:.0f} deg) "
                    f"{self.front_wide_min_m:.3f} m <= "
                    f"{EXIT_FORWARD_HARD_STOP_M:.2f} m. Forward halted."
                )
                self.last_estop_log_time = now
            return False

        # Stage C: caution-zone slowdown linearly between hard stop and caution.
        if self.front_wide_min_m < EXIT_FORWARD_CAUTION_DISTANCE_M:
            denom = EXIT_FORWARD_CAUTION_DISTANCE_M - EXIT_FORWARD_HARD_STOP_M
            scale = clamp(
                (self.front_wide_min_m - EXIT_FORWARD_HARD_STOP_M) / denom,
                0.20,
                1.0,
            )
            speed_cap = SPEED_MPS * scale
        else:
            speed_cap = SPEED_MPS

        return self.drive_to_xy(
            (SLOT_X_M, AISLE_Y_M),
            reverse=False,
            max_speed=speed_cap,
        )

    def run_decide_free(self) -> bool:
        if self.latest_scan is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        # Collect every valid (body-angle deg, range) pair from the full scan.
        msg = self.latest_scan
        rays: List[Tuple[float, float]] = []
        for index, distance in enumerate(msg.ranges):
            usable = self.usable_scan_range(distance, msg)
            if usable is None:
                continue
            angle = msg.angle_min + (index * msg.angle_increment)
            angle_deg = math.degrees(normalize_angle(angle))
            rays.append((angle_deg, usable))

        if not rays:
            self.publish_cmd(0.0, 0.0)
            now = self.get_clock().now()
            if now - self.last_idle_warn_time >= Duration(seconds=IDLE_WARN_PERIOD_S):
                self.get_logger().warn(
                    "DECIDE_FREE: no valid LiDAR returns across full scan."
                )
                self.last_idle_warn_time = now
            return False

        # Sliding window over the full 360 deg: score each candidate body
        # angle by the median range within +- FREE_SCAN_WINDOW_HALF_DEG.
        # Median (vs max) prefers directions with sustained openness, not a
        # single long ray squeezing through a narrow gap.
        half_window = FREE_SCAN_WINDOW_HALF_DEG
        best_angle_deg = 0.0
        best_score = -math.inf
        cand = -180.0
        while cand < 180.0:
            in_window: List[float] = []
            for ang_deg, rng in rays:
                d = ((ang_deg - cand + 180.0) % 360.0) - 180.0
                if abs(d) <= half_window:
                    in_window.append(rng)
            if in_window:
                in_window.sort()
                score = in_window[len(in_window) // 2]
                improved = score > best_score + 1e-4
                tied_closer_to_forward = (
                    abs(score - best_score) <= 1e-4
                    and abs(cand) < abs(best_angle_deg)
                )
                if improved or tied_closer_to_forward:
                    best_score = score
                    best_angle_deg = cand
            cand += FREE_SCAN_BIN_DEG

        self.last_chosen_angle_deg = best_angle_deg
        self.last_chosen_score = best_score
        self.free_target_yaw = normalize_angle(
            self.current_yaw + math.radians(best_angle_deg)
        )

        self.get_logger().info(
            f"DECIDE_FREE: scanned {len(rays)} rays, "
            f"window=+-{half_window:.0f} deg. "
            f"best body angle={best_angle_deg:.1f} deg, "
            f"window-median range={best_score:.2f} m, "
            f"target_yaw={self.free_target_yaw:.3f} rad."
        )
        self.publish_cmd(0.0, 0.0)
        return True

    def run_turn_to_free(self) -> bool:
        if self.current_yaw is None or self.free_target_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        yaw_error = shortest_angular_distance(
            self.free_target_yaw, self.current_yaw
        )
        if abs(yaw_error) <= YAW_TOLERANCE_RAD:
            self.get_logger().info(
                "TURN_TO_FREE: aligned with empty-space direction."
            )
            return True

        # Ackermann forward arc: linear must be nonzero or the steering
        # plugin keeps the wheels stationary.
        arc_angular = clamp(
            yaw_error * 2.0,
            -MAX_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.publish_cmd(TURN_ARC_SPEED_MPS, arc_angular)
        return False

    def run_drive_free(self) -> bool:
        if (
            self.current_position is None
            or self.current_yaw is None
            or self.free_target_yaw is None
        ):
            self.publish_cmd(0.0, 0.0)
            return False

        # Path-length odometry accumulator (persists across redirects).
        if self.free_drive_last_position is not None:
            dx = self.current_position[0] - self.free_drive_last_position[0]
            dy = self.current_position[1] - self.free_drive_last_position[1]
            self.free_drive_traveled += math.hypot(dx, dy)
        self.free_drive_last_position = self.current_position

        if self.free_drive_traveled >= FREE_DRIVE_DISTANCE_M:
            self.get_logger().info(
                f"DRIVE_FREE: traveled {self.free_drive_traveled:.3f} m "
                f">= {FREE_DRIVE_DISTANCE_M:.2f} m. Stopping."
            )
            return True

        # Mid-flight redirect: front obstacle within 0.6 m -> re-decide.
        if self.front_obstacle_distance_m <= FREE_OBSTACLE_REDIRECT_M:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"DRIVE_FREE: front obstacle {self.front_obstacle_distance_m:.3f} m"
                f" <= {FREE_OBSTACLE_REDIRECT_M:.2f} m."
                f" Triggering re-decision (traveled "
                f"{self.free_drive_traveled:.3f} m)."
            )
            self.free_redirect_request = True
            return False

        # Heading drift correction along chosen yaw.
        yaw_err = shortest_angular_distance(
            self.free_target_yaw, self.current_yaw
        )
        angular = clamp(
            yaw_err * 1.5,
            -MAX_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.publish_cmd(DRIVE_FREE_SPEED_MPS, angular)
        return False

    def drive_to_xy(
        self,
        target: Tuple[float, float],
        reverse: bool,
        max_speed: float = SPEED_MPS,
    ) -> bool:
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
        speed = clamp(distance * 0.45, 0.015, max_speed)
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

    def extract_sector_ranges(
        self,
        msg: LaserScan,
        start_deg: float,
        end_deg: float,
    ) -> List[float]:
        sector_ranges: List[float] = []
        for index, distance in enumerate(msg.ranges):
            usable = self.usable_scan_range(distance, msg)
            if usable is None:
                continue

            angle = msg.angle_min + (index * msg.angle_increment)
            angle_deg = math.degrees(normalize_angle(angle))
            if start_deg <= angle_deg <= end_deg:
                sector_ranges.append(usable)
        return sector_ranges

    def usable_scan_range(self, distance: float, msg: LaserScan) -> Optional[float]:
        if math.isnan(distance):
            return None
        if math.isinf(distance):
            return msg.range_max
        if distance < msg.range_min:
            return distance
        return min(distance, msg.range_max)

    def publish_cmd(self, linear: float, angular: float) -> None:
        linear = clamp(linear, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS)
        angular = clamp(angular, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.last_cmd_linear = linear
        self.last_cmd_angular = angular
        self.cmd_pub.publish(cmd)

    def maybe_log_status(self, extra: str = "") -> None:
        now = self.get_clock().now()
        if now - self.last_status_log_time < Duration(seconds=STATUS_LOG_PERIOD_S):
            return

        x = self.current_position[0] if self.current_position else math.nan
        y = self.current_position[1] if self.current_position else math.nan
        yaw = self.current_yaw if self.current_yaw is not None else math.nan
        message = (
            f"state={self.state.value} pose=({x:.3f}, {y:.3f}, {yaw:.3f}) "
            f"cmd=({self.last_cmd_linear:.3f}, {self.last_cmd_angular:.3f}) "
            f"front={self.front_obstacle_distance_m:.3f} "
            f"wide={self.front_wide_min_m:.3f} "
            f"chosen=({self.last_chosen_angle_deg:.0f}deg,"
            f"{self.last_chosen_score:.2f}m) "
            f"trav={self.free_drive_traveled:.2f}"
        )
        if extra:
            message += f", {extra}"
        self.get_logger().info(message)
        self.last_status_log_time = now

    def transition_to(self, new_state: ExitState) -> None:
        if self.state == new_state:
            return

        self.get_logger().info(f"Transition: {self.state.value} -> {new_state.value}")
        self.state = new_state
        self.state_logged.discard(new_state)
        if new_state == ExitState.DECIDE_FREE:
            self.free_target_yaw = None
        elif new_state == ExitState.DRIVE_FREE:
            self.free_drive_last_position = None
        elif new_state == ExitState.FINISH:
            self.get_logger().info("limo2 exit sequence complete.")

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
    node = Limo2ExitHandler()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
