#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional, Tuple

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node


CONTROL_PERIOD_S = 0.05
STATUS_LOG_PERIOD_S = 1.0

ENTITY_NAME = "moving_passenger_car"
MODEL_Z_M = 0.02
SPEED_MPS = 0.05

SLOT_X_M = 0.0
SLOT_Y_M = -0.75
AISLE_Y_M = 0.0
INITIAL_YAW_RAD = math.pi / 2.0

TURN_RADIUS_M = 0.45
EXIT_X_M = -4.20
EXIT_YAW_RAD = math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class ExitState(Enum):
    BACK_OUT = "BACK_OUT"
    TURN_TO_MINUS_X = "TURN_TO_MINUS_X"
    EXIT_STRAIGHT = "EXIT_STRAIGHT"
    FINISH = "FINISH"


class CarExitController(Node):
    def __init__(self) -> None:
        super().__init__("car_exit_controller")

        self.declare_parameter("entity_name", ENTITY_NAME)
        self.declare_parameter("speed_mps", SPEED_MPS)
        self.declare_parameter("exit_x_m", EXIT_X_M)
        self.declare_parameter("set_entity_state_service", "/set_entity_state")

        self.entity_name = str(self.get_parameter("entity_name").value)
        self.speed_mps = clamp(
            abs(float(self.get_parameter("speed_mps").value)),
            0.001,
            SPEED_MPS,
        )
        self.exit_x_m = float(self.get_parameter("exit_x_m").value)
        self.set_entity_state_service = str(
            self.get_parameter("set_entity_state_service").value
        )

        self.client = self.create_client(
            SetEntityState,
            self.set_entity_state_service,
        )
        self.timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.state = ExitState.BACK_OUT
        self.state_logged = set()
        self.last_status_log_time = self.get_clock().now()
        self.pending_future = None

        self.back_out_start_time = None
        self.turn_start_time = None
        self.exit_start_time = None
        self.turn_start_pose: Optional[Tuple[float, float, float]] = None
        self.exit_start_pose: Optional[Tuple[float, float, float]] = None

        self.back_out_distance_m = abs(AISLE_Y_M - SLOT_Y_M)
        self.back_out_duration_s = self.back_out_distance_m / self.speed_mps
        self.turn_duration_s = (TURN_RADIUS_M * (math.pi / 2.0)) / self.speed_mps

        self.get_logger().info(
            "car_exit_controller ready: moving_passenger_car BACK_OUT -> "
            "TURN_TO_MINUS_X -> EXIT_STRAIGHT -> FINISH"
        )
        self.get_logger().info(
            f"speed={self.speed_mps:.3f} m/s, slot=({SLOT_X_M:.2f}, {SLOT_Y_M:.2f}), "
            f"aisle_y={AISLE_Y_M:.2f}, exit_x={self.exit_x_m:.2f}, "
            f"service={self.set_entity_state_service}"
        )

    def control_loop(self) -> None:
        self.check_pending_result()
        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.0)
            self.maybe_log_status(
                f"waiting for {self.set_entity_state_service}"
            )
            return

        self.log_state_once()
        now = self.get_clock().now()

        if self.state == ExitState.BACK_OUT:
            pose = self.compute_back_out_pose(now)
            self.send_entity_pose(*pose)
            if pose[1] >= AISLE_Y_M:
                self.transition_to(ExitState.TURN_TO_MINUS_X)
            self.maybe_log_status()
            return

        if self.state == ExitState.TURN_TO_MINUS_X:
            pose = self.compute_turn_pose(now)
            self.send_entity_pose(*pose)
            yaw_error = abs(
                math.atan2(
                    math.sin(EXIT_YAW_RAD - pose[2]),
                    math.cos(EXIT_YAW_RAD - pose[2]),
                )
            )
            if yaw_error <= math.radians(1.0):
                self.transition_to(ExitState.EXIT_STRAIGHT)
            self.maybe_log_status()
            return

        if self.state == ExitState.EXIT_STRAIGHT:
            pose = self.compute_exit_pose(now)
            self.send_entity_pose(*pose)
            if pose[0] <= self.exit_x_m:
                self.transition_to(ExitState.FINISH)
            self.maybe_log_status()
            return

        if self.state == ExitState.FINISH:
            exit_y = (
                self.exit_start_pose[1]
                if self.exit_start_pose is not None
                else AISLE_Y_M + TURN_RADIUS_M
            )
            self.send_entity_pose(self.exit_x_m, exit_y, EXIT_YAW_RAD)
            self.maybe_log_status("exit complete")

    def compute_back_out_pose(self, now) -> Tuple[float, float, float]:
        if self.back_out_start_time is None:
            self.back_out_start_time = now

        elapsed = (now - self.back_out_start_time).nanoseconds / 1e9
        progress = clamp(elapsed / self.back_out_duration_s, 0.0, 1.0)
        y = SLOT_Y_M + (AISLE_Y_M - SLOT_Y_M) * progress
        return SLOT_X_M, y, INITIAL_YAW_RAD

    def compute_turn_pose(self, now) -> Tuple[float, float, float]:
        if self.turn_start_time is None:
            self.turn_start_time = now
            self.turn_start_pose = (SLOT_X_M, AISLE_Y_M, INITIAL_YAW_RAD)

        elapsed = (now - self.turn_start_time).nanoseconds / 1e9
        progress = clamp(elapsed / self.turn_duration_s, 0.0, 1.0)
        theta = progress * (math.pi / 2.0)

        # Quarter-circle turn away from the LIMO side: x decreases, yaw ends at pi.
        x = SLOT_X_M - TURN_RADIUS_M * math.sin(theta)
        y = AISLE_Y_M + TURN_RADIUS_M * (1.0 - math.cos(theta))
        yaw = INITIAL_YAW_RAD + theta
        return x, y, yaw

    def compute_exit_pose(self, now) -> Tuple[float, float, float]:
        if self.exit_start_time is None:
            self.exit_start_time = now
            self.exit_start_pose = (
                SLOT_X_M - TURN_RADIUS_M,
                AISLE_Y_M + TURN_RADIUS_M,
                EXIT_YAW_RAD,
            )

        elapsed = (now - self.exit_start_time).nanoseconds / 1e9
        distance = elapsed * self.speed_mps
        x = self.exit_start_pose[0] - distance
        return x, self.exit_start_pose[1], EXIT_YAW_RAD

    def send_entity_pose(self, x: float, y: float, yaw: float) -> None:
        if self.pending_future is not None and not self.pending_future.done():
            return

        request = SetEntityState.Request()
        state = EntityState()
        state.name = self.entity_name
        state.reference_frame = "world"
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = MODEL_Z_M
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        state.twist = Twist()
        if self.state == ExitState.BACK_OUT:
            state.twist.linear.y = self.speed_mps
        elif self.state == ExitState.TURN_TO_MINUS_X:
            state.twist.linear.x = -self.speed_mps
            state.twist.angular.z = self.speed_mps / TURN_RADIUS_M
        elif self.state == ExitState.EXIT_STRAIGHT:
            state.twist.linear.x = -self.speed_mps

        request.state = state
        self.pending_future = self.client.call_async(request)

    def check_pending_result(self) -> None:
        if self.pending_future is None or not self.pending_future.done():
            return

        try:
            result = self.pending_future.result()
        except Exception as exc:
            self.get_logger().warn(f"/set_entity_state failed: {exc}")
            self.pending_future = None
            return

        if result is not None and not result.success:
            self.get_logger().warn(f"/set_entity_state rejected: {result.status_message}")
        self.pending_future = None

    def transition_to(self, new_state: ExitState) -> None:
        if self.state == new_state:
            return

        self.get_logger().info(f"Transition: {self.state.value} -> {new_state.value}")
        self.state = new_state
        self.state_logged.discard(new_state)
        if new_state == ExitState.TURN_TO_MINUS_X:
            self.turn_start_time = None
        elif new_state == ExitState.EXIT_STRAIGHT:
            self.exit_start_time = None
        elif new_state == ExitState.FINISH:
            self.get_logger().info("moving_passenger_car has exited the parking lot.")

    def log_state_once(self) -> None:
        if self.state in self.state_logged:
            return
        self.get_logger().info(f"State -> {self.state.value}")
        self.state_logged.add(self.state)

    def maybe_log_status(self, extra: str = "") -> None:
        now = self.get_clock().now()
        if now - self.last_status_log_time < Duration(seconds=STATUS_LOG_PERIOD_S):
            return

        message = f"state={self.state.value}, speed={self.speed_mps:.3f} m/s"
        if extra:
            message += f", {extra}"
        self.get_logger().info(message)
        self.last_status_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarExitController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
