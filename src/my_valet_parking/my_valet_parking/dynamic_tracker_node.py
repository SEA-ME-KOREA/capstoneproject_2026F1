#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


CONTROL_PERIOD_S = 0.05
STATUS_LOG_PERIOD_S = 1.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class SideProjection:
    points_odom: np.ndarray
    ranges: np.ndarray
    close_points_odom: np.ndarray
    representative_clearance_m: float
    clear_ratio: float
    close_count: int
    forward_unit_odom: np.ndarray
    side_unit_odom: np.ndarray


class DynamicTrackerNode(Node):
    """Tracks a side-parked vehicle in odom frame and publishes the freed slot.

    Ego-motion compensation is done by projecting each selected LiDAR ray into
    the odom frame using the latest odometry pose:

        p_odom = [x, y] + R(yaw) * [range*cos(theta), range*sin(theta)]

    Once the initial side cluster disappears from the same odom-frame ROI, the
    node publishes the estimated empty slot center as geometry_msgs/PoseStamped.
    """

    def __init__(self) -> None:
        super().__init__("dynamic_tracker_node")

        self.scan_topic = self.declare_parameter("scan_topic", "/limo1/scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/limo1/odom").value
        self.target_topic = self.declare_parameter(
            "target_topic", "/limo1/target_slot"
        ).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        self.side_scan_center_deg = float(
            self.declare_parameter("side_scan_center_deg", 90.0).value
        )
        self.side_scan_half_deg = float(
            self.declare_parameter("side_scan_half_deg", 8.0).value
        )
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.05).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 10.0).value)
        self.occupied_distance_m = float(
            self.declare_parameter("occupied_distance_m", 0.45).value
        )
        self.baseline_capture_distance_m = float(
            self.declare_parameter("baseline_capture_distance_m", 0.75).value
        )
        self.clear_distance_m = float(
            self.declare_parameter("clear_distance_m", 0.65).value
        )
        min_clear_ratio_default = float(
            self.declare_parameter("min_clear_ratio", 0.25).value
        )
        self.min_clear_ratio = float(
            self.declare_parameter(
                "clear_ratio_min", min_clear_ratio_default
            ).value
        )
        self.side_percentile = float(
            self.declare_parameter("side_percentile", 70.0).value
        )
        self.min_side_rays = int(self.declare_parameter("min_side_rays", 3).value)
        self.baseline_min_points = int(
            self.declare_parameter("baseline_min_points", 3).value
        )

        self.track_box_length_m = float(
            self.declare_parameter("track_box_length_m", 0.35).value
        )
        self.track_box_width_m = float(
            self.declare_parameter("track_box_width_m", 0.30).value
        )
        self.clear_count_fraction = float(
            self.declare_parameter("clear_count_fraction", 0.25).value
        )
        self.clear_max_points = int(
            self.declare_parameter("clear_max_points", 2).value
        )
        self.clear_hold_s = float(self.declare_parameter("clear_hold_s", 0.70).value)
        self.slot_center_offset_m = float(
            self.declare_parameter("slot_center_offset_m", 0.30).value
        )

        self.target_yaw_rad = float(
            self.declare_parameter("target_yaw_rad", -math.pi / 2.0).value
        )
        self.use_side_axis_yaw = bool(
            self.declare_parameter("use_side_axis_yaw", False).value
        )
        self.allow_fallback_without_baseline = bool(
            self.declare_parameter("allow_fallback_without_baseline", False).value
        )
        self.fallback_target_x = float(
            self.declare_parameter("fallback_target_x", 0.0).value
        )
        self.fallback_target_y = float(
            self.declare_parameter("fallback_target_y", -0.75).value
        )

        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.10).value
        )

        # Chassis self-occlusion mask: rear-left/right sectors blocked by the
        # robot body. Stored in radians, normalized to (-pi, pi].
        self.self_occlusion_mask_rad = (
            (math.radians(130.0), math.radians(170.0)),
            (math.radians(-170.0), math.radians(-130.0)),
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)
        self.debug_timer = self.create_timer(STATUS_LOG_PERIOD_S, self.debug_log_loop)

        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_rx_time = None
        self.current_pose: Optional[Tuple[float, float, float]] = None

        self.baseline_center_odom: Optional[np.ndarray] = None
        self.baseline_forward_unit_odom: Optional[np.ndarray] = None
        self.baseline_side_unit_odom: Optional[np.ndarray] = None
        self.baseline_roi_count = 0
        self.seen_occupied = False

        self.clear_since = None
        self.target_acquired = False
        self.target_pose: Optional[PoseStamped] = None
        self.last_publish_time = self.get_clock().now()

        self.last_side_clearance_m = math.inf
        self.last_clear_ratio = 0.0
        self.last_close_count = 0
        self.last_roi_count = 0
        self.last_projection_valid = False

        self.get_logger().info(
            "dynamic_tracker_node ready: "
            f"scan={self.scan_topic}, odom={self.odom_topic}, "
            f"target={self.target_topic}, side={self.side_scan_center_deg:+.1f} deg "
            f"+/- {self.side_scan_half_deg:.1f} deg"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_rx_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def control_loop(self) -> None:
        if self.latest_scan is None or self.current_pose is None:
            return

        projection = self.project_side_scan(self.latest_scan)
        if projection is None:
            self.last_projection_valid = False
            return

        self.last_projection_valid = True
        self.last_side_clearance_m = projection.representative_clearance_m
        self.last_clear_ratio = projection.clear_ratio
        self.last_close_count = projection.close_count

        if not self.target_acquired:
            # The baseline ROI is anchored in the odom frame at the parked
            # vehicle's location, so the "is the slot still occupied?" check
            # must look at ALL LiDAR rays projected into odom -- not just the
            # +90 +/- 8 deg side-scan window. Restricting it to the side window
            # caused the tracker to lose sight of LIMO2 as soon as the ego
            # vehicle moved a few cm during EVADE, producing a false slot-
            # cleared verdict and short-circuiting the reverse maneuver.
            all_points_odom = self.project_all_rays_to_odom(self.latest_scan)
            self.update_baseline_if_occupied(projection, all_points_odom)
            roi_count = self.count_points_in_tracked_roi(all_points_odom)
            self.last_roi_count = roi_count
            self.evaluate_clearance(projection, roi_count)

        if self.target_acquired:
            self.publish_target_if_due()

    def project_side_scan(self, msg: LaserScan) -> Optional[SideProjection]:
        x, y, yaw = self.current_pose
        center = math.radians(self.side_scan_center_deg)
        half = math.radians(self.side_scan_half_deg)
        low = center - half
        high = center + half

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_unit = np.array([cos_yaw, sin_yaw], dtype=np.float32)
        side_yaw = yaw + center
        side_unit = np.array(
            [math.cos(side_yaw), math.sin(side_yaw)], dtype=np.float32
        )

        points = []
        ranges = []
        close_points = []
        for index, raw_distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)

            # Drop rays inside the chassis self-occlusion sectors.
            if any(
                lo <= angle <= hi for lo, hi in self.self_occlusion_mask_rad
            ):
                continue

            distance = self.usable_scan_range(raw_distance, msg)
            if distance is None:
                continue

            if not self.angle_in_sector(angle, low, high):
                continue

            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            odom_x = x + (cos_yaw * local_x) - (sin_yaw * local_y)
            odom_y = y + (sin_yaw * local_x) + (cos_yaw * local_y)
            point = [odom_x, odom_y]
            points.append(point)
            ranges.append(distance)
            if distance <= self.baseline_capture_distance_m:
                close_points.append(point)

        if len(ranges) < self.min_side_rays:
            return None

        ranges_np = np.asarray(ranges, dtype=np.float32)
        points_np = np.asarray(points, dtype=np.float32)
        close_points_np = np.asarray(close_points, dtype=np.float32)
        representative = float(np.percentile(ranges_np, self.side_percentile))
        clear_ratio = float(np.count_nonzero(ranges_np >= self.clear_distance_m)) / float(
            ranges_np.size
        )
        close_count = int(np.count_nonzero(ranges_np <= self.occupied_distance_m))

        return SideProjection(
            points_odom=points_np,
            ranges=ranges_np,
            close_points_odom=close_points_np,
            representative_clearance_m=representative,
            clear_ratio=clear_ratio,
            close_count=close_count,
            forward_unit_odom=forward_unit,
            side_unit_odom=side_unit,
        )

    def project_all_rays_to_odom(self, msg: LaserScan) -> np.ndarray:
        """Project every valid LiDAR ray (full FOV) into the odom frame.

        Unlike project_side_scan this does NOT filter by the +/- 8 deg side
        window. Used for the baseline-anchored ROI check so the tracker keeps
        "seeing" the parked vehicle at its odom-frame position regardless of
        how the ego vehicle has rotated or translated.
        """
        if self.current_pose is None:
            return np.zeros((0, 2), dtype=np.float32)

        x, y, yaw = self.current_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points = []
        for index, raw_distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if any(lo <= angle <= hi for lo, hi in self.self_occlusion_mask_rad):
                continue
            distance = self.usable_scan_range(raw_distance, msg)
            if distance is None:
                continue
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            odom_x = x + (cos_yaw * local_x) - (sin_yaw * local_y)
            odom_y = y + (sin_yaw * local_x) + (cos_yaw * local_y)
            points.append([odom_x, odom_y])

        if not points:
            return np.zeros((0, 2), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def update_baseline_if_occupied(
        self,
        projection: SideProjection,
        all_points_odom: np.ndarray,
    ) -> None:
        if projection.close_points_odom.shape[0] < self.baseline_min_points:
            return

        candidate_center = np.mean(projection.close_points_odom, axis=0)
        if self.baseline_center_odom is None:
            self.baseline_center_odom = candidate_center
            self.baseline_forward_unit_odom = projection.forward_unit_odom
            self.baseline_side_unit_odom = projection.side_unit_odom
        else:
            alpha = 0.2
            self.baseline_center_odom = (
                (1.0 - alpha) * self.baseline_center_odom
                + alpha * candidate_center
            )
            self.baseline_forward_unit_odom = self.normalize_vector(
                (1.0 - alpha) * self.baseline_forward_unit_odom
                + alpha * projection.forward_unit_odom
            )
            self.baseline_side_unit_odom = self.normalize_vector(
                (1.0 - alpha) * self.baseline_side_unit_odom
                + alpha * projection.side_unit_odom
            )

        self.seen_occupied = True
        # Count baseline ROI hits using the full-FOV projection so the value is
        # comparable to the runtime count consumed by evaluate_clearance.
        roi_count = self.count_points_in_tracked_roi(all_points_odom)
        self.baseline_roi_count = max(
            self.baseline_roi_count,
            roi_count,
            int(projection.close_points_odom.shape[0]),
        )

    def evaluate_clearance(
        self, projection: SideProjection, current_roi_count: int
    ) -> None:
        side_open = (
            projection.representative_clearance_m >= self.clear_distance_m
            and projection.clear_ratio >= self.min_clear_ratio
        )

        if not self.seen_occupied:
            self.clear_since = None
            return

        if self.baseline_center_odom is None:
            raw_clear = self.allow_fallback_without_baseline and side_open
        elif self.allow_fallback_without_baseline:
            raw_clear = side_open
        else:
            clear_count_limit = max(
                self.clear_max_points,
                int(round(self.baseline_roi_count * self.clear_count_fraction)),
            )
            object_left = current_roi_count <= clear_count_limit
            raw_clear = object_left and side_open

        now = self.get_clock().now()
        if not raw_clear:
            self.clear_since = None
            return

        if self.clear_since is None:
            self.clear_since = now
            return

        if now - self.clear_since < Duration(seconds=self.clear_hold_s):
            return

        if (
            self.baseline_center_odom is not None
            and self.baseline_side_unit_odom is not None
        ):
            target_center = (
                self.baseline_center_odom
                + self.baseline_side_unit_odom * self.slot_center_offset_m
            )
            self.lock_target(
                target_center,
                self.baseline_side_unit_odom,
                "side-open after occupied detection",
            )
        else:
            self.lock_target(
                np.array(
                    [self.fallback_target_x, self.fallback_target_y],
                    dtype=np.float32,
                ),
                projection.side_unit_odom,
                "fallback side-open after occupied detection",
            )

    def lock_target(
        self, center_odom: np.ndarray, side_unit_odom: np.ndarray, reason: str
    ) -> None:
        if self.target_acquired:
            return

        target_yaw = self.target_yaw_rad
        if self.use_side_axis_yaw:
            target_yaw = math.atan2(float(side_unit_odom[1]), float(side_unit_odom[0]))

        msg = PoseStamped()
        msg.header.frame_id = self.odom_frame
        msg.pose.position.x = float(center_odom[0])
        msg.pose.position.y = float(center_odom[1])
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quaternion(target_yaw)

        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        self.target_pose = msg
        self.target_acquired = True
        self.target_pub.publish(msg)
        self.last_publish_time = now
        self.get_logger().info(
            "[TRACKER] target_slot acquired: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, "
            f"yaw={target_yaw:.3f} rad, reason={reason}, "
            f"baseline_count={self.baseline_roi_count}, "
            f"current_count={self.last_roi_count}"
        )

    def count_points_in_tracked_roi(self, points_odom: np.ndarray) -> int:
        if (
            self.baseline_center_odom is None
            or self.baseline_forward_unit_odom is None
            or self.baseline_side_unit_odom is None
            or points_odom.size == 0
        ):
            return 0

        rel = points_odom - self.baseline_center_odom
        forward = rel @ self.baseline_forward_unit_odom
        side = rel @ self.baseline_side_unit_odom
        in_roi = (
            np.abs(forward) <= self.track_box_length_m * 0.5
        ) & (
            np.abs(side) <= self.track_box_width_m * 0.5
        )
        return int(np.count_nonzero(in_roi))

    def publish_target_if_due(self) -> None:
        if self.target_pose is None:
            return

        now = self.get_clock().now()
        if now - self.last_publish_time < Duration(seconds=self.publish_period_s):
            return

        self.target_pose.header.stamp = now.to_msg()
        self.target_pub.publish(self.target_pose)
        self.last_publish_time = now

    def usable_scan_range(
        self, distance: float, msg: LaserScan
    ) -> Optional[float]:
        if math.isnan(distance):
            return None
        if math.isinf(distance):
            return self.max_range_m

        min_range = max(msg.range_min, self.min_range_m)
        max_range = self.max_range_m
        if msg.range_max > 0.0 and math.isfinite(msg.range_max):
            max_range = min(max_range, msg.range_max)

        if distance < min_range:
            return None
        return min(distance, max_range)

    @staticmethod
    def angle_in_sector(angle: float, low: float, high: float) -> bool:
        low_n = normalize_angle(low)
        high_n = normalize_angle(high)
        if low_n <= high_n:
            return low_n <= angle <= high_n
        return angle >= low_n or angle <= high_n

    @staticmethod
    def normalize_vector(vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-6:
            return vector
        return vector / norm

    def debug_log_loop(self) -> None:
        scan_received = self.latest_scan is not None
        odom_received = self.current_pose is not None
        scan_age_s = math.inf
        if self.last_scan_rx_time is not None:
            scan_age_s = (
                self.get_clock().now() - self.last_scan_rx_time
            ).nanoseconds / 1e9
        scan_alive = scan_received and scan_age_s <= STATUS_LOG_PERIOD_S * 2.0
        baseline = "yes" if self.baseline_center_odom is not None else "no"

        self.get_logger().info(
            "[Tracker DEBUG] "
            f"scan_received={scan_received}, "
            f"scan_alive={scan_alive}, "
            f"scan_age={scan_age_s:.2f}s, "
            f"odom_received={odom_received}, "
            f"projection_valid={self.last_projection_valid}, "
            f"side={self.last_side_clearance_m:.3f} m, "
            f"clear_ratio={self.last_clear_ratio * 100.0:.1f}%, "
            f"close={self.last_close_count}, "
            f"roi={self.last_roi_count}/{self.baseline_roi_count}, "
            f"baseline={baseline}, seen_occupied={self.seen_occupied}, "
            f"acquired={self.target_acquired}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
