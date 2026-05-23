#!/usr/bin/env python3
"""
mission_manager.py
══════════════════════════════════════════════════════════════
원격 주차 미션 마스터 FSM 노드

FSM 상태:
  IDLE              ← /start_remote_parking 서비스 대기
  LIMO1_EVADE       ← LIMO 1 후진 (차로 → B3 슬롯, 이중주차 해제)
  LIMO2_EXIT_INIT   ← LIMO 2 출차 경로 계산 + 시작
  LIMO2_EXITING     ← LIMO 2 출차 완료 감지 대기
  LIMO1_SCAN        ← LiDAR + 카메라로 A3 슬롯 비어있음 확인
  LIMO1_REPARK      ← LIMO 1 A3 슬롯 재주차 (3단계 FSM)
  FINISH            ← 완료 / 정지
  ABORT             ← 비상 정지

서비스:
  /start_remote_parking  (std_srvs/Trigger) : 미션 시작 버튼
  /reset_parking         (std_srvs/Trigger) : 미션 리셋

구독:
  /limo1/odom                       (nav_msgs/Odometry)
  /limo1/scan                       (sensor_msgs/LaserScan)
  /limo1/rear_camera/depth/image_raw (sensor_msgs/Image)
  /limo1/front_camera/depth/image_raw(sensor_msgs/Image)
  /limo2/odom                       (nav_msgs/Odometry)
  /limo1/target_slot                (geometry_msgs/PoseStamped)
  /remote_parking/reset             (std_msgs/Bool)

발행:
  /limo1/cmd_vel  (geometry_msgs/Twist)
  /limo2/cmd_vel  (geometry_msgs/Twist)
  /remote_parking/status (std_msgs/String) : 현재 FSM 상태
══════════════════════════════════════════════════════════════
"""

import math
import time
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


# ══════════════════════════════════════════════════════════════
# FSM 상태 정의
# ══════════════════════════════════════════════════════════════
class MissionState(Enum):
    IDLE            = auto()
    LIMO1_EVADE     = auto()   # LIMO 1 차로 따라 후진 (이중주차 해제)
    WAIT_FOR_SELECTION = auto() # 사용자 터미널 입력 대기
    LIMO2_EXIT_INIT = auto()   # 출차 시작
    LIMO2_EXITING   = auto()   # LIMO 2 출차 완료 대기
    LIMO1_SCAN      = auto()   # 슬롯 비어있음 확인
    LIMO1_REPARK    = auto()   # LIMO 1 A3 재주차
    FINISH          = auto()
    ABORT           = auto()



# ══════════════════════════════════════════════════════════════
# 유틸리티
# ══════════════════════════════════════════════════════════════
def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def pos_from_odom(odom: Odometry) -> Tuple[float, float]:
    p = odom.pose.pose.position
    return p.x, p.y


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def dist2d(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)


# ══════════════════════════════════════════════════════════════
# 메인 노드
# ══════════════════════════════════════════════════════════════
class RemoteParkingManager(Node):

    # ── 상수 (파라미터로 오버라이드 가능) ─────────────────────────
    CTRL_HZ          = 20.0     # 제어 루프 Hz
    ESTOP_DIST_M     = 0.18     # 비상 정지 장애물 거리 (m)

    # LIMO 1 후진
    EVADE_SPEED      = -0.12    # 후진 선속도 (m/s), 음수
    EVADE_YAW_KP     = 1.2      # 후진 중 직진 보정 P 게인
    LIMO1_MIN_TURN_R = 0.42     # 최소 회전 반경 (m) = wheelbase / tan(max_steer)

    # LIMO 2 출차 (Pure Pursuit)
    LIMO2_EXIT_SPEED   = 0.20    # m/s (전진 속도)
    LIMO2_WHEELBASE    = 0.24    # m
    LIMO2_LOOKAHEAD    = 0.25    # m (lookahead 거리)
    LIMO2_MAX_STEER    = 0.5236  # rad (최대 조향각 30°)
    LIMO2_ARRIVE_TOL   = 0.12   # m (도착 허용 오차)
    LIMO2_EXIT_TIMEOUT = 30.0   # s (안전 타임아웃)
    LIMO2_WP_SPACING   = 0.08   # m (웨이포인트 간격)

    # LIMO 1 재주차 (Hybrid A* + 세그먼트별 추종 + 최종 정렬)
    REPARK_FWD_SPEED  = 0.08    # m/s (전진)
    REPARK_BWD_SPEED  = -0.06   # m/s (후진)
    REPARK_LOOKAHEAD  = 0.15    # m
    REPARK_WHEELBASE  = 0.24    # m
    REPARK_MAX_STEER  = 0.5236  # rad (30°)
    REPARK_ARRIVE     = 0.04    # m (최종 위치 허용 오차)
    REPARK_YAW_TOL    = 0.052   # rad (~3°, 최종 yaw 허용 오차)
    REPARK_HOLD_S     = 0.8     # s (도착 확인 유지 시간)
    REPARK_TIMEOUT    = 60.0    # s (안전 타임아웃)
    # 최종 정렬 단계
    ALIGN_DIST_ENTER  = 0.12    # m (정렬 시작 거리)
    ALIGN_SPEED       = 0.03    # m/s (저속 정렬)
    ALIGN_YAW_KP      = 1.5     # yaw 보정 P 게인
    MAX_LINEAR        = 0.08    # m/s (호환용)
    # 슬롯 확인 타임아웃
    SCAN_TIMEOUT_S    = 10.0

    def __init__(self):
        super().__init__('remote_parking_manager')

        # ── 파라미터 선언 ────────────────────────────────────────
        self.declare_parameter('limo1_evade_x',  -0.5)
        self.declare_parameter('limo1_evade_y',   0.35)
        self.declare_parameter('limo2_exit_x',    3.0)
        self.declare_parameter('limo2_exit_y',    0.0)
        self.declare_parameter('repark_x',        0.225)
        self.declare_parameter('repark_y',        0.985)
        self.declare_parameter('repark_yaw',      1.5708)
        self.declare_parameter('max_linear_speed', 0.05)

        # ── 파라미터 로드 ────────────────────────────────────────
        self.evade_goal  = (
            self.get_parameter('limo1_evade_x').value,
            self.get_parameter('limo1_evade_y').value,
        )
        self.limo2_exit = (
            self.get_parameter('limo2_exit_x').value,
            self.get_parameter('limo2_exit_y').value,
        )
        self.repark_goal = (
            self.get_parameter('repark_x').value,
            self.get_parameter('repark_y').value,
        )
        self.repark_yaw_goal = self.get_parameter('repark_yaw').value

        # ── 상태 변수 ────────────────────────────────────────────
        self.state       = MissionState.IDLE
        # (unused, kept for compat)

        # LIMO 1
        self.limo1_odom: Optional[Odometry] = None
        self.limo1_scan: Optional[LaserScan] = None
        self.limo1_rear_depth: Optional[Image] = None
        self.limo1_front_depth: Optional[Image] = None
        self.front_obstacle_m: float = 999.0
        self.rear_obstacle_m:  float = 999.0

        # LIMO 2, A2, A4
        self.limo2_odom: Optional[Odometry] = None
        self.limoa2_odom: Optional[Odometry] = None
        self.limoa4_odom: Optional[Odometry] = None
        self.selected_exit_car = 'a3'
        self.limo2_scan: Optional[LaserScan] = None
        self.limo2_front_obstacle_m: float = 999.0
        self.limo2_rear_obstacle_m:  float = 999.0
        self.limo2_exit_done = False
        self._limo2_exit_start: Optional[float] = None
        self._limo2_waypoints: list = []

        # 슬롯 탐지
        self.target_slot: Optional[PoseStamped] = None
        self.scan_start_time: Optional[float] = None

        # 후진 yaw 고정
        self._evade_yaw: float = 0.0
        self._evade_blocked_since: Optional[float] = None

        # 재주차 (Hybrid A* path + 세그먼트별 추종)
        self._repark_path: list = []
        self._repark_segments: list = []
        self._repark_seg_idx: int = 0
        self._repark_wp_idx: int = 0
        self._repark_start_time: Optional[float] = None
        self.prec_ok_since: Optional[float] = None

        # ── QoS ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        # ── 구독 ─────────────────────────────────────────────────
        self.create_subscription(Odometry,   '/limo1/odom',  self._limo1_odom_cb,  sensor_qos)
        self.create_subscription(LaserScan,  '/limo1/scan',  self._limo1_scan_cb,  sensor_qos)
        self.create_subscription(Image, '/limo1/rear_camera/depth/image_raw',
                                 self._rear_depth_cb,  sensor_qos)
        self.create_subscription(Image, '/limo1/front_camera/depth/image_raw',
                                 self._front_depth_cb, sensor_qos)
        self.create_subscription(Odometry,  '/limo2/odom',  self._limo2_odom_cb,  sensor_qos)
        self.create_subscription(Odometry,  '/limo_a2/odom',  self._limoa2_odom_cb,  sensor_qos)
        self.create_subscription(Odometry,  '/limo_a4/odom',  self._limoa4_odom_cb,  sensor_qos)
        self.create_subscription(LaserScan, '/limo2/scan',  self._limo2_scan_cb, sensor_qos)
        self.create_subscription(Image, '/limo2/rear_camera/depth/image_raw',
                                 self._limo2_rear_depth_cb, sensor_qos)
        self.create_subscription(PoseStamped, '/limo1/target_slot', self._target_slot_cb, 1)
        self.create_subscription(Bool, '/remote_parking/reset', self._reset_cb, 1)
        self.create_subscription(String, '/select_exit_car', self._select_car_cb, 10)

        # ── 발행 ─────────────────────────────────────────────────
        self.limo1_cmd = self.create_publisher(Twist, '/limo1/cmd_vel', 1)
        self.limo2_cmd = self.create_publisher(Twist, '/limo2/cmd_vel', 1)
        self.limoa2_cmd = self.create_publisher(Twist, '/limo_a2/cmd_vel', 1)
        self.limoa4_cmd = self.create_publisher(Twist, '/limo_a4/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/remote_parking/status', 1)

        # ── 서비스 서버 ──────────────────────────────────────────
        self.start_srv  = self.create_service(Trigger, '/start_remote_parking', self._start_cb)
        self.reset_srv  = self.create_service(Trigger, '/reset_parking',        self._reset_srv_cb)
        self.create_service(Trigger, '/select_exit_car/a2', lambda req, res: self._select_car_srv(req, res, 'a2'))
        self.create_service(Trigger, '/select_exit_car/a3', lambda req, res: self._select_car_srv(req, res, 'a3'))
        self.create_service(Trigger, '/select_exit_car/a4', lambda req, res: self._select_car_srv(req, res, 'a4'))

        # ── 제어 루프 타이머 (20Hz) ──────────────────────────────
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)

        self.get_logger().info('🚗 Remote Parking Manager 시작 — IDLE 대기 중')
        self.get_logger().info('   /start_remote_parking 서비스 호출로 미션 시작')

    # ══════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════
    def _limo1_odom_cb(self, msg: Odometry):
        self.limo1_odom = msg

    def _limo1_scan_cb(self, msg: LaserScan):
        self.limo1_scan = msg
        self._update_obstacle_distances(msg)

    def _rear_depth_cb(self, msg: Image):
        self.limo1_rear_depth = msg
        self._update_rear_obstacle(msg)

    def _front_depth_cb(self, msg: Image):
        self.limo1_front_depth = msg

    def _limo2_odom_cb(self, msg: Odometry):
        self.limo2_odom = msg

    def _limoa2_odom_cb(self, msg: Odometry):
        self.limoa2_odom = msg

    def _limoa4_odom_cb(self, msg: Odometry):
        self.limoa4_odom = msg

    def _limo2_rear_depth_cb(self, msg: Image):
        try:
            if msg.encoding in ('32FC1',):
                data = np.frombuffer(msg.data, dtype=np.float32).copy()
                data = data[np.isfinite(data) & (data > 0.01)]
                if len(data) > 0:
                    self.limo2_rear_obstacle_m = float(np.percentile(data, 5))
                else:
                    self.limo2_rear_obstacle_m = 999.0
        except Exception:
            pass

    def _limo2_scan_cb(self, msg: LaserScan):
        self.limo2_scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(
            np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max),
            ranges, np.inf
        )
        self.limo2_front_obstacle_m = self._scan_min_in_range(msg, ranges, 0.0, 30.0)
        rear = self._scan_min_in_range(msg, ranges, 180.0, 30.0)
        if rear < 999.0:
            self.limo2_rear_obstacle_m = rear

    def _target_slot_cb(self, msg: PoseStamped):
        self.target_slot = msg

    def _reset_cb(self, msg: Bool):
        if msg.data:
            self._do_reset()

    # ── 서비스 핸들러 ─────────────────────────────────────────────
    def _start_cb(self, request, response):
        if self.state != MissionState.IDLE:
            self.get_logger().info(
                f'⚠️ 이전 미션 상태 {self.state.name} → 자동 리셋 후 재시작')
            self._do_reset()

        if self.limo1_odom is None or self.limo2_odom is None:
            response.success = False
            response.message = '오도메트리 미수신. 로봇 준비 상태를 확인하세요.'
            return response

        self._transition_to(MissionState.LIMO1_EVADE)
        response.success = True
        response.message = '원격 주차 미션 시작 — LIMO 1 후진 시작'
        self.get_logger().info('🚀 미션 시작!')
        return response

    def _reset_srv_cb(self, request, response):
        self._do_reset()
        response.success = True
        response.message = 'IDLE 상태로 리셋 완료'
        return response

    def _do_reset(self):
        self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
        self._publish_cmd(self.limo2_cmd, 0.0, 0.0)
        self._publish_cmd(self.limoa2_cmd, 0.0, 0.0)
        self._publish_cmd(self.limoa4_cmd, 0.0, 0.0)
        self.limo2_exit_done = False
        self.target_slot = None
        self.scan_start_time = None
        self.prec_ok_since = None
        self._repark_path = []
        self._repark_segments = []
        self._repark_seg_idx = 0
        self._repark_wp_idx = 0
        self._repark_start_time = None
        self._transition_to(MissionState.IDLE)
        self.get_logger().info('🔄 미션 리셋 → IDLE')

    # ══════════════════════════════════════════════════════════════
    # 메인 제어 루프 (20Hz)
    # ══════════════════════════════════════════════════════════════
    def _control_loop(self):
        self._publish_status()

        if self.state == MissionState.IDLE:
            return

        elif self.state == MissionState.LIMO1_EVADE:
            self._run_limo1_evade()

        elif self.state == MissionState.WAIT_FOR_SELECTION:
            pass # Thread handles input

        elif self.state == MissionState.LIMO2_EXIT_INIT:
            self._run_limo2_exit_init()

        elif self.state == MissionState.LIMO2_EXITING:
            self._run_limo2_exiting()

        elif self.state == MissionState.LIMO1_SCAN:
            self._run_limo1_scan()

        elif self.state == MissionState.LIMO1_REPARK:
            self._run_limo1_repark()

        elif self.state == MissionState.FINISH:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            self._publish_cmd(self.limo2_cmd, 0.0, 0.0)
            self._publish_cmd(self.limoa2_cmd, 0.0, 0.0)
            self._publish_cmd(self.limoa4_cmd, 0.0, 0.0)

        elif self.state == MissionState.ABORT:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            self._publish_cmd(self.limo2_cmd, 0.0, 0.0)
            self._publish_cmd(self.limoa2_cmd, 0.0, 0.0)
            self._publish_cmd(self.limoa4_cmd, 0.0, 0.0)

    # ══════════════════════════════════════════════════════════════
    # STATE: LIMO1_EVADE  (LIMO 1 차로 따라 후진, 이중주차 해제)
    # ══════════════════════════════════════════════════════════════
    def _run_limo1_evade(self):
        if self.limo1_odom is None:
            return

        cx, cy   = pos_from_odom(self.limo1_odom)
        cur_yaw  = yaw_from_odom(self.limo1_odom)
        gx, gy   = self.evade_goal

        # 후방 장애물 체크 (후방 카메라 + LiDAR)
        if self.rear_obstacle_m < self.ESTOP_DIST_M:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            now = time.monotonic()
            if self._evade_blocked_since is None:
                self._evade_blocked_since = now
                self.get_logger().warn(
                    f'⚠️ 후방 장애물 {self.rear_obstacle_m:.2f}m — 정지')
            elif now - self._evade_blocked_since > 1.5:
                self.get_logger().info(
                    f'✅ LIMO 1 후방 장애물로 후진 종료 ({cx:.3f}, {cy:.3f})')
                self._transition_to(MissionState.WAIT_FOR_SELECTION)
            return
        self._evade_blocked_since = None

        # 목표 도달 확인
        remaining = dist2d(cx, cy, gx, gy)
        if remaining < 0.04:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            self.get_logger().info(
                f'✅ LIMO 1 후진 완료 ({cx:.3f}, {cy:.3f})')
            self._transition_to(MissionState.WAIT_FOR_SELECTION)
            return

        linear = self.EVADE_SPEED
        yaw_err = normalize_angle(self._evade_yaw - cur_yaw)
        if abs(yaw_err) < 0.03:
            yaw_err = 0.0
        angular = float(np.clip(-self.EVADE_YAW_KP * yaw_err, -0.3, 0.3))

        self._publish_cmd(self.limo1_cmd, linear, angular)

    def _try_select_car(self, car: str) -> tuple:
        if self.state != MissionState.WAIT_FOR_SELECTION:
            return False, f'현재 상태({self.state.name})에서는 차량 선택 불가'
        if car not in ['a2', 'a3', 'a4']:
            return False, f'잘못된 입력: {car}. a2, a3, a4 중 하나를 선택하세요.'
        odom, _, _ = self._get_active_exit_car_by_name(car)
        if odom is None:
            return False, f'{car} 오도메트리 미수신. 로봇 스폰 상태를 확인하세요.'
        self.selected_exit_car = car
        self.get_logger().info(f"✅ 차량 선택 완료: {car}")
        self._transition_to(MissionState.LIMO2_EXIT_INIT)
        return True, f'{car} 출차 시작'

    def _get_active_exit_car_by_name(self, car: str):
        if car == 'a2':
            return self.limoa2_odom, self.limoa2_cmd, -0.225
        elif car == 'a4':
            return self.limoa4_odom, self.limoa4_cmd, 0.675
        else:
            return self.limo2_odom, self.limo2_cmd, 0.225

    def _select_car_cb(self, msg: String):
        car = msg.data.strip().lower()
        ok, reason = self._try_select_car(car)
        if not ok:
            self.get_logger().warn(f"❌ {reason}")

    def _select_car_srv(self, request, response, car: str):
        ok, reason = self._try_select_car(car)
        response.success = ok
        response.message = reason
        return response

    def _get_active_exit_car(self):
        return self._get_active_exit_car_by_name(self.selected_exit_car)

    # ══════════════════════════════════════════════════════════════
    # STATE: LIMO2_EXIT_INIT  (출차 시작)
    # ══════════════════════════════════════════════════════════════
    def _run_limo2_exit_init(self):
        """출차: 선택된 차량의 웨이포인트 생성 + Pure Pursuit 시작."""
        self.limo2_exit_done = False
        self._limo2_exit_start = self.get_clock().now().nanoseconds * 1e-9
        
        odom, cmd, init_x = self._get_active_exit_car()
        self.repark_goal = (init_x, self.repark_goal[1]) # 목표를 출차한 슬롯으로 동적 업데이트
        self._limo2_waypoints = self._generate_exit_waypoints(init_x)
        
        n = len(self._limo2_waypoints)
        self.get_logger().info(
            f'🚗 {self.selected_exit_car} 출차 시작 (Pure Pursuit, {n}개 웨이포인트)')
        self._transition_to(MissionState.LIMO2_EXITING)

    # ══════════════════════════════════════════════════════════════
    # STATE: LIMO2_EXITING  (Pure Pursuit 웨이포인트 추종)
    #
    # 미리 생성한 웨이포인트를 Pure Pursuit 알고리즘으로 추종.
    # cmd_vel 발행으로 ackermann 물리 엔진 구동 (실차 이식 가능).
    #
    # 경로: 슬롯 남쪽 직진 → sinusoidal 곡선(남→동) → 동쪽 직진
    # ══════════════════════════════════════════════════════════════
    def _run_limo2_exiting(self):
        odom, cmd, _ = self._get_active_exit_car()
        if odom is None:
            return

        cx, cy = pos_from_odom(odom)
        cur_yaw = yaw_from_odom(odom)
        waypoints = self._limo2_waypoints

        # 안전 타임아웃
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._limo2_exit_start if self._limo2_exit_start else 0.0
        if elapsed > self.LIMO2_EXIT_TIMEOUT:
            self._publish_cmd(cmd, 0.0, 0.0)
            self.limo2_exit_done = True
            self.get_logger().warn(
                f'⚠️ {self.selected_exit_car} 출차 타임아웃 ({self.LIMO2_EXIT_TIMEOUT}s)')
            self._transition_to(MissionState.LIMO1_SCAN)
            return

        # 진행 로그 (2초마다)
        if not hasattr(self, '_limo2_exit_log_t'):
            self._limo2_exit_log_t = 0.0
        if now - self._limo2_exit_log_t > 2.0:
            self._limo2_exit_log_t = now
            self.get_logger().info(
                f'  [EXITING] Pure Pursuit elapsed={elapsed:.1f}s '
                f'pos=({cx:.3f},{cy:.3f}) yaw={cur_yaw:.2f}')

        # 최종 웨이포인트 도달 확인
        if waypoints:
            final_wp = waypoints[-1]
            if dist2d(cx, cy, final_wp[0], final_wp[1]) < self.LIMO2_ARRIVE_TOL:
                self._publish_cmd(cmd, 0.0, 0.0)
                self.limo2_exit_done = True
                self.get_logger().info(
                    f'✅ {self.selected_exit_car} 출차 완료 pos=({cx:.3f},{cy:.3f})')
                self._transition_to(MissionState.LIMO1_SCAN)
                return

        # Lookahead 포인트 탐색
        lookahead_pt = self._find_limo2_lookahead(cx, cy)
        if lookahead_pt is None:
            self._publish_cmd(cmd, 0.0, 0.0)
            self.limo2_exit_done = True
            self.get_logger().info(
                f'✅ {self.selected_exit_car} 출차 완료 (경로 소진) pos=({cx:.3f},{cy:.3f})')
            self._transition_to(MissionState.LIMO1_SCAN)
            return

        # Pure Pursuit 조향 계산
        dx = lookahead_pt[0] - cx
        dy = lookahead_pt[1] - cy
        local_x = math.cos(cur_yaw) * dx + math.sin(cur_yaw) * dy
        local_y = -math.sin(cur_yaw) * dx + math.cos(cur_yaw) * dy

        ld = math.hypot(local_x, local_y)
        if ld < 1e-6:
            self._publish_cmd(cmd, self.LIMO2_EXIT_SPEED, 0.0)
            return

        curvature = 2.0 * local_y / (ld * ld)
        delta = math.atan(self.LIMO2_WHEELBASE * curvature)
        delta = float(np.clip(delta, -self.LIMO2_MAX_STEER, self.LIMO2_MAX_STEER))

        linear = self.LIMO2_EXIT_SPEED
        angular = linear * math.tan(delta) / self.LIMO2_WHEELBASE
        self._publish_cmd(cmd, linear, angular)

    def _generate_exit_waypoints(self, start_x):
        """출차 경로 웨이포인트 생성 (sinusoidal 곡선 프로파일).

        3단계 경로를 시뮬레이션하여 (x, y) 웨이포인트 리스트 반환:
          Phase 1: 슬롯 탈출 직진 (남쪽, 2.5s)
          Phase 2: sinusoidal 곡선 (남→동, 4.0s, 총 +π/2 회전)
          Phase 3: 동쪽 직진 (4.0s)
        """
        x, y, yaw = start_x, 0.985, -math.pi / 2.0
        dt_sim = 0.01
        speed = self.LIMO2_EXIT_SPEED
        phase1_t, phase2_t, phase3_t = 2.5, 4.0, 4.0
        total_t = phase1_t + phase2_t + phase3_t

        waypoints = [(x, y)]
        last_wp = (x, y)
        t = 0.0

        while t < total_t:
            if t < phase1_t:
                omega = 0.0
            elif t < phase1_t + phase2_t:
                tc = t - phase1_t
                T = phase2_t
                omega = (math.pi ** 2 / (4.0 * T)) * math.sin(math.pi * tc / T)
            else:
                omega = 0.0

            yaw += omega * dt_sim
            x += speed * math.cos(yaw) * dt_sim
            y += speed * math.sin(yaw) * dt_sim
            t += dt_sim

            if math.hypot(x - last_wp[0], y - last_wp[1]) >= self.LIMO2_WP_SPACING:
                waypoints.append((x, y))
                last_wp = (x, y)

        if math.hypot(x - last_wp[0], y - last_wp[1]) > 0.01:
            waypoints.append((x, y))

        return waypoints

    def _find_limo2_lookahead(self, cx: float, cy: float):
        """경로에서 lookahead 거리에 해당하는 포인트 탐색."""
        waypoints = self._limo2_waypoints
        if not waypoints:
            return None

        Ld = self.LIMO2_LOOKAHEAD

        nearest_idx = 0
        min_dist = float('inf')
        for i, (wx, wy) in enumerate(waypoints):
            d = dist2d(cx, cy, wx, wy)
            if d < min_dist:
                min_dist = d
                nearest_idx = i

        for i in range(nearest_idx, len(waypoints)):
            wx, wy = waypoints[i]
            if dist2d(cx, cy, wx, wy) >= Ld:
                return (wx, wy)

        last = waypoints[-1]
        if dist2d(cx, cy, last[0], last[1]) < self.LIMO2_ARRIVE_TOL:
            return None
        return last

    # ══════════════════════════════════════════════════════════════
    # STATE: LIMO1_SCAN  (A3 슬롯 비어있음 확인)
    # ══════════════════════════════════════════════════════════════
    def _run_limo1_scan(self):
        """
        LiDAR + 전방 Depth 카메라로 A3 슬롯이 비어있음을 확인.
        dynamic_tracker_node가 /limo1/target_slot 을 발행하면 완료.
        타임아웃 10초 내 미확인 시 fallback으로 재주차 진행.
        """
        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info('🔍 A3 슬롯 비어있음 스캔 중...')
            return

        # target_slot 수신 → 슬롯 확인 완료
        if self.target_slot is not None:
            self.get_logger().info(
                f'✅ 슬롯 감지 완료: '
                f'({self.target_slot.pose.position.x:.3f}, '
                f'{self.target_slot.pose.position.y:.3f})')
            self._transition_to(MissionState.LIMO1_REPARK)
            return

        # 타임아웃 → fallback 사용
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.scan_start_time
        if elapsed > self.SCAN_TIMEOUT_S:
            self.get_logger().warn(
                f'⚠️ 슬롯 스캔 타임아웃 ({self.SCAN_TIMEOUT_S}s) — '
                f'fallback 목표 ({self.repark_goal[0]}, {self.repark_goal[1]}) 사용')
            self._transition_to(MissionState.LIMO1_REPARK)

    # ══════════════════════════════════════════════════════════════
    # STATE: LIMO1_REPARK  (A3 슬롯 재주차 — Hybrid A* + 세그먼트별 추종)
    # ══════════════════════════════════════════════════════════════
    def _run_limo1_repark(self):
        if self.limo1_odom is None:
            return

        cx, cy  = pos_from_odom(self.limo1_odom)
        cur_yaw = yaw_from_odom(self.limo1_odom)
        gx, gy  = self.repark_goal
        gyaw    = self.repark_yaw_goal
        now_t   = self.get_clock().now().nanoseconds * 1e-9

        # ── 최초 진입: Hybrid A* 경로 계획 + 세그먼트 분리 ──
        if not self._repark_path:
            from remote_parking_manager.hybrid_astar_planner import plan_repark_path
            self.get_logger().info(
                f'  [REPARK] Hybrid A* 경로 계획 중... '
                f'start=({cx:.3f},{cy:.3f},{cur_yaw:.2f}) '
                f'goal=({gx:.3f},{gy:.3f},{gyaw:.2f})')
            t0 = time.monotonic()
            path = plan_repark_path(cx, cy, cur_yaw, gx, gy, gyaw)
            dt = time.monotonic() - t0
            if path is None or len(path) < 2:
                self.get_logger().error(
                    f'  [REPARK] 경로 계획 실패 ({dt:.2f}s) — ABORT')
                self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
                self._transition_to(MissionState.ABORT)
                return
            self._repark_path = path
            self._repark_segments = self._split_path_segments(path)
            self._repark_seg_idx = 0
            self._repark_wp_idx = 0
            self._repark_start_time = now_t
            self.prec_ok_since = None
            seg_info = [(s['dir'], len(s['pts'])) for s in self._repark_segments]
            self.get_logger().info(
                f'  [REPARK] 경로 계획 완료: {len(path)}개 포인트, '
                f'{len(self._repark_segments)}개 세그먼트, {dt:.2f}s')
            self.get_logger().info(
                f'  [REPARK] 세그먼트: {seg_info}')
            return

        # ── 타임아웃 ──
        if self._repark_start_time and now_t - self._repark_start_time > self.REPARK_TIMEOUT:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            self.get_logger().warn(f'  [REPARK] 타임아웃 ({self.REPARK_TIMEOUT}s)')
            self._transition_to(MissionState.ABORT)
            return

        # ── 진행 로그 (3초마다) ──
        if not hasattr(self, '_repark_log_t'):
            self._repark_log_t = 0.0
        if now_t - self._repark_log_t > 3.0:
            self._repark_log_t = now_t
            remaining = dist2d(cx, cy, gx, gy)
            elapsed = now_t - self._repark_start_time if self._repark_start_time else 0.0
            self.get_logger().info(
                f'  [REPARK] t={elapsed:.1f}s pos=({cx:.3f},{cy:.3f}) '
                f'yaw={cur_yaw:.2f} dist={remaining:.3f} '
                f'seg={self._repark_seg_idx}/{len(self._repark_segments)} '
                f'wp={self._repark_wp_idx}')

        # ── 도착 판정 (위치 + yaw) ──
        remaining = dist2d(cx, cy, gx, gy)
        yaw_err_final = abs(normalize_angle(gyaw - cur_yaw))
        if remaining < self.REPARK_ARRIVE and yaw_err_final < self.REPARK_YAW_TOL:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            if self.prec_ok_since is None:
                self.prec_ok_since = now_t
                self.get_logger().info(
                    f'  [REPARK] 슬롯 도착 ({cx:.3f},{cy:.3f}) '
                    f'yaw_err={math.degrees(yaw_err_final):.1f}°, 확인 중...')
            elif now_t - self.prec_ok_since >= self.REPARK_HOLD_S:
                self.get_logger().info('🏁 재주차 완료! A3 슬롯 정착 (Hybrid A*)')
                self._transition_to(MissionState.FINISH)
            return
        self.prec_ok_since = None

        # ── 최종 정렬 단계: 슬롯 근접 시 저속 + yaw 보정 ──
        if remaining < self.ALIGN_DIST_ENTER and self._repark_seg_idx >= len(self._repark_segments):
            yaw_err = normalize_angle(gyaw - cur_yaw)
            if abs(yaw_err) < 0.02:
                yaw_err = 0.0

            if remaining < self.REPARK_ARRIVE:
                speed = 0.0
            else:
                speed = self.ALIGN_SPEED

            angular = float(np.clip(self.ALIGN_YAW_KP * yaw_err, -0.3, 0.3))
            self._publish_cmd(self.limo1_cmd, speed, angular)
            return

        # ── 세그먼트별 경로 추종 ──
        if self._repark_seg_idx >= len(self._repark_segments):
            if remaining < self.ALIGN_DIST_ENTER:
                return
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            if remaining < 0.15 and yaw_err_final < 0.30:
                self.get_logger().info(
                    f'  [REPARK] 경로 소진, 정렬 단계 진입 '
                    f'(dist={remaining:.3f}, yaw_err={math.degrees(yaw_err_final):.1f}°)')
            else:
                self.get_logger().warn('  [REPARK] 세그먼트 소진 — 재계획')
                self._repark_path = []
            return

        seg = self._repark_segments[self._repark_seg_idx]
        seg_dir = seg['dir']
        seg_pts = seg['pts']

        # 현재 세그먼트 끝점 도달 → 다음 세그먼트로
        seg_end = seg_pts[-1]
        if dist2d(cx, cy, seg_end[0], seg_end[1]) < 0.06:
            self._publish_cmd(self.limo1_cmd, 0.0, 0.0)
            self._repark_seg_idx += 1
            self._repark_wp_idx = 0
            if self._repark_seg_idx < len(self._repark_segments):
                nd = self._repark_segments[self._repark_seg_idx]['dir']
                self.get_logger().info(
                    f'  [REPARK] 세그먼트 {self._repark_seg_idx} 시작 '
                    f'(dir={nd:+d})')
            return

        # 세그먼트 내 lookahead 포인트 찾기
        target = self._find_segment_lookahead(
            cx, cy, seg_pts, self._repark_wp_idx)
        if target is None:
            self._repark_seg_idx += 1
            self._repark_wp_idx = 0
            return

        target_pt, new_wp_idx = target
        self._repark_wp_idx = new_wp_idx

        # 슬롯 근접 시 감속
        if remaining < 0.20:
            fwd_speed = self.ALIGN_SPEED
            bwd_speed = -self.ALIGN_SPEED
        else:
            fwd_speed = self.REPARK_FWD_SPEED
            bwd_speed = self.REPARK_BWD_SPEED

        # 목표 방향 계산
        dx = target_pt[0] - cx
        dy = target_pt[1] - cy
        target_angle = math.atan2(dy, dx)

        if seg_dir > 0:
            yaw_err = normalize_angle(target_angle - cur_yaw)
            speed = fwd_speed
        else:
            yaw_err = normalize_angle(target_angle - cur_yaw - math.pi)
            speed = bwd_speed

        if abs(yaw_err) < 0.03:
            yaw_err = 0.0

        Kp = 2.0
        angular = float(np.clip(Kp * yaw_err, -0.8, 0.8))
        max_ang = abs(speed) / self.LIMO1_MIN_TURN_R
        angular = float(np.clip(angular, -max_ang, max_ang))

        self._publish_cmd(self.limo1_cmd, speed, angular)

    def _split_path_segments(self, path):
        """경로를 동일 방향 세그먼트로 분리."""
        segments = []
        cur_dir = path[0][3]
        cur_pts = [path[0]]
        for p in path[1:]:
            if p[3] == cur_dir:
                cur_pts.append(p)
            else:
                segments.append({'dir': cur_dir, 'pts': cur_pts})
                cur_dir = p[3]
                cur_pts = [p]
        if cur_pts:
            segments.append({'dir': cur_dir, 'pts': cur_pts})
        return segments

    def _find_segment_lookahead(self, cx, cy, seg_pts, wp_idx):
        """세그먼트 내에서 lookahead 포인트를 찾음."""
        Ld = self.REPARK_LOOKAHEAD
        n = len(seg_pts)

        # 가장 가까운 웨이포인트 (현재 인덱스 근처)
        best_idx = min(wp_idx, n - 1)
        best_dist = float('inf')
        lo = max(0, wp_idx - 3)
        hi = min(n, wp_idx + 20)
        for i in range(lo, hi):
            d = dist2d(cx, cy, seg_pts[i][0], seg_pts[i][1])
            if d < best_dist:
                best_dist = d
                best_idx = i

        # lookahead 거리 이상인 첫 포인트
        for i in range(best_idx, n):
            if dist2d(cx, cy, seg_pts[i][0], seg_pts[i][1]) >= Ld:
                return (seg_pts[i][0], seg_pts[i][1]), best_idx

        # 세그먼트 끝 반환
        last = seg_pts[-1]
        return (last[0], last[1]), best_idx

    # ══════════════════════════════════════════════════════════════
    # 장애물 거리 업데이트
    # ══════════════════════════════════════════════════════════════
    def _scan_min_in_range(self, scan: LaserScan, ranges: np.ndarray,
                           center_deg: float, half_deg: float) -> float:
        n = len(ranges)
        if n == 0:
            return 999.0
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        lo = math.radians(center_deg - half_deg)
        hi = math.radians(center_deg + half_deg)
        mask = (angles >= lo) & (angles <= hi)
        if not np.any(mask):
            return 999.0
        return float(np.min(ranges[mask]))

    def _update_obstacle_distances(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(
            np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max),
            ranges, np.inf
        )
        self.front_obstacle_m = self._scan_min_in_range(scan, ranges, 0.0, 30.0)
        rear = self._scan_min_in_range(scan, ranges, 180.0, 30.0)
        if rear < 999.0:
            self.rear_obstacle_m = rear

    def _update_rear_obstacle(self, img: Image):
        try:
            if img.encoding in ('32FC1',):
                data = np.frombuffer(img.data, dtype=np.float32).copy()
                data = data[np.isfinite(data) & (data > 0.01)]
                if len(data) > 0:
                    self.rear_obstacle_m = float(np.percentile(data, 5))
                else:
                    self.rear_obstacle_m = 999.0
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════
    # 유틸리티
    # ══════════════════════════════════════════════════════════════
    def _publish_cmd(self, pub, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = self.state.name
        self.status_pub.publish(msg)

    def _transition_to(self, new_state: MissionState):
        old = self.state.name
        self.state = new_state
        if new_state == MissionState.LIMO1_EVADE and self.limo1_odom is not None:
            self._evade_yaw = yaw_from_odom(self.limo1_odom)
            self._evade_blocked_since = None
        if new_state == MissionState.LIMO1_REPARK:
            self._repark_path = []
            self._repark_segments = []
            self._repark_seg_idx = 0
            self._repark_wp_idx = 0
            self._repark_start_time = None
            self.prec_ok_since = None
        self.get_logger().info(f'🔀 {old} → {new_state.name}')
        if new_state == MissionState.WAIT_FOR_SELECTION:
            self.get_logger().info("=" * 60)
            self.get_logger().info("어느 차량을 출차하시겠습니까? (a2, a3, a4)")
            self.get_logger().info("새 터미널을 열고 다음 명령어를 실행하세요:")
            self.get_logger().info("  bash scripts/select_car.sh a3")
            self.get_logger().info("=" * 60)


# ══════════════════════════════════════════════════════════════
# 진입점
# ══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = RemoteParkingManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Remote Parking Manager 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
