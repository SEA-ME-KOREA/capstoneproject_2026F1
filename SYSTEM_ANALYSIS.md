# Remote Parking System 분석 문서

## 목차

1. [전체 프로그램 흐름](#1-전체-프로그램-흐름)
2. [LIMO 1 주요 알고리즘](#2-limo-1-주요-알고리즘)
3. [LIMO 2 주요 알고리즘](#3-limo-2-주요-알고리즘)

---

## 1. 전체 프로그램 흐름

### 1.1 시스템 개요

이중주차된 LIMO 1이 뒤로 빠지고, A3 슬롯에 주차된 LIMO 2가 출차한 뒤, LIMO 1이 비어진 A3 슬롯에 재주차하는 원격 주차 시나리오이다.

**주차장 배치 (축척 1/5.6):**

```
              North Wall (y=1.585)
 ┌──────────────────────────────────────────────┐
 │  [A1]        [A2]      ★A3★       [A4]       │
 │  (-0.675)   (-0.225)  (0.225)    (0.675)     │
 │              y = 0.985 (슬롯 중심)            │
 │                                               │
 │─────────── 차로 (y ≈ 0.35) ─────────────────│
 │                                               │
 │  [B1]        [B2]       [B3]       [B4]       │
 │              y = -0.985                       │
 └──────────────────────────────────────────────┘
              South Wall (y=-1.585)
```

- A1, A2, A4, B1~B4: 정적 차량 7대 (Gazebo 월드에 고정 배치)
- A3: LIMO 2가 주차된 슬롯 (미션 목표 슬롯)

### 1.2 패키지 구조

| 패키지 | 타입 | 역할 |
|--------|------|------|
| `remote_parking_world` | CMake | Gazebo 월드, 로봇 스폰, Launch 파일 |
| `limo_car` | CMake | LIMO 로봇 URDF/Xacro 모델 정의 |
| `remote_parking_manager` | Python | 마스터 FSM (`mission_manager`) + 경로 계획기 (`hybrid_astar_planner`) |
| `my_valet_parking` | Python | LiDAR 슬롯 탐지 (`dynamic_tracker_node`) 등 제어 노드 |

### 1.3 실행 순서 및 노드 기동 타이밍

```
[Terminal 1] ros2 launch remote_parking_world world_server.launch.py gui:=true
    → gzserver 시작 (정적 월드 + 7대 차량 로드)
    → gzclient 시작 (GUI)

[Terminal 2] ros2 launch remote_parking_world spawn_robots.launch.py
    → T=0s:  robot_state_publisher × 2 (limo1, limo2 네임스페이스)
    → T=3s:  Gazebo에 LIMO 1, LIMO 2 엔티티 스폰
    → T=5s:  dynamic_tracker_node 시작 (LiDAR 슬롯 탐지)
    → T=6s:  mission_manager 시작 (마스터 FSM, IDLE 상태 대기)
    → T=0s:  rviz2 시작 (시각화)

[Terminal 3] ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
    → mission_manager의 _start_cb() 호출
    → 양쪽 로봇 오도메트리 수신 확인 후 FSM을 IDLE → LIMO1_EVADE로 전이
```

### 1.4 초기 로봇 배치

| 로봇 | 위치 (x, y, z) | 방향 (yaw) | 상태 |
|------|----------------|-----------|------|
| LIMO 1 | (0.225, 0.35, 0.145) | 0 rad (+X 방향) | 차로에서 A3 앞 이중주차 |
| LIMO 2 | (0.225, 0.985, 0.145) | -π/2 rad (-Y 방향) | A3 슬롯 정상 주차 |

### 1.5 FSM (Finite State Machine) 상태 전이

```
IDLE
  │  /start_remote_parking 서비스 호출
  ▼
LIMO1_EVADE ─────────────────────────────────────────────────────────
  │  LIMO 1이 차로를 따라 후진하여 LIMO 2 출차 경로 확보
  │  종료조건: 목표 위치 (-0.5, 0.35) 도달 OR 후방 장애물 1.5초 이상 감지
  ▼
LIMO2_EXIT_INIT ─────────────────────────────────────────────────────
  │  LIMO 2의 출차 경로(sinusoidal 웨이포인트) 계산
  │  즉시 다음 상태로 전이
  ▼
LIMO2_EXITING ───────────────────────────────────────────────────────
  │  LIMO 2가 Pure Pursuit으로 웨이포인트 추종하며 출차
  │  종료조건: 최종 웨이포인트 도달(허용 오차 0.12m) OR 타임아웃 30초
  ▼
LIMO1_SCAN ──────────────────────────────────────────────────────────
  │  dynamic_tracker_node가 A3 슬롯 비어있음을 확인
  │  종료조건: /limo1/target_slot 수신 OR 타임아웃 10초 (fallback 좌표 사용)
  ▼
LIMO1_REPARK ────────────────────────────────────────────────────────
  │  Hybrid A* 경로 계획 → 세그먼트별 Pure Pursuit 추종 → 최종 정렬
  │  종료조건: 목표 위치 오차 <4cm + yaw 오차 <3° 를 0.8초 유지 OR 타임아웃 60초
  ▼
FINISH ──────────────────────────────────────────────────────────────
  │  양쪽 로봇 정지 (cmd_vel = 0)
  ▼
(IDLE 대기 — 재호출 가능)

※ ABORT: 경로 계획 실패, 타임아웃 등 비상 시 양쪽 로봇 정지
```

### 1.6 센서 사용 흐름

#### 런타임 노드 구성 및 토픽 흐름

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo                                  │
│  /limo1/odom ──┐   /limo1/scan ──┐   /limo1/rear_camera/... ──┐│
│  /limo2/odom ──┤   /limo2/scan ──┤   /limo1/front_camera/...──┤│
│                │                 │   /limo2/rear_camera/... ──┤│
└────────────────┼─────────────────┼────────────────────────────┤┘
                 │                 │                             │
                 ▼                 ▼                             ▼
  ┌──────────────────────────────────────────────────────────────┐
  │             mission_manager (remote_parking_manager)         │
  │                                                              │
  │  구독:                                                       │
  │    /limo1/odom         → 위치·yaw 추출                       │
  │    /limo1/scan         → 전방·후방 장애물 거리 (±30° 섹터)    │
  │    /limo1/rear_camera  → 후방 깊이 영상 → 5th 백분위 거리    │
  │    /limo1/front_camera → 전방 깊이 영상                      │
  │    /limo2/odom         → LIMO 2 위치·yaw 추출                │
  │    /limo2/scan         → LIMO 2 전방·후방 장애물 거리         │
  │    /limo2/rear_camera  → LIMO 2 후방 깊이 영상               │
  │    /limo1/target_slot  → 슬롯 위치 (dynamic_tracker에서 수신) │
  │                                                              │
  │  발행:                                                       │
  │    /limo1/cmd_vel      → LIMO 1 속도 명령                    │
  │    /limo2/cmd_vel      → LIMO 2 속도 명령                    │
  │    /remote_parking/status → 현재 FSM 상태 문자열             │
  └──────────────────────────────────────────────────────────────┘
                 ▲
                 │ /limo1/target_slot
  ┌──────────────────────────────────────────────────────────────┐
  │            dynamic_tracker_node (my_valet_parking)           │
  │                                                              │
  │  구독:                                                       │
  │    /limo1/scan  → +90°±8° 사이드 스캔 + 전체 FOV 투영        │
  │    /limo1/odom  → ego 위치 보정 (odom 프레임 투영)           │
  │                                                              │
  │  발행:                                                       │
  │    /limo1/target_slot → 비어진 슬롯 위치 (PoseStamped)       │
  └──────────────────────────────────────────────────────────────┘
```

#### 센서별 용도 정리

| 센서 | 토픽 | 사용 노드 | 용도 |
|------|------|----------|------|
| Odometry (LIMO 1) | `/limo1/odom` | mission_manager, dynamic_tracker | 위치·yaw 추적, odom 프레임 좌표 변환 |
| Odometry (LIMO 2) | `/limo2/odom` | mission_manager | 출차 중 위치·yaw 추적 |
| 2D LiDAR (LIMO 1) | `/limo1/scan` | mission_manager, dynamic_tracker | 전방/후방 장애물 감지, 사이드 슬롯 점유 판정 |
| 2D LiDAR (LIMO 2) | `/limo2/scan` | mission_manager | 전방/후방 장애물 감지 |
| 후방 깊이 카메라 (LIMO 1) | `/limo1/rear_camera/depth/image_raw` | mission_manager | 후진 시 후방 장애물 거리 (32FC1 → 5th 백분위) |
| 전방 깊이 카메라 (LIMO 1) | `/limo1/front_camera/depth/image_raw` | mission_manager | 전방 장애물 보조 감지 |
| 후방 깊이 카메라 (LIMO 2) | `/limo2/rear_camera/depth/image_raw` | mission_manager | 출차 시 후방 장애물 거리 |

#### 장애물 감지 방식

- **LiDAR 섹터 분석**: `_scan_min_in_range()` — 지정 각도 범위(중심 ± 반각)의 최솟값 추출
  - 전방: 0° ± 30° → `front_obstacle_m`
  - 후방: 180° ± 30° → `rear_obstacle_m`
- **깊이 카메라**: 32FC1 인코딩 → `np.percentile(data, 5)` (하위 5% 거리 = 가장 가까운 장애물)
- **비상 정지 기준**: 장애물 거리 < 0.18m (`ESTOP_DIST_M`)

---

## 2. LIMO 1 주요 알고리즘

LIMO 1은 3단계 미션을 수행한다: **후진 회피** → **슬롯 탐지** → **재주차**.

### 2.1 LIMO1_EVADE: 직선 후진 (이중주차 해제)

**목적**: LIMO 2가 출차할 수 있도록 차로를 따라 후진하여 A3 슬롯 앞 공간 확보.

**알고리즘**: P 제어 기반 직선 후진

```
입력: 현재 위치 (cx, cy), 현재 yaw, 목표 위치 (-0.5, 0.35)
출력: cmd_vel (linear.x, angular.z)

1. 후방 장애물 확인
   - rear_obstacle_m < 0.18m → 정지
   - 1.5초 이상 연속 정지 → 후진 종료 (LIMO2_EXIT_INIT으로 전이)

2. 목표 도달 확인
   - dist(현재, 목표) < 0.04m → 후진 종료

3. 속도 명령 생성
   - linear = -0.12 m/s (고정 후진 속도)
   - yaw_err = 시작 yaw - 현재 yaw
   - |yaw_err| < 0.03 → 보정 불필요 (데드존)
   - angular = clip(-1.2 × yaw_err, -0.3, 0.3)   ← P 게인 = 1.2
```

**핵심 포인트**: 시작 시점의 yaw를 기준값으로 잡아 후진 중 차체가 틀어지지 않도록 P 제어로 보정한다.

### 2.2 LIMO1_SCAN: LiDAR 기반 슬롯 탐지

**목적**: LIMO 2가 떠난 A3 슬롯이 실제로 비었는지 확인하고, 재주차 목표 좌표를 결정한다.

**실행 노드**: `dynamic_tracker_node` (별도 노드, mission_manager는 결과만 수신)

#### 2.2.1 사이드 스캔 투영 (Side Scan Projection)

```
1. LiDAR에서 +90° ± 8° 범위의 레이 추출 (로봇 우측)
2. 각 레이를 odom 프레임으로 변환:
   p_odom = [ego_x, ego_y] + R(ego_yaw) × [range×cos(θ), range×sin(θ)]
3. 통계 산출:
   - representative_clearance = 70th 백분위 거리
   - clear_ratio = (거리 ≥ 0.65m인 레이 수) / (전체 레이 수)
   - close_count = 거리 ≤ 0.45m인 레이 수
```

#### 2.2.2 베이스라인 앵커링 (Baseline Anchoring)

```
1. 가까운 포인트(≤ 0.75m)가 3개 이상이면 → 점유 물체 감지
2. 해당 포인트의 평균 위치를 odom 프레임에 앵커링 (baseline_center_odom)
3. EMA(α=0.2)로 위치를 점진 업데이트
4. 앵커 주변에 추적 박스(0.35m × 0.30m) 설정
```

#### 2.2.3 클리어런스 판정 (Clearance Evaluation)

```
1. 전체 FOV LiDAR를 odom 프레임에 투영 (사이드 윈도우 아님)
2. 추적 박스 내 포인트 수(roi_count) 집계
3. 비어있음 조건:
   - roi_count ≤ max(2, baseline_roi_count × 0.25)  ← 물체 사라짐
   - representative_clearance ≥ 0.65m               ← 사이드 개방
   - clear_ratio ≥ 0.25                              ← 25% 이상 레이가 멀리 감지
4. 위 조건이 0.70초 연속 유지 → 슬롯 비어있음 확정

5. 목표 좌표 생성:
   target = baseline_center + side_unit × 0.30m (슬롯 중심 오프셋)
```

**Fallback**: 10초 내 `target_slot`을 수신하지 못하면 하드코딩 좌표 (0.225, 0.985) 사용.

### 2.3 LIMO1_REPARK: Hybrid A* 경로 계획 + 세그먼트별 추종

**목적**: 현재 위치에서 A3 슬롯 (0.225, 0.985, yaw=π/2)까지 전진·후진을 조합하여 주차.

#### 2.3.1 점유 격자 지도 (Occupancy Grid)

```
범위: x ∈ [-1.1, 1.2], y ∈ [-0.6, 1.7]
해상도: 0.02m/셀
장애물:
  - A1(-0.675, 0.985), A2(-0.225, 0.985), A4(0.675, 0.985) → 정적 차량
  - 북쪽 벽 (y=1.585), 서쪽 벽 (x=-0.95)
  - 각 차량 크기: 0.40m × 0.26m (마진 포함)
```

#### 2.3.2 Hybrid A* 탐색

일반 A*는 격자 기반이라 차량의 비홀로노믹(non-holonomic) 제약을 반영하지 못한다. Hybrid A*는 격자 인덱스로 중복을 제거하면서 실제 차량 운동학에 맞는 연속 상태를 탐색한다.

```
상태: (x, y, yaw)
격자 해상도: xy=0.03m, yaw=5°
조향각: [-30°, -15°, 0°, +15°, +30°] (5단계)
방향: 전진(+1), 후진(-1)
스텝 크기: 0.06m/확장

확장 1회:
  nx = x + step × dir × cos(yaw + β/2)
  ny = y + step × dir × sin(yaw + β/2)
  nyaw = yaw + β
  여기서 β = step × dir × tan(steer) / wheelbase

비용 함수:
  기본 비용 = step (0.06)
  후진 페널티 = ×2.5
  조향 페널티 = ×1.1
  방향 전환 페널티 = +0.3

휴리스틱:
  h = dist(현재, 목표) + 0.5 × min_turn_radius × |yaw_err|

충돌 검사:
  차량 풋프린트 (0.31m × 0.19m) + 마진 (0.02m) 전체를 격자 위에 투영

RS 접속 시도:
  매 5회 확장마다 현재 노드에서 목표까지 Reeds-Shepp 곡선 연결 시도
  충돌 없으면 즉시 경로 확정 → 탐색 조기 종료
```

#### 2.3.3 Reeds-Shepp 곡선

전진·후진·회전을 조합하여 두 배치(pose) 사이의 최단 경로를 생성하는 해석적 방법이다.

```
3가지 기본 공식: LSL, LSR, LRL
  L = 좌회전, R = 우회전, S = 직진

4가지 변환 (각 공식에 적용):
  원본:         (x, y, φ)   → 전진
  시간 반전:    (-x, y, -φ) → 후진
  반사:         (x, -y, -φ) → L↔R 교체
  시간반전+반사: (-x,-y, φ) → L↔R 교체 + 후진

= 3 × 4 = 12가지 경로 후보 중 최단 + 충돌 없는 경로 선택
```

#### 2.3.4 세그먼트별 경로 추종

Hybrid A*가 생성한 경로는 전진/후진 구간이 혼합되어 있다. 이를 동일 방향 세그먼트로 분리하여 순차 추종한다.

```
경로 = [(x, y, yaw, dir), ...] → 세그먼트 분리 (dir 변경점 기준)

각 세그먼트 추종 (20Hz):
  1. 현재 위치에서 가장 가까운 웨이포인트 탐색
  2. lookahead 거리(0.15m) 이상인 첫 포인트를 목표로 설정
  3. 목표 방향 계산:
     - 전진: yaw_err = atan2(dy, dx) - cur_yaw
     - 후진: yaw_err = atan2(dy, dx) - cur_yaw - π
  4. 속도 명령:
     - 전진: 0.08 m/s (슬롯 근접 시 0.03 m/s)
     - 후진: -0.06 m/s (슬롯 근접 시 -0.03 m/s)
     - angular = clip(2.0 × yaw_err, -max_ang, max_ang)
     - max_ang = |speed| / min_turn_radius (0.42m)
  5. 세그먼트 끝점 도달(0.06m 이내) → 다음 세그먼트로

세그먼트 전체 소진 후 → 최종 정렬 단계 진입
```

#### 2.3.5 최종 정렬 (Final Alignment)

```
진입 조건: 목표까지 거리 < 0.12m (ALIGN_DIST_ENTER)

1. 저속 전진: 0.03 m/s (ALIGN_SPEED)
2. yaw 보정: angular = clip(1.5 × yaw_err, -0.3, 0.3)
3. 데드존: |yaw_err| < 0.02 → 보정 안 함

도착 판정:
  - 위치 오차 < 0.04m (REPARK_ARRIVE)
  - yaw 오차 < 0.052 rad (~3°, REPARK_YAW_TOL)
  - 위 조건을 0.8초 연속 유지 → FINISH
```

### 2.4 LIMO 1 알고리즘 요약 다이어그램

```
LIMO1_EVADE                    LIMO1_SCAN                 LIMO1_REPARK
┌─────────────┐      ┌──────────────────────────┐    ┌──────────────────────────┐
│ P제어 직선   │      │ dynamic_tracker_node     │    │ 1. Occupancy Grid 구축   │
│ 후진        │      │                          │    │ 2. Hybrid A* 탐색        │
│             │      │ LiDAR →odom 투영         │    │    (RS 곡선 접속 시도)    │
│ 후방 장애물  │ ──→  │ 베이스라인 앵커링         │ ──→│ 3. 세그먼트 분리          │
│ 감지 (LiDAR │      │ 추적 박스 ROI 집계        │    │ 4. 구간별 Pure Pursuit    │
│ + 깊이 카메라)│      │ 0.70s 유지 → 슬롯 확정   │    │ 5. 최종 정렬 (P제어)     │
│             │      │                          │    │ 6. 0.8s 유지 → 완료      │
│ 속도: -0.12 │      │ → /limo1/target_slot     │    │                          │
│ m/s         │      │   발행                   │    │ 속도: ±0.08 (→0.03) m/s  │
└─────────────┘      └──────────────────────────┘    └──────────────────────────┘
```

---

## 3. LIMO 2 주요 알고리즘

LIMO 2는 단일 미션을 수행한다: A3 슬롯에서 **출차**.

### 3.1 출차 경로 생성 (Sinusoidal Waypoint Generation)

**목적**: A3 슬롯(-Y 방향)에서 시작하여 남쪽으로 빠진 뒤 동쪽(+X)으로 빠져나가는 부드러운 출차 경로를 생성한다.

```
시작 배치: (0.225, 0.985, yaw=-π/2)
속도: 0.20 m/s (일정)
시뮬레이션 dt: 0.01s
웨이포인트 간격: 0.08m

Phase 1 (0~2.5s): 직진 — 슬롯 탈출
  ω = 0
  → 남쪽(-Y)으로 0.5m 이동

Phase 2 (2.5~6.5s): Sinusoidal 곡선 — 방향 전환
  tc = t - 2.5
  ω(tc) = (π² / 4T) × sin(π × tc / T),  T = 4.0s
  → 총 회전량 = π/2 (90°, 남→동 방향 전환)
  → 가속도 연속인 부드러운 S자 회전

Phase 3 (6.5~10.5s): 직진 — 동쪽으로 탈출
  ω = 0
  → 동쪽(+X)으로 0.8m 이동

각 스텝:
  yaw += ω × dt
  x += speed × cos(yaw) × dt
  y += speed × sin(yaw) × dt
  0.08m 이상 이동마다 웨이포인트 기록
```

**Sinusoidal 프로파일을 사용하는 이유**: 일반적인 일정 각속도 회전은 곡선 시작/끝에서 각가속도가 불연속이다. Sinusoidal 프로파일(`sin(π·t/T)` 형태)은 시작과 끝에서 ω=0이 되어 각가속도가 연속적이며, Ackermann 조향의 물리적 한계에 맞는 부드러운 전환을 보장한다.

### 3.2 Pure Pursuit 경로 추종

**목적**: 생성된 웨이포인트 리스트를 추종하여 실제 차량을 이동시킨다.

```
파라미터:
  Wheelbase    = 0.24m
  Lookahead    = 0.25m
  Max Steering = 30° (0.5236 rad)
  도착 허용 오차 = 0.12m

매 제어 주기 (20Hz):
  1. 현재 위치에서 가장 가까운 웨이포인트 탐색
  2. 가장 가까운 점부터 순방향으로 순회하며
     거리 ≥ Lookahead(0.25m)인 첫 포인트를 lookahead 포인트로 설정
  3. lookahead 포인트를 로봇 로컬 좌표로 변환:
     dx = target_x - cx
     dy = target_y - cy
     local_x =  cos(yaw) × dx + sin(yaw) × dy
     local_y = -sin(yaw) × dx + cos(yaw) × dy
  4. 곡률 계산:
     ld = sqrt(local_x² + local_y²)
     curvature = 2 × local_y / ld²
  5. 조향각 계산:
     δ = atan(wheelbase × curvature)
     δ = clip(δ, -30°, +30°)
  6. 속도 명령:
     linear  = 0.20 m/s
     angular = linear × tan(δ) / wheelbase
  7. 종료 판정:
     - 최종 웨이포인트까지 거리 < 0.12m → 출차 완료
     - 경과 시간 > 30s → 타임아웃 종료
```

### 3.3 LIMO 2 출차 흐름 요약

```
LIMO2_EXIT_INIT                          LIMO2_EXITING
┌────────────────────────────┐     ┌────────────────────────────────┐
│ sinusoidal 웨이포인트 생성  │     │ Pure Pursuit 추종 (20Hz)       │
│                            │     │                                │
│ Phase 1: 직진 남쪽 (2.5s)  │     │ 1. nearest waypoint 탐색       │
│ Phase 2: S곡선 (4.0s)      │ ──→ │ 2. lookahead point (≥0.25m)    │
│ Phase 3: 직진 동쪽 (4.0s)  │     │ 3. 곡률 → 조향각 → cmd_vel    │
│                            │     │ 4. 도착 or 타임아웃 → 종료     │
│ ≈26개 웨이포인트 생성       │     │                                │
│ (0.08m 간격)               │     │ 속도: 0.20 m/s                 │
└────────────────────────────┘     └────────────────────────────────┘
```

---

## 부록: 주요 파라미터 일람

### 차량 물리 사양 (공통)

| 파라미터 | 값 | 단위 |
|---------|-----|------|
| 질량 | 4.34 | kg |
| 차체 크기 | 0.31 × 0.19 × 0.12 | m (L×W×H) |
| 바퀴 반경 | 0.045 | m |
| 휠베이스 | 0.24 | m |
| 최소 회전 반경 | 0.42 | m |
| 최대 조향각 | 30° | deg |

### mission_manager 주요 상수

| 상수 | 값 | 용도 |
|-----|-----|------|
| `CTRL_HZ` | 20 Hz | 제어 루프 주기 |
| `ESTOP_DIST_M` | 0.18 m | 비상 정지 장애물 거리 |
| `EVADE_SPEED` | -0.12 m/s | LIMO 1 후진 속도 |
| `EVADE_YAW_KP` | 1.2 | 후진 yaw P 게인 |
| `LIMO2_EXIT_SPEED` | 0.20 m/s | LIMO 2 출차 속도 |
| `LIMO2_LOOKAHEAD` | 0.25 m | 출차 Pure Pursuit lookahead |
| `LIMO2_ARRIVE_TOL` | 0.12 m | 출차 도착 허용 오차 |
| `REPARK_FWD_SPEED` | 0.08 m/s | 재주차 전진 속도 |
| `REPARK_BWD_SPEED` | -0.06 m/s | 재주차 후진 속도 |
| `REPARK_LOOKAHEAD` | 0.15 m | 재주차 lookahead |
| `REPARK_ARRIVE` | 0.04 m | 재주차 도착 허용 오차 |
| `REPARK_YAW_TOL` | 0.052 rad (~3°) | 재주차 yaw 허용 오차 |
| `REPARK_HOLD_S` | 0.8 s | 도착 확인 유지 시간 |
| `SCAN_TIMEOUT_S` | 10.0 s | 슬롯 스캔 타임아웃 |
| `ALIGN_SPEED` | 0.03 m/s | 최종 정렬 저속 |
| `ALIGN_YAW_KP` | 1.5 | 최종 정렬 yaw P 게인 |

### dynamic_tracker_node 주요 파라미터

| 파라미터 | 값 | 용도 |
|---------|-----|------|
| `side_scan_center_deg` | 90° | 사이드 스캔 중심 (우측) |
| `side_scan_half_deg` | 8° | 사이드 스캔 반각 |
| `occupied_distance_m` | 0.45 m | 점유 판정 거리 |
| `clear_distance_m` | 0.65 m | 개방 판정 거리 |
| `baseline_capture_distance_m` | 0.75 m | 베이스라인 캡처 거리 |
| `clear_hold_s` | 0.70 s | 개방 상태 유지 시간 |
| `slot_center_offset_m` | 0.30 m | 슬롯 중심 오프셋 |
| `track_box_length_m` | 0.35 m | 추적 박스 전후 길이 |
| `track_box_width_m` | 0.30 m | 추적 박스 좌우 폭 |
| `side_percentile` | 70% | 대표 거리 백분위 |
| `min_clear_ratio` | 0.25 | 최소 개방 비율 |

### Hybrid A* 플래너 파라미터

| 파라미터 | 값 | 용도 |
|---------|-----|------|
| XY 해상도 | 0.03 m | 격자 인덱싱 |
| Yaw 해상도 | 5° | 격자 인덱싱 |
| 스텝 크기 | 0.06 m | 노드 확장 거리 |
| 조향각 | [-30°, -15°, 0°, +15°, +30°] | 확장 방향 |
| RS 접속 시도 간격 | 5회 | 매 5회 확장마다 |
| 후진 비용 배수 | 2.5× | 전진 대비 |
| 조향 비용 배수 | 1.1× | 직진 대비 |
| 방향 전환 비용 | +0.3 | 전진↔후진 전환 |
| 충돌 마진 | 0.02 m | 차체 주변 여유 |
| 최대 반복 | 80,000회 | 탐색 상한 |
