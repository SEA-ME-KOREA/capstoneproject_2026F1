# Remote Parking Workspace 1 - 시스템 분석 문서

---

## 목차

1. [전체 시스템 흐름](#1-전체-시스템-흐름)
2. [LIMO1 (사용자 차량) 핵심 로직](#2-limo1-사용자-차량-핵심-로직)
3. [LIMO2 및 상대 차량 핵심 로직](#3-limo2-및-상대-차량-핵심-로직)
4. [요약](#4-요약)

---

## 1. 전체 시스템 흐름

### 1.1 시스템 개요

본 시스템은 **ROS2 + Gazebo** 기반의 **원격 주차 시뮬레이션 시스템**이다. 이중주차(double parking)된 사용자 차량(LIMO1)이 후진하여 길을 비우고, 사용자가 선택한 상대 차량(A2/A3/A4 중 하나)이 자율 출차한 뒤, LIMO1이 빈 슬롯에 자동으로 재주차하는 전 과정을 자율적으로 수행한다.

### 1.2 패키지 구성

| 패키지 | 역할 |
|--------|------|
| `remote_parking_manager` | 미션 마스터 FSM 노드 + Hybrid A* 경로 계획기 |
| `my_valet_parking` | LIMO 개별 제어 노드 모음 (슬롯 추적, 독립 출차, 키보드 텔레옵 등) |
| `remote_parking_world` | Gazebo 월드 파일, URDF/XACRO, launch 파일, 유틸 스크립트 |
| `limo_car` | LIMO 차량 URDF 모델 및 Gazebo 플러그인 정의 |

### 1.3 주차장 배치 및 로봇 배치

```
                   북쪽 벽 (y=1.585)
   ┌───────────────────────────────────────────────────────────────┐
   │ [A-1] [A0] [A1]  [A2★] [★A3★] [A4★]  [A5]  [A6]           │ ← 주차 슬롯 (y=0.985)
   │-1.575-1.125-0.675-0.225 0.225  0.675  1.125  1.575          │
   │                                                               │
   │             [LIMO1]                                           │ ← 이중주차 위치
   │             x=0.225, y=0.35, yaw=0                           │    (A3 앞 차로)
   │                                                               │
   │─── 차로 에지 y=+0.535 ──── 차로 (center y≈0) ──── y=-0.535 ──│
   │                                                               │
   │ [B-1] [B0] [B1]  [B2]   [B3]   [B4]   [B5]  [B6]           │ ← 주차 슬롯 (y=-0.985)
   └───────────────────────────────────────────────────────────────┘
                   남쪽 벽 (y=-1.585)
      서쪽 벽 (x=-1.90)                              출구 → +X (동쪽 개방)
```

**차량 배치:**

| 구분 | 슬롯 | 상태 |
|------|------|------|
| 정적 차량 5대 | Row A: A-1(-1.575), A0(-1.125), A1(-0.675), A5(1.125), A6(1.575) | Gazebo 월드에 고정 |
| 정적 차량 8대 | Row B: B-1(-1.575), B0(-1.125), B1(-0.675), B2(-0.225), B3(0.225), B4(0.675), B5(1.125), B6(1.575) | Gazebo 월드에 고정 |
| **LIMO1** | 차로 (0.225, 0.35) yaw=0 | 이중주차 — A3 앞을 막고 있음 |
| **LIMO2** | A3 (0.225, 0.985) yaw=-π/2 | 동적 스폰 |
| **LIMO A2** | A2 (-0.225, 0.985) yaw=-π/2 | 동적 스폰 |
| **LIMO A4** | A4 (0.675, 0.985) yaw=-π/2 | 동적 스폰 |

★ A2, A3, A4: 사용자 선택에 따라 출차 가능한 차량

### 1.4 사용 센서 총괄

| 센서 | 토픽 | 용도 |
|------|------|------|
| 2D LiDAR | `/limo1/scan`, `/limo2/scan` | 장애물 감지, 슬롯 점유 판단, E-stop |
| Odometry | `/limo1/odom`, `/limo2/odom`, `/limo_a2/odom`, `/limo_a4/odom` | 위치/자세 추적, 경로 추종 |
| 후방 Depth 카메라 | `/limo1/rear_camera/depth/image_raw` | 후진 시 후방 장애물 거리 측정 |
| 전방 Depth 카메라 | `/limo1/front_camera/depth/image_raw` | 전방 장애물 감지 |
| 후방 Depth 카메라 (LIMO2) | `/limo2/rear_camera/depth/image_raw` | 출차 시 후방 장애물 거리 |

### 1.5 전체 미션 FSM (mission_manager.py)

미션 마스터 FSM은 `RemoteParkingManager` 노드가 20Hz 제어 루프로 관리한다.

```
IDLE
  │  /start_remote_parking 서비스 호출
  ▼
LIMO1_EVADE ──────────────────────────────────────────┐
  │  LIMO1이 후진하여 이중주차 해제                      │
  │  (후방 장애물 감지 또는 목표 (-0.9, 0.35) 도달)      │
  ▼                                                    │
WAIT_FOR_SELECTION                                     │
  │  사용자가 출차할 차량 선택 (a2/a3/a4)                │
  │  bash scripts/select_car.sh a3                     │
  │  → /select_exit_car/a3 서비스 호출                  │
  ▼                                                    │
LIMO2_EXIT_INIT                                        │
  │  선택된 차량의 출차 웨이포인트 생성                   │
  │  repark_goal을 출차 슬롯 좌표로 동적 업데이트         │
  ▼                                                    │
LIMO2_EXITING                                          │
  │  Pure Pursuit으로 출차 경로 추종                     │
  │  (도착/타임아웃 30s 시 종료)                         │
  ▼                                                    │
LIMO1_SCAN                                             │
  │  LiDAR로 슬롯 비어있음 확인                          │
  │  (dynamic_tracker_node가 target_slot 발행 대기)     │
  │  (타임아웃 10s → fallback 좌표 사용)                 │
  ▼                                                    │
LIMO1_REPARK                                           │
  │  Hybrid A* 경로 계획 → 세그먼트별 추종               │
  │  → 최종 정렬 → 도착 판정                             │
  ▼                                                    │
FINISH ←──────────────────────────────────────── ABORT │
  모든 차량 정지                       경로 실패/타임아웃 │
```

### 1.6 노드 간 통신 구조

```
[mission_manager] ──pub──→ /limo1/cmd_vel ──→ LIMO1 구동
                  ──pub──→ /limo2/cmd_vel ──→ LIMO2 구동
                  ──pub──→ /limo_a2/cmd_vel ──→ LIMO A2 구동
                  ──pub──→ /limo_a4/cmd_vel ──→ LIMO A4 구동
                  ──pub──→ /remote_parking/status
                  ←─sub── /limo1/odom, /limo2/odom, /limo_a2/odom, /limo_a4/odom
                  ←─sub── /limo1/scan, /limo2/scan
                  ←─sub── /limo1/rear_camera/depth/image_raw
                  ←─sub── /limo1/front_camera/depth/image_raw
                  ←─sub── /limo2/rear_camera/depth/image_raw
                  ←─sub── /limo1/target_slot (from dynamic_tracker_node)
                  ←─sub── /select_exit_car (토픽, 보조)
                  ←─sub── /remote_parking/reset
                  ←─srv── /start_remote_parking (미션 시작)
                  ←─srv── /reset_parking (미션 리셋)
                  ←─srv── /select_exit_car/a2, /a3, /a4 (차량 선택)

[dynamic_tracker_node] ←─sub── /limo1/scan, /limo1/odom
                       ──pub──→ /limo1/target_slot
```

### 1.7 Launch 순서

1. **world_server.launch.py** (1회 실행) - Gazebo 물리 엔진 + 정적 월드 (13대 정적 차량 포함) 로드
2. **spawn_robots.launch.py** (테스트마다 실행):
   - 0s: robot_state_publisher × 4 + joint_state_publisher × 4 시작
   - 3s: Gazebo 로봇 스폰 × 4 (LIMO1, LIMO2, LIMO_A2, LIMO_A4)
   - 5s: dynamic_tracker_node 시작
   - 6s: mission_manager (마스터 FSM) 시작
   - RViz 시각화 시작

---

## 2. LIMO1 (사용자 차량) 핵심 로직

LIMO1은 이중주차를 해제하고, 상대 차량이 출차한 뒤 빈 슬롯에 재주차하는 전 과정을 수행하는 **주 행위자(primary actor)**이다. mission_manager가 직접 `/limo1/cmd_vel`을 발행하여 제어한다.

### 2.1 LIMO1_EVADE: 후진 이중주차 해제

#### 목적
슬롯 앞을 막고 있는 LIMO1을 후진시켜 상대 차량의 출차 경로를 확보한다.

#### 사용 센서
- **Odometry** (`/limo1/odom`): 현재 위치/yaw 추적
- **후방 Depth 카메라** (`/limo1/rear_camera/depth/image_raw`): 후방 장애물 거리 측정
- **2D LiDAR** (`/limo1/scan`): 후방 180도 영역 장애물 거리 보조 측정

#### 알고리즘 작동 방식

```
1. 진입 시 현재 yaw를 _evade_yaw로 저장 (직진 기준각)

2. 매 제어 주기 (50ms):
   a. 후방 장애물 거리 체크:
      - rear_obstacle_m < 0.18m (ESTOP_DIST_M)이면 즉시 정지
      - 1.5초 이상 연속 차단 시 → WAIT_FOR_SELECTION으로 전환
      
   b. 목표 도달 체크:
      - 현재 위치와 evade_goal(-0.9, 0.35) 사이 거리 < 0.04m이면 완료
      → WAIT_FOR_SELECTION으로 전환
      
   c. 후진 제어:
      - linear = -0.12 m/s (EVADE_SPEED, 후진)
      - yaw_err = _evade_yaw - 현재 yaw (직진 유지용)
      - |yaw_err| < 0.03 → 데드존 (보정 안 함)
      - angular = clip(-1.2 * yaw_err, -0.3, 0.3)  (P 제어)
```

#### 후방 장애물 거리 측정 방법

두 가지 센서를 융합하여 `rear_obstacle_m` 값을 결정한다:

1. **LiDAR 기반** (`_update_obstacle_distances`):
   - LiDAR 스캔 데이터에서 180도 ± 30도 (후방) 영역의 최소 거리 추출
   - 유효 범위 필터링: `isfinite & (range > range_min) & (range < range_max)`

2. **Depth 카메라 기반** (`_update_rear_obstacle`):
   - 32FC1 인코딩 Depth 이미지에서 유효 픽셀 추출 (finite & > 0.01m)
   - **5번째 백분위수** (하위 5%)를 대표 거리로 사용 → 가장 가까운 장애물 감지
   - Depth 카메라 값이 LiDAR 값을 덮어씀 (더 정밀)

### 2.2 WAIT_FOR_SELECTION: 출차 차량 선택 대기

#### 목적
사용자가 어떤 차량(A2/A3/A4)을 출차할지 결정하도록 대기한다.

#### 작동 방식
- `/select_exit_car/a2`, `/select_exit_car/a3`, `/select_exit_car/a4` 서비스 호출로 차량 선택
- 별도 터미널에서 `bash scripts/select_car.sh a3` 실행
  - 스크립트 내부: `ros2 service call /select_exit_car/a3 std_srvs/srv/Trigger '{}'`
- 보조: `/select_exit_car` 토픽으로도 문자열 메시지 수신 가능 (a2, a3, a4 중 하나)
- 유효한 선택이 들어오면 → `LIMO2_EXIT_INIT`으로 전환
- 선택된 차량에 따라 odom/cmd_vel 퍼블리셔 동적 바인딩 + `repark_goal` 동적 업데이트

| 선택 | Odometry 토픽 | cmd_vel 토픽 | 초기 X 좌표 | 재주차 목표 |
|------|-------------|------------|-----------|-----------|
| a2 | `/limo_a2/odom` | `/limo_a2/cmd_vel` | -0.225 | (-0.225, 0.985) |
| a3 | `/limo2/odom` | `/limo2/cmd_vel` | 0.225 | (0.225, 0.985) |
| a4 | `/limo_a4/odom` | `/limo_a4/cmd_vel` | 0.675 | (0.675, 0.985) |

### 2.3 LIMO1_SCAN: 빈 슬롯 탐지

#### 목적
상대 차량이 출차한 뒤, 해당 슬롯이 실제로 비어있는지 LiDAR로 확인한다.

#### 사용 센서
- **2D LiDAR** (`/limo1/scan`) → `dynamic_tracker_node`가 처리

#### 알고리즘 작동 방식

`dynamic_tracker_node`가 독립 노드로 실행되며, 다음 과정을 수행한다:

```
1. Side Scan Projection (측면 스캔 투영):
   - LiDAR 스캔에서 +90도 ± 8도 (좌측면) 영역 추출
   - 차체 자기간섭 구간 (±130~170도) 마스킹
   - 각 레이를 odom 프레임으로 변환:
     p_odom = [x,y] + R(yaw) * [range*cos(θ), range*sin(θ)]

2. Baseline 구축 (점유 상태 기록):
   - 측면 0.75m 이내의 포인트가 3개 이상이면 "차량 존재"로 판단
   - 포인트 클러스터의 중심을 baseline_center_odom으로 저장
   - EMA (α=0.2) 필터로 baseline 위치를 부드럽게 업데이트
   - odom 프레임의 ROI(0.35m x 0.30m) 안에 있는 포인트 수 기록

3. Clearance 판정 (빈 슬롯 확인):
   - 전체 FOV 레이를 odom 프레임으로 투영 (사이드 윈도우 아님)
     ※ EVADE 중 ego 이동에 의한 오감지 방지를 위해 전체 FOV 사용
   - ROI 안 포인트 수가 baseline 대비 25% 이하로 감소
   - 측면 대표 거리(70번째 백분위수) >= 0.65m
   - clear_ratio (0.65m 이상 레이 비율) >= 25%
   - 위 조건이 0.7초 이상 유지되면 → target_slot 발행

4. Target Slot 발행:
   - baseline_center + side_unit * 0.30m 위치를 빈 슬롯 중심으로 산출
   - PoseStamped 메시지로 /limo1/target_slot에 0.1초 간격 발행
```

mission_manager는 `/limo1/target_slot` 수신 시 → `LIMO1_REPARK` 전환. 10초 타임아웃 시 `repark_goal` 파라미터 좌표를 fallback으로 사용 (선택된 슬롯에 따라 동적으로 결정됨).

### 2.4 LIMO1_REPARK: Hybrid A* 자동 재주차

#### 목적
LIMO1을 비워진 슬롯까지 자율 주행시켜 정밀하게 주차한다. **가장 복잡한 단계**이다.

#### 사용 센서
- **Odometry** (`/limo1/odom`): 실시간 위치/yaw 추적

#### 알고리즘 상세

##### (A) Hybrid A* 경로 계획 (`hybrid_astar_planner.py`)

```
입력: start(x,y,yaw), goal(x,y,yaw)
출력: [(x, y, yaw, direction), ...] 경로 포인트 리스트

1. Occupancy Grid 구축:
   - 범위: x[-1.5, 1.2], y[-0.6, 1.7], 해상도 0.02m
   - 장애물 후보 슬롯: A0(-1.125), A1(-0.675), A2(-0.225), A3(0.225), A4(0.675)
     → 출차한 슬롯(goal_x)은 장애물에서 제외
   - 북벽(y=1.585), 서벽(x=-1.90)을 장애물로 등록
   - 차량 크기: 0.40m x 0.26m (마진 포함)

2. Hybrid A* 탐색:
   - 상태 공간: (x, y, yaw) — 연속 좌표 + 이산화된 그리드 인덱스
   - xy 해상도: 0.03m, yaw 해상도: 5도
   - 스텝 크기: 0.06m
   - 조향각 후보: [-30, -15, 0, 15, 30]도
   - 방향: 전진(+1), 후진(-1) 모두 탐색
   
3. 비용 함수:
   - 기본 비용: step 크기 (0.06m)
   - 후진 패널티: x2.5 (후진 선호 억제)
   - 조향 패널티: x1.1 (직진 선호)
   - 방향 전환 패널티: +0.3 (잦은 전후진 전환 억제)
   
4. 휴리스틱:
   - h = euclidean_dist + 0.5 * min_turn_radius * |yaw_diff|

5. Reeds-Shepp 곡선 가지치기:
   - 매 5 노드마다 현재 위치에서 goal까지 RS 곡선 시도
   - 충돌 없는 RS 경로 발견 시 → 즉시 경로 확정 (탐색 종료)
   - 3가지 기본 공식(LSL, LSR, LRL) × 4 변환 = 12가지 RS 경로 유형
   
6. 충돌 검사:
   - 차량 footprint(0.31m x 0.19m + 마진 0.02m) 전체를 그리드에서 확인
   - footprint 내 어떤 셀이라도 점유되어 있으면 충돌로 판정
```

##### (B) 세그먼트별 경로 추종

```
1. 경로를 동일 방향(전진/후진) 세그먼트로 분리
   예: [전진 구간 → 후진 구간 → 전진 구간]

2. 각 세그먼트 내 Pure Pursuit 스타일 추종:
   - 현재 위치에서 가장 가까운 웨이포인트 탐색 (±3~20 인덱스 범위)
   - lookahead 거리(0.15m) 이상인 첫 번째 포인트를 목표로 설정
   
3. 조향 계산:
   - target_angle = atan2(dy, dx)
   - 전진: yaw_err = target_angle - 현재 yaw
   - 후진: yaw_err = target_angle - 현재 yaw - π
   - angular = clip(2.0 * yaw_err, -0.8, 0.8)
   - 최소 회전 반경 제한: |angular| <= |speed| / 0.42m
   
4. 속도:
   - 전진: 0.08 m/s, 후진: -0.06 m/s
   - 슬롯 근접(0.20m 이내): 0.03 m/s로 감속

5. 세그먼트 끝점 도달(0.06m 이내) → 정지 → 다음 세그먼트로 전환
```

##### (C) 최종 정렬

```
1. 슬롯 근접(0.12m 이내) + 모든 세그먼트 소진 시 정렬 단계 진입

2. 저속 정렬 제어:
   - 속도: 0.03 m/s (슬롯까지 남은 거리가 0.04m 이하면 0)
   - yaw 보정: angular = clip(1.5 * yaw_err, -0.3, 0.3)
   - 데드존: |yaw_err| < 0.02 → 보정 안 함

3. 도착 판정:
   - 위치 오차 < 0.04m AND yaw 오차 < 3도
   - 위 조건이 0.8초 이상 연속 유지 → FINISH로 전환
   
4. 안전 장치:
   - 60초 타임아웃 → ABORT
   - 경로 소진 시 재계획 시도
```

### 2.5 LIMO1 로직 전체 순서도

```
[미션 시작]
    │
    ▼
(1) LIMO1_EVADE: 후진 이중주차 해제
    센서: Odometry + 후방 Depth 카메라 + LiDAR(후방)
    알고리즘: P 제어 직진 후진, 후방 장애물 E-stop
    종료조건: 목표(-0.9, 0.35) 도달 OR 1.5초 이상 후방 차단
    │
    ▼
(2) WAIT_FOR_SELECTION: 출차 차량 선택
    입력: /select_exit_car/a2|a3|a4 서비스 호출
    │
    ▼
(3) LIMO2_EXIT_INIT + LIMO2_EXITING: 상대 차량 출차 (아래 3장 참조)
    │
    ▼
(4) LIMO1_SCAN: 빈 슬롯 탐지
    센서: LiDAR (dynamic_tracker_node 경유)
    알고리즘: Side Scan → Odom 투영 → ROI 점유 비교 → Clear 판정
    │
    ▼
(5) LIMO1_REPARK: 자동 재주차
    센서: Odometry
    알고리즘: Hybrid A* 경로 계획 → 세그먼트별 추종 → 최종 정렬
    │
    ▼
[FINISH]
```

---

## 3. LIMO2 및 상대 차량 핵심 로직

### 3.1 출차 차량 선택 메커니즘

사용자가 `select_car.sh` 스크립트(서비스 호출 방식)로 a2/a3/a4 중 하나를 선택하면, mission_manager가 해당 차량의 odom과 cmd_vel 퍼블리셔를 동적으로 바인딩한다.

| 선택 | Odometry 토픽 | cmd_vel 토픽 | 초기 X 좌표 |
|------|-------------|------------|-----------|
| a2 | `/limo_a2/odom` | `/limo_a2/cmd_vel` | -0.225 |
| a3 | `/limo2/odom` | `/limo2/cmd_vel` | 0.225 |
| a4 | `/limo_a4/odom` | `/limo_a4/cmd_vel` | 0.675 |

선택 결과에 따라 `repark_goal`도 해당 슬롯 좌표로 동적 업데이트된다.

### 3.2 LIMO2_EXIT_INIT: 출차 경로 생성

#### 알고리즘 - Sinusoidal 곡선 웨이포인트 생성

출차 경로는 3단계 시뮬레이션으로 미리 생성한다:

```
초기 상태: (start_x, 0.985, yaw=-π/2) → 슬롯 안에서 남쪽을 향함
  start_x = 선택된 차량의 초기 X 좌표 (-0.225 / 0.225 / 0.675)

Phase 1: 슬롯 탈출 직진 (2.5초)
  - 직선 남쪽 이동, omega = 0
  - 슬롯을 빠져나와 차로 영역에 진입

Phase 2: Sinusoidal 곡선 회전 (4.0초)
  - omega(t) = (π²/4T) * sin(πt/T)   (T = 4.0)
  - 누적 회전: +π/2 (남쪽→동쪽으로 부드럽게 방향 전환)
  - Sinusoidal 프로파일: 시작/끝에서 각속도 0, 중간에서 최대
  - 급격한 조향 변화 없이 부드러운 곡선 생성

Phase 3: 동쪽 직진 (4.0초)
  - 직선 동쪽 이동, omega = 0
  - 주차장 출구 방향으로 이탈

시뮬레이션 파라미터:
  - dt = 0.01초 (정밀 이산화)
  - speed = 0.20 m/s
  - 웨이포인트 간격: 0.08m 이상일 때만 저장
```

#### 생성된 경로 형태

```
  슬롯
  │
  │  Phase 1 (직진 남쪽)
  │
  │
   ╲
    ╲  Phase 2 (sinusoidal 곡선, -π/2 → 0)
     ╲
      ────────────── Phase 3 (직진 동쪽) ──→ 출구
```

### 3.3 LIMO2_EXITING: Pure Pursuit 경로 추종

#### 사용 센서
- **Odometry** (선택된 차량의 odom): 현재 위치/yaw 추적

#### 알고리즘 상세

```
매 제어 주기 (50ms):

1. 안전 타임아웃 체크:
   - 30초 초과 시 강제 종료 → LIMO1_SCAN 전환

2. 최종 웨이포인트 도달 체크:
   - 마지막 웨이포인트까지 거리 < 0.12m → 출차 완료

3. Lookahead 포인트 탐색:
   a. 전체 웨이포인트에서 현재 위치에 가장 가까운 인덱스 탐색
   b. 해당 인덱스부터 순방향으로, 현재 위치에서 0.25m 이상 거리인 첫 포인트 선택
   c. 없으면 마지막 웨이포인트 사용

4. Pure Pursuit 조향 계산:
   a. 목표점을 로컬 좌표계로 변환:
      local_x = cos(yaw)*dx + sin(yaw)*dy
      local_y = -sin(yaw)*dx + cos(yaw)*dy
      
   b. 곡률 계산:
      curvature = 2 * local_y / (ld²)
      
   c. 조향각 계산:
      delta = atan(wheelbase * curvature)
      delta = clip(delta, -30°, +30°)
      
   d. cmd_vel 변환:
      linear  = 0.20 m/s
      angular = linear * tan(delta) / wheelbase

5. 선택된 차량의 cmd_vel 토픽으로 Twist 발행
```

#### Pure Pursuit 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| LIMO2_EXIT_SPEED | 0.20 m/s | 전진 속도 |
| LIMO2_WHEELBASE | 0.24 m | 축간 거리 |
| LIMO2_LOOKAHEAD | 0.25 m | Lookahead 거리 |
| LIMO2_MAX_STEER | 30도 | 최대 조향각 |
| LIMO2_ARRIVE_TOL | 0.12 m | 도착 허용 오차 |
| LIMO2_WP_SPACING | 0.08 m | 웨이포인트 간격 |

### 3.4 독립 실행 가능한 LIMO2 출차 노드 (limo2_exit_handler.py)

mission_manager와 별도로, LIMO2 전용 독립 출차 노드도 존재한다. 이 노드는 자체 FSM으로 동작하며 LiDAR 기반 자율 판단을 수행한다.

#### FSM 상태

```
WAIT (10초 대기)
  │
  ▼
EXIT_SLOT (슬롯 탈출)
  │  yaw 정렬 → 전진 → 통로 도달
  ▼
DECIDE_FREE (빈 공간 방향 탐색)
  │  360도 LiDAR 스캔 → 슬라이딩 윈도우(±30도)로 최적 방향 탐색
  │  방향별 중앙값(median) 거리 기준 최대 개방 방향 선택
  ▼
TURN_TO_FREE (빈 공간 방향으로 회전)
  │  Ackermann 전방 호(arc) 회전 (linear 필수)
  ▼
DRIVE_FREE (빈 공간으로 직진 3m)
  │  장애물 0.6m 이내 감지 시 → DECIDE_FREE로 리다이렉트
  ▼
FINISH
```

#### 핵심 알고리즘: 빈 공간 탐색 (DECIDE_FREE)

```
1. 전체 LiDAR 스캔에서 유효 레이 (angle_deg, range) 수집

2. 후보 각도 -180도 ~ +180도를 5도 간격으로 순회:
   각 후보에 대해 ±30도 윈도우 안의 레이 거리 중앙값(median) 계산

3. 중앙값이 가장 큰 방향 = 가장 넓게 열린 방향
   (동점일 경우 정면에 가까운 방향 선호)

4. 선택된 방향을 world 좌표 yaw로 변환하여 free_target_yaw 설정
```

#### 사용 센서
- **Odometry** (`/limo2/odom`): 위치/yaw
- **2D LiDAR** (`/limo2/scan`): 전방 장애물 E-stop, 빈 공간 방향 탐색

### 3.5 보조 노드 (car_exit_controller.py)

Gazebo의 `SetEntityState` 서비스를 이용하여 `moving_passenger_car` 엔티티를 직접 이동시키는 노드이다. 실제 물리 시뮬레이션 없이 키네마틱하게 출차를 재현한다.

```
BACK_OUT: 슬롯에서 통로까지 직선 후진 (y: -0.75 → 0.0)
TURN_TO_MINUS_X: 반경 0.45m 사분원 호(quarter-circle) 회전
EXIT_STRAIGHT: -X 방향 직진 (x: → -4.20)
FINISH: 정지
```

### 3.6 키보드 텔레옵 노드 (limo2_arrow_teleop.py)

LIMO2를 방향키로 수동 제어할 수 있는 텔레옵 노드이다.

- 토픽: `/limo2/cmd_vel`
- 전진: 0.1 m/s, 후진: -0.1 m/s
- 회전 시 보조 직진: 0.05 m/s (Ackermann 구동 특성상 linear=0이면 조향 불가)
- 입력: pynput 키보드 리스너 (방향키)

---

## 4. 요약

### 4.1 시스템 한 줄 요약

> **이중주차된 LIMO1이 후진으로 길을 비우고, 사용자가 선택한 상대 차량(A2/A3/A4)이 Pure Pursuit으로 자율 출차한 뒤, LIMO1이 Hybrid A* + Reeds-Shepp 경로 계획으로 빈 슬롯에 정밀 재주차하는 ROS2 기반 원격 주차 자율주행 시스템이다.**

### 4.2 핵심 알고리즘 요약

| 단계 | 알고리즘 | 핵심 센서 |
|------|---------|----------|
| LIMO1 이중주차 해제 | P 제어 직진 후진 + Depth 카메라 E-stop | Odometry, 후방 Depth, LiDAR |
| 출차 차량 선택 | 서비스 호출 (a2/a3/a4) | - |
| 상대 차량 출차 경로 생성 | Sinusoidal 곡선 프로파일 웨이포인트 | - |
| 상대 차량 출차 추종 | Pure Pursuit (Lookahead 0.25m) | Odometry |
| 빈 슬롯 탐지 | LiDAR Side Scan → Odom 투영 → ROI 점유 비교 | LiDAR, Odometry |
| 재주차 경로 계획 | Hybrid A* + Reeds-Shepp 곡선 (12가지 유형) | Odometry (grid는 정적) |
| 재주차 경로 추종 | 세그먼트별 Pure Pursuit + 최소 회전 반경 제한 | Odometry |
| 최종 정렬 | 저속 P 제어 (위치 + yaw) + 0.8초 홀드 판정 | Odometry |

### 4.3 시스템 특징

1. **마스터-슬레이브 아키텍처**: mission_manager가 전체 미션 FSM을 관리하고, 모든 차량의 cmd_vel을 직접 제어
2. **다중 차량 지원**: A2/A3/A4 중 사용자 선택에 따라 동적으로 출차 대상 변경 (서비스 호출 방식)
3. **센서 융합**: LiDAR + Depth 카메라 융합으로 신뢰성 있는 장애물/슬롯 감지
4. **안전 메커니즘**: E-stop (0.18m), 타임아웃(30s/60s), ABORT 상태
5. **Ackermann 구동**: LIMO 차량의 Ackermann 조향 특성을 반영한 제어 (최소 회전 반경 0.42m)
6. **Hybrid A* + Reeds-Shepp**: 전후진 혼합 경로 계획으로 좁은 주차장 환경에서 실현 가능한 경로 생성
7. **확장된 주차장**: 16슬롯(A-1~A6, B-1~B6) 구성, 정적 차량 13대 + 동적 로봇 4대
