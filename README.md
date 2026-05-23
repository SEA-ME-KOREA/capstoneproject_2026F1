# remote_parking_ws_1

ROS 2 Humble 기반 LIMO 플랫폼 원격 자율 주차 시스템

이중주차된 LIMO1이 후진으로 길을 비우고, 사용자가 선택한 상대 차량(A2/A3/A4)이 자율 출차한 뒤, LIMO1이 Hybrid A* 경로 계획으로 빈 슬롯에 정밀 재주차하는 시스템이다.

## 패키지 구성

```
remote_parking_ws_1/src/
├── remote_parking_world/      # Gazebo 월드 + 로봇 스폰 + URDF + 유틸 스크립트
├── remote_parking_manager/    # 미션 마스터 FSM 노드 + Hybrid A* 경로 계획기
├── my_valet_parking/          # LiDAR 슬롯 탐지, 독립 출차, 키보드 텔레옵 등
└── limo_car/                  # LIMO 차량 URDF 모델 및 Gazebo 플러그인
```

## 전제 조건 (사전 설치 필요)

- ROS 2 Humble
- Gazebo Classic 11
- Python 3 (`numpy`, `pynput`)

## 빌드

```bash
cd ~/remote_parking_ws_1
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행 순서

### 1단계: Gazebo 서버 시작 (1회만 실행)
```bash
ros2 launch remote_parking_world world_server.launch.py gui:=true
```
- gzserver 시작 (정적 월드 + 정적 차량 13대 로드)
- GUI 없이 실행: `gui:=false`

### 2단계: 로봇 스폰 + 노드 시작 (테스트마다 실행)
```bash
ros2 launch remote_parking_world spawn_robots.launch.py
```
- LIMO1, LIMO2, LIMO_A2, LIMO_A4 스폰
- dynamic_tracker_node, mission_manager, RViz 자동 시작

### 3단계: 미션 시작
```bash
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```
- LIMO1이 후진하여 이중주차 해제 → WAIT_FOR_SELECTION 상태로 전이

### 4단계: 출차 차량 선택 (새 터미널)
```bash
bash scripts/select_car.sh a3    # a2, a3, a4 중 선택
```
- 선택된 차량이 자율 출차 → 빈 슬롯 탐지 → LIMO1 자동 재주차

### 재시도 (Gazebo 재시작 없이)
```bash
bash src/remote_parking_world/scripts/reset_robots.sh teleport
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```

## 주요 토픽 및 서비스

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/start_remote_parking` | `std_srvs/Trigger` | 미션 시작 |
| `/reset_parking` | `std_srvs/Trigger` | 미션 리셋 (IDLE로 복귀) |
| `/select_exit_car/a2` | `std_srvs/Trigger` | A2 슬롯 차량 출차 선택 |
| `/select_exit_car/a3` | `std_srvs/Trigger` | A3 슬롯 차량 출차 선택 |
| `/select_exit_car/a4` | `std_srvs/Trigger` | A4 슬롯 차량 출차 선택 |

### 토픽

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/remote_parking/status` | 발행 | 현재 FSM 상태 문자열 |
| `/limo1/cmd_vel` | 발행 | LIMO 1 속도 명령 |
| `/limo2/cmd_vel` | 발행 | LIMO 2 속도 명령 |
| `/limo_a2/cmd_vel` | 발행 | LIMO A2 속도 명령 |
| `/limo_a4/cmd_vel` | 발행 | LIMO A4 속도 명령 |
| `/limo1/odom` | 구독 | LIMO 1 오도메트리 |
| `/limo2/odom` | 구독 | LIMO 2 오도메트리 |
| `/limo_a2/odom` | 구독 | LIMO A2 오도메트리 |
| `/limo_a4/odom` | 구독 | LIMO A4 오도메트리 |
| `/limo1/scan` | 구독 | LIMO 1 LiDAR |
| `/limo2/scan` | 구독 | LIMO 2 LiDAR |
| `/limo1/rear_camera/depth/image_raw` | 구독 | LIMO 1 후방 깊이 카메라 |
| `/limo1/front_camera/depth/image_raw` | 구독 | LIMO 1 전방 깊이 카메라 |
| `/limo2/rear_camera/depth/image_raw` | 구독 | LIMO 2 후방 깊이 카메라 |
| `/limo1/target_slot` | 발행 | 빈 슬롯 위치 (dynamic_tracker → mission_manager) |

## FSM 상태 흐름

```
IDLE → LIMO1_EVADE → WAIT_FOR_SELECTION → LIMO2_EXIT_INIT → LIMO2_EXITING
→ LIMO1_SCAN → LIMO1_REPARK → FINISH
```

## 유틸 스크립트

| 스크립트 | 위치 | 용도 |
|---------|------|------|
| `select_car.sh` | `src/remote_parking_world/scripts/` | 출차 차량 선택 (a2/a3/a4) |
| `reset_robots.sh` | `src/remote_parking_world/scripts/` | 로봇 위치 초기화 (teleport/respawn) |

## 문서

- `SYSTEM_ANALYSIS.md` — 전체 알고리즘 상세 분석 (Hybrid A*, Pure Pursuit, 센서 융합 등)
- `SYSTEM_OVERVIEW.md` — 시스템 개요, FSM 흐름, 노드별 핵심 로직, 보조 노드 설명
