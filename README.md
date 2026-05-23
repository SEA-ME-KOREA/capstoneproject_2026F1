# remote_parking_ws

ROS 2 Humble 기반 LIMO 플랫폼 원격 자율 주차 시스템

## 패키지 구성

```
remote_parking_ws/src/
├── remote_parking_world/   # Gazebo 월드 + 로봇 스폰 + URDF
└── remote_parking_manager/ # 미션 마스터 FSM 노드
```

## 전제 조건 (사전 설치 필요)

- `parking_ws` 빌드 완료 (limo_car, my_valet_parking 패키지 포함)
- `LIMO_platform` 빌드 완료 (wego_2d_nav, robot_localization 등)
- ROS 2 Humble, Gazebo Classic 11

## 빌드

```bash
cd ~/remote_parking_ws
source /opt/ros/humble/setup.bash
source ~/parking_ws/install/setup.bash     # limo_car, my_valet_parking
source ~/LIMO_platform/install/setup.bash  # wego_2d_nav 등
colcon build --symlink-install
source install/setup.bash
```

## 실행 순서 (★ 핵심 최적화: Gazebo 1회만 시작)

### 1단계: Gazebo 서버 시작 (1회만 실행)
```bash
ros2 launch remote_parking_world world_server.launch.py gui:=true
```

### 2단계: 로봇 스폰 + 노드 시작 (테스트마다 실행)
```bash
ros2 launch remote_parking_world spawn_robots.launch.py
```

### 3단계: 미션 시작 (원격 버튼 시뮬)
```bash
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```

### 재시도 (Gazebo 재시작 없이 ~0.5초)
```bash
bash src/remote_parking_world/scripts/reset_robots.sh teleport
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```

## 주요 토픽

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/start_remote_parking` | 서비스 | 미션 시작 버튼 |
| `/reset_parking` | 서비스 | 미션 리셋 |
| `/remote_parking/status` | 발행 | 현재 FSM 상태 |
| `/limo1/cmd_vel` | 발행 | LIMO 1 속도 명령 |
| `/limo2/cmd_vel` | 발행 | LIMO 2 속도 명령 |
| `/limo1/scan` | 구독 | LIMO 1 LiDAR |
| `/limo1/odom` | 구독 | LIMO 1 오도메트리 |
| `/limo1/rear_camera/depth/image_raw` | 구독 | LIMO 1 후방 카메라 |

## 검증된 버그 수정 목록

| # | 파일 | 내용 |
|---|------|------|
| 1 | world_server.launch.py | 존재하지 않는 패키지 import 제거 |
| 2 | world_server.launch.py | IfCondition import 누락 수정 |
| 3 | spawn_robots.launch.py | ament_python_file_backend → ament_index_python |
| 4 | spawn_robots.launch.py | xacro Python API 호출 방식 수정 |
| 5 | spawn_robots.launch.py | xacro 인자 robot_name → robot_namespace |
| 6→12 | spawn_robots.launch.py | spawn z: 0.065 → 0.145 (base_link root 기반) |
| 7→14 | limo_with_rear_camera.urdf.xacro | include 경로: urdf/ → gazebo/ |
| 8 | limo_with_rear_camera.urdf.xacro | arg 이름 robot_name → robot_namespace |
| 9→15 | limo_with_rear_camera.urdf.xacro | 후방 카메라 namespace 처리 방식 수정 |
| 10 | mission_manager.py | 장애물 감지 각도 ±20° → ±30° |
| 11 | spawn_robots.launch.py | dynamic_tracker 누락 파라미터 추가 |
| 13 | spawn_robots.launch.py | spawn 명령에 -robot_namespace 플래그 추가 |
