# 원격 자율 주차 시스템 — 코딩 에이전트 구현 가이드

> **목적**: 이 문서 하나로 전체 시스템을 처음부터 구현·디버깅할 수 있도록 작성됨.  
> **플랫폼**: AgileX LIMO Pro · ROS 2 Humble · Gazebo Classic 11 · Ubuntu 22.04

---

## 1. 시나리오 & 목표

```
[초기 상태]
  LIMO 1 (사용자 차) ── 차로에서 LIMO 2 앞 이중주차 (차로 막음)
  LIMO 2 (상대 차)  ── A3 슬롯에 정상 주차 (LIMO 1에 막혀 출차 불가)

[사용자 버튼 1회 누름]  ──→  ros2 service call /start_remote_parking

  ① LIMO 1 후진 (B3 슬롯으로 후진, 차로 개방)
  ② LIMO 2 자율 출차 (차로 → 출구, 웨이포인트 P제어)
  ③ LIMO 1이 LiDAR로 빈 슬롯(A3) 감지
  ④ LIMO 1 A3 재주차 (3단계 FSM)
  ⑤ 완료 — 양 로봇 정지
```

**핵심 제약**
- 원격 버튼: ROS2 서비스 호출로 시뮬
- 카메라 스트리밍: 불필요
- 슬롯 탐지: LiDAR(동적) + 후방 Depth 카메라(보조)
- 시뮬레이터 빠른 기동: Gazebo 1회 시작 후 로봇만 리셋

---

## 2. 전제 조건 (사전 준비)

### 필수 워크스페이스 (이미 존재해야 함)

```
~/parking_ws/          ← parking_ws 레포
  src/
    limo_car/          ← URDF + Gazebo xacro
    limo_gazebosim/    ← 기존 월드 파일
    my_valet_parking/  ← dynamic_tracker_node, limo_valet_parking_node

~/LIMO_platform/       ← LIMO_platform 레포
  src/
    wego_2d_nav/       ← Nav2 파라미터
    robot_localization/← EKF
```

### limo_car 패키지 디렉토리 구조 확인

```
limo_car/
├── urdf/
│   ├── limo_ackerman_base.xacro      ← base_footprint 주석처리됨, root=base_link
│   ├── limo_anteil.xacro
│   └── limo_steering_hinge.xacro
├── gazebo/                           ★ Gazebo 플러그인은 이 디렉토리
│   ├── ackermann_with_sensor.xacro   ← include 대상 (sensor.xacro + ackermann.xacro)
│   ├── ackermann.xacro               ← 드라이브 플러그인
│   └── sensor.xacro                  ← LiDAR/카메라/IMU 플러그인
└── launch/
    └── ackermann.launch.py           ← gazebo/ackermann_with_sensor.xacro 로드
```

### URDF 핵심 수치 (ackermann.xacro 기준)

| 항목 | 값 | 비고 |
|------|----|------|
| root link | `base_link` | base_footprint 주석 처리됨 |
| wheel_vertical_offset | `-0.10 m` | base_link 기준 바퀴 중심 z |
| wheel_radius | `0.045 m` | |
| **spawn z** | **`0.145 m`** | 바퀴 하단이 z=0에 닿는 값 |
| wheelbase | `0.24 m` | |
| track | `0.168 m` | |
| cmd_vel 토픽 | `cmd_vel` (상대) | spawn -robot_namespace 로 namespacing |
| odom 토픽 | `odom` (상대) | |
| scan 토픽 | `~/out → scan` | namespace=/ 이나 spawn이 override |

### Gazebo 센서 토픽 (spawn -robot_namespace limo1 기준)

| 센서 | 토픽 |
|------|------|
| LiDAR | `/limo1/scan` |
| Depth (RGB) | `/limo1/rgb/image_raw` |
| Depth (깊이) | `/limo1/depth/image_raw` |
| 후방 카메라 (신규) | `/limo1/rear_camera/depth/image_raw` |
| IMU | `/limo/imu` (**하드코딩**, 두 로봇 공유) |
| Odometry | `/limo1/odom` |
| cmd_vel 수신 | `/limo1/cmd_vel` |

---

## 3. 새 워크스페이스 구조

```
~/remote_parking_ws/
├── README.md
└── src/
    ├── remote_parking_world/          ← Gazebo 월드 + 스폰 launch
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── worlds/
    │   │   └── remote_parking_static.world
    │   ├── urdf/
    │   │   └── limo_with_rear_camera.urdf.xacro
    │   ├── launch/
    │   │   ├── world_server.launch.py   ← 1회만 실행
    │   │   └── spawn_robots.launch.py   ← 테스트마다 실행
    │   └── scripts/
    │       └── reset_robots.sh          ← 빠른 리셋 (~0.5초)
    └── remote_parking_manager/         ← 미션 마스터 FSM
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/remote_parking_manager
        └── remote_parking_manager/
            ├── __init__.py
            └── mission_manager.py
```

---

## 4. 핵심 아키텍처

### 2단계 Launch 분리 (시뮬레이터 속도 최적화)

```
[1회 실행]   world_server.launch.py
               gzserver + 정적 월드 로드 (주차장 + 정적 차량 7대)
               → Gazebo 프로세스 유지, 재시작 없음

[매 테스트]  spawn_robots.launch.py
               LIMO 1·2 스폰 + 모든 ROS2 노드 시작
               타이밍: RSP(0s) → 스폰(3s) → 제어노드(5s) → FSM(6s)

[리셋]       bash reset_robots.sh teleport
               set_entity_state로 위치만 이동 (~0.5초)
               Gazebo 재시작 없음
```

### FSM 상태 전이

```
IDLE
  │ /start_remote_parking 서비스 수신
  ▼
LIMO1_EVADE        ── LIMO 1 후진 (0.225,0) → (0.225,-0.985) [B3 슬롯]
  │ 후진 완료 (오도메트리 기준 도달)
  ▼
LIMO2_EXIT_INIT    ── LIMO 2 웨이포인트 초기화
  │ 즉시 전환
  ▼
LIMO2_EXITING      ── LIMO 2 출차 P제어 (4개 웨이포인트)
  │ 모든 웨이포인트 완료
  ▼
LIMO1_SCAN         ── LiDAR로 A3 슬롯 비어있음 확인
  │ target_slot 수신 OR 타임아웃(10s) → fallback 사용
  ▼
LIMO1_REPARK       ── APPROACH → ARC_TURN → PRECISION
  │ 정밀 주차 완료 (yaw<1°, x_err<8mm, 0.6s 유지)
  ▼
FINISH             ── 양 로봇 정지
```

### 패키지 역할 분담

| 기능 | 출처 | 파일 |
|------|------|------|
| URDF + Gazebo 플러그인 | `parking_ws/limo_car` | `gazebo/ackermann_with_sensor.xacro` |
| 후방 카메라 추가 | **신규** | `remote_parking_world/urdf/limo_with_rear_camera.urdf.xacro` |
| Gazebo 월드 | **신규** | `remote_parking_world/worlds/remote_parking_static.world` |
| LiDAR 슬롯 탐지 | `parking_ws/my_valet_parking` | `dynamic_tracker_node.py` |
| 미션 FSM | **신규** | `remote_parking_manager/mission_manager.py` |

---

## 5. 주차장 월드 설계 (법령 기준 1/5.6 축척)

```
실제 슬롯: 2.5m × 5.0m  →  Gazebo: 0.45m × 0.90m
실제 차로: 6.0m          →  Gazebo: 1.07m
LIMO 크기:               →  0.34m(W) × 0.78m(L)

[월드 Top-view]                    +Y (북)

  North wall ────────────────────── Y = +1.585
  [A1:red][A2:blue][A3:LIMO2][A4:silver]  Y = +0.985
  Lane edge ───────────────────────  Y = +0.535
              LANE (center Y=0)
              ← LIMO 1 이중주차 →
  Lane edge ───────────────────────  Y = -0.535
  [B1:yel][B2:grn][B3:empty][B4:ora]      Y = -0.985
  South wall ────────────────────── Y = -1.585

  West wall X=-0.95         출구 → +X

슬롯 X 좌표: A1/B1=-0.675, A2/B2=-0.225, A3/B3=+0.225, A4/B4=+0.675
A3, B3: 정적 모델 없음 (LIMO 1·2 스폰 위치)
```

### 초기 스폰 포즈

| 로봇 | x | y | z | yaw |
|------|---|---|---|-----|
| LIMO 1 | 0.225 | 0.0 | **0.145** | +π/2 |
| LIMO 2 | 0.225 | 0.985 | **0.145** | -π/2 |

> z=0.145 근거: base_link root, wheel_vertical_offset=-0.10, wheel_radius=0.045 → 바퀴 하단 = spawn_z - 0.145 = 0

---

## 6. 전체 구현 파일

### 6-1. `remote_parking_world/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>remote_parking_world</name>
  <version>0.1.0</version>
  <description>Remote parking Gazebo world and robot spawn launch</description>
  <maintainer email="you@example.com">Jihyun</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>my_valet_parking</exec_depend>
  <exec_depend>remote_parking_manager</exec_depend>
  <ament_cmake_depend>ament_cmake</ament_cmake_depend>
</package>
```

### 6-2. `remote_parking_world/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(remote_parking_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds models launch scripts urdf config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/reset_robots.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
```

### 6-3. `remote_parking_world/worlds/remote_parking_static.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="remote_parking">

    <!-- 물리 최적화: 1000Hz→500Hz, quick solver, 그림자OFF -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- ROS2 플러그인 -->
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>

    <!-- 조명 (그림자 OFF) -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 지면 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 아스팔트 -->
    <model name="parking_asphalt">
      <static>true</static>
      <pose>0.05 0 0.001 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 3.17 0.002</size></box></geometry></collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 3.17 0.002</size></box></geometry>
          <material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.22 0.22 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 북쪽 벽 -->
    <model name="wall_north">
      <static>true</static><pose>0.05 1.585 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 0.10 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 0.10 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 남쪽 벽 -->
    <model name="wall_south">
      <static>true</static><pose>0.05 -1.585 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 0.10 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 0.10 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 서쪽 벽 -->
    <model name="wall_west">
      <static>true</static><pose>-0.95 0 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.10 3.17 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>0.10 3.17 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 차로 경계선 북 Y=+0.535 -->
    <model name="line_lane_north"><static>true</static><pose>0.05 0.535 0.003 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>1.80 0.02 0.002</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>1.80 0.02 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link>
    </model>

    <!-- 차로 경계선 남 Y=-0.535 -->
    <model name="line_lane_south"><static>true</static><pose>0.05 -0.535 0.003 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>1.80 0.02 0.002</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>1.80 0.02 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Row A 슬롯 구획선 5개 (X = -0.90 ~ +0.90) -->
    <model name="line_a0"><static>true</static><pose>-0.90 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a1"><static>true</static><pose>-0.45 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a2"><static>true</static><pose>0.0 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a3"><static>true</static><pose>0.45 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a4"><static>true</static><pose>0.90 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>

    <!-- Row B 슬롯 구획선 5개 -->
    <model name="line_b0"><static>true</static><pose>-0.90 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b1"><static>true</static><pose>-0.45 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b2"><static>true</static><pose>0.0 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b3"><static>true</static><pose>0.45 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b4"><static>true</static><pose>0.90 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>

    <!-- 정적 차량 (static=true, box geometry, A3/B3 비워둠)
         크기: 0.78m(길이) × 0.34m(폭) × 0.13m(높이), center z=0.065
         Row A yaw=-π/2 (앞면 차로 방향), Row B yaw=+π/2 -->

    <!-- A1 red -->
    <model name="static_car_a1"><static>true</static><pose>-0.675 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
      </link>
    </model>
    <!-- A2 blue -->
    <model name="static_car_a2"><static>true</static><pose>-0.225 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.1 0.1 0.85 1</ambient><diffuse>0.1 0.1 0.85 1</diffuse></material></visual>
      </link>
    </model>
    <!-- A3: LIMO 2 동적 스폰 -->
    <!-- A4 silver -->
    <model name="static_car_a4"><static>true</static><pose>0.675 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <!-- B1 yellow -->
    <model name="static_car_b1"><static>true</static><pose>-0.675 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>0.9 0.8 0.0 1</diffuse></material></visual>
      </link>
    </model>
    <!-- B2 green -->
    <model name="static_car_b2"><static>true</static><pose>-0.225 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.1 0.75 0.1 1</ambient><diffuse>0.1 0.75 0.1 1</diffuse></material></visual>
      </link>
    </model>
    <!-- B3: LIMO 1 임시 후진 슬롯 — 비워둠 -->
    <!-- B4 orange -->
    <model name="static_car_b4"><static>true</static><pose>0.675 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.95 0.5 0.0 1</ambient><diffuse>0.95 0.5 0.0 1</diffuse></material></visual>
      </link>
    </model>

  </world>
</sdf>
```

### 6-4. `remote_parking_world/urdf/limo_with_rear_camera.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="limo" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- robot_namespace: spawn_entity -robot_namespace 와 동일한 값 설정
       후방 카메라 플러그인 식별용; 실제 토픽 namespacing은 spawn이 처리 -->
  <xacro:arg name="robot_namespace" default="/"/>
  <xacro:property name="robot_ns" value="$(arg robot_namespace)"/>

  <!-- 기존 LIMO Ackermann URDF (Gazebo 플러그인 포함)
       경로: limo_car/gazebo/ (urdf/ 아님!)
       포함 내용:
         - libgazebo_ros_ackermann_drive (cmd_vel, odom)
         - libgazebo_ros_ray_sensor (scan)
         - libgazebo_ros_camera (rgb, depth)
         - libgazebo_ros_imu_sensor (imu)
       주의: ackermann_with_sensor.xacro 는 외부 인자를 받지 않음 -->
  <xacro:include filename="$(find limo_car)/gazebo/ackermann_with_sensor.xacro"/>

  <!-- 후방 카메라 링크 (base_link 기준 후방 -0.30m, 높이 +0.10m) -->
  <link name="rear_camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.025 0.090 0.025"/></geometry>
      <material name="cam_mat"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.025 0.090 0.025"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="rear_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="rear_camera_link"/>
    <origin xyz="-0.30 0 0.10" rpy="0 0 3.14159"/>
  </joint>

  <!-- 광학 프레임 (ROS 관례: x-right, y-down, z-forward) -->
  <link name="rear_camera_optical_frame"/>
  <joint name="rear_camera_optical_joint" type="fixed">
    <parent link="rear_camera_link"/>
    <child  link="rear_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <!-- Gazebo 후방 Depth 카메라 플러그인
       namespace 미지정 → spawn_entity -robot_namespace 가 자동 적용
       limo1: /limo1/rear_camera/depth/image_raw
       limo2: /limo2/rear_camera/depth/image_raw -->
  <gazebo reference="rear_camera_link">
    <sensor name="rear_depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>
        <image><width>320</width><height>240</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>3.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
      </camera>
      <plugin name="rear_depth_camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>image_raw:=rear_camera/depth/image_raw</remapping>
          <remapping>camera_info:=rear_camera/depth/camera_info</remapping>
          <remapping>points:=rear_camera/points</remapping>
        </ros>
        <camera_name>rear_camera</camera_name>
        <frame_name>rear_camera_optical_frame</frame_name>
        <min_depth>0.05</min_depth>
        <max_depth>3.0</max_depth>
        <hack_baseline>0.0</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### 6-5. `remote_parking_world/launch/world_server.launch.py`

```python
#!/usr/bin/env python3
"""
world_server.launch.py — 1회만 실행
gzserver + 정적 월드 로드 (LIMO 없음)
gui:=true 로 gzclient 포함 가능
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('remote_parking_world')
    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'remote_parking_static.world'
    ])
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[gazebo_model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),
        DeclareLaunchArgument('gui', default_value='false',
                              description='gzclient 도 시작할지 여부'),

        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_state.so',
                world_file,
            ],
            output='screen',
            additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path},
        ),

        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
```

### 6-6. `remote_parking_world/launch/spawn_robots.launch.py`

```python
#!/usr/bin/env python3
"""
spawn_robots.launch.py — 테스트마다 실행
전제: world_server.launch.py 가 이미 실행 중

타이밍:
  0s  → robot_state_publisher × 2 시작
  3s  → LIMO 1·2 Gazebo 스폰
  5s  → dynamic_tracker_node 시작
  6s  → remote_parking_manager FSM 시작
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace

# ── 초기 스폰 포즈 ──────────────────────────────────────────────────
# z=0.145: base_link root, wheel_offset=-0.10, wheel_r=0.045 → 바퀴 하단=z=0
LIMO1_INIT = dict(x='0.225', y='0.0',   z='0.145', R='0', P='0', Y='1.5708')
LIMO2_INIT = dict(x='0.225', y='0.985', z='0.145', R='0', P='0', Y='-1.5708')


def generate_launch_description():
    pkg_world = get_package_share_directory('remote_parking_world')
    urdf_xacro = os.path.join(pkg_world, 'urdf', 'limo_with_rear_camera.urdf.xacro')

    # xacro 런타임 처리 (Command 치환자)
    limo1_desc = Command([FindExecutable(name='xacro'), ' ', urdf_xacro, ' robot_namespace:=limo1'])
    limo2_desc = Command([FindExecutable(name='xacro'), ' ', urdf_xacro, ' robot_namespace:=limo2'])

    # robot_state_publisher 그룹
    limo1_group = GroupAction([
        PushRosNamespace('limo1'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': limo1_desc, 'use_sim_time': True}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}]),
    ])
    limo2_group = GroupAction([
        PushRosNamespace('limo2'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': limo2_desc, 'use_sim_time': True}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}]),
    ])

    # Gazebo 스폰 (3초 후 — RSP 준비 대기)
    # -robot_namespace 필수: Gazebo 플러그인 토픽 namespacing
    spawn_limo1 = TimerAction(period=3.0, actions=[ExecuteProcess(cmd=[
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'limo1',
        '-topic',  '/limo1/robot_description',
        '-robot_namespace', 'limo1',
        '-x', LIMO1_INIT['x'], '-y', LIMO1_INIT['y'], '-z', LIMO1_INIT['z'],
        '-R', LIMO1_INIT['R'], '-P', LIMO1_INIT['P'], '-Y', LIMO1_INIT['Y'],
        '-timeout', '120.0',
    ], output='screen')])

    spawn_limo2 = TimerAction(period=3.0, actions=[ExecuteProcess(cmd=[
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'limo2',
        '-topic',  '/limo2/robot_description',
        '-robot_namespace', 'limo2',
        '-x', LIMO2_INIT['x'], '-y', LIMO2_INIT['y'], '-z', LIMO2_INIT['z'],
        '-R', LIMO2_INIT['R'], '-P', LIMO2_INIT['P'], '-Y', LIMO2_INIT['Y'],
        '-timeout', '120.0',
    ], output='screen')])

    # dynamic_tracker_node (5초 후 — 스폰 완료 대기)
    dynamic_tracker = TimerAction(period=5.0, actions=[Node(
        package='my_valet_parking', executable='dynamic_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic':   '/limo1/scan',
            'odom_topic':   '/limo1/odom',
            'target_topic': '/limo1/target_slot',
            'odom_frame':   'odom',
            'occupied_distance_m':            0.45,
            'clear_distance_m':               0.65,
            'baseline_capture_distance_m':    0.75,
            'clear_hold_s':                   0.70,
            'slot_center_offset_m':           0.30,
            'allow_fallback_without_baseline': True,  # 초기 baseline 없이 탐지
            'fallback_target_x':              0.225,
            'fallback_target_y':              0.985,
            'target_yaw_rad':                -1.5708,
            'side_scan_center_deg':           90.0,
            'side_scan_half_deg':             8.0,
            'min_clear_ratio':                0.25,
            'track_box_length_m':             0.35,
            'track_box_width_m':              0.30,
            'publish_period_s':               0.10,
        }],
    )])

    # mission_manager FSM (6초 후)
    mission_manager = TimerAction(period=6.0, actions=[Node(
        package='remote_parking_manager', executable='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time':    True,
            'limo1_evade_x':   0.225,
            'limo1_evade_y':  -0.985,
            'limo2_exit_x':    3.0,
            'limo2_exit_y':    0.0,
            'repark_x':        0.225,
            'repark_y':        0.985,
            'repark_yaw':     -1.5708,
            'max_linear_speed': 0.05,
            'max_angular_speed': 0.8,
        }],
    )])

    return LaunchDescription([
        limo1_group, limo2_group,
        spawn_limo1, spawn_limo2,
        dynamic_tracker,
        mission_manager,
    ])
```

### 6-7. `remote_parking_world/scripts/reset_robots.sh`

```bash
#!/usr/bin/env bash
# 빠른 리셋: Gazebo 재시작 없이 로봇 위치만 초기화
# 사용법: bash reset_robots.sh [teleport|respawn]
# teleport (기본): set_entity_state로 즉시 이동 (~0.5초)
# respawn: delete + respawn (~5초, 완전 초기화)

set -e
MODE=${1:-"teleport"}

# z=0.145: ackermann.xacro 기준 바퀴 하단 z=0 되는 높이
LIMO1_POSE="{position: {x: 0.225, y: 0.0,   z: 0.145}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}"
LIMO2_POSE="{position: {x: 0.225, y: 0.985, z: 0.145}, orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"
ZERO_TWIST="{linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0}}"
LIMO1_INIT="-x 0.225 -y 0.0   -z 0.145 -R 0 -P 0 -Y  1.5708"
LIMO2_INIT="-x 0.225 -y 0.985 -z 0.145 -R 0 -P 0 -Y -1.5708"

echo "━━━ Remote Parking Reset (MODE: $MODE) ━━━"

# FSM 리셋 신호
ros2 topic pub --once /remote_parking/reset std_msgs/Bool "data: true" 2>/dev/null || true

# 정지 명령
ros2 topic pub --once /limo1/cmd_vel geometry_msgs/Twist \
  "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true
ros2 topic pub --once /limo2/cmd_vel geometry_msgs/Twist \
  "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true
sleep 0.5

if [ "$MODE" = "teleport" ]; then
    ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
      "{state: {name: 'limo1', pose: $LIMO1_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}"
    ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
      "{state: {name: 'limo2', pose: $LIMO2_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}"
    echo "✅ 텔레포트 리셋 완료"

elif [ "$MODE" = "respawn" ]; then
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo1'}" 2>/dev/null || true
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo2'}" 2>/dev/null || true
    sleep 2
    ros2 run gazebo_ros spawn_entity.py -entity limo1 -topic /limo1/robot_description \
      -robot_namespace limo1 $LIMO1_INIT &
    ros2 run gazebo_ros spawn_entity.py -entity limo2 -topic /limo2/robot_description \
      -robot_namespace limo2 $LIMO2_INIT &
    wait
    echo "✅ 재스폰 리셋 완료"
fi

echo "→ 다음 테스트: ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'"
```

### 6-8. `remote_parking_manager/remote_parking_manager/mission_manager.py`

```python
#!/usr/bin/env python3
"""
mission_manager.py — 원격 주차 미션 마스터 FSM

FSM: IDLE → LIMO1_EVADE → LIMO2_EXIT_INIT → LIMO2_EXITING
       → LIMO1_SCAN → LIMO1_REPARK (APPROACH→ARC_TURN→PRECISION) → FINISH

서비스:
  /start_remote_parking  (std_srvs/Trigger)  미션 시작
  /reset_parking         (std_srvs/Trigger)  리셋

구독:
  /limo1/odom, /limo1/scan
  /limo1/rear_camera/depth/image_raw (후방 장애물)
  /limo2/odom
  /limo1/target_slot (dynamic_tracker_node 발행)
  /remote_parking/reset (Bool)

발행:
  /limo1/cmd_vel, /limo2/cmd_vel
  /remote_parking/status (String)
"""

import math
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionState(Enum):
    IDLE            = auto()
    LIMO1_EVADE     = auto()
    LIMO2_EXIT_INIT = auto()
    LIMO2_EXITING   = auto()
    LIMO1_SCAN      = auto()
    LIMO1_REPARK    = auto()
    FINISH          = auto()
    ABORT           = auto()


class ParkStage(Enum):
    APPROACH  = auto()
    ARC_TURN  = auto()
    PRECISION = auto()


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


def pos_from_odom(odom: Odometry) -> Tuple[float, float]:
    p = odom.pose.pose.position
    return p.x, p.y


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def dist2d(x1, y1, x2, y2) -> float:
    return math.hypot(x2-x1, y2-y1)


class RemoteParkingManager(Node):

    CTRL_HZ          = 20.0
    ESTOP_DIST_M     = 0.18
    EVADE_SPEED      = -0.04
    EVADE_YAW_KP     = 1.8
    LIMO2_LINEAR     = 0.05
    LIMO2_ANGULAR_KP = 2.0
    MAX_LINEAR       = 0.05
    ARC_ANGULAR_MIN  = 0.35
    ARC_ANGULAR_MAX  = 1.2
    ARC_KP           = 1.5
    PREC_KP_LAT      = 1.4
    PREC_KP_YAW      = 2.2
    PREC_FINISH_DIST = 0.005
    PREC_FINISH_YAW  = 0.018
    PREC_HOLD_S      = 0.6
    SCAN_TIMEOUT_S   = 10.0

    def __init__(self):
        super().__init__('remote_parking_manager')

        self.declare_parameter('limo1_evade_x',    0.225)
        self.declare_parameter('limo1_evade_y',   -0.985)
        self.declare_parameter('limo2_exit_x',     3.0)
        self.declare_parameter('limo2_exit_y',     0.0)
        self.declare_parameter('repark_x',         0.225)
        self.declare_parameter('repark_y',         0.985)
        self.declare_parameter('repark_yaw',      -1.5708)
        self.declare_parameter('max_linear_speed', 0.05)

        self.evade_goal  = (self.get_parameter('limo1_evade_x').value,
                            self.get_parameter('limo1_evade_y').value)
        self.limo2_exit  = (self.get_parameter('limo2_exit_x').value,
                            self.get_parameter('limo2_exit_y').value)
        self.repark_goal = (self.get_parameter('repark_x').value,
                            self.get_parameter('repark_y').value)
        self.repark_yaw_goal = self.get_parameter('repark_yaw').value

        self.state      = MissionState.IDLE
        self.park_stage = ParkStage.APPROACH

        self.limo1_odom: Optional[Odometry] = None
        self.limo1_scan: Optional[LaserScan] = None
        self.limo2_odom: Optional[Odometry] = None
        self.target_slot: Optional[PoseStamped] = None

        self.front_obstacle_m = 999.0
        self.rear_obstacle_m  = 999.0
        self.limo2_waypoints  = []
        self.limo2_wp_idx     = 0
        self.scan_start_time: Optional[float] = None
        self.prec_ok_since:   Optional[float] = None
        self.approach_target: Optional[Tuple[float, float]] = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.create_subscription(Odometry,    '/limo1/odom',  self._l1_odom_cb,  sensor_qos)
        self.create_subscription(LaserScan,   '/limo1/scan',  self._l1_scan_cb,  sensor_qos)
        self.create_subscription(Image, '/limo1/rear_camera/depth/image_raw',
                                 self._rear_depth_cb, sensor_qos)
        self.create_subscription(Odometry,    '/limo2/odom',  self._l2_odom_cb,  sensor_qos)
        self.create_subscription(PoseStamped, '/limo1/target_slot', self._slot_cb, 1)
        self.create_subscription(Bool, '/remote_parking/reset', self._reset_cb, 1)

        self.limo1_cmd  = self.create_publisher(Twist,  '/limo1/cmd_vel', 1)
        self.limo2_cmd  = self.create_publisher(Twist,  '/limo2/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/remote_parking/status', 1)

        self.create_service(Trigger, '/start_remote_parking', self._start_cb)
        self.create_service(Trigger, '/reset_parking',        self._reset_srv_cb)
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)

        self._build_limo2_waypoints()
        self.get_logger().info('🚗 RemoteParkingManager 준비 — /start_remote_parking 대기')

    # ── 웨이포인트 ────────────────────────────────────────────────────
    def _build_limo2_waypoints(self):
        self.limo2_waypoints = [
            (0.225,  0.2,  -math.pi/2),   # 슬롯 → 차로 진입
            (0.225,  0.0,  -math.pi/2),   # 차로 중앙
            (0.50,   0.0,   0.0),          # 우회전 준비
            (self.limo2_exit[0], self.limo2_exit[1], 0.0),  # 출구
        ]
        self.limo2_wp_idx = 0

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _l1_odom_cb(self, m): self.limo1_odom = m
    def _l2_odom_cb(self, m): self.limo2_odom = m
    def _slot_cb(self, m):    self.target_slot = m

    def _l1_scan_cb(self, msg: LaserScan):
        self.limo1_scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges) & (ranges > msg.range_min)
                          & (ranges < msg.range_max), ranges, np.inf)
        n = len(ranges)
        if n == 0: return
        # ±30° (limo_valet_parking_node.py 기준 ESTOP_FRONT_START/END_DEG=±30)
        span = int(30 / math.degrees(msg.angle_increment))
        fi = np.concatenate([np.arange(0, span), np.arange(n-span, n)])
        self.front_obstacle_m = float(np.min(ranges[fi]))
        c = n // 2
        self.rear_obstacle_m  = float(np.min(ranges[max(0,c-span):min(n,c+span)]))

    def _rear_depth_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                d = np.frombuffer(msg.data, dtype=np.float32).copy()
                d = d[np.isfinite(d) & (d > 0.01)]
                if len(d) > 0:
                    self.rear_obstacle_m = min(self.rear_obstacle_m, float(np.percentile(d, 5)))
        except Exception: pass

    def _reset_cb(self, msg: Bool):
        if msg.data: self._do_reset()

    def _start_cb(self, req, res):
        if self.state != MissionState.IDLE:
            res.success = False; res.message = f'진행 중: {self.state.name}'
            return res
        if self.limo1_odom is None or self.limo2_odom is None:
            res.success = False; res.message = '오도메트리 미수신'
            return res
        self._transition_to(MissionState.LIMO1_EVADE)
        res.success = True; res.message = '미션 시작'
        return res

    def _reset_srv_cb(self, req, res):
        self._do_reset()
        res.success = True; res.message = 'IDLE 리셋'
        return res

    def _do_reset(self):
        self._pub(self.limo1_cmd, 0, 0); self._pub(self.limo2_cmd, 0, 0)
        self.target_slot = None; self.scan_start_time = None
        self.prec_ok_since = None; self._build_limo2_waypoints()
        self._transition_to(MissionState.IDLE)

    # ── 제어 루프 ─────────────────────────────────────────────────────
    def _control_loop(self):
        s = String(); s.data = self.state.name; self.status_pub.publish(s)

        if   self.state == MissionState.IDLE:            return
        elif self.state == MissionState.LIMO1_EVADE:     self._evade()
        elif self.state == MissionState.LIMO2_EXIT_INIT: self._exit_init()
        elif self.state == MissionState.LIMO2_EXITING:   self._exiting()
        elif self.state == MissionState.LIMO1_SCAN:      self._scan()
        elif self.state == MissionState.LIMO1_REPARK:    self._repark()
        elif self.state in (MissionState.FINISH, MissionState.ABORT):
            self._pub(self.limo1_cmd, 0, 0); self._pub(self.limo2_cmd, 0, 0)

    # ── EVADE ─────────────────────────────────────────────────────────
    def _evade(self):
        if self.limo1_odom is None: return
        if self.rear_obstacle_m < self.ESTOP_DIST_M:
            self._pub(self.limo1_cmd, 0, 0); return
        cx, cy = pos_from_odom(self.limo1_odom)
        gx, gy = self.evade_goal
        if dist2d(cx, cy, gx, gy) < 0.04:
            self._pub(self.limo1_cmd, 0, 0)
            self._transition_to(MissionState.LIMO2_EXIT_INIT); return
        cur_yaw = yaw_from_odom(self.limo1_odom)
        tgt_yaw = norm_angle(math.atan2(gy-cy, gx-cx) + math.pi)
        yaw_err = norm_angle(tgt_yaw - cur_yaw)
        remaining = dist2d(cx, cy, gx, gy)
        linear  = max(self.EVADE_SPEED, min(-0.01, -remaining * 0.08))
        angular = float(np.clip(self.EVADE_YAW_KP * yaw_err, -1.5, 1.5))
        self._pub(self.limo1_cmd, linear, angular)

    # ── LIMO 2 출차 ───────────────────────────────────────────────────
    def _exit_init(self):
        self._build_limo2_waypoints()
        self._transition_to(MissionState.LIMO2_EXITING)

    def _exiting(self):
        if self.limo2_odom is None: return
        if self.limo2_wp_idx < len(self.limo2_waypoints):
            wp = self.limo2_waypoints[self.limo2_wp_idx]
            if self._drive_limo2(wp[0], wp[1], wp[2]):
                self.limo2_wp_idx += 1
        else:
            self._pub(self.limo2_cmd, 0, 0)
            self._transition_to(MissionState.LIMO1_SCAN)

    def _drive_limo2(self, gx, gy, g_yaw) -> bool:
        cx, cy  = pos_from_odom(self.limo2_odom)
        cur_yaw = yaw_from_odom(self.limo2_odom)
        d = dist2d(cx, cy, gx, gy)
        if d < 0.05:
            ye = norm_angle(g_yaw - cur_yaw)
            if abs(ye) < 0.15:
                self._pub(self.limo2_cmd, 0, 0); return True
            self._pub(self.limo2_cmd, 0, float(np.clip(self.LIMO2_ANGULAR_KP*ye, -0.8, 0.8)))
            return False
        tgt = math.atan2(gy-cy, gx-cx)
        ye  = norm_angle(tgt - cur_yaw)
        lin = float(np.clip(self.LIMO2_LINEAR*(1-abs(ye)/math.pi), 0.01, self.LIMO2_LINEAR))
        ang = float(np.clip(self.LIMO2_ANGULAR_KP*ye, -1.0, 1.0))
        self._pub(self.limo2_cmd, lin, ang)
        return False

    # ── 슬롯 스캔 ─────────────────────────────────────────────────────
    def _scan(self):
        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now().nanoseconds * 1e-9
        if self.target_slot is not None:
            self._transition_to(MissionState.LIMO1_REPARK); return
        elapsed = self.get_clock().now().nanoseconds*1e-9 - self.scan_start_time
        if elapsed > self.SCAN_TIMEOUT_S:
            self.get_logger().warn('슬롯 스캔 타임아웃 → fallback 사용')
            self._transition_to(MissionState.LIMO1_REPARK)

    # ── 재주차 ────────────────────────────────────────────────────────
    def _repark(self):
        if self.limo1_odom is None: return
        if self.front_obstacle_m < self.ESTOP_DIST_M:
            self._pub(self.limo1_cmd, 0, 0); return
        if   self.park_stage == ParkStage.APPROACH:  self._approach()
        elif self.park_stage == ParkStage.ARC_TURN:  self._arc_turn()
        elif self.park_stage == ParkStage.PRECISION:  self._precision()

    def _approach(self):
        gx, gy = self.repark_goal
        cx, cy = pos_from_odom(self.limo1_odom)
        if self.approach_target is None:
            self.approach_target = (gx, gy - 0.40)
        ax, ay = self.approach_target
        d = dist2d(cx, cy, ax, ay)
        if d < 0.05:
            self._pub(self.limo1_cmd, 0, 0)
            self.park_stage = ParkStage.ARC_TURN; return
        cur_yaw = yaw_from_odom(self.limo1_odom)
        ye  = norm_angle(math.atan2(ay-cy, ax-cx) - cur_yaw)
        lin = float(np.clip(0.5*d, 0.01, self.MAX_LINEAR))
        ang = float(np.clip(2.0*ye, -1.2, 1.2))
        self._pub(self.limo1_cmd, lin, ang)

    def _arc_turn(self):
        cur_yaw = yaw_from_odom(self.limo1_odom)
        ye = norm_angle(self.repark_yaw_goal - cur_yaw)
        if abs(ye) < 0.08:
            self._pub(self.limo1_cmd, 0, 0)
            self.park_stage = ParkStage.PRECISION; return
        sign = 1.0 if ye > 0 else -1.0
        ang  = sign * float(np.clip(abs(self.ARC_KP*ye), self.ARC_ANGULAR_MIN, self.ARC_ANGULAR_MAX))
        self._pub(self.limo1_cmd, 0.02, ang)

    def _precision(self):
        gx, gy   = self.repark_goal
        cx, cy   = pos_from_odom(self.limo1_odom)
        cur_yaw  = yaw_from_odom(self.limo1_odom)
        remaining = dist2d(cx, cy, gx, gy)
        yaw_err  = norm_angle(self.repark_yaw_goal - cur_yaw)
        x_err    = gx - cx
        if remaining <= 0.04:
            cond = abs(yaw_err) < self.PREC_FINISH_YAW and abs(x_err) < 0.008 \
                   and remaining < self.PREC_FINISH_DIST
            if cond:
                now = self.get_clock().now().nanoseconds * 1e-9
                if self.prec_ok_since is None:
                    self.prec_ok_since = now
                elif now - self.prec_ok_since >= self.PREC_HOLD_S:
                    self._pub(self.limo1_cmd, 0, 0)
                    self.get_logger().info('🏁 재주차 완료!')
                    self._transition_to(MissionState.FINISH); return
            else:
                self.prec_ok_since = None
        desired_yaw = self.repark_yaw_goal + self.PREC_KP_LAT * x_err
        ang = float(np.clip(self.PREC_KP_YAW * norm_angle(desired_yaw - cur_yaw), -1.5, 1.5))
        lin = float(np.clip(remaining * 0.4, 0.010, 0.022))
        self._pub(self.limo1_cmd, lin, ang)

    # ── 유틸 ─────────────────────────────────────────────────────────
    def _pub(self, pub, lin: float, ang: float):
        m = Twist(); m.linear.x = float(lin); m.angular.z = float(ang); pub.publish(m)

    def _transition_to(self, s: MissionState):
        self.get_logger().info(f'🔀 {self.state.name} → {s.name}')
        self.state = s
        if s == MissionState.LIMO1_REPARK:
            self.park_stage = ParkStage.APPROACH
            self.approach_target = None
            self.prec_ok_since = None


def main(args=None):
    rclpy.init(args=args)
    node = RemoteParkingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6-9. `remote_parking_manager/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>remote_parking_manager</name>
  <version>0.1.0</version>
  <description>Remote parking master FSM node</description>
  <maintainer email="you@example.com">Jihyun</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>gazebo_msgs</exec_depend>
  <exec_depend>numpy</exec_depend>
</package>
```

### 6-10. `remote_parking_manager/setup.py`

```python
from setuptools import setup, find_packages

package_name = 'remote_parking_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mission_manager = remote_parking_manager.mission_manager:main',
        ],
    },
)
```

### 6-11. `remote_parking_manager/setup.cfg`

```ini
[develop]
script_dir=$base/lib/remote_parking_manager
[install]
install_scripts=$base/lib/remote_parking_manager
```

---

## 7. 빌드 & 실행

### 빌드

```bash
cd ~/remote_parking_ws
source /opt/ros/humble/setup.bash
source ~/parking_ws/install/setup.bash      # limo_car, my_valet_parking
source ~/LIMO_platform/install/setup.bash   # 필요시
colcon build --symlink-install
source install/setup.bash
```

### 실행 순서

```bash
# ─── Terminal 1: Gazebo 서버 (1회만) ───────────────────────────────
ros2 launch remote_parking_world world_server.launch.py gui:=true

# ─── Terminal 2: 로봇 스폰 + 노드 (테스트마다) ─────────────────────
ros2 launch remote_parking_world spawn_robots.launch.py

# ─── Terminal 3: 미션 시작 ─────────────────────────────────────────
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'

# ─── 재시도 (Gazebo 재시작 없이 ~0.5초) ───────────────────────────
bash ~/remote_parking_ws/src/remote_parking_world/scripts/reset_robots.sh teleport
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```

### 상태 모니터링

```bash
# FSM 상태
ros2 topic echo /remote_parking/status

# 슬롯 탐지 여부
ros2 topic echo /limo1/target_slot

# 로봇 위치
ros2 topic echo /limo1/odom --no-arr
ros2 topic echo /limo2/odom --no-arr
```

---

## 8. 수정된 버그 전체 목록

| # | 파일 | 버그 내용 | 수정 |
|---|------|-----------|------|
| 1 | world_server.launch.py | `ament_python_file_backend` 존재하지 않는 패키지 import | 제거 |
| 2 | world_server.launch.py | `IfCondition` import 누락 | 추가 |
| 3 | spawn_robots.launch.py | 동일 잘못된 패키지 import | `ament_index_python` 으로 수정 |
| 4 | spawn_robots.launch.py | `import xacro` + `make_robot_description()` — launch 파일에서 호출 구조 오류 | 함수 제거, `Command` 치환자 유지 |
| 5 | spawn_robots.launch.py | xacro 인자 `robot_name` → `robot_namespace` (URDF 파라미터명 불일치) | 수정 |
| 6→12 | spawn_robots.launch.py | spawn z=`0.065` → `0.145` (base_link root, 바퀴 높이 계산 오류) | 수정 |
| 7→14 | limo_with_rear_camera.urdf.xacro | include: `urdf/limo_ackerman_base.xacro` (Gazebo 플러그인 없음) | `gazebo/ackermann_with_sensor.xacro` 로 수정 |
| 8 | limo_with_rear_camera.urdf.xacro | `<xacro:arg name="robot_name">` → `robot_namespace` | 수정 |
| 9→15 | limo_with_rear_camera.urdf.xacro | 후방 카메라 `<namespace>` hardcoding → 이중 namespace 충돌 | 비워서 spawn이 처리하도록 변경 |
| 10 | mission_manager.py | 장애물 감지 각도 `±20°` → `±30°` (`limo_valet_parking_node.py` 기준) | 수정 |
| 11 | spawn_robots.launch.py | `dynamic_tracker_node` 에 `allow_fallback_without_baseline` 누락 | 추가 |
| 12 | reset_robots.sh | 리셋 위치 z=`0.065` → `0.145` | 수정 |
| 13 | spawn_robots.launch.py | spawn 명령에 `-robot_namespace limo1/limo2` 누락 (Gazebo 플러그인 namespacing 실패) | 추가 |

---

## 9. 알려진 제한 및 주의사항

### TF 충돌 (비치명적)
LIMO 1·2가 동일 URDF 링크명(`base_link`, `laser_link` 등)을 공유하므로 `/tf` 트리에서 충돌 발생.  
**영향**: RViz에서 두 로봇이 겹쳐 보일 수 있음.  
**비영향**: 오도메트리 기반 제어(`/limo1/odom`, `/limo2/odom`)는 정상 동작.  
**해결책 (선택)**: 각 robot_state_publisher에 `frame_prefix` 또는 `tf_prefix` 파라미터 추가.

### IMU 토픽 하드코딩 (`sensor.xacro` 확인)
```xml
<topicName>/limo/imu</topicName>  <!-- 절대 경로, namespace 미적용 -->
```
LIMO 1·2 양쪽이 `/limo/imu` 로 발행 → 충돌.  
**영향**: IMU 미사용 시 무관. EKF 통합 시 토픽 분리 필요.

### base_footprint 미존재 (`ackermann.xacro` 확인)
```xml
<!-- <link name="base_footprint"/> -->  ← 주석 처리됨
<robot_base_frame>base_footprint</robot_base_frame>  ← 플러그인 설정에는 존재
```
플러그인이 존재하지 않는 프레임을 참조하므로 TF 경고 발생 가능.  
**해결책 (선택)**: `ackermann.xacro` 의 주석을 해제하고 spawn z를 재계산.

### 시뮬레이터 속도 최적화 효과
| 항목 | 기존 (start_parking_limo_master.sh) | 신규 |
|------|--------------------------------------|------|
| 매 실행 colcon build | ~10분 | **없음** |
| gzserver sleep | 15초 | 서비스 대기 |
| 물리 엔진 Hz | 1000 Hz | **500 Hz** |
| 그림자 렌더링 | ON | **OFF** |
| 재시도 소요 시간 | ~15분 | **~0.5초** |
