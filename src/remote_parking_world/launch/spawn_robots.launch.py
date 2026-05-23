#!/usr/bin/env python3
"""
spawn_robots.launch.py
══════════════════════════════════════════════════════════════
★ 이 launch 파일은 "테스트마다" 실행합니다.
  - LIMO 1 (사용자 차) 스폰: 차로에서 A3 앞 이중주차 상태
  - LIMO 2 (A3 슬롯 주차 상태)
  - LIMO A2 (A2 슬롯 주차 상태)
  - LIMO A4 (A4 슬롯 주차 상태)
  - robot_state_publisher × 4 시작
  - 모든 ROS2 제어 노드 시작
  - remote_parking_manager (미션 마스터 FSM) 시작

전제조건:
  world_server.launch.py 가 이미 실행 중이어야 합니다.

초기 포즈:
  LIMO 1: (x=0.225, y=0.35,  z=0.145, yaw=0)     → A3 차선 근처 이중주차 (차로와 평행, +X 방향)
  LIMO 2: (x=0.225, y=0.985, z=0.145, yaw=-π/2)  → A3 슬롯 주차
  LIMO A2: (x=-0.225, y=0.985, z=0.145, yaw=-π/2) → A2 슬롯 주차
  LIMO A4: (x=0.675, y=0.985, z=0.145, yaw=-π/2)  → A4 슬롯 주차
══════════════════════════════════════════════════════════════
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# ── 초기 스폰 좌표 ─────────────────────────────────────────────────
LIMO1_INIT = dict(x='0.225',  y='0.35',  z='0.145',
                  R='0', P='0', Y='0.0')      # yaw = 0 (차로와 평행, A3 차선 근처)
LIMO2_INIT = dict(x='0.225',  y='0.985', z='0.145',
                  R='0', P='0', Y='-1.5708')  # yaw = -π/2
LIMOA2_INIT = dict(x='-0.225', y='0.985', z='0.145',
                   R='0', P='0', Y='-1.5708') # yaw = -π/2
LIMOA4_INIT = dict(x='0.675',  y='0.985', z='0.145',
                   R='0', P='0', Y='-1.5708') # yaw = -π/2

def generate_launch_description():
    pkg_world = get_package_share_directory('remote_parking_world')
    pkg_manager = get_package_share_directory('remote_parking_manager')

    limo_xacro = os.path.join(pkg_world, 'urdf', 'limo_kinematic.urdf.xacro')

    # ── robot_description 생성 (Command 치환자 → 런타임에 xacro 실행) ─
    def make_desc(ns):
        return ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', limo_xacro,
                f' robot_namespace:={ns}'
            ]),
            value_type=str
        )

    limo1_desc = make_desc('limo1')
    limo2_desc = make_desc('limo2')
    limoa2_desc = make_desc('limo_a2')
    limoa4_desc = make_desc('limo_a4')

    def make_group(ns, desc):
        return GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': desc,
                    'use_sim_time': True,
                    'publish_frequency': 50.0,
                }],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ])

    limo1_group = make_group('limo1', limo1_desc)
    limo2_group = make_group('limo2', limo2_desc)
    limoa2_group = make_group('limo_a2', limoa2_desc)
    limoa4_group = make_group('limo_a4', limoa4_desc)

    def make_spawn(entity_name, init_dict):
        return TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', entity_name,
                        '-topic', f'/{entity_name}/robot_description',
                        '-robot_namespace', entity_name,
                        '-x', init_dict['x'], '-y', init_dict['y'],
                        '-z', init_dict['z'],
                        '-R', init_dict['R'], '-P', init_dict['P'],
                        '-Y', init_dict['Y'],
                        '-timeout', '120.0',
                    ],
                    output='screen',
                )
            ],
        )

    spawn_limo1 = make_spawn('limo1', LIMO1_INIT)
    spawn_limo2 = make_spawn('limo2', LIMO2_INIT)
    spawn_limoa2 = make_spawn('limo_a2', LIMOA2_INIT)
    spawn_limoa4 = make_spawn('limo_a4', LIMOA4_INIT)

    # ════════════════════════════════════════════════════════════════
    # Dynamic Tracker Node (LiDAR 슬롯 탐지 - parking_ws 재활용)
    # ════════════════════════════════════════════════════════════════
    dynamic_tracker = TimerAction(
        period=5.0,  # 스폰 완료 후 시작
        actions=[
            Node(
                package='my_valet_parking',
                executable='dynamic_tracker_node',
                name='dynamic_tracker_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                parameters=[{
                    'use_sim_time': True,
                    'scan_topic':   '/limo1/scan',
                    'odom_topic':   '/limo1/odom',
                    'target_topic': '/limo1/target_slot',
                    'odom_frame':   'odom',
                    # 슬롯 감지 파라미터
                    'occupied_distance_m':         0.45,
                    'clear_distance_m':            0.65,
                    'baseline_capture_distance_m': 0.75,
                    'clear_hold_s':                0.70,
                    'slot_center_offset_m':        0.30,
                    'allow_fallback_without_baseline': True,
                    'fallback_target_x':           0.225,
                    'fallback_target_y':           0.985,
                    'target_yaw_rad':             -1.5708,  # -π/2 (A3 슬롯 방향)
                    'side_scan_center_deg':        90.0,
                    'side_scan_half_deg':          8.0,
                    'min_clear_ratio':             0.25,
                    'track_box_length_m':          0.35,
                    'track_box_width_m':           0.30,
                    'publish_period_s':            0.10,
                }],
            )
        ],
    )

    # ════════════════════════════════════════════════════════════════
    # Remote Parking Manager (마스터 FSM)
    # ════════════════════════════════════════════════════════════════
    mission_manager = TimerAction(
        period=6.0,  # 모든 노드 준비 후 마지막 시작
        actions=[
            Node(
                package='remote_parking_manager',
                executable='mission_manager',
                name='remote_parking_manager',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'use_sim_time': True,
                    # LIMO 1 초기/목표 좌표
                    'limo1_init_x':   0.225,
                    'limo1_init_y':   0.35,
                    'limo1_init_yaw': 0.0,
                    # LIMO 1 후진 목표 (차로 서쪽으로 직선 후진)
                    'limo1_evade_x':  -0.5,
                    'limo1_evade_y':   0.35,
                    # LIMO 2 (기본) 출차 목표 (주차장 출구)
                    'limo2_exit_x':   3.0,
                    'limo2_exit_y':   0.0,
                    # 재주차 목표 (초기화시 A3. 이후 코드에서 동적변경됨)
                    'repark_x':  0.225,
                    'repark_y':  0.985,
                    'repark_yaw': 1.5708,
                    # 속도 제한
                    'max_linear_speed':  0.05,
                    'max_angular_speed': 0.8,
                }],
            )
        ],
    )

    # ════════════════════════════════════════════════════════════════
    # RViz (선택적 시각화)
    # ════════════════════════════════════════════════════════════════
    rviz_config = os.path.join(pkg_world, 'config', 'remote_parking.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # 1단계: robot_state_publisher 시작
        limo1_group,
        limo2_group,
        limoa2_group,
        limoa4_group,
        # 2단계: 3초 후 Gazebo 스폰
        spawn_limo1,
        spawn_limo2,
        spawn_limoa2,
        spawn_limoa4,
        # 3단계: 5초 후 제어 노드
        dynamic_tracker,
        # 4단계: 6초 후 마스터 FSM
        mission_manager,
        # 시각화
        rviz_node,
    ])
