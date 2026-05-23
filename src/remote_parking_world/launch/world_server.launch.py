#!/usr/bin/env python3
"""
world_server.launch.py
══════════════════════════════════════════════════════════════
★ 이 launch 파일은 "1회만" 실행합니다.
  - gzserver (물리 엔진) 만 시작
  - 정적 월드(주차장 환경 + 정적 차량 7대) 로드
  - gzclient(GUI)는 선택적으로 별도 실행 가능 (gui:=true)

실행:
  # 서버만 시작 (헤드리스, 빠름)
  ros2 launch remote_parking_world world_server.launch.py

  # GUI 포함
  ros2 launch remote_parking_world world_server.launch.py gui:=true

이후 테스트 반복 시:
  ros2 launch remote_parking_world spawn_robots.launch.py   # 로봇 스폰
  bash scripts/reset_robots.sh                              # 위치 리셋 (Gazebo 재시작 불필요)
══════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition          # BUG #2 수정: import 추가
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# BUG #1 수정: 존재하지 않는 'ament_python_file_backend' import 제거
# FindPackageShare 로 대체 (이미 아래에서 사용 중)


def generate_launch_description():
    pkg_share = FindPackageShare('remote_parking_world')

    # ── GAZEBO 모델 경로 등록 ──────────────────────────────────────
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'remote_parking_static.world'
    ])

    # ── Arguments ─────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='true이면 gzclient(GUI) 도 함께 시작'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Gazebo 상세 로그 출력'
    )

    # ── 환경 변수 설정 ─────────────────────────────────────────────
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            gazebo_model_path,
            ':',
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]
    )

    # ── gzserver ──────────────────────────────────────────────────
    # 핵심 최적화:
    #   -s libgazebo_ros_factory.so  : spawn_entity 서비스
    #   -s libgazebo_ros_state.so    : set_entity_state 서비스(reset용)
    #   world 파일에 이미 물리 최적화 파라미터 포함
    gzserver_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
    )

    # ── gzclient (선택, gui:=true 일 때만) ────────────────────────
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        set_model_path,
        gui_arg,
        verbose_arg,
        gzserver_cmd,
        gzclient_cmd,
    ])
