#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════
# reset_robots.sh
# ★ Gazebo 를 재시작하지 않고 로봇들의 위치만 초기화
# ══════════════════════════════════════════════════════════════════════
#
# 사용법:
#   bash scripts/reset_robots.sh          # 기본 (teleport)
#   bash scripts/reset_robots.sh teleport  # set_entity_state로 텔레포트 (더 빠름)
#
# ══════════════════════════════════════════════════════════════════════

set -e

MODE=${1:-"teleport"}   # 기본: teleport (빠른 리셋)

# ── 초기 포즈 상수 (spawn_robots.launch.py 와 동일) ──────────────────
# LIMO 1: x=0.225, y=0.35, z=0.145, yaw=0.0 (차로와 평행, +X 방향)
LIMO1_POSE="{position: {x: 0.225, y: 0.35, z: 0.145}, \
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
ZERO_TWIST="{linear: {x: 0.0, y: 0.0, z: 0.0}, \
              angular: {x: 0.0, y: 0.0, z: 0.0}}"

# LIMO 2 (A3): x=0.225, y=0.985, z=0.145, yaw=-π/2
LIMO2_POSE="{position: {x: 0.225, y: 0.985, z: 0.145}, \
             orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"

# LIMO A2: x=-0.225, y=0.985, z=0.145, yaw=-π/2
LIMOA2_POSE="{position: {x: -0.225, y: 0.985, z: 0.145}, \
             orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"

# LIMO A4: x=0.675, y=0.985, z=0.145, yaw=-π/2
LIMOA4_POSE="{position: {x: 0.675, y: 0.985, z: 0.145}, \
             orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"

LIMO1_INIT_ARGS="-x 0.225 -y 0.35  -z 0.145 -R 0 -P 0 -Y 0.0"
LIMO2_INIT_ARGS="-x 0.225 -y 0.985 -z 0.145 -R 0 -P 0 -Y -1.5708"
LIMOA2_INIT_ARGS="-x -0.225 -y 0.985 -z 0.145 -R 0 -P 0 -Y -1.5708"
LIMOA4_INIT_ARGS="-x 0.675 -y 0.985 -z 0.145 -R 0 -P 0 -Y -1.5708"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Remote Parking — Robot Reset (MODE: $MODE)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── 미션 매니저 FSM 리셋 (서비스 호출 — 응답까지 대기하므로 확실) ──
echo "[1/3] 미션 매니저 IDLE 상태로 리셋 + 전체 로봇 정지..."
ros2 service call /reset_parking std_srvs/srv/Trigger '{}' > /dev/null

sleep 0.3

if [ "$MODE" = "teleport" ]; then
  # ════════════════════════════════════════════════════════════════
  # TELEPORT MODE: set_entity_state 로 즉시 위치 이동
  # ════════════════════════════════════════════════════════════════
  echo "[2/3] 텔레포트 진행 중..."
  ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: 'limo1', pose: $LIMO1_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}" > /dev/null

  ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: 'limo2', pose: $LIMO2_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}" > /dev/null

  ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: 'limo_a2', pose: $LIMOA2_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}" > /dev/null

  ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: 'limo_a4', pose: $LIMOA4_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}" > /dev/null

  echo "[3/3] 텔레포트 후 정지 재확인..."
  ros2 service call /reset_parking std_srvs/srv/Trigger '{}' > /dev/null

  echo ""
  echo "✅ 텔레포트 리셋 완료"

elif [ "$MODE" = "respawn" ]; then
  # ════════════════════════════════════════════════════════════════
  # RESPAWN MODE: delete → respawn
  # ════════════════════════════════════════════════════════════════
  echo "[2/3] 기존 로봇 삭제..."
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo1'}" 2>/dev/null || true
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo2'}" 2>/dev/null || true
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo_a2'}" 2>/dev/null || true
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo_a4'}" 2>/dev/null || true

  sleep 2

  echo "[3/3] 로봇 재스폰..."
  ros2 run gazebo_ros spawn_entity.py -entity limo1 -topic /limo1/robot_description -robot_namespace limo1 $LIMO1_INIT_ARGS &
  ros2 run gazebo_ros spawn_entity.py -entity limo2 -topic /limo2/robot_description -robot_namespace limo2 $LIMO2_INIT_ARGS &
  ros2 run gazebo_ros spawn_entity.py -entity limo_a2 -topic /limo_a2/robot_description -robot_namespace limo_a2 $LIMOA2_INIT_ARGS &
  ros2 run gazebo_ros spawn_entity.py -entity limo_a4 -topic /limo_a4/robot_description -robot_namespace limo_a4 $LIMOA4_INIT_ARGS &

  wait

  echo ""
  echo "✅ 재스폰 리셋 완료 (~5초)"

else
  echo "❌ 알 수 없는 MODE: $MODE"
  echo "   사용법: $0 [teleport|respawn]"
  exit 1
fi

echo ""
echo "  다음 테스트를 시작하려면:"
echo "  ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'"
