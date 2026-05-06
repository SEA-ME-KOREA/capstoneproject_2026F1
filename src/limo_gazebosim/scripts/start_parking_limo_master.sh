#!/usr/bin/env bash

set -e

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_ROOT="${HOME}/LIMO_simulation"
WS_SETUP="${HOME}/LIMO_simulation/install/setup.bash"
WORLD_PATH="${HOME}/LIMO_simulation/install/limo_gazebosim/share/limo_gazebosim/worlds/parking_lot_scaled_vehicles.world"
LOG_DIR="${HOME}/LIMO_simulation/log"
GAZEBO_LOG="${LOG_DIR}/gzserver_debug.log"
BUILD_LOG="${LOG_DIR}/colcon_build.log"
LIMO_DESCRIPTION_LOG="${LOG_DIR}/limo_description.log"
GZCLIENT_LOG="${LOG_DIR}/gzclient.log"
SPAWN_LOG="${LOG_DIR}/spawn_entity.log"
SPAWN_SERVICE="/spawn_entity"
SPAWN_TIMEOUT_SECONDS=60
ROBOT_DESCRIPTION_TOPIC="/robot_description"

mkdir -p "${LOG_DIR}"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

echo "[1/8] Clearing existing gazebo, limo, and rviz processes..."
for pattern in gazebo limo rviz; do
  while read -r pid; do
    if [ -n "${pid}" ] && [ "${pid}" != "$$" ]; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done
sleep 2

echo "[2/8] Removing build, install, and log directories for a clean rebuild..."
cd "${WS_ROOT}"
rm -rf build/ install/ log/
mkdir -p "${LOG_DIR}"

echo "[3/8] Building workspace with colcon build --symlink-install..."
colcon build --symlink-install >"${BUILD_LOG}" 2>&1

if [ ! -f "${WS_SETUP}" ]; then
  echo "Missing workspace setup after build: ${WS_SETUP}" >&2
  exit 1
fi

if [ ! -f "${WORLD_PATH}" ]; then
  echo "Missing world file after build: ${WORLD_PATH}" >&2
  exit 1
fi

echo "[4/8] Sourcing workspace overlay..."
source "${WS_SETUP}"

PACKAGE_MODEL_PATH="${HOME}/LIMO_simulation/install/limo_gazebosim/share/limo_gazebosim/models"
export GAZEBO_MODEL_PATH="${HOME}/.gazebo/models:${PACKAGE_MODEL_PATH}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

echo "[INFO] Starting Step 1: gzserver..."
sleep 2
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && export GAZEBO_MODEL_PATH='${GAZEBO_MODEL_PATH}' && ros2 launch gazebo_ros gazebo.launch.py gui:=false server:=true init:=true factory:=true verbose:=true world:='${WORLD_PATH}'" \
  >"${GAZEBO_LOG}" 2>&1 < /dev/null &
GAZEBO_LAUNCH_PID=$!
sleep 15

echo "[INFO] Starting Step 2: gzclient..."
sleep 2
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && gzclient" \
  >"${GZCLIENT_LOG}" 2>&1 < /dev/null &
GZCLIENT_PID=$!
sleep 5

echo "[INFO] Starting Step 3: robot_state_publisher and spawn_entity..."
existing_rsp_count=$(bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 node list 2>/dev/null | grep -Fxc '/robot_state_publisher' || true")
if [ "${existing_rsp_count}" -gt 1 ]; then
  echo "[WARN] Multiple /robot_state_publisher nodes detected (${existing_rsp_count}); pruning duplicates before launch."
  while read -r pid; do
    if [ -n "${pid}" ] && [ "${pid}" != "$$" ]; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -f "robot_state_publisher" 2>/dev/null || true)
  sleep 2
  existing_rsp_count=0
fi

if [ "${existing_rsp_count}" -eq 0 ]; then
  sleep 2
  setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 launch limo_car ackermann.launch.py use_sim_time:=true" \
    >"${LIMO_DESCRIPTION_LOG}" 2>&1 < /dev/null &
  LIMO_DESCRIPTION_PID=$!
  sleep 5
else
  echo "[INFO] Existing /robot_state_publisher detected (${existing_rsp_count}); skipping duplicate launch."
  LIMO_DESCRIPTION_PID=""
fi

echo "[6/8] Waiting up to ${SPAWN_TIMEOUT_SECONDS}s for ${SPAWN_SERVICE} service..."
spawn_ready=0
for ((i=1; i<=SPAWN_TIMEOUT_SECONDS; i++)); do
  if ros2 service list 2>/dev/null | grep -Fxq "${SPAWN_SERVICE}"; then
    spawn_ready=1
    break
  fi
  sleep 1
done

if [ "${spawn_ready}" -ne 1 ]; then
  echo "Timed out waiting for ${SPAWN_SERVICE}. Check ${GAZEBO_LOG}" >&2
  exit 1
fi

topic_ready=0
for ((i=1; i<=30; i++)); do
  if ros2 topic list 2>/dev/null | grep -Fxq "${ROBOT_DESCRIPTION_TOPIC}"; then
    topic_ready=1
    break
  fi
  sleep 1
done

if [ "${topic_ready}" -ne 1 ]; then
  echo "Timed out waiting for ${ROBOT_DESCRIPTION_TOPIC}. Check ${LIMO_DESCRIPTION_LOG}" >&2
  exit 1
fi

sleep 2
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity limo_physics_fixed -x 0.1897 -y -0.8162 -z 0.45 -Y 1.565 -timeout 120.0" \
  >"${SPAWN_LOG}" 2>&1 < /dev/null &
SPAWN_PID=$!
sleep 5

echo "Launch sequence submitted."
echo "Gazebo launch PID: ${GAZEBO_LAUNCH_PID}"
echo "LIMO description PID: ${LIMO_DESCRIPTION_PID}"
echo "gzclient PID: ${GZCLIENT_PID}"
echo "Spawn PID: ${SPAWN_PID}"
echo "Logs: ${LOG_DIR}"
echo "[INFO] Simulation Environment Ready."

wait
