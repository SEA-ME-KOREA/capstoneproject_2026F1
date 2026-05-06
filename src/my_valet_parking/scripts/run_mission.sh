#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo "[INFO] Cleaning up old mission processes..."
pkill -9 -f "ros2 run my_valet_parking parking_detector.py" 2>/dev/null || true
pkill -9 -f "ros2 run my_valet_parking limo_parking_planner.py" 2>/dev/null || true
pkill -9 -f "parking_detector.py" 2>/dev/null || true
pkill -9 -f "limo_parking_planner.py" 2>/dev/null || true

chmod +x "${WS_DIR}/src/my_valet_parking/scripts/parking_detector.py"
chmod +x "${WS_DIR}/src/my_valet_parking/scripts/limo_parking_planner.py"

source "${WS_DIR}/install/setup.bash"

cleanup() {
  jobs -p | xargs -r kill 2>/dev/null || true
}

trap cleanup EXIT

echo "[INFO] Starting Perception Node..."
ros2 run my_valet_parking parking_detector.py &
DETECTOR_PID=$!

sleep 2

echo "[INFO] Starting Planner Node..."
ros2 run my_valet_parking limo_parking_planner.py &
PLANNER_PID=$!

echo "parking_detector PID: ${DETECTOR_PID}"
echo "limo_parking_planner PID: ${PLANNER_PID}"

wait
