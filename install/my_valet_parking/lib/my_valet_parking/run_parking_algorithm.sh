#!/bin/bash

PIDS=()

cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo
    echo ">>> 주차 알고리즘 노드 종료 중..."
    ros2 topic pub --once /limo1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null
        fi
    done
    wait "${PIDS[@]}" 2>/dev/null
}

trap cleanup SIGINT SIGTERM EXIT

source /opt/ros/humble/setup.bash
source ~/parking_ws/install/setup.bash

echo "======================================================"
echo " [LIMO 1 & 2] Multi-Node 자율 주차 시나리오 가동"
echo " - Tracker   : /limo1/scan + /limo1/odom -> /limo1/target_slot"
echo " - Controller: /limo1/target_slot 수신 후 PULL_FORWARD -> 주차"
echo " - Limo 2    : 방향키 텔레옵으로 사용자가 직접 출차"
echo "======================================================"
echo
echo " [LIMO2 방향키 조작] 다음 명령을 별도 터미널에서 실행:"
echo
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/parking_ws/install/setup.bash"
echo "   ros2 run my_valet_parking limo2_arrow_teleop"
echo
echo "   ↑ : 전진(+0.08 m/s)   ↓ : 후진(-0.08 m/s)"
echo "   ← : 좌회전(+0.3 rad/s) → : 우회전(-0.3 rad/s)"
echo "   Space / 키 떼면 정지,  q 종료"
echo "======================================================"

ros2 run my_valet_parking dynamic_tracker_node --ros-args \
    -p scan_topic:=/limo1/scan \
    -p odom_topic:=/limo1/odom \
    -p target_topic:=/limo1/target_slot \
    -p odom_frame:=odom &
TRACKER_PID=$!
PIDS+=("${TRACKER_PID}")

ros2 run my_valet_parking limo_valet_parking_node --ros-args \
    -r __ns:=/limo1 \
    -r odom:=/limo1/odom \
    -r scan:=/limo1/scan \
    -r rgb/image_raw:=/limo1/rgb/image_raw \
    -r depth/image_raw:=/limo1/depth_camera/depth/image_raw \
    -r target_slot:=/limo1/target_slot \
    -r cmd_vel:=/limo1/cmd_vel &
CONTROLLER_PID=$!
PIDS+=("${CONTROLLER_PID}")

echo
echo ">>> dynamic_tracker_node 실행 (PID=${TRACKER_PID})"
echo ">>> limo_valet_parking_node 실행 (PID=${CONTROLLER_PID})"
echo ">>> Ctrl+C를 누르면 두 노드를 모두 종료합니다."
echo

wait
