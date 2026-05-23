#!/bin/bash
CAR=${1:-a3}
echo "[$CAR] 차량 출차를 선택합니다..."
ros2 service call /select_exit_car/$CAR std_srvs/srv/Trigger '{}'
