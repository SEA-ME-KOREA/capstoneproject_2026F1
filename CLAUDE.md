# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble desktop full
- **Simulator**: Gazebo (classic)
- **Workspace root**: `~/LIMO_simulation` (the colcon workspace; this repo is the workspace root)

## Build & Source

```bash
# From the workspace root (~/LIMO_simulation):
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After sourcing, set the Gazebo model path so the parking-lot SDF models resolve:

```bash
export GAZEBO_MODEL_PATH=~/.gazebo/models:~/LIMO_simulation/install/limo_gazebosim/share/limo_gazebosim/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
```

## Running the Simulation

**Full parking-lot simulation (recommended — automated orchestration):**
```bash
bash src/limo_gazebosim/scripts/start_parking_limo_master.sh
```
This script: cleans and rebuilds the workspace, launches `gzserver`, `gzclient`, `robot_state_publisher`, and spawns the LIMO robot at the parking-lot pose.

**Minimal Gazebo + RViz launch (manual):**
```bash
ros2 launch limo_gazebosim limo_ackermann_rviz.launch.py
```
Spawns LIMO in `parking_lot_scaled_vehicles.world` with RViz.

**Empty-world Gazebo simulation:**
```bash
ros2 launch limo_car ackermann_gazebo.launch.py
```

**RViz-only URDF preview:**
```bash
ros2 launch limo_car display_ackermann.launch.py
```

**Manual teleoperation (once simulation is running):**
```bash
ros2 run rqt_robot_steering rqt_robot_steering
# or
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Running the Autonomous Mission Nodes

**Valet parking pipeline** (perception + planner, requires simulation running):
```bash
bash src/my_valet_parking/scripts/run_mission.sh
```
Starts `parking_detector.py` and `limo_parking_planner.py` as separate processes.

**Individual mission controllers** (via installed entry points):
```bash
ros2 run my_valet_parking limo_f1tenth_pure_pursuit
ros2 run my_valet_parking limo_evasion_controller
ros2 run my_valet_parking limo_lane_mission
```

## Running Tests

```bash
cd ~/LIMO_simulation
colcon test --packages-select my_valet_parking
colcon test-result --verbose
```

Tests in `src/my_valet_parking/test/` cover copyright, flake8 linting, and pep257 docstring style.

## Package Architecture

The workspace contains six source packages under `src/`:

| Package | Type | Purpose |
|---|---|---|
| `limo_msgs` | ament_cmake | Custom `LimoStatus.msg` for hardware status |
| `limo_base` | ament_cmake (C++) | Hardware driver: serial comms, base node, TF publisher |
| `limo_description` | ament_cmake | URDF/xacro robot description (diff-drive and ackermann variants) |
| `limo_car` | ament_cmake | Ackermann URDF/xacro with Gazebo plugins + launch files |
| `limo_gazebosim` | ament_cmake | Parking-lot worlds, scaled vehicle SDF models, orchestration scripts |
| `my_valet_parking` | ament_python | Autonomous mission nodes (Python) |

`src/tmp/limo_ros2_tmp/` is a cloned upstream reference snapshot; do not modify it.

### `my_valet_parking` Node Overview

All nodes subscribe to `/odom` (nav_msgs/Odometry), `/scan` (sensor_msgs/LaserScan), and `/rgb/image_raw` (sensor_msgs/Image), and publish `/cmd_vel` (geometry_msgs/Twist).

**`limo_f1tenth_pure_pursuit`** — Primary mission controller. State machine:
`IDLE → SCAN_INITIAL → EVADE_FORWARD → PURE_PURSUIT_RETURN → VISION_ALIGN → FINISH`
Drives 1 m forward (with obstacle avoidance), records the path, then reverses along the logged path using pure-pursuit. Final alignment uses OpenCV template matching (TM_CCOEFF_NORMED) fused with LiDAR distance comparison (10% tolerance threshold).

**`limo_evasion_controller`** — Simpler evasion-and-return controller. State machine:
`IDLE → SCAN_INITIAL → EVADE_FORWARD → WAIT_REAR → RETURN_HOME → VISION_ALIGN`
Uses smoothed acceleration ramping for commands. Returns via local-frame waypoint tracking.

**`limo_lane_mission`** — Lane-based alignment node. State machine:
`CAPTURE_INITIAL → DRIVE_FORWARD → DRIVE_BACKWARD → CORRECTION → COMPLETE`
Detects white parking lines via HSV thresholding + image moments; corrects lateral offset using pixel error.

**`ParkingDetector`** (`scripts/parking_detector.py`) — Perception node. Detects parking slot occupancy using Canny edges + Hough line segments. Publishes `MarkerArray` on `/parking/slot_markers` and slot states (`Int32MultiArray`) on `/parking/slot_states`. Slot grid: 3 rows × 10 slots each.

**`LimoParkingPlanner`** (`scripts/limo_parking_planner.py`) — Planner node. Subscribes to `/parking/slot_states` from `ParkingDetector`. Drives forward 1 m, records the first detected empty slot pose, reverses to that pose, then corrects with vision.

### Key Topics

| Topic | Direction | Type |
|---|---|---|
| `/cmd_vel` | Published by mission nodes | geometry_msgs/Twist |
| `/odom` | Subscribed | nav_msgs/Odometry |
| `/scan` | Subscribed | sensor_msgs/LaserScan |
| `/rgb/image_raw` | Subscribed | sensor_msgs/Image |
| `/parking/slot_states` | ParkingDetector → Planner | std_msgs/Int32MultiArray |
| `/parking/slot_markers` | Published by ParkingDetector | visualization_msgs/MarkerArray |
| `/debug/image_raw` | Published by vision nodes | sensor_msgs/Image |
| `/lookahead_marker` | Published by pure-pursuit | visualization_msgs/Marker |

### Robot Models

`limo_car/urdf/` contains the Ackermann URDF/xacro. `limo_description/urdf/` has both Ackermann and four-wheel-diff variants. The Gazebo `.gazebo` files attach `libgazebo_ros_ackermann_drive` and `libgazebo_ros_ray_sensor` plugins.

### Worlds and Models

`limo_gazebosim/worlds/parking_lot_scaled_vehicles.world` — primary simulation world with scaled-down car models (hatchback, prius hybrid, SUV). `limo_gazebosim/models/` contains the SDF + mesh assets; `scripts/scale_vehicle_models.py` and `setup_scaled_vehicle_models.py` generated the scaled variants.
