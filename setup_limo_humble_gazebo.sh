#!/usr/bin/env bash

set -euo pipefail

WS_DIR="${HOME}/LIMO_simulation"
SRC_DIR="${WS_DIR}/src"
REPO_DIR="${SRC_DIR}/limo_ros2"
REPO_URL="https://github.com/agilexrobotics/limo_ros2.git"
REPO_BRANCH="humble"

if [[ ! -d "${WS_DIR}" ]]; then
  echo "Workspace not found: ${WS_DIR}" >&2
  exit 1
fi

mkdir -p "${SRC_DIR}"

echo "[1/6] Remove conflicting repositories"
rm -rf "${SRC_DIR}/limo_ros2" "${SRC_DIR}/ugv_gazebo_sim"

echo "[2/6] Clone limo_ros2 (${REPO_BRANCH})"
git clone --branch "${REPO_BRANCH}" --single-branch "${REPO_URL}" "${REPO_DIR}"

echo "[3/6] Install ROS 2 Humble Gazebo control dependencies"
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-position-controllers \
  ros-humble-velocity-controllers \
  ros-humble-xacro

echo "[4/6] Expose nested ROS packages to workspace/src when needed"
mapfile -t nested_packages < <(
  find "${REPO_DIR}" \
    -mindepth 2 \
    -maxdepth 6 \
    -type f \
    -name package.xml \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    ! -path '*/log/*' \
    -printf '%h\n' | sort -u
)

for pkg_dir in "${nested_packages[@]}"; do
  pkg_name="$(basename "${pkg_dir}")"

  if [[ "${pkg_dir}" == "${REPO_DIR}" ]]; then
    continue
  fi

  if [[ ! -e "${SRC_DIR}/${pkg_name}" ]]; then
    ln -s "${pkg_dir}" "${SRC_DIR}/${pkg_name}"
    echo "  linked: ${SRC_DIR}/${pkg_name} -> ${pkg_dir}"
  fi
done

echo "[5/6] Clean colcon cache and rebuild"
rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"

source /opt/ros/humble/setup.bash
cd "${WS_DIR}"
colcon build --symlink-install

echo "[6/6] Source workspace and print launch command"
source "${WS_DIR}/install/setup.bash"

cat <<'EOF'

Environment is ready.
Run:
  cd ~/LIMO_simulation
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch limo_gazebosim limo_ackermann_rviz.launch.py

EOF
