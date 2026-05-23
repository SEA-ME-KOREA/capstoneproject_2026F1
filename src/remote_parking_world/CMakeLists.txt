cmake_minimum_required(VERSION 3.8)
project(remote_parking_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  launch
  scripts
  urdf
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/reset_robots.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
