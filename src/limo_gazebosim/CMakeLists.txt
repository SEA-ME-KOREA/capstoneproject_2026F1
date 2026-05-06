cmake_minimum_required(VERSION 3.8)
project(limo_gazebosim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds scripts models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
