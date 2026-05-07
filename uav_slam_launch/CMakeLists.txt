cmake_minimum_required(VERSION 3.8)
project(uav_slam_launch)

# C++17 required for ROS2 Jazzy / Ubuntu 24.04
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/benchmark_uav.sh
  config/cyclonedds_env.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
