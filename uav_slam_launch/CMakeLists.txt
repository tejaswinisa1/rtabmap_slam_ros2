cmake_minimum_required(VERSION 3.8)
project(uav_slam_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
