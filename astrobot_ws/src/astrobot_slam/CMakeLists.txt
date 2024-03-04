cmake_minimum_required(VERSION 3.8)
project(astrobot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
