cmake_minimum_required(VERSION 3.8)
project(astrobot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config include src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
