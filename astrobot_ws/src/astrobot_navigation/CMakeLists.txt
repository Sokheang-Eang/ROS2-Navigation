cmake_minimum_required(VERSION 3.8)
project(astrobot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
