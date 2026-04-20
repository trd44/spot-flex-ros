cmake_minimum_required(VERSION 3.8)
project(spot_flex_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_moveit_vnc.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
