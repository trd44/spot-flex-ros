cmake_minimum_required(VERSION 3.8)
project(spot_flex_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_virtual_display.sh
  scripts/stop_virtual_display.sh
  scripts/virtual_display_env.sh
  scripts/launch_gazebo_vnc.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
