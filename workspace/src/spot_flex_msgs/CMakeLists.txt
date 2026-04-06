cmake_minimum_required(VERSION 3.8)
project(spot_flex_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FetchItem.action"
  "action/FindObject.action"
  "action/ExecutePolicy.action"
  "srv/GetPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
