cmake_minimum_required(VERSION 3.8)
project(spot_flex_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FetchItem.action"
  "action/FindBoxGraspPoint.action"
  "action/FindCabinetHandle.action"
  "action/FindObject.action"
  "action/ExecutePolicy.action"
  "srv/GetPlan.srv"
  DEPENDENCIES std_msgs
)

ament_package()
