cmake_minimum_required(VERSION 3.16)
project(autolivery_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/PayloadState.msg"
  "msg/MissionStatus.msg"
  "msg/Weather.msg"
  "srv/PlanPath.srv"
  "action/Deliver.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
