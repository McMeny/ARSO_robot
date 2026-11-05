cmake_minimum_required(VERSION 3.8)
project(walker_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hazard.msg"
  "msg/HazardArray.msg"
  "msg/ProbeResult.msg"
  "msg/WalkerTrajectory.msg"
  "msg/Tentacles.msg"
  "msg/GripState.msg"
  "msg/SteeringStatus.msg"
  "msg/HandleForce.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
