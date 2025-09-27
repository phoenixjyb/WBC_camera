cmake_minimum_required(VERSION 3.8)
project(mobile_arm_whole_body_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/CameraPoseTarget.msg"
  "msg/CameraPoseStatus.msg"
  "msg/TrackingPhaseState.msg"
)

set(srv_files
  "srv/PlanArmRamp.srv"
  "srv/PlanBaseRamp.srv"
  "srv/PlanTrackingSegment.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
