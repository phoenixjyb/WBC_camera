cmake_minimum_required(VERSION 3.5)
project(mobile_arm_whole_body)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(msg_files
  "msg/BaseProfile.msg"
  "msg/CoordinatorStatus.msg"
  "msg/EndEffectorReference.msg"
  "msg/StageInfo.msg"
  "msg/SyncStatus.msg"
  "msg/WholeBodyCommand.msg"
  "msg/WholeBodyMetrics.msg"
)

set(action_files
  "action/ExecuteWholeBody.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs trajectory_msgs
)

install(DIRECTORY urdf srdf config launch scripts meshes
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

include_directories(include)

add_library(wbc_moveit_solver
  src/moveit_ik_solver.cpp)
ament_target_dependencies(wbc_moveit_solver
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs)
target_include_directories(wbc_moveit_solver PUBLIC include)

add_executable(whole_body_input_node src/whole_body_input_node.cpp)
ament_target_dependencies(whole_body_input_node
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs)

add_executable(whole_body_coordinator_node src/whole_body_coordinator_node.cpp)
ament_target_dependencies(whole_body_coordinator_node
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs)

add_executable(whole_body_ramp_planner_node src/whole_body_ramp_planner_node.cpp)
ament_target_dependencies(whole_body_ramp_planner_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  trajectory_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(whole_body_input_node ${cpp_typesupport_target})
target_link_libraries(whole_body_coordinator_node ${cpp_typesupport_target} wbc_moveit_solver)
target_link_libraries(whole_body_ramp_planner_node ${cpp_typesupport_target} wbc_moveit_solver)

install(TARGETS
  wbc_moveit_solver
  whole_body_input_node
  whole_body_coordinator_node
  whole_body_ramp_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
