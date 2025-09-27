cmake_minimum_required(VERSION 3.8)
project(mobile_arm_whole_body_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mobile_arm_whole_body_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(fcl REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_launch_test(test/test_supervisor_handshake.py
    TIMEOUT 20
  )
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_srvs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  mobile_arm_whole_body_interfaces
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2_geometry_msgs
  tf2_eigen
  shape_msgs
  fcl
)

add_executable(trajectory_ingestor_node src/trajectory_ingestor_node.cpp)
ament_target_dependencies(trajectory_ingestor_node ${dependencies})

add_executable(whole_body_supervisor_node src/whole_body_supervisor_node.cpp)
ament_target_dependencies(whole_body_supervisor_node ${dependencies})

add_executable(arm_trajectory_publisher_node src/arm_trajectory_publisher_node.cpp)
ament_target_dependencies(arm_trajectory_publisher_node ${dependencies})

add_executable(base_motion_commander_node src/base_motion_commander_node.cpp)
ament_target_dependencies(base_motion_commander_node ${dependencies})

add_executable(planner_stub_node src/planner_stub_node.cpp)
ament_target_dependencies(planner_stub_node ${dependencies})

add_executable(whole_body_planner_node src/whole_body_planner_node.cpp)
ament_target_dependencies(whole_body_planner_node ${dependencies})

add_executable(base_planner_node src/base_planner_node.cpp)
ament_target_dependencies(base_planner_node ${dependencies})

add_executable(obstacle_manager_node src/obstacle_manager_node.cpp)
ament_target_dependencies(obstacle_manager_node ${dependencies})

add_library(trajectory_utils_lib src/trajectory_utils.cpp)
target_include_directories(trajectory_utils_lib PUBLIC include)
ament_target_dependencies(trajectory_utils_lib
  rclcpp
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs)

target_link_libraries(whole_body_supervisor_node trajectory_utils_lib)
target_link_libraries(whole_body_planner_node trajectory_utils_lib)

if(BUILD_TESTING)
  ament_add_gtest(test_trajectory_utils test/test_trajectory_utils.cpp)
  if(TARGET test_trajectory_utils)
    target_link_libraries(test_trajectory_utils trajectory_utils_lib)
    ament_target_dependencies(test_trajectory_utils
      rclcpp
      geometry_msgs
      nav_msgs
      trajectory_msgs
      tf2_geometry_msgs)
  endif()
endif()

install(TARGETS
  trajectory_ingestor_node
  whole_body_supervisor_node
  arm_trajectory_publisher_node
  base_motion_commander_node
  planner_stub_node
  whole_body_planner_node
  base_planner_node
  obstacle_manager_node
  trajectory_utils_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(trajectory_utils_lib)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs trajectory_msgs tf2_geometry_msgs)

ament_package()
