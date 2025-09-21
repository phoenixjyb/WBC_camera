cmake_minimum_required(VERSION 3.5)
project(mobile_arm_whole_body)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf config launch scripts meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
