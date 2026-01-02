cmake_minimum_required(VERSION 3.10)

project(rparm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  gazebo
  ros2_control
  config
  worlds
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
