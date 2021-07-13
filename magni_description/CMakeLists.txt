cmake_minimum_required(VERSION 3.5)
project(magni_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY ros2_control urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()