cmake_minimum_required(VERSION 3.8)
project(pal_robotiq_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes robots urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
