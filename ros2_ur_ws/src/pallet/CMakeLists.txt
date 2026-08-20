cmake_minimum_required(VERSION 3.8)

project(pallet)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
