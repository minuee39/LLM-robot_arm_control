cmake_minimum_required(VERSION 3.8)

project(pallet)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes scripts urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/pallet_follow_joint_trajectory_bridge.py
    scripts/pallet_isaac_gripperless_physics_test.py
    scripts/pallet_isaac_moveit_bridge.py
    scripts/run_pallet_isaac_gripperless_physics_test.sh
    scripts/run_pallet_isaac_moveit_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
