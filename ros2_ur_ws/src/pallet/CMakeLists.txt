cmake_minimum_required(VERSION 3.8)

project(pallet)

find_package(ament_cmake REQUIRED)

install(FILES SINGLE_AXIS_POSITION.md SINGLE_AXIS_GRAVITY.md COMMISSIONING_RESULT_20261022.md SINGLE_AXIS_CURRENT_TEST.md MOTOR_FEEDBACK_RVIZ.md HAND_GUIDING.md MASS_REVISED_MIGRATION.md DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY config launch meshes scripts urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/pallet_single_axis_gravity.py
    scripts/pallet_watchdog_test.py
    scripts/pallet_single_axis_current_test.py
    scripts/pallet_single_axis_position.py
    scripts/pallet_position_preflight.py
    scripts/pallet_current_limits.py
    scripts/pallet_multi_axis_position.py
    scripts/pallet_hold_all_motors.py
    scripts/pallet_restore_motor_defaults.py
    scripts/pallet_j5_response_compare.py
    scripts/pallet_pp_watchdog_test.py
    scripts/pallet_serial_trajectory_bridge.py
    scripts/pallet_gravity_preview.py
    scripts/pallet_motor_joint_state.py
    scripts/pallet_hand_guiding.py
    scripts/pallet_follow_joint_trajectory_bridge.py
    scripts/pallet_robstride_trajectory_bridge.py
    scripts/pallet_isaac_gripperless_physics_test.py
    scripts/pallet_isaac_moveit_bridge.py
    scripts/pallet_planning_scene_floor.py
    scripts/robstride_can_id.py
    scripts/run_pallet_isaac_gripperless_physics_test.sh
    scripts/run_pallet_isaac_moveit_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/serial_trajectory_core.py scripts/single_axis_position_core.py scripts/single_axis_gravity_core.py scripts/robstride_trajectory_core.py scripts/hand_guiding_core.py scripts/motor_joint_state_core.py scripts/robstride_serial_readonly.py scripts/single_axis_current_core.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
