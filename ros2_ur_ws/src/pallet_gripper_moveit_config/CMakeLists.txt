cmake_minimum_required(VERSION 3.22)
project(pallet_gripper_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/isaac_action_bridge.py
    scripts/isaac_sim.py
    scripts/run_isaac.sh
    scripts/startup_gate.py
    scripts/workcell_scene.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/workcell_scenario.py DESTINATION lib/${PROJECT_NAME})
