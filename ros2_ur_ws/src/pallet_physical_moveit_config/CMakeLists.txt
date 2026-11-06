cmake_minimum_required(VERSION 3.22)
project(pallet_physical_moveit_config)

find_package(ament_cmake REQUIRED)
# Retain only the exact plugin binaries validated for this scoped workaround.
find_package(moveit_ros_planning 2.12.4 EXACT REQUIRED)
add_library(pallet_moveit_shutdown_fix SHARED src/plugin_code_lifetime.cpp)
target_compile_features(pallet_moveit_shutdown_fix PRIVATE cxx_std_17)
target_link_libraries(pallet_moveit_shutdown_fix PRIVATE ${CMAKE_DL_LIBS})
install(TARGETS pallet_moveit_shutdown_fix LIBRARY DESTINATION lib/${PROJECT_NAME})

find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_library(pallet_adaptive_display SHARED src/adaptive_motion_planning_display.cpp)
target_compile_features(pallet_adaptive_display PRIVATE cxx_std_17)
ament_target_dependencies(pallet_adaptive_display rviz_common rclcpp rclcpp_action control_msgs sensor_msgs pluginlib ament_index_cpp)
target_link_libraries(pallet_adaptive_display Qt5::Widgets moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core)
pluginlib_export_plugin_description_file(rviz_common adaptive_display_plugin.xml)
install(TARGETS pallet_adaptive_display LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  add_executable(adaptive_collision_audit test/adaptive_collision_audit.cpp)
  ament_target_dependencies(adaptive_collision_audit rclcpp ament_index_cpp)
  target_link_libraries(adaptive_collision_audit Qt5::Widgets moveit_ros_planning::moveit_planning_scene_monitor)
  add_executable(adaptive_preview_smoke test/adaptive_preview_smoke.cpp)
  target_compile_features(adaptive_preview_smoke PRIVATE cxx_std_17)
  ament_target_dependencies(adaptive_preview_smoke rviz_common rclcpp ament_index_cpp)
  target_link_libraries(adaptive_preview_smoke Qt5::Widgets moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
  ament_add_pytest_test(test_tcp_measurement test/test_tcp_measurement.py)
  ament_add_pytest_test(test_gripper_calibration test/test_gripper_calibration.py)
  ament_add_pytest_test(test_cad_tcp_reference test/test_cad_tcp_reference.py)
endif()

install(PROGRAMS scripts/gripper_direct.py scripts/verify_adaptive_bridge.py scripts/verify_mock_execution.py scripts/tcp_measurement.py scripts/gripper_calibration.py scripts/cad_tcp_reference.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(FILES ADAPTIVE_SERIAL_BRIDGE.md README.md HARDWARE_VALIDATION.md SHUTDOWN_FIX.md TCP_MEASUREMENT.md SERIAL_BRIDGE.md SERIAL_RELIABILITY.md
  DESTINATION share/${PROJECT_NAME})
