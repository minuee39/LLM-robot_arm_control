cmake_minimum_required(VERSION 3.22)
project(pallet_physical_moveit_config)

find_package(ament_cmake REQUIRED)
# Retain only the exact plugin binaries validated for this scoped workaround.
find_package(moveit_ros_planning 2.12.4 EXACT REQUIRED)
add_library(pallet_moveit_shutdown_fix SHARED src/plugin_code_lifetime.cpp)
target_compile_features(pallet_moveit_shutdown_fix PRIVATE cxx_std_17)
target_link_libraries(pallet_moveit_shutdown_fix PRIVATE ${CMAKE_DL_LIBS})
install(TARGETS pallet_moveit_shutdown_fix LIBRARY DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model_consistency test/test_model_consistency.py)
  ament_add_pytest_test(test_tcp_measurement test/test_tcp_measurement.py)
  ament_add_pytest_test(test_gripper_calibration test/test_gripper_calibration.py)
  ament_add_pytest_test(test_cad_tcp_reference test/test_cad_tcp_reference.py)
endif()

install(PROGRAMS scripts/verify_mock_execution.py scripts/tcp_measurement.py scripts/gripper_calibration.py scripts/cad_tcp_reference.py
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

install(FILES README.md HARDWARE_VALIDATION.md SHUTDOWN_FIX.md TCP_MEASUREMENT.md
  DESTINATION share/${PROJECT_NAME})
