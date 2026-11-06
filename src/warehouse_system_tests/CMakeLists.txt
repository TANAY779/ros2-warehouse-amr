cmake_minimum_required(VERSION 3.8)
project(warehouse_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The Python package: helpers shared by the launch tests (ros_helpers), and the test tools
# behind the commands below (route_driver, localization_eval, map_timelapse, bags).
ament_python_install_package(${PROJECT_NAME})

# Test tools, run with `ros2 run warehouse_system_tests <name>`: thin scripts that call the
# modules above. Simulation only: never part of the robot stack (plan P2-D11).
install(PROGRAMS
  scripts/drive_route
  scripts/evaluate_localization
  scripts/map_timelapse
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto skips a missing linter silently; make a missing formatter fail loudly.
  find_package(ament_cmake_clang_format REQUIRED)
  # clang-format replaces uncrustify as the C++ formatter (docs/decisions/ADR-003).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Run each launch test in its own ROS domain, picked by domain_coordinator (unless
  # ROS_DOMAIN_ID is already set), so tests running in parallel never see each other.
  set(isolated_runner "${ament_cmake_ros_DIR}/run_test_isolated.py")

  # Mock hardware, no simulator: runs everywhere, including PR CI.
  add_launch_test(test/test_mock_bringup.py
    TIMEOUT 180
    RUNNER "${isolated_runner}")

  # Gazebo with rendering sensors: needs a GPU or EGL. Every case is reported as skipped
  # when the environment sets WAREHOUSE_SKIP_GZ_TESTS=1, as CI does (see README.md).
  add_launch_test(test/test_gz_bringup.py
    TIMEOUT 600
    RUNNER "${isolated_runner}")

  # The test tools' logic, without a ROS graph: seconds each.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_route_driver test/test_route_driver.py TIMEOUT 60)
  ament_add_pytest_test(test_localization_eval test/test_localization_eval.py TIMEOUT 120)
  ament_add_pytest_test(test_map_timelapse test/test_map_timelapse.py TIMEOUT 60)
endif()

ament_package()
