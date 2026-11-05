cmake_minimum_required(VERSION 3.8)
project(warehouse_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Helpers shared by the launch tests: import warehouse_system_tests.ros_helpers.
ament_python_install_package(${PROJECT_NAME})

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
endif()

ament_package()
