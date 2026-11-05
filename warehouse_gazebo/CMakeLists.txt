cmake_minimum_required(VERSION 3.8)
project(warehouse_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto skips a missing linter silently; make a missing formatter fail loudly.
  find_package(ament_cmake_clang_format REQUIRED)
  # clang-format replaces uncrustify as the C++ formatter (docs/decisions/ADR-003).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  # Checks the worlds, the bridge config and Gazebo's URDF conversion without rendering, so it
  # runs in PR CI, which skips the Gazebo launch test (warehouse_system_tests).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_config test/test_gazebo_config.py)
endif()

ament_package()
