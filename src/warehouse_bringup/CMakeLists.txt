cmake_minimum_required(VERSION 3.8)
project(warehouse_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# The Python package warehouse_bringup: the teleop window and its hold-to-drive logic.
ament_python_install_package(${PROJECT_NAME})

# `ros2 run warehouse_bringup teleop_window` runs lib/warehouse_bringup/teleop_window. RENAME
# drops the .py, which the linters need to find the file. ament's symlink install cannot
# rename, so with --symlink-install this small file is copied: rebuild after editing it (the
# code in the package above stays symlinked).
install(
  PROGRAMS scripts/teleop_window.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_window
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto skips a missing linter silently; make a missing formatter fail loudly.
  find_package(ament_cmake_clang_format REQUIRED)
  # clang-format replaces uncrustify as the C++ formatter (docs/decisions/ADR-003).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # The teleop logic, with no Qt and no ROS.
  ament_add_pytest_test(test_hold_to_drive test/test_hold_to_drive.py TIMEOUT 60)
  # The teleop window with synthetic key events, on Qt's offscreen platform (no screen needed).
  ament_add_pytest_test(test_teleop_window test/test_teleop_window.py TIMEOUT 60)
endif()

ament_package()
