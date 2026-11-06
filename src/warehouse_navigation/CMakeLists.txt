cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

find_package(ament_cmake REQUIRED)

# maps/ holds the saved warehouse map and its README. The README is always there, so the
# folder installs even before a map has been saved.
install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto skips a missing linter silently; make a missing formatter fail loudly.
  find_package(ament_cmake_clang_format REQUIRED)
  # clang-format replaces uncrustify as the C++ formatter (docs/decisions/ADR-003).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  # Checks the SLAM, AMCL and map server parameter files without starting anything.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav_params test/test_nav_params.py)
endif()

ament_package()
