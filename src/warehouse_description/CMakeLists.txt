cmake_minimum_required(VERSION 3.8)
project(warehouse_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto skips a missing linter silently; make a missing formatter fail loudly.
  find_package(ament_cmake_clang_format REQUIRED)
  # clang-format replaces uncrustify as the C++ formatter (docs/decisions/ADR-003).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  # Builds the xacro for every hardware option and checks it against robot.yaml.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)
endif()

ament_package()
