cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

find_package(ament_cmake REQUIRED)

# maps/ holds the saved warehouse map and its README. The README is always there, so the
# folder installs even before a map has been saved.
install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
