cmake_minimum_required(VERSION 3.8)
project(warehouse_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The Python package: the test tools behind the commands below (route_driver,
# localization_eval, map_timelapse, measure_load, goal_set, nav_benchmark, navigation_eval) and
# their helpers (ros_helpers, bags, zones).
ament_python_install_package(${PROJECT_NAME})

# Test tools, run with `ros2 run warehouse_system_tests <name>`: thin scripts that call the
# modules above. Simulation only: never part of the robot stack (plan P2-D11).
install(PROGRAMS
  scripts/drive_route
  scripts/evaluate_localization
  scripts/evaluate_navigation
  scripts/goal_set
  scripts/map_timelapse
  scripts/measure_load
  scripts/nav_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

# config/goal_sets holds the frozen goal sets that goal_set writes and nav_benchmark reads
# (share/warehouse_system_tests/config/goal_sets/warehouse_50.yaml).
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
