cmake_minimum_required(VERSION 3.8)
project(warehouse_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The Python package: the test tools behind the commands below (route_driver,
# localization_eval, map_timelapse) and their helpers (ros_helpers, bags).
ament_python_install_package(${PROJECT_NAME})

# Test tools, run with `ros2 run warehouse_system_tests <name>`: thin scripts that call the
# modules above. Simulation only: never part of the robot stack (plan P2-D11).
install(PROGRAMS
  scripts/drive_route
  scripts/evaluate_localization
  scripts/map_timelapse
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
