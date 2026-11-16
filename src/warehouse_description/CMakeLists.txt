cmake_minimum_required(VERSION 3.8)
project(warehouse_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The Python package warehouse_description: robot_config (robot.yaml and the numbers derived
# from it, for other packages' Python code: the warehouse layout's fit rules, Nav2's footprint,
# the safety fields' sweep model) and safety_fields (the tool below).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

# `ros2 run warehouse_description safety_fields`: a thin script that calls the module above.
install(PROGRAMS
  scripts/safety_fields
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
