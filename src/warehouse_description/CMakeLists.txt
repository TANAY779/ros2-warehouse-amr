cmake_minimum_required(VERSION 3.8)
project(warehouse_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The Python module warehouse_description.robot_config: robot.yaml and the numbers derived from
# it, for other packages' Python code (the warehouse layout's fit rules, later Nav2 checks).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
