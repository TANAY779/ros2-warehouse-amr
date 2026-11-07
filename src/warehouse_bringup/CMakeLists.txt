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
# drops the .py from the command's name. ament's symlink install cannot rename, so with
# --symlink-install this small file is copied: rebuild after editing it (the code in the
# package above stays symlinked).
install(
  PROGRAMS scripts/teleop_window.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_window
)

ament_package()
