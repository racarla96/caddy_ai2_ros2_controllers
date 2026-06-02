cmake_minimum_required(VERSION 3.16)
project(caddy_ai2_ros2_controllers)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  caddy_ai2_ros2_controllers/bicycle_cmd_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bicycle_cmd_relay
)

ament_package()
