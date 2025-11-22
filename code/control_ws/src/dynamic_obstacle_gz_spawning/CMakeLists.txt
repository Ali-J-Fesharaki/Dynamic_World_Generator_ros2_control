cmake_minimum_required(VERSION 3.5)
project(dynamic_obstacle_gz_spawning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(multi_robot_scripts)

install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

