cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_export_interfaces)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_index REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_export_interfaces-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
