cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_auto)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_auto-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
