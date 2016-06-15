cmake_minimum_required(VERSION 3.5)

project(ament_cmake_auto NONE)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_auto-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
