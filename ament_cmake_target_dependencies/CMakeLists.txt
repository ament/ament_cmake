cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_target_dependencies)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_environment REQUIRED)
find_package(ament_cmake_index REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_target_dependencies-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
