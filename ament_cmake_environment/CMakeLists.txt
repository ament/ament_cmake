cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_environment)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_index REQUIRED)

# include CMake functions of this package
set(ament_cmake_environment_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_cmake_environment-extras.cmake" NO_POLICY_SCOPE)

ament_package(
  CONFIG_EXTRAS "ament_cmake_environment-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
