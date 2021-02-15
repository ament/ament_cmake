cmake_minimum_required(VERSION 3.5)

project(ament_cmake_python NONE)

find_package(ament_cmake_core REQUIRED)

set(ament_cmake_python_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_cmake_python-extras.cmake")
ament_python_install_package(${PROJECT_NAME} NO_DATA)

ament_package(
  CONFIG_EXTRAS "ament_cmake_python-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
