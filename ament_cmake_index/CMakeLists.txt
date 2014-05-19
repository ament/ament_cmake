cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_index)

find_package(ament_cmake_core REQUIRED)

# include CMake functions of this package
set(ament_cmake_index_DIR "cmake")
include("ament_cmake_index-extras.cmake" NO_POLICY_SCOPE)

ament_package(
  CONFIG_EXTRAS "ament_cmake_index-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
