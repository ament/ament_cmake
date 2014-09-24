cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)

ament_export_dependencies(
  "ament_cmake_core"
  "ament_cmake_export_definitions"
  "ament_cmake_export_dependencies"
  "ament_cmake_export_include_directories"
  "ament_cmake_export_interfaces"
  "ament_cmake_export_libraries"
  "ament_cmake_gmock"
  "ament_cmake_gtest"
  "ament_cmake_libraries"
  "ament_cmake_nose"
  "ament_cmake_python"
  "ament_cmake_target_dependencies"
  "ament_cmake_test"
)

ament_package()
