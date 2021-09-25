cmake_minimum_required(VERSION 3.5)

project(ament_cmake_gen_version_h)

find_package(ament_cmake_core REQUIRED)

ament_package(CONFIG_EXTRAS "ament_cmake_gen_version_h-extras.cmake")

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
