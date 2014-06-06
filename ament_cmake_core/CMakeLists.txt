cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_core)

find_package(ament_cmake_symlink_install REQUIRED)

# include CMake functions of this package
set(ament_cmake_core_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("${ament_cmake_core_DIR}/all.cmake" NO_POLICY_SCOPE)

ament_package_xml()

message(STATUS "ament_cmake_core ${ament_cmake_core_VERSION}")

ament_package(
  CONFIG_EXTRAS "ament_cmake_core-extras.cmake.in"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# install package marker file into index manually
# since this package does not depend on ament_cmake_index
set(marker_file
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_core/packages/${PROJECT_NAME}")
file(WRITE "${marker_file}" "")
install(
  FILES "${marker_file}"
  DESTINATION "share/ament_index/resource_index/packages"
)
