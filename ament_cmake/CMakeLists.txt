cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake)

find_package(ament_cmake_symlink_install REQUIRED)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ament_cmakeConfig.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# install package marker file into index
# it is done manually here since this package does not depend on ament_cmake_index
set(marker_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake/packages/${PROJECT_NAME}")
file(WRITE "${marker_file}" "")
install(
  FILES "${marker_file}"
  DESTINATION "share/ament_index/resource_index/packages"
)
