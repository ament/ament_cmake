# copied from
# ament_cmake_target_dependencies/ament_cmake_target_dependencies-extras.cmake

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_libraries REQUIRED)

include(
  "${ament_cmake_target_dependencies_DIR}/ament_target_dependencies.cmake")
