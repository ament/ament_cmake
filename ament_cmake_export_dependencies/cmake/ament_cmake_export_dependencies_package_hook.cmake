# generate and register extra file for dependencies
set(_generated_extra_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
configure_file(
  "${ament_cmake_export_dependencies_DIR}/ament_cmake_export_dependencies-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")
