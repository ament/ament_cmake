# generate and register extra file for definitions
set(_generated_extra_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_definitions/ament_cmake_export_definitions-extras.cmake")
configure_file(
  "${ament_cmake_export_definitions_DIR}/ament_cmake_export_definitions-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")
