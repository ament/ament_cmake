# list relative paths before absolute paths
set(_AMENT_CMAKE_EXPORT_INCLUDE_DIRECTORIES ${_AMENT_EXPORT_RELATIVE_INCLUDE_DIRECTORIES} ${_AMENT_EXPORT_ABSOLUTE_INCLUDE_DIRECTORIES})

# generate and register extra file for include directories
set(_generated_extra_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
configure_file(
  "${ament_cmake_export_include_directories_DIR}/ament_cmake_export_include_directories-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")
