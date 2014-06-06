# deduplicate _AMENT_EXPORT_LIBRARY_TARGETS, _AMENT_EXPORT_ABSOLUTE_LIBRARIES
# and _AMENT_EXPORT_LIBRARY_NAMES
# while maintaining library order
# as well as build configuration keywords
# TODO

# generate and register extra file for libraries
set(_generated_extra_file
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
configure_file(
  "${ament_cmake_export_libraries_DIR}/ament_cmake_export_libraries-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")
