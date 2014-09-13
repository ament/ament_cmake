# generate and register extra file for interfaces
set(_generated_extra_file
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_interfaces/ament_cmake_export_interfaces-extras.cmake")
configure_file(
  "${ament_cmake_export_interfaces_DIR}/ament_cmake_export_interfaces-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")

# install export files for interfaces
if(NOT "${_AMENT_CMAKE_EXPORT_INTERFACES} " STREQUAL " ")
  foreach(_interface ${_AMENT_CMAKE_EXPORT_INTERFACES})
    install(
      EXPORT "${_interface}"
      DESTINATION share/${PROJECT_NAME}/cmake
      NAMESPACE "${PROJECT_NAME}::"
      FILE "${_interface}Export.cmake"
    )
  endforeach()
endif()
