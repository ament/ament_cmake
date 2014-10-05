# extract information from ament_package.templates
if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR
    "ament_cmake_package_templates: variable 'PYTHON_EXECUTABLE' must not be "
    "empty")
endif()

# stamp script to generate CMake code
set(_generator
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/package_templates/templates_2_cmake.py")
stamp("${_generator}")

# invoke generator script
set(_generated_file
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_package_templates/templates.cmake")
set(_cmd
  "${PYTHON_EXECUTABLE}"
  "${_generator}"
  "${_generated_file}"
)
execute_process(
  COMMAND ${_cmd}
  RESULT_VARIABLE _res
)
if(NOT _res EQUAL 0)
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  message(FATAL_ERROR
    "execute_process(${_cmd_str}) returned error code ${_res}")
endif()

# load extracted variables into cmake
include("${_generated_file}")
