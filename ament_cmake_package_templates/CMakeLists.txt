cmake_minimum_required(VERSION 2.8.3)

project(ament_cmake_package_templates)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_index REQUIRED)

# extract information from ament_package.templates
if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR
    "ament_cmake_package_templates: variable 'PYTHON_EXECUTABLE' must not be empty")
endif()

# stamp script to generate CMake code
set(generator "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates_2_cmake.py")
stamp("${generator}")

# invoke generator script
set(generated_file "${CMAKE_CURRENT_BINARY_DIR}/templates.cmake")
set(cmd
  "${PYTHON_EXECUTABLE}"
  "${generator}"
  "${generated_file}"
)
execute_process(
  COMMAND ${cmd}
  RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
  string(REPLACE ";" " " cmd_str "${cmd}")
  message(FATAL_ERROR
    "execute_process(${cmd_str}) returned error code ${res}")
endif()

# load extracted variables into cmake
include("${generated_file}")

ament_package(
  CONFIG_EXTRAS "ament_cmake_package_templates-extras.cmake.in"
)
