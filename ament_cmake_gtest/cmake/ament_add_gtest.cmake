# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# include CMake functions
include(CMakeParseArguments)

#
# Add a gtest.
#
# Call add_executable(target ARGN), link it against the gtest library
# and register the executable as a test.
#
# :param target: the target name which will also be used as the test name
# :type target: string
# :param ARGN: the list of source files
# :type ARGN: list of strings
# :param TIMEOUT: the test timeout in seconds, default: 60
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory for invoking the
#   executable in, default: CMAKE_SOURCE_DIR
# :type WORKING_DIRECTORY: string
#
# @public
#
macro(ament_add_gtest target)
  _ament_cmake_gtest_find_gtest()
  if(GTEST_FOUND)
    _ament_add_gtest("${target}" ${ARGN})
  endif()
endmacro()

function(_ament_add_gtest target)
  cmake_parse_arguments(ARG "" "TIMEOUT" "" ${ARGN})
  if(NOT ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "ament_add_gtest() must be invoked with at least one source file")
  endif()

  # should be EXCLUDE_FROM_ALL if it would be possible
  # to add this target as a dependency to the "test" target
  add_executable("${target}" ${ARG_UNPARSED_ARGUMENTS})
  target_link_libraries("${target}" gtest)

  get_target_property(target_path "${target}" RUNTIME_OUTPUT_DIRECTORY)
  if(NOT target_path)
    set(target_path "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  set(cmd
    "${target_path}/${target}"
    "--gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}.xml")
  if(ARG_TIMEOUT)
    set(ARG_TIMEOUT "TIMEOUT" ${ARG_TIMEOUT})
  endif()
  if(ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY "WORKING_DIRECTORY" "${ARG_WORKING_DIRECTORY}")
  endif()

  ament_add_test(
    "${target}"
    COMMAND ${cmd}
    ${ARG_TIMEOUT}
    ${ARG_WORKING_DIRECTORY}
  )
  set_tests_properties(
    "${target}"
    PROPERTIES REQUIRED_FILES "${target_path}/${target}"
  )
endfunction()
