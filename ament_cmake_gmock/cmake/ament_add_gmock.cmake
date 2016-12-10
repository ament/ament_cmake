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
# Add a gmock.
#
# Call add_executable(target ARGN), link it against the gmock library
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
# :param SKIP_LINKING_MAIN_LIBRARIES: if set skip linking against the gmock
#   main libraries
# :type SKIP_LINKING_MAIN_LIBRARIES: option
#
# @public
#
macro(ament_add_gmock target)
  _ament_cmake_gmock_find_gmock()
  if(GMOCK_FOUND)
    _ament_add_gmock("${target}" ${ARGN})
  endif()
endmacro()

function(_ament_add_gmock target)
  cmake_parse_arguments(ARG "SKIP_LINKING_MAIN_LIBRARIES" "TIMEOUT" "" ${ARGN})
  if(NOT ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "ament_add_gmock() must be invoked with at least one source file")
  endif()

  # should be EXCLUDE_FROM_ALL if it would be possible
  # to add this target as a dependency to the "test" target
  add_executable("${target}" ${ARG_UNPARSED_ARGUMENTS})
  target_include_directories("${target}" PUBLIC "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries("${target}" ${GMOCK_LIBRARIES})
  if(NOT ARG_SKIP_LINKING_MAIN_LIBRARIES)
    target_link_libraries("${target}" ${GMOCK_MAIN_LIBRARIES})
  endif()

  # TODO consider using a generator expression instead
  # $<TARGET_FILE:target>
  # until then set the policy explicitly in order to avoid warning with newer CMake versions.
  if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
  endif()
  get_target_property(executable "${target}" LOCATION)
  string(REPLACE "$(Configuration)" "$<CONFIGURATION>" executable "${executable}")
  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}.gtest.xml")
  set(cmd
    "${executable}"
    "--gtest_output=xml:${result_file}")
  if(ARG_TIMEOUT)
    set(ARG_TIMEOUT "TIMEOUT" ${ARG_TIMEOUT})
  endif()
  if(ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY "WORKING_DIRECTORY" "${ARG_WORKING_DIRECTORY}")
  endif()

  ament_add_test(
    "${target}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gmock/${target}.txt"
    RESULT_FILE "${result_file}"
    ${ARG_TIMEOUT}
    ${ARG_WORKING_DIRECTORY}
  )
  set_tests_properties(
    "${target}"
    PROPERTIES
    REQUIRED_FILES "${executable}"
    LABELS "gmock"
  )
endfunction()
