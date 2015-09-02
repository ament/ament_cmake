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
# Add a test.
#
# A test is expected to generated a JUnit result file
# ${AMENT_TEST_RESULTS_DIR}/$PROJECT_NAME}/${testname}.xml.
# Failing to do so is considered a failed test.
#
# :param testname: the name of the test
# :type testname: string
# :param COMMAND: the command including its arguments to invoke
# :type COMMAND: list of strings
# :param OUTPUT_FILE: the path of the file to pipe the output to
# :type OUTPUT_FILE: string
# :param TIMEOUT: the test timeout in seconds, default: 60
# :type TIMEOUT: integer
# :param WORKING_DIRECTORY: the working directory for invoking the
#   command in, default: CMAKE_SOURCE_DIR
# :type WORKING_DIRECTORY: string
# :param GENERATE_RESULT_FOR_RETURN_CODE_ZERO: generate a test result
#   file when the command invocation returns with code zero
#   command in, default: CMAKE_SOURCE_DIR
# :type GENERATE_RESULT_FOR_RETURN_CODE_ZERO: option
#
# @public
#
function(ament_add_test testname)
  cmake_parse_arguments(ARG "GENERATE_RESULT_FOR_RETURN_CODE_ZERO"
    "OUTPUT_FILE;RESULT_FILE;TIMEOUT;WORKING_DIRECTORY" "COMMAND" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_add_test() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_COMMAND)
    message(FATAL_ERROR
      "ament_add_test() must be invoked with the COMMAND argument")
  endif()
  if(NOT ARG_RESULT_FILE)
    set(ARG_RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml")
  endif()
  if(NOT ARG_TIMEOUT)
    set(ARG_TIMEOUT 60)
  endif()
  if(NOT ARG_TIMEOUT GREATER 0)
    message(FATAL_ERROR "ament_add_test() the TIMEOUT argument must be a "
      "valid number and greater than zero")
  endif()
  if(NOT ARG_WORKING_DIRECTORY)
    set(WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
  endif()

  # wrap command with run_test script to ensure test result generation
  set(cmd_wrapper "${PYTHON_EXECUTABLE}" "-u" "${ament_cmake_test_DIR}/run_test.py"
    "${ARG_RESULT_FILE}")
  if(ARG_GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
    list(APPEND cmd_wrapper "--generate-result-on-success")
  endif()
  if(ARG_OUTPUT_FILE)
    list(APPEND cmd_wrapper "--output-file" "${ARG_OUTPUT_FILE}")
  endif()
  list(APPEND cmd_wrapper "--command" ${ARG_COMMAND})

  add_test(
    NAME "${testname}"
    COMMAND ${cmd_wrapper}
    WORKING_DIRECTORY "${ARG_WORKING_DIRECTORY}"
  )
  set_tests_properties(
    "${testname}"
    PROPERTIES TIMEOUT ${ARG_TIMEOUT}
  )
endfunction()
