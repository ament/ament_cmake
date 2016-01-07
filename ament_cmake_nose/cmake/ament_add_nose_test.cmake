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
# Add a nose test.
#
# :param testname: the name of the test
# :type testname: string
# :param path: the path to a file or folder where ``nosetests``
#   should be invoked on
# :type path: string
# :param TIMEOUT: the test timeout in seconds, default: 60
# :type TIMEOUT: integer
# :param ENV: list of env vars to set; listed as ``VAR=value``
# :type ENV: list of strings
# :param APPEND_ENV: list of env vars to append if already set, otherwise set;
#   listed as ``VAR=value``
# :type APPEND_ENV: list of strings
# :param APPEND_LIBRARY_DIRS: list of library dirs to append to the appropriate
#   OS specific env var, a la LD_LIBRARY_PATH
# :type APPEND_LIBRARY_DIRS: list of strings
#
# @public
#
macro(ament_add_nose_test testname path)
  _ament_cmake_nose_find_nosetests()
  if(NOSETESTS)
    _ament_add_nose_test("${testname}" "${path}" ${ARGN})
  endif()
endmacro()

function(_ament_add_nose_test testname path)
  cmake_parse_arguments(ARG
    ""
    "TIMEOUT"
    "APPEND_ENV;APPEND_LIBRARY_DIRS;ENV"
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_add_nose_test() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT IS_ABSOLUTE "${path}")
    set(path "${CMAKE_CURRENT_SOURCE_DIR}/${path}")
  endif()
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR
      "ament_add_nose_test() the path '${path}' does not exist")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xunit.xml")
  set(cmd
    "${NOSETESTS}" "${path}" "--with-xunit"
    "--xunit-file=${result_file}")
  if(NOT "${NOSETESTS_VERSION}" VERSION_LESS "1.3.5")
    list(APPEND cmd "--xunit-testsuite-name=${PROJECT_NAME}.nosetests")
    if(NOT "${NOSETESTS_VERSION}" VERSION_LESS "1.3.8")
      list(APPEND cmd "--xunit-prefix-with-testsuite-name")
    endif()
  endif()

  if(ARG_ENV)
    set(ARG_ENV "ENV" ${ARG_ENV})
  endif()
  if(ARG_APPEND_ENV)
    set(ARG_APPEND_ENV "APPEND_ENV" ${ARG_APPEND_ENV})
  endif()
  if(ARG_APPEND_LIBRARY_DIRS)
    set(ARG_APPEND_LIBRARY_DIRS "APPEND_LIBRARY_DIRS" ${ARG_APPEND_LIBRARY_DIRS})
  endif()
  if(ARG_TIMEOUT)
    set(ARG_TIMEOUT "TIMEOUT" "${ARG_TIMEOUT}")
  endif()

  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_nose/${testname}.txt"
    RESULT_FILE "${result_file}"
    ${ARG_ENV}
    ${ARG_APPEND_ENV}
    ${ARG_APPEND_LIBRARY_DIRS}
    ${ARG_TIMEOUT}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
  set_tests_properties(
    "${testname}"
    PROPERTIES
    LABELS "nose"
  )
endfunction()

# TODO provide function to register all found tests separately
