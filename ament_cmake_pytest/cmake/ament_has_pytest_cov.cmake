# Copyright 2020 Open Source Robotics Foundation, Inc.
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

#
# Check if the Python module `pytest-cov` was found and get its version if it is.
#
# :param var: the output variable name
# :type var: string
# :param version_var: the variable name for the version of pytest-cov found, only
#   set if it was found
# :type version_var: string
# :param QUIET: suppress the CMake warning if pytest-cov is not found, if not set
#   and pytest-cov was not found a CMake warning is printed
# :type QUIET: option
# :param PYTHON_EXECUTABLE: absolute path to the Python interpreter to be used,
#   default to the CMake variable with the same name returned by
#   FindPythonInterp
# :type PYTHON_EXECUTABLE: string
#
# @public
#
function(ament_has_pytest_cov var version_var)
  cmake_parse_arguments(ARG "QUIET" "PYTHON_EXECUTABLE" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_has_pytest_cov() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_PYTHON_EXECUTABLE)
    set(ARG_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  endif()

  set(cmd "${ARG_PYTHON_EXECUTABLE}" "-m" "pytest" "--version")
  execute_process(
    COMMAND ${cmd}
    RESULT_VARIABLE res
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(res EQUAL 0)
    # check if pytest-cov is in the list of plugins
    # (actual output of the command is in ${error} and not ${output})
    string(REGEX MATCH "pytest-cov-[0-9]\.[0-9]\.[0-9]" pytest_cov_full_version "${error}")
    if(pytest_cov_full_version)
      # extract version
      string(REGEX MATCH "[0-9]\.[0-9]\.[0-9]" pytest_cov_version "${pytest_cov_full_version}")
      set(${var} TRUE PARENT_SCOPE)
      set(${version_var} ${pytest_cov_version} PARENT_SCOPE)
    else()
      if(NOT ARG_QUIET)
        message(WARNING "Failed to find `pytest` plugin `pytest-cov`")
      endif()
      set(${var} FALSE PARENT_SCOPE)
    endif()
  else()
    if(NOT ARG_QUIET)
      string(REPLACE ";" " " cmd_str "${cmd}")
      message(WARNING
        "Failed to find Python module `pytest` which is needed "
        "for `pytest-cov`: '${cmd_str}' returned error code ${res}")
    endif()
    set(${var} FALSE PARENT_SCOPE)
  endif()
endfunction()
