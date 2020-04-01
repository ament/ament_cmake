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
# :param PYTHON_EXECUTABLE: absolute path to the Python interpreter to be used,
#   default to the CMake variable with the same name returned by
#   FindPythonInterp
# :type PYTHON_EXECUTABLE: string
#
# @public
#
function(ament_get_pytest_cov_version var)
  cmake_parse_arguments(ARG "PYTHON_EXECUTABLE" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_get_pytest_cov_version() called with unused arguments: "
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
      set(${var} ${pytest_cov_version} PARENT_SCOPE)
    else()
      set(${var} FALSE PARENT_SCOPE)
    endif()
  else()
    set(${var} FALSE PARENT_SCOPE)
  endif()
endfunction()
