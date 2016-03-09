# Copyright 2014 Open Source Robotics Foundation, Inc.
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

if("${PYTHON_HOST_EXECUTABLE} " STREQUAL " ")
  set(PYTHON_VERSION "" CACHE STRING
    "Specify specific Python version to use ('major.minor' or 'major')")
  # if not specified otherwise use Python 3
  if(NOT PYTHON_VERSION)
    set(PYTHON_VERSION "3")
  endif()

  find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
else()
  set(PYTHON_EXECUTABLE "${PYTHON_HOST_EXECUTABLE}")
endif()
message(STATUS "Using PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
