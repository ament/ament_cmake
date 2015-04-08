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

# copied from ament_cmake_nose/ament_cmake_nose-extras.cmake

# find nosetests once
macro(_ament_cmake_nose_find_nosetests)
  if(NOT DEFINED _AMENT_CMAKE_NOSE_FIND_NOSETESTS)
    set(_AMENT_CMAKE_NOSE_FIND_NOSETESTS TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    find_package(ament_cmake_test QUIET REQUIRED)

    find_program(NOSETESTS NAMES
      "nosetests${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
      "nosetests-${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
      "nosetests${PYTHON_VERSION_MAJOR}"
      "nosetests-${PYTHON_VERSION_MAJOR}"
      "nosetests")
    if(NOSETESTS)
      message(STATUS "Using Python nosetests: ${NOSETESTS}")
    else()
      if("${PYTHON_VERSION_MAJOR} " STREQUAL "3 ")
        set(_python_nosetests_package "python3-nose")
      else()
        set(_python_nosetests_package "python-nose")
      endif()
      message(WARNING
        "'nosetests' not found, Python nose tests can not be run (e.g. on "
        "Ubuntu/Debian install the package '${_python_nosetests_package}')")
    endif()
  endif()
endmacro()

include("${ament_cmake_nose_DIR}/ament_add_nose_test.cmake")
include("${ament_cmake_nose_DIR}/ament_find_nosetests.cmake")
