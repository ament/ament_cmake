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

# prevent multiple inclusion
if(DEFINED _AMENT_CMAKE_CORE_INCLUDED)
  message(FATAL_ERROR "ament_cmake_core/cmake/core/all.cmake included "
    "multiple times")
endif()
set(_AMENT_CMAKE_CORE_INCLUDED TRUE)

if(NOT DEFINED ament_cmake_core_DIR)
  message(FATAL_ERROR "ament_cmake_core_DIR is not set")
endif()

# enable all new policies (if available)
macro(_set_cmake_policy_to_new_if_available policy)
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endmacro()
_set_cmake_policy_to_new_if_available(CMP0000)
_set_cmake_policy_to_new_if_available(CMP0001)
_set_cmake_policy_to_new_if_available(CMP0002)
_set_cmake_policy_to_new_if_available(CMP0003)
_set_cmake_policy_to_new_if_available(CMP0004)
_set_cmake_policy_to_new_if_available(CMP0005)
_set_cmake_policy_to_new_if_available(CMP0006)
_set_cmake_policy_to_new_if_available(CMP0007)
_set_cmake_policy_to_new_if_available(CMP0008)
_set_cmake_policy_to_new_if_available(CMP0009)
_set_cmake_policy_to_new_if_available(CMP0010)
_set_cmake_policy_to_new_if_available(CMP0011)
_set_cmake_policy_to_new_if_available(CMP0012)
_set_cmake_policy_to_new_if_available(CMP0013)
_set_cmake_policy_to_new_if_available(CMP0014)
_set_cmake_policy_to_new_if_available(CMP0015)
_set_cmake_policy_to_new_if_available(CMP0016)
_set_cmake_policy_to_new_if_available(CMP0017)
_set_cmake_policy_to_new_if_available(CMP0018)
_set_cmake_policy_to_new_if_available(CMP0019)
_set_cmake_policy_to_new_if_available(CMP0020)
_set_cmake_policy_to_new_if_available(CMP0021)
_set_cmake_policy_to_new_if_available(CMP0022)
_set_cmake_policy_to_new_if_available(CMP0023)

# the following operations must be performed inside a project context
if(NOT PROJECT_NAME)
  project(ament_cmake_internal NONE)
endif()

# include CMake functions
include(CMakeParseArguments)

# various functions / macros
foreach(filename
  "ament_execute_extensions"
  "ament_package"
  "ament_package_xml"
  "ament_register_extension"
  "assert_file_exists"
  "list_append_unique"
  "normalize_path"
  "python"
  "stamp"
  "string_ends_with"
)
  include(${ament_cmake_core_DIR}/core/${filename}.cmake)
endforeach()

# ensure that no current package name is set
unset(_AMENT_PACKAGE_NAME)
