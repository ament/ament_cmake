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

# include CMake functions
include(CMakeParseArguments)

#
# Register a package name with the resource index.
#
# :param PACKAGE_NAME: the package name (default: ${PROJECT_NAME})
# :type PACKAGE_NAME: string
#
function(ament_index_register_package)
  cmake_parse_arguments(ARG "" "PACKAGE_NAME" "" ${ARGN})
  if(ARGN)
    message(FATAL_ERROR
      "ament_index_register_package() called with unused arguments: ${ARGN}")
  endif()

  # register package name with the resource type 'packages'
  ament_index_register_resource("packages" ${ARGN})
endfunction()
