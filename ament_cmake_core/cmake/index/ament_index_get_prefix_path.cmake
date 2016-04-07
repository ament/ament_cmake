# Copyright 2016 Open Source Robotics Foundation, Inc.
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
# Get the prefix path including the folder from the binary dir.
#
# :param var: the output variable name for the prefix path
# :type var: string
#
# @public
#
function(ament_index_get_prefix_path var)
  if(NOT "${ARGN} " STREQUAL " ")
    message(FATAL_ERROR "ament_index_get_prefix_path() called with unused "
      "arguments: ${ARGN}")
  endif()

  string(REPLACE ":" ";" prefix_path "$ENV{AMENT_PREFIX_PATH}")

  # Remove CMAKE_INSTALL_PREFIX if it is in the list of paths to search,
  # and add it to the list at the front
  # TODO(dirk-thomas) check if this can be removed or the insert can be done
  list(REMOVE_ITEM prefix_path "${CMAKE_INSTALL_PREFIX}")
  list(INSERT prefix_path 0 "${CMAKE_INSTALL_PREFIX}")

  # prepend path from binary dir
  list(INSERT prefix_path 0 "${CMAKE_BINARY_DIR}/ament_cmake_index")

  set(${var} "${prefix_path}" PARENT_SCOPE)
endfunction()
