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

#
# Get all registered package resources of a specific type from the index.
#
# :param var: the output variable name
# :type var: list of resource names
# :param resource_type: the type of the resource
# :type resource_type: string
#
# @public
#
function(ament_index_get_resources var resource_type)
  if("${resource_type} " STREQUAL " ")
    message(FATAL_ERROR
      "ament_index_get_resources() called without a 'resource_type'")
  endif()

  if(NOT "${ARGN} " STREQUAL " ")
    message(FATAL_ERROR "ament_index_get_resources() called with unused "
      "arguments: ${ARGN}")
  endif()

  ament_index_get_prefix_path(raw_paths_to_search)
  # Remove any empty strings and make sure slashes are consistent
  set(paths_to_search)
  foreach(path IN LISTS raw_paths_to_search)
    if(NOT "${path} " STREQUAL " ")
      string(REPLACE "\\" "/" normalized_path "${path}")
      list_append_unique(paths_to_search "${normalized_path}")
    endif()
  endforeach()
  set(all_resources "")
  foreach(path IN LISTS paths_to_search)
    file(GLOB resources
      RELATIVE "${path}/share/ament_index/resource_index/${resource_type}"
      "${path}/share/ament_index/resource_index/${resource_type}/*")
    list_append_unique(all_resources ${resources})
  endforeach()
  set(${var} "${all_resources}" PARENT_SCOPE)
endfunction()
