# Copyright 2021 Open Source Robotics Foundation, Inc.
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
# Order targets according to the chain of ament prefixes.
#
# :param output_targets: the output list of ordered targets.
# :type output_targets: list of targets
# :param ARGN: a list of targets.
# :type ARGN: list of targets
#
# @public
#
macro(ament_target_dependencies_order output_targets)
  set(_ament_prefix_path_list "$ENV{AMENT_PREFIX_PATH}")
  if(NOT WIN32)
    string(REPLACE ":" ";" _ament_prefix_path_list "${_ament_prefix_path_list}")
  endif()
  _ament_target_dependencies_order(${output_targets} "${_ament_prefix_path_list}" ${ARGN})
endmacro()


# This implementation was copied and adapted from "ament_include_directories_order"
function(_ament_target_dependencies_order output_targets prefixes)
  # create list of empty slots, one per prefix and one for unknown prefixes
  list(LENGTH prefixes prefix_count)
  foreach(index RANGE ${prefix_count})
    set(slot_${index} "")
  endforeach()

  # append the target to the first prefix matching any of it's include directories
  foreach(interface ${ARGN})
    get_target_property(include_dirs ${interface} INTERFACE_INCLUDE_DIRECTORIES)

    set(match FALSE)
    foreach(include_dir ${include_dirs})
      string(LENGTH "${include_dir}" include_dir_length)

      set(index 0)
      while(TRUE)
        if(NOT ${index} LESS ${prefix_count})
          # no match
          break()
        endif()

        list(GET prefixes ${index} prefix)
        # exact match
        if(prefix STREQUAL include_dir)
          set(match TRUE)
          break()
        endif()

        string(LENGTH "${prefix}" prefix_length)
        if(${prefix_length} LESS ${include_dir_length})
          math(EXPR prefix_length_plus_one "${prefix_length} + 1")
          string(SUBSTRING "${include_dir}"
            0 ${prefix_length_plus_one} include_dir_prefix)
          # prefix match
          if("${prefix}/" STREQUAL "${include_dir_prefix}")
            set(match TRUE)
            break()
          endif()
        endif()

        math(EXPR index "${index} + 1")
      endwhile()
      if(match)
        list(APPEND slot_${index} ${interface})
        break()
      endif()
    endforeach()
    if(NOT match)
      list(APPEND slot_${index} ${interface})
    endif()
  endforeach()

  # join the slot lists
  set(ordered "")
  foreach(index RANGE ${prefix_count})
    list(APPEND ordered ${slot_${index}})
  endforeach()
  set(${output_targets} "${ordered}" PARENT_SCOPE)
endfunction()
