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

#
# Add the definitions, include directories and libraries of packages to a
# target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be _DEFINITIONS, _INCLUDE_DIRS, _LIBRARIES, and
# _LINK_FLAGS.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names, which could start with PUBLIC keyword.
#   If it starts with PUBLIC, this keyword is used in the target_link_libraries call.
#   If not, the non-keyword target_link_libraries call is used.
# :type ARGN: list of strings
#
# @public
#
function(ament_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    set(options PUBLIC)
    set(oneValueArgs)
    set(multiValueArgs)
    cmake_parse_arguments(AMENT_TARGET_DEPENDENCIES
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
      ${ARGN})
    set(definitions "")
    set(include_dirs "")
    set(libraries "")
    set(link_flags "")
    set(TARGET_LINK_LIBRARIES_VISIBILITY)
    if(AMENT_TARGET_DEPENDENCIES_PUBLIC)
      if(NOT ${ARGV1} STREQUAL PUBLIC)
        message(FATAL_ERROR "ament_target_dependencies() PUBLIC keyword is only allowed before the package names list")
      endif()
      set(TARGET_LINK_LIBRARIES_VISIBILITY PUBLIC)
    endif()
    foreach(package_name ${AMENT_TARGET_DEPENDENCIES_UNPARSED_ARGUMENTS})
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      list_append_unique(definitions ${${package_name}_DEFINITIONS})
      list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
      list(APPEND libraries ${${package_name}_LIBRARIES})
      list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
    endforeach()
    target_compile_definitions(${target}
      PUBLIC ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_include_directories(${target}
      PUBLIC ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target}
      ${TARGET_LINK_LIBRARIES_VISIBILITY} ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()
