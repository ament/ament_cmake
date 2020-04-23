# Copyright 2014-2020 Open Source Robotics Foundation, Inc.
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
# Add the interface targets or definitions, include directories and libraries
# of packages to a target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be either _INTERFACES or _DEFINITIONS, _INCLUDE_DIRS,
# _LIBRARIES, and _LINK_FLAGS.
# If _INTERFACES is not empty it will be used exclusively, otherwise the other
# variables are being used.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names, which can optionally start with an
#   INTERFACE or PUBLIC keyword.
#   If it starts with INTERFACE or PUBLIC, this keyword is used in the
#   target_* calls.
# :type ARGN: list of strings
#
# @public
#
function(ament_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    cmake_parse_arguments(ARG "INTERFACE;PUBLIC" "" "" ${ARGN})
    set(optional_keyword "")
    set(required_keyword "PUBLIC")
    if(ARG_INTERFACE)
      if(NOT "${ARGV1}" STREQUAL "INTERFACE")
        message(FATAL_ERROR "ament_target_dependencies() INTERFACE keyword is only allowed before the package names")
      endif()
      set(optional_keyword INTERFACE)
      set(required_keyword INTERFACE)
    endif()
    if(ARG_PUBLIC)
      if(NOT "${ARGV1}" STREQUAL "PUBLIC")
        message(FATAL_ERROR "ament_target_dependencies() PUBLIC keyword is only allowed before the package names")
      endif()
      set(optional_keyword PUBLIC)
    endif()
    set(definitions "")
    set(include_dirs "")
    set(interfaces "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARG_UNPARSED_ARGUMENTS})
      if(NOT "${${package_name}_FOUND}")
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      if(NOT "${${package_name}_INTERFACES}" STREQUAL "")
        # if a package provides modern CMake interface targets use them
        # exclusively assuming the classic CMake variables only exist for
        # backward compatibility
        list_append_unique(interfaces ${${package_name}_INTERFACES})
      else()
        # otherwise use the classic CMake variables
        list_append_unique(definitions ${${package_name}_DEFINITIONS})
        list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
        list(APPEND libraries ${${package_name}_LIBRARIES})
        list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
      endif()
    endforeach()
    target_compile_definitions(${target}
      ${required_keyword} ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_link_libraries(${target}
      ${optional_keyword} ${interfaces})
    target_include_directories(${target}
      ${required_keyword} ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target}
      ${optional_keyword} ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()
