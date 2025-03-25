# Copyright 2025 Open Source Robotics Foundation, Inc.
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
# Make a target depend on everything provided by another CMake package.
#
# This function is intended to be used internally by ament_cmake_auto.
#
# :param target: the name of the target
# :type target: string
# :param SYSTEM: Optional. If given, and if a package provides old
#   style standard CMake variables instead of modern CMake targets, then
#   the include directories from this dependency will be treated as system
#   includes.
#   This property has no effect if the package being depended upon provides
#   modern CMake targets.
# :type SYSTEM: None
# :param SCOPE: Optional. If given it must be one of PUBLIC, PRIVATE, or INTERFACE.
#   See target_link_libraries() documentation for more info about SCOPE. If unset
#   then it defaults to unset for CMake functions that support that, and PUBLIC for
#   functions that don't.
# :type SCOPE: string
# :param PACKAGES: a list of package names
# :type PACKAGES: list of strings
#
# @private
#
function(ament_auto_depend_on_packages target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_auto_depend_on_packages() the first argument must be a valid target name")
  endif()
  cmake_parse_arguments(ARG
    "SYSTEM"
    "SCOPE"
    "PACKAGES"
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_auto_depend_on_packages() called with "
      "unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  set(allowed_scopes PUBLIC PRIVATE INTERFACE)
  if(ARG_SCOPE AND NOT ARG_SCOPE IN_LIST allowed_scopes)
    message(FATAL_ERROR "If SCOPE is specified, it must be one of: ${allowed_scopes}. Got: '${ARG_SCOPE}'")
  endif()

  if(ARG_SYSTEM)
    # convert bool -> string that is passed to target_include_directories
    set(_system "SYSTEM")
  else()
    set(_system "")
  endif()

  # Most target_*() macros require a keyword, but
  # target_link_libraries has special behavior when the keyword is
  # unset, so default to PUBLIC for most functions when unset
  set(_implied_scope ${ARG_SCOPE})
  if(NOT _implied_scope)
    set(_implied_scope "PUBLIC")
  endif()

  foreach(package_name ${ARG_PACKAGES})
    if(NOT "${${package_name}_FOUND}")
      message(FATAL_ERROR "'${package_name}' must be found with find_package() prior to passing it to ament_auto_depend_on_packages()")
    endif()

    if(${package_name}_TARGETS)
      # Use modern CMake targets
      target_link_libraries(${target} ${ARG_SCOPE} ${${package_name}_TARGETS})
    else()
      # Use standard CMake variables
      # https://cmake.org/cmake/help/latest/manual/cmake-developer.7.html#standard-variable-names
      if(${package_name}_INCLUDE_DIRS)
        # Order include directories to mitigate issues that come from
        # overriding packages without having a package-specifc include directory
        ament_include_directories_order(ordered_include_dirs ${${package_name}_INCLUDE_DIRS})
        target_include_directories(${target} ${_system} ${_implied_scope} ${ordered_include_dirs})
      endif()
      if(${package_name}_LIBRARIES)
        # Deduplicate libraries to speed up linking in leaf packages.
        ament_libraries_deduplicate(unique_libraries ${${package_name}_LIBRARIES})
        target_link_libraries(${target} ${ARG_SCOPE} ${unique_libraries})
      endif()
      if(${package_name}_LIBRARY_DIRS)
        # Remove duplicates might not be necessary here, but doesn't hurt
        list(REMOVE_DUPLICATES ${${package_name}_LIBRARY_DIRS})
        target_link_directories(${target} ${_implied_scope} ${${package_name}_LIBRARY_DIRS})
      endif()
      if(${package_name}_DEFINITIONS)
        # Remove duplicates might not be necessary here, but doesn't hurt
        list(REMOVE_DUPLICATES ${${package_name}_DEFINITIONS})
        target_compile_definitions(${target} ${_implied_scope} ${${package_name}_DEFINITIONS})
      endif()
    endif()
  endforeach()
endfunction()
