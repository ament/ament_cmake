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
# Export dependencies to downstream packages.
#
# Behavior depends on whether ``COMPONENTS`` parameter is provided. If provided,
# ``ament_export_dependencies`` expects a single package name, followed a list
# of components to export. Otherwise, it expects a list of packages to export.
# Each package name must be find_package()-able with the exact same case.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be INCLUDE_DIRS and LIBRARIES.
#
# :param ARGN: a list of package names
# :type ARGN: list of strings
# :param COMPONENTS: optional list of components. In such cas, only 1 package
#                    name shall be listed
# :type COMPONENTS: list of strings
#
# @public
#
macro(ament_export_dependencies)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_dependencies() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_dependencies_register_package_hook()

    cmake_parse_arguments(_ARGS "" "" "COMPONENTS" ${ARGN})
    if(NOT "${_ARGS_COMPONENTS}" STREQUAL "")
      # parse a single dependency with its components
      list(LENGTH _ARGS_UNPARSED_ARGUMENTS _unparsed_length)
      if (NOT ${_unparsed_length} EQUAL "1")
        message(FATAL_ERROR
          "ament_export_dependencies() called with unknown arguments: ${ARGN}")
      endif()
      # pack the dependency and all its components "dep:compA:compB:compC"
      # TODO use AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR-like variable instead of hardcoded ":"?
      set (_unpacked "${_ARGS_UNPARSED_ARGUMENTS}")
      list(APPEND _unpacked "${_ARGS_COMPONENTS}")
      string(REGEX REPLACE ";" ":" _packed "${_unpacked}")
      list(APPEND _AMENT_CMAKE_EXPORT_DEPENDENCIES "${_packed}")

    else()
      foreach(_arg ${ARGN})
        # only pass package name
        # will be resolved by downstream packages
        # must be find_package()-able
        # and provide _INCLUDE_DIRS and _LIBRARIES
        list(APPEND _AMENT_CMAKE_EXPORT_DEPENDENCIES "${_arg}")
      endforeach()

    endif() 
  endif()

endmacro()
