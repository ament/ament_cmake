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
# Invoke find_package() for all build and buildtool dependencies.
#
# :param REQUIRED: an optional list of package names that are known
# required CMake dependencies. For these dependencies, find_package() will be
# invoked with REQUIRED.
# :type REQUIRED: list of strings
#
# All found package names are appended to the
# ``${PROJECT_NAME}_FOUND_BUILD_DEPENDS`` /
# ``${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS`` variables.
#
# The content of the package specific variables of build dependencies
# ending with ``_DEFINITIONS``, ``_INCLUDE_DIRS`` and ``_LIBRARIES``
# are appended to the same variables starting with
# ``${PROJECT_NAME}_FOUND_``.
#
# @public
#
macro(ament_auto_find_build_dependencies)
  cmake_parse_arguments(_ARG "" "" "REQUIRED" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_auto_find_build_dependencies() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # ensure that the caller isn't expecting additional packages to be found
  set(_build_and_buildtool_depends
    ${${PROJECT_NAME}_BUILD_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
  )
  set(_additional_packages "")
  foreach(_package_name ${_ARG_REQUIRED})
    if(NOT _package_name IN_LIST _build_and_buildtool_depends)
      list(APPEND _additional_packages ${_package_name})
    endif()
  endforeach()
  if(_additional_packages)
    message(FATAL_ERROR "ament_auto_find_build_dependencies() called with "
      "required packages that are not listed as a build/buildtool dependency in "
      "the package.xml: ${_additional_packages}")
  endif()

  # try to find_package() all build dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILD_DEPENDS})
    if(_dep IN_LIST _ARG_REQUIRED)
      find_package(${_dep} REQUIRED QUIET)
    else()
      find_package(${_dep} QUIET)
    endif()
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS ${_dep})

      list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${${_dep}_DEFINITIONS})
      list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${${_dep}_INCLUDE_DIRS})
      list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${${_dep}_LIBRARIES})
    endif()
  endforeach()

  # try to find_package() all buildtool dependencies
  foreach(_dep ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})
    if(_dep IN_LIST _ARG_REQUIRED)
      find_package(${_dep} REQUIRED QUIET)
    else()
      find_package(${_dep} QUIET)
    endif()
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS ${_dep})
    endif()
  endforeach()
endmacro()
