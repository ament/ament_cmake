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
# Install a Python package (and its recursive subpackages).
#
# Can be called multiple times with the same package name to merge
# multiple source directories into a single installed package.
# When called more than once, source directories are merged at build time
# and the last call's parameters take precedence (last registered
# directory wins on file conflicts). Callers providing build-time-generated
# files should pass DEPENDS to ensure generation completes before the merge.
#
# :param package_name: the Python package name
# :type package_name: string
# :param PACKAGE_DIR: the path to the Python package directory (default:
#   <package_name> folder relative to the CMAKE_CURRENT_LIST_DIR)
# :type PACKAGE_DIR: string
# :param VERSION: the Python package version (default: package.xml version)
# :param VERSION: string
# :param SETUP_CFG: the path to a setup.cfg file (default:
#   setup.cfg file at CMAKE_CURRENT_LIST_DIR root, if any)
# :param SETUP_CFG: string
# :param DESTINATION: the path to the Python package installation
#   directory (default: PYTHON_INSTALL_DIR)
# :type DESTINATION: string
# :param SCRIPTS_DESTINATION: the path to the Python package scripts'
#   installation directory, scripts (if any) will be ignored if not set
# :type SCRIPTS_DESTINATION: string
# :param SKIP_COMPILE: if set do not byte-compile the installed package
# :type SKIP_COMPILE: option
# :param DEPENDS: build targets that must complete before
#   the package files are synced to the build directory
# :type DEPENDS: list of strings
#
macro(ament_python_install_package)
  _ament_cmake_python_register_extension_hook()
  _ament_cmake_python_register_environment_hook()
  _ament_cmake_python_install_package(${ARGN})
endmacro()

function(_ament_cmake_python_install_package package_name)
  cmake_parse_arguments(
    ARG "SKIP_COMPILE" "PACKAGE_DIR;VERSION;SETUP_CFG;DESTINATION;SCRIPTS_DESTINATION" "DEPENDS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_python_install_package() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_PACKAGE_DIR)
    set(ARG_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/${package_name}")
  endif()
  if(NOT IS_ABSOLUTE "${ARG_PACKAGE_DIR}")
    set(ARG_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/${ARG_PACKAGE_DIR}")
  endif()

  if(NOT ARG_VERSION)
    # Use package.xml version
    if(NOT _AMENT_PACKAGE_NAME)
      ament_package_xml()
    endif()
    set(ARG_VERSION "${${PROJECT_NAME}_VERSION}")
  endif()

  if(NOT EXISTS "${ARG_PACKAGE_DIR}/__init__.py")
    message(FATAL_ERROR "ament_python_install_package() the Python package "
      "folder '${ARG_PACKAGE_DIR}' doesn't contain an '__init__.py' file")
  endif()

  if(NOT ARG_SETUP_CFG)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/setup.cfg")
      set(ARG_SETUP_CFG "${CMAKE_CURRENT_LIST_DIR}/setup.cfg")
    endif()
  elseif(NOT IS_ABSOLUTE "${ARG_SETUP_CFG}")
    set(ARG_SETUP_CFG "${CMAKE_CURRENT_LIST_DIR}/${ARG_SETUP_CFG}")
  endif()

  if(NOT ARG_DESTINATION)
    if(NOT PYTHON_INSTALL_DIR)
      message(FATAL_ERROR "ament_python_install_package() variable "
        "'PYTHON_INSTALL_DIR' must not be empty")
    endif()
    set(ARG_DESTINATION ${PYTHON_INSTALL_DIR})
  endif()

  get_property(_pkgs GLOBAL PROPERTY AMENT_CMAKE_PYTHON_PKGS)
  list(FIND _pkgs "${package_name}" _idx)
  if(_idx EQUAL -1)
    set_property(GLOBAL APPEND PROPERTY AMENT_CMAKE_PYTHON_PKGS "${package_name}")
  else()
    message(STATUS "ament_python_install_package: extending '${package_name}'")
  endif()

  set_property(GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_SKIP_COMPILE "${ARG_SKIP_COMPILE}")
  set_property(GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_VERSION "${ARG_VERSION}")
  set_property(GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_SETUP_CFG "${ARG_SETUP_CFG}")
  set_property(GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_DESTINATION "${ARG_DESTINATION}")
  set_property(GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_SCRIPTS_DESTINATION "${ARG_SCRIPTS_DESTINATION}")
  set_property(GLOBAL APPEND PROPERTY AMENT_CMAKE_PYTHON_${package_name}_DEPENDS ${ARG_DEPENDS})
  set_property(GLOBAL APPEND PROPERTY AMENT_CMAKE_PYTHON_${package_name}_PACKAGE_DIRS "${ARG_PACKAGE_DIR}")
endfunction()
