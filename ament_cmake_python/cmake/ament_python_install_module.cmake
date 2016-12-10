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
# Install a Python module.
#
# :param module_file: the Python module file
# :type MODULE_FILE: string
# :param DESTINATION: the base package to install the module to, rooted on
#   PYTHON_INSTALL_DIR (default: install as a top level module)
# :type DESTINATION: string
#
macro(ament_python_install_module)
  _ament_cmake_python_register_environment_hook()
  _ament_cmake_python_install_module(${ARGN})
endmacro()

function(_ament_cmake_python_install_module module_file)
  cmake_parse_arguments(ARG "" "DESTINATION" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_python_install_module() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT IS_ABSOLUTE "${module_file}")
    set(module_file "${CMAKE_CURRENT_LIST_DIR}/${module_file}")
  endif()
  if(NOT EXISTS "${module_file}")
    message(FATAL_ERROR "ament_python_install_module() the Python module file "
      "'${module_file}' doesn't exist")
  endif()

  if(NOT PYTHON_INSTALL_DIR)
    message(FATAL_ERROR "ament_python_install_module() variable "
      "'PYTHON_INSTALL_DIR' must not be empty")
  endif()

  set(destination "")
  set(module_name "")
  get_filename_component(name "${module_file}" NAME_WE)

  if(NOT ARG_DESTINATION)
    set(destination "${PYTHON_INSTALL_DIR}")
    set(module_name "${name}")
  else()
    set(destination "${PYTHON_INSTALL_DIR}/${ARG_DESTINATION}")
    set(module_name "${ARG_DESTINATION}/${name}")
  endif()

  install(
    FILES "${module_file}"
    DESTINATION "${destination}"
  )
  # TODO optionally compile Python file

  get_filename_component(name "${module_file}" NAME_WE)
  list(FIND AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES "${destination}" index)
  if(NOT index EQUAL -1)
    message(FATAL_ERROR "ament_python_install_module() a Python module file "
      "or package with the same name '${destination}' has been installed before")
  endif()
  list(APPEND AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES "${destination}")
  set(AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES
    "${AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES}" PARENT_SCOPE)
endfunction()
