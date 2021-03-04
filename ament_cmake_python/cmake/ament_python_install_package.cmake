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
# Install a Python package (and its recursive subpackages) as a flat Python egg
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
# :param SKIP_COMPILE: if set do not byte-compile the installed package
# :type SKIP_COMPILE: option
# :param NO_DATA: if set do not install any package data
# :type NO_DATA: option
#
macro(ament_python_install_package)
  _ament_cmake_python_register_environment_hook()
  _ament_cmake_python_install_package(${ARGN})
  _ament_cmake_python_install_package_hook()
endmacro()

function(_ament_cmake_python_install_package package_name)
  cmake_parse_arguments(ARG "SKIP_COMPILE;NO_DATA" "PACKAGE_DIR;VERSION;SETUP_CFG" "" ${ARGN})
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

  set(build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${package_name}")

  if(ARG_NO_DATA)
    string(CONFIGURE "\
import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='${package_name}',
    version='${ARG_VERSION}',
    packages=find_packages(
        include=('${package_name}', '${package_name}.*')),
)
" setup_py_content)
  else()
    string(CONFIGURE "\
import os
from setuptools import find_packages
from setuptools import setup

from ament_cmake_python import find_packages_data

setup(
    name='${package_name}',
    version='${ARG_VERSION}',
    packages=find_packages(
        include=('${package_name}', '${package_name}.*')),
    package_data=find_packages_data(
        include=('${package_name}', '${package_name}.*'))
)
" setup_py_content)
  endif()

  file(GENERATE
    OUTPUT "${build_dir}/setup.py"
    CONTENT "${setup_py_content}"
  )

  if(ARG_SETUP_CFG)
    add_custom_target(
      ament_cmake_python_symlink_${package_name}_setup ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${ARG_SETUP_CFG}" "${build_dir}/setup.cfg"
    )
  endif()

  add_custom_target(
    ament_cmake_python_symlink_${package_name} ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      "${ARG_PACKAGE_DIR}" "${build_dir}/${package_name}"
  )

  set(install_dir "${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}")
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" install_prefix)
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}/bin" install_scripts_dir)

  if(NOT AMENT_CMAKE_SYMLINK_INSTALL)
    # Install as flat Python egg to mimic https://github.com/colcon/colcon-core
    # handling of pure Python packages for non-symlink installs.
    if(NOT ARG_SKIP_COMPILE)
      set(extra_install_args --compile)
    else()
      set(extra_install_args --no-compile)
    endif()

    # NOTE(hidmic): Allow setup.py install to build, as there is no way to
    # determine the Python package's source dependencies for proper build
    # invalidation.
    install(CODE
      "set(install_dir ${install_dir})
       set(extra_install_args ${extra_install_args})
       if(DEFINED ENV{DESTDIR} AND NOT \"\$ENV{DESTDIR}\" STREQUAL \"\")
         list(APPEND extra_install_args --root \$ENV{DESTDIR})
         file(TO_CMAKE_PATH \"\$ENV{DESTDIR}/\${install_dir}\" install_dir)
       endif()
       message(STATUS
         \"Installing: ${package_name} as flat Python egg under \${install_dir}\")
       execute_process(
         COMMAND
           \"${PYTHON_EXECUTABLE}\" setup.py install
             --single-version-externally-managed
             --install-scripts ${install_scripts_dir}
             --prefix ${install_prefix}
             --record install.log
             \${extra_install_args}
         WORKING_DIRECTORY \"${build_dir}\"
         OUTPUT_QUIET
       )"
    )
  else()
    # NOTE(hidmic): Make sure build directory makes it into the PYTHONPATH.
    # See https://github.com/colcon/colcon-core/pull/163 for further reference.
    file(TO_NATIVE_PATH "${build_dir}" build_prefix)

    if(NOT "${package_name}" STREQUAL "${PROJECT_NAME}")
      set(hook_name "${package_name}_python_develop")
    else()
      set(hook_name "python_develop")
    endif()

    if(WIN32)
      set(_ext "bat")
    else()
      set(_ext "sh")
    endif()

    configure_file(
      "${ament_cmake_python_DIR}/environment_hooks/${hook_name}.${_ext}.in"
      "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/environment_hooks/${hook_name}.${_ext}"
      @ONLY
    )

    # register information for .dsv generation
    set(_AMENT_CMAKE_PYTHON_INSTALL_HOOK_DESC
      "prepend-non-duplicate;PYTHONPATH;${build_prefix}"
      PARENT_SCOPE)

    set(_AMENT_CMAKE_PYTHON_INSTALL_HOOK
      "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/environment_hooks/${hook_name}.${_ext}"
      PARENT_SCOPE)

    # Install as Python egg link to mimic https://github.com/colcon/colcon-core
    # handling of pure Python packages for symlink installs.
    install(CODE
      "message(STATUS
         \"Symlinking: ${package_name} as flat Python egg under ${install_dir}\")
       execute_process(
         COMMAND
           \"${PYTHON_EXECUTABLE}\" setup.py develop
             --prefix ${install_prefix}
             --script-dir ${install_scripts_dir}
             --editable --no-deps
             --build-directory build
         WORKING_DIRECTORY \"${build_dir}\"
         OUTPUT_QUIET
       )"
    )
  endif()

  if(package_name IN_LIST AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES)
    message(FATAL_ERROR
      "ament_python_install_package() a Python module file or package with "
      "the same name '${package_name}' has been installed before")
  endif()
  list(APPEND AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES "${package_name}")
  set(AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES
    "${AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES}" PARENT_SCOPE)
endfunction()

macro(_ament_cmake_python_install_package_hook)
  if(_AMENT_CMAKE_PYTHON_INSTALL_HOOK)
    get_filename_component(hook_name "${_AMENT_CMAKE_PYTHON_INSTALL_HOOK}" NAME_WE)
    set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_${hook_name}
      "${_AMENT_CMAKE_PYTHON_INSTALL_HOOK_DESC}")
    ament_environment_hooks("${_AMENT_CMAKE_PYTHON_INSTALL_HOOK}")

    unset(_AMENT_CMAKE_PYTHON_INSTALL_HOOK_DESC)
    unset(_AMENT_CMAKE_PYTHON_INSTALL_HOOK)
  endif()
endmacro()
