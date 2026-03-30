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

function(ament_cmake_python_install_registered_packages)
  get_property(_pkgs GLOBAL PROPERTY AMENT_CMAKE_PYTHON_PKGS)
  foreach(pkg IN LISTS _pkgs)
    _ament_cmake_python_install_package_impl(${pkg})
  endforeach()
endfunction()

function(_ament_cmake_python_install_package_impl package_name)
  foreach(_prop IN ITEMS SKIP_COMPILE VERSION SETUP_CFG DESTINATION SCRIPTS_DESTINATION PACKAGE_DIRS DEPENDS)
    get_property(_${_prop} GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_${_prop})
  endforeach()

  _ament_cmake_python_prepare_build(${package_name})
  _ament_cmake_python_copy_build_files(${package_name})

  # Technically, we should call find_package(Python3) first to ensure that Python3::Interpreter
  # is available.  But we skip this here because this macro requires ament_cmake, and ament_cmake
  # calls find_package(Python3) for us.
  get_executable_path(python_interpreter Python3::Interpreter BUILD)

  _ament_cmake_python_generate_egg(${package_name})

  if(_SCRIPTS_DESTINATION)
  _ament_cmake_python_install_scripts(${package_name})
  endif()

  _ament_cmake_python_install_sources(${package_name})

  if(NOT _SKIP_COMPILE)
    _ament_cmake_python_byte_compile(${package_name})
  endif()

endfunction()

macro(_ament_cmake_python_prepare_build package_name)
  set(_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${package_name}")

  string(CONFIGURE "\
from setuptools import find_packages
from setuptools import setup

setup(
  name='${package_name}',
  version='${_VERSION}',
  packages=find_packages(
      include=('${package_name}', '${package_name}.*')),
)
" setup_py_content)

  file(GENERATE
    OUTPUT "${_build_dir}/setup.py"
    CONTENT "${setup_py_content}"
  )

endmacro()

macro(_ament_cmake_python_copy_build_files package_name)
  set(_sync_target "ament_cmake_python_sync_${package_name}")

  set(_sync_commands)
  foreach(_dir IN LISTS _PACKAGE_DIRS)
    list(APPEND _sync_commands
      COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${_dir}" "${_build_dir}/${package_name}")
  endforeach()

  if(_SETUP_CFG)
    list(APPEND _sync_commands
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${_SETUP_CFG}" "${_build_dir}/setup.cfg")
  endif()

  add_custom_target(${_sync_target} ${_sync_commands})

  if(_DEPENDS)
    add_dependencies(${_sync_target} ${_DEPENDS})
  endif()

endmacro()

macro(_ament_cmake_python_generate_egg package_name)
  add_custom_target(
    ament_cmake_python_build_${package_name}_egg ALL
    COMMAND ${python_interpreter} setup.py egg_info
    WORKING_DIRECTORY "${_build_dir}"
    DEPENDS ${_sync_target}
  )

  set(python_version "py${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

  set(egg_name "${package_name}")
  set(egg_install_name "${egg_name}-${_VERSION}")
  set(egg_install_name "${egg_install_name}-${python_version}")

  install(
    DIRECTORY "${_build_dir}/${egg_name}.egg-info/"
    DESTINATION "${_DESTINATION}/${egg_install_name}.egg-info"
  )
endmacro()

macro(_ament_cmake_python_install_scripts package_name)
  file(MAKE_DIRECTORY "${_build_dir}/scripts")  # setup.py may or may not create it

  add_custom_target(
    ament_cmake_python_build_${package_name}_scripts ALL
    COMMAND ${python_interpreter} setup.py install_scripts -d scripts
    WORKING_DIRECTORY "${_build_dir}"
    DEPENDS ${_sync_target}
  )

  if(NOT AMENT_CMAKE_SYMLINK_INSTALL)
    # Not needed for nor supported by symlink installs
    set(_extra_install_args USE_SOURCE_PERMISSIONS)
  endif()

  install(
    DIRECTORY "${_build_dir}/scripts/"
    DESTINATION "${_SCRIPTS_DESTINATION}/"
    ${_extra_install_args}
  )
endmacro()

macro(_ament_cmake_python_install_sources package_name)
  list(LENGTH _PACKAGE_DIRS _num_dirs)
  if(_num_dirs EQUAL 1)
    # For single dir we install from source to maintain the original behavior
    install(
      DIRECTORY "${_PACKAGE_DIRS}/"
      DESTINATION "${_DESTINATION}/${package_name}"
      PATTERN "*.pyc"       EXCLUDE
      PATTERN "__pycache__" EXCLUDE
    )
  elseif(AMENT_CMAKE_SYMLINK_INSTALL)
    set(_DIRS_TO_INSTALL "${_PACKAGE_DIRS}")
    list(TRANSFORM _DIRS_TO_INSTALL APPEND "/")
    foreach(_dir IN LISTS _DIRS_TO_INSTALL)
      install(
        DIRECTORY "${_dir}"
        DESTINATION "${_DESTINATION}/${package_name}"
        PATTERN "*.pyc"       EXCLUDE
        PATTERN "__pycache__" EXCLUDE
      )
    endforeach()
  else()
    install(
      DIRECTORY "${_build_dir}/${package_name}/"
      DESTINATION "${_DESTINATION}/${package_name}"
      PATTERN "*.pyc"       EXCLUDE
      PATTERN "__pycache__" EXCLUDE
    )
  endif()
endmacro()

macro(_ament_cmake_python_byte_compile package_name)
  get_executable_path(python_interpreter_config Python3::Interpreter CONFIGURE)
  # compile Python files
  install(CODE
    "execute_process(
      COMMAND
      \"${python_interpreter_config}\" \"-m\" \"compileall\"
      \"${CMAKE_INSTALL_PREFIX}/${_DESTINATION}/${package_name}\"
    )"
  )
endmacro()

ament_cmake_python_install_registered_packages()
