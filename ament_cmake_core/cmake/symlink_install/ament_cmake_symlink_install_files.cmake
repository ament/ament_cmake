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

set(__AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX "0"
  CACHE INTERNAL "Index for unique symlink install files")

#
# Reimplement CMake install(FILES) command to use symlinks instead of copying
# resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_files files_keyword)
  if(NOT files_keyword STREQUAL "FILES")
    message(FATAL_ERROR "ament_cmake_symlink_install_files() first argument "
      "must be 'FILES', not '${files_keyword}'")
  endif()

  set(unsupported_keywords
    "PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(FILES ${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")

    # generate unique files
    set(generated_file_base
      "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_symlink_install_files_${__AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX}")
    set(generated_file_generator_suffix "${generated_file_base}_$<CONFIG>.cmake")
    set(generated_file_variable_suffix "${generated_file_base}_\${CMAKE_INSTALL_CONFIG_NAME}.cmake")
    math(EXPR __AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX
      "${__AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX} + 1")
    set(__AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX "${__AMENT_CMAKE_SYMLINK_INSTALL_FILES_INDEX}"
      CACHE INTERNAL "Index for unique symlink install files")

    file(GENERATE OUTPUT "${generated_file_generator_suffix}"
      CONTENT
      "ament_cmake_symlink_install_files(\"${CMAKE_CURRENT_SOURCE_DIR}\" FILES ${argn_quoted})\n")
    ament_cmake_symlink_install_append_install_code(
      "include(\"${generated_file_variable_suffix}\")"
      COMMENTS "install(FILES ${argn_quoted})"
    )
  endif()
endfunction()
