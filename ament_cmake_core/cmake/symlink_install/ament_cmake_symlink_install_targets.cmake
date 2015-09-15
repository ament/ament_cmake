# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  if(NOT "${ARGV0} " STREQUAL "TARGETS ")
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() first argument "
      "must be 'TARGETS', not '${ARGV0}'")
  endif()

  set(unsupported_keywords
    "EXPORT"
    "FRAMEWORK"
    "BUNDLE"
    "PRIVATE_HEADER"
    "PUBLIC_HEADER"
    "RESOURCE"
    "INCLUDES"
    "PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "NAMELINK_ONLY"
    "NAMELINK_SKIP"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    cmake_parse_arguments(ARG "ARCHIVE;LIBRARY;RUNTIME;OPTIONAL" "DESTINATION"
      "TARGETS" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
        "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    # convert target names into absolute files
    set(target_files "")
    foreach(target ${ARG_TARGETS})
      if(NOT TARGET ${target})
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() '${target}' is not a target")
      endif()
      get_target_property(is_imported "${target}" IMPORTED)
      if(is_imported)
        message(FATAL_ERROR "ament_cmake_symlink_install_targets() "
          "'${target}' is an imported target")
      endif()
      # TODO consider using a generator expression instead
      # $<TARGET_FILE:target>
      # until then set the policy explicitly in order to avoid warning with newer CMake versions.
      if(POLICY CMP0026)
        cmake_policy(SET CMP0026 OLD)
      endif()
      get_property(location TARGET ${target} PROPERTY LOCATION)
      list(APPEND target_files "${location}")
    endforeach()

    string(REPLACE ";" "\" \"" target_files_quoted
      "\"TARGET_FILES;${target_files}\"")
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")

    # join destination keyword with kind of target (e.g. ARCHIVE)
    # to simplify parsing in the next CMake function
    string(REPLACE "\"ARCHIVE\" \"DESTINATION\"" "\"ARCHIVE_DESTINATION\"" argn_quoted "${argn_quoted}")
    string(REPLACE "\"LIBRARY\" \"DESTINATION\"" "\"LIBRARY_DESTINATION\"" argn_quoted "${argn_quoted}")
    string(REPLACE "\"RUNTIME\" \"DESTINATION\"" "\"RUNTIME_DESTINATION\"" argn_quoted "${argn_quoted}")

    ament_cmake_symlink_install_append_install_code(
      "ament_cmake_symlink_install_targets(${target_files_quoted};${argn_quoted})"
      COMMENTS "install(${argn_quoted})"
    )
  endif()
endfunction()
