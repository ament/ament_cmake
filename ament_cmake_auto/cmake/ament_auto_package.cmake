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
# Export information, install files and targets, execute the
# extension point ``ament_auto_package`` and invoke
# ``ament_package()``.
#
# :param INSTALL_TO_PATH: if set, install executables to `bin` so that
#   they are available on the `PATH`.
#   By default they are being installed into `lib/${PROJECT_NAME}`.
#   It is currently not possible to install some executable into `bin`
#   and some into `lib/${PROJECT_NAME}`.
#   Libraries are not affected by this option.
#   They are always installed into `lib` and `dll`s into `bin`.
# :type INSTALL_TO_PATH: option
# :param INSTALL_TO_SHARE: a list of directories to be installed to the
#   package's share directory
# :type INSTALL_TO_SHARE: list of strings
#
# Export all found build dependencies which are also run
# dependencies.
# If the package has an include directory install all recursively
# found header files (ending in h, hh, hpp, hxx) and export the
# include directory.
# Export and install all library targets and install all executable
# targets.
#
# @public
#

macro(ament_auto_package)
  cmake_parse_arguments(_ARG "INSTALL_TO_PATH" "" "INSTALL_TO_SHARE" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_auto_find_build_dependencies() called with "
      "unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  # export all found build dependencies which are also run dependencies
  set(_run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS})
  foreach(_dep
      ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
      ${${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS})
    if(_dep IN_LIST _run_depends)
      ament_export_dependencies("${_dep}")
    endif()
  endforeach()

  # export and install include directory of this package if it has one
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    ament_export_include_directories("include")
    install(DIRECTORY include/ DESTINATION include)
  endif()

  # export and install all libraries
  if(NOT ${PROJECT_NAME}_LIBRARIES STREQUAL "")
    ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
    install(
      TARGETS ${${PROJECT_NAME}_LIBRARIES}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  # install all executables
  if(NOT ${PROJECT_NAME}_EXECUTABLES STREQUAL "")
    if(_ARG_INSTALL_TO_PATH)
      set(_destination "bin")
    else()
      set(_destination "lib/${PROJECT_NAME}")
    endif()
    install(
      TARGETS ${${PROJECT_NAME}_EXECUTABLES}
      DESTINATION ${_destination}
    )
  endif()

  # install directories to share
  foreach(_dir ${_ARG_INSTALL_TO_SHARE})
    install(
      DIRECTORY "${_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endforeach()

  ament_execute_extensions(ament_auto_package)

  ament_package()
endmacro()
