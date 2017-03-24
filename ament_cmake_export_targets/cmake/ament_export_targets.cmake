# Copyright 2017 Open Source Robotics Foundation, Inc.
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
# Export targets to downstream packages.
#
# Each target name must be valid target.
# The targets are being installed to `lib` (archive and library) or `bin`
# (runtime).
#
# After being find_package()-ed the variable
# `<project_name>_EXPORTED_TARGET_NAMES` contains the passed target names.
# For each target name an imported target with the name
# `<project_name>::<target_name>` is created.
#
# :param ARGN: a list of target names
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_targets)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_targets() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_targets_register_package_hook()
    foreach(_arg ${ARGN})
      # ensure argument is a valid target name
      if(NOT TARGET "${_arg}")
        message(FATAL_ERROR
          "ament_export_targets() argument '${_arg}' is not a target")
      endif()
      list(APPEND _AMENT_CMAKE_EXPORT_TARGETS "${_arg}")
    endforeach()
  endif()
endmacro()
