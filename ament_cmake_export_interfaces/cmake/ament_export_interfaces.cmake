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
# Export interfaces to downstream packages.
#
# Each interface name must have been used to install targets using
# ``install(TARGETS ... EXPORT name ...)``.
# The ``install(EXPORT ...)`` invocation is handled by this macros.
#
# :param ARGN: a list of export names
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_interfaces)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_interfaces() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_interfaces_register_package_hook()
    foreach(_arg ${ARGN})
      list(APPEND _AMENT_CMAKE_EXPORT_INTERFACES "${_arg}")
    endforeach()
  endif()
endmacro()
