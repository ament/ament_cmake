# Copyright 2026 Open Source Robotics Foundation, Inc.
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
# Prepend a value to an environment variable in the generated package.dsv file.
#
# Usage:
#   ament_environment_prepend_non_duplicate(VAR value [IF_EXISTS])
#
function(ament_environment_prepend_non_duplicate var value)
  set(options IF_EXISTS)
  cmake_parse_arguments(ARG "${options}" "" "" ${ARGN})

  if(ARG_IF_EXISTS)
    set(prefix "IF_EXISTS")
  else()
    set(prefix "ALWAYS")
  endif()

  list(APPEND _AMENT_CMAKE_ENVIRONMENT_PREPEND_NON_DUPLICATE "${prefix}|${var}|${value}")
  set(_AMENT_CMAKE_ENVIRONMENT_PREPEND_NON_DUPLICATE "${_AMENT_CMAKE_ENVIRONMENT_PREPEND_NON_DUPLICATE}" PARENT_SCOPE)
endfunction()
