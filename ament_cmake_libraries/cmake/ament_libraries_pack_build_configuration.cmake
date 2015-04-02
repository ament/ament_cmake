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
# Pack a list of libraries with optional build configuration keywords.
#
# Each keyword is joined with its library using a separator.
# A packed library list can be deduplicated easily.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
macro(ament_libraries_pack_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} _lib)
    if("${_lib}" MATCHES "^debug|optimized|general$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "ament_libraries_pack_build_configuration() the list of libraries '${_argn}' ends with '${_lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${_lib}${AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${_lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()
