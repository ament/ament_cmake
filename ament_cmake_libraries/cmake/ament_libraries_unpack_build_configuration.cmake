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
# Unpack a list of libraries with optional build configuration keyword prefixes.
#
# Libraries prefixed with a keyword are split into the keyword and the library.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
macro(ament_libraries_unpack_build_configuration VAR)
  set(${VAR} "")
  foreach(_lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" _lib "${_lib}")
    list(APPEND ${VAR} "${_lib}")
  endforeach()
endmacro()
