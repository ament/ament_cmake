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
# Deduplicate libraries.
#
# If the list contains duplicates only the last value is kept.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries.
#   Each element might either a build configuration keyword or a library.
# :type ARGN: list of strings
#
# @public
#
macro(ament_libraries_deduplicate VAR)
  ament_libraries_pack_build_configuration(_packed ${ARGN})
  set(_unique "")
  foreach(_lib ${_packed})
    # remove existing value if it exists
    list(REMOVE_ITEM _unique ${_lib})
    # append value to the end
    list(APPEND _unique ${_lib})
  endforeach()
  ament_libraries_unpack_build_configuration(${VAR} ${_unique})
endmacro()
