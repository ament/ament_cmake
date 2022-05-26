# Copyright 2022 Open Source Robotics Foundation, Inc.
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
# Explicit call to add default CMake options (e.g. BUILD_SHARED_LIBS)
#
# .. note:: It can be called multiple times, but should be called
#   once for any package that requires the options.
#
# :param EXCLUDE_BUILD_SHARED_LIBS: Exclude the BUILD_SHARED_LIBS option
#
# @public
#
macro(ament_add_default_options)
  # TODO(CH3): Would be good to parse args to skip options next time.
  set(options EXCLUDE_BUILD_SHARED_LIBS)
  set(oneValueArgs)
  set(multiValueArgs)
  cmake_parse_arguments(ament_add_default_options
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT ament_add_default_options_EXCLUDE_BUILD_SHARED_LIBS)
    option(
      BUILD_SHARED_LIBS
      "Global flag to cause add_library() to create shared libraries if on. \
    If set to true, this will cause all libraries to be built shared \
    unless the library was explicitly added as a static library."
      ON)
  endif()
endmacro()
