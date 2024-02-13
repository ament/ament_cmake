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

# ament_cmake needs a Python 3 interpreter
# If a specific Python version is required then find it before finding ament_cmake
# Example:
#   find_package(Python3 3.8 REQUIRED)
#   find_package(ament_cmake REQUIRED)
if(NOT TARGET Python3::Interpreter)
  # By default, without the settings below, find_package(Python3) will attempt
  # to find the newest python version it can, and additionally will find the
  # most specific version.  For instance, on a system that has
  # /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
  # /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
  # The behavior we want is to prefer the "system" installed version unless the
  # user specifically tells us othewise through the Python3_EXECUTABLE hint.
  # Setting CMP0094 to NEW means that the search will stop after the first
  # python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
  # the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
  # latter functionality is only available in CMake 3.20 or later, so we need
  # at least that version.
  cmake_minimum_required(VERSION 3.20)
  cmake_policy(SET CMP0094 NEW)
  set(Python3_FIND_UNVERSIONED_NAMES FIRST)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()
