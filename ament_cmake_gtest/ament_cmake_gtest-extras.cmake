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

# copied from ament_cmake_gtest/ament_cmake_gtest-extras.cmake

# find gtest and create library targets once
macro(_ament_cmake_gtest_find_gtest)
  if(NOT DEFINED _AMENT_CMAKE_GTEST_FIND_GTEST)
    set(_AMENT_CMAKE_GTEST_FIND_GTEST TRUE)

    find_package(ament_cmake_test QUIET REQUIRED)
    find_package(GTest REQUIRED)
    ament_find_gtest()
  endif()
endmacro()

include("${ament_cmake_gtest_DIR}/ament_add_gtest.cmake")
include("${ament_cmake_gtest_DIR}/ament_add_gtest_executable.cmake")
include("${ament_cmake_gtest_DIR}/ament_add_gtest_test.cmake")
include("${ament_cmake_gtest_DIR}/ament_find_gtest.cmake")
