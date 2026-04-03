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

# copied from ament_cmake_gmock/ament_cmake_gmock-extras.cmake

# find gmock and create library targets once
macro(_ament_cmake_gmock_find_gmock)
  if(NOT DEFINED _AMENT_CMAKE_GMOCK_FIND_GMOCK)
    set(_AMENT_CMAKE_GMOCK_FIND_GMOCK TRUE)

    find_package(ament_cmake_test QUIET REQUIRED)
    # Not a typo. GTest provides GMock
    find_package(GTest REQUIRED)
    ament_find_gmock()
  endif()
endmacro()


include("${ament_cmake_gmock_DIR}/ament_add_gmock.cmake")
include("${ament_cmake_gmock_DIR}/ament_add_gmock_executable.cmake")
include("${ament_cmake_gmock_DIR}/ament_add_gmock_test.cmake")
include("${ament_cmake_gmock_DIR}/ament_find_gmock.cmake")
