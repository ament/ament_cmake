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

# copied from ament_cmake_test/ament_cmake_test-extras.cmake

# register environment hook for libraries once
option(AMENT_ENABLE_TESTING "Enable testing" OFF)
# option()
set(
  AMENT_TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results"
  CACHE STRING "The path where test results are generated"
)

if(AMENT_ENABLE_TESTING)
  enable_testing()
endif()

find_package(ament_cmake_core REQUIRED)

include("${ament_cmake_test_DIR}/ament_add_test.cmake")
