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

# copied from ament_cmake_core/ament_cmake_environment_hooks-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION
  "Generate environment files in the package share folder" ON)

include(
  "${ament_cmake_core_DIR}/environment_hooks/ament_environment_hooks.cmake")
include(
  "${ament_cmake_core_DIR}/environment_hooks/ament_generate_package_environment.cmake")

ament_register_extension("ament_package" "ament_cmake_core"
  "environment_hooks/ament_cmake_environment_hooks_package_hook.cmake")
