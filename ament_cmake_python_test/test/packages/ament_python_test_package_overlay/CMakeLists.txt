# Copyright 2025 Open Source Robotics Foundation, Inc.
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

cmake_minimum_required(VERSION 3.12)

project(ament_python_test_package_overlay)

find_package(ament_cmake_core REQUIRED)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
    ament_python_test_package_overlay
    PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/ament_python_test_package"
)
ament_python_install_package(
    ament_python_test_package_overlay
    PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/ament_python_test_package_overlay"
)

ament_package()
