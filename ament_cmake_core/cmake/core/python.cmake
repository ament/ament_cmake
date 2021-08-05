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
if (NOT TARGET Python3::Interpreter)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()
