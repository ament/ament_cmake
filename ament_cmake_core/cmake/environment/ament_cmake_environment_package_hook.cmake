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

if(AMENT_CMAKE_ENVIRONMENT_GENERATION)
  ament_generate_environment()
endif()

function(ament_cmake_environment_generate_package_run_dependencies_marker)
  # use all run dependencies from the package.xml
  set(run_depends "")
  list_append_unique(run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS}
    ${${PROJECT_NAME}_TEST_DEPENDS})

  # register direct run dependencies
  ament_index_register_resource("package_run_dependencies" CONTENT "${run_depends}")
endfunction()

function(ament_cmake_environment_generate_parent_prefix_path_marker)
  ament_index_register_resource("parent_prefix_path" CONTENT "$ENV{AMENT_PREFIX_PATH}")
endfunction()

if(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION)
  ament_cmake_environment_generate_package_run_dependencies_marker()
  ament_cmake_environment_generate_parent_prefix_path_marker()
endif()
