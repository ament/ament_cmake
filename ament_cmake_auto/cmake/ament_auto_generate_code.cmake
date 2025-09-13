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
# Generate code from msg/srv/action interface files.
#
# Requires that the package declares all of the following in `package.xml`:
# - runtime dependency on `rosidl_default_runtime`
# - buildtool dependency on `rosidl_default_generators`
# - membership in group `rosidl_interface_packages`
# If any is missing, the macro fails with a hint message.
#
# :param TARGETS: list of targets in same package that should be linked against
#   same package's interface typesupport library so they can use the generated code of interfaces.
# :type TARGETS: list of strings
#
# @public
#
macro(ament_auto_generate_code)
  cmake_parse_arguments(_ARG "" "" "TARGETS" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_auto_generate_code called with unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  # Ensure package.xml is parsed so *_DEPENDS variables are available
  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # Validate required <exec_depend> tags
  if(NOT rosidl_default_runtime IN_LIST ${PROJECT_NAME}_EXEC_DEPENDS)
    message(FATAL_ERROR
      "ament_auto_generate_code: '${PROJECT_NAME}' must declare an exec_depend on 'rosidl_default_runtime' to use generated interfaces at runtime.\n"
      "Hint:\n"
      "  - package.xml: add <exec_depend>rosidl_default_runtime</exec_depend>\n")
  endif()

  # Validate required <buildtool_depend> tags
  if(rosidl_default_generators IN_LIST ${PROJECT_NAME}_BUILDTOOL_DEPENDS)
    message(FATAL_ERROR
      "ament_auto_generate_code: '${PROJECT_NAME}' must declare a buildtool dependency on 'rosidl_default_generators' to generate interfaces.\n"
      "Hint:\n"
      "  - package.xml: add <buildtool_depend>rosidl_default_generators</buildtool_depend>\n")
  endif()

  # Validate required <member_of_group> tags
  if(NOT rosidl_interface_packages IN_LIST ${PROJECT_NAME}_MEMBER_OF_GROUPS)
    message(FATAL_ERROR
      "ament_auto_generate_code: '${PROJECT_NAME}' should join a `rosidl_interface_packages` group so tools can discover interface packages.\n"
      "Hint:\n"
      "  - package.xml: add <member_of_group>rosidl_interface_packages</member_of_group>\n")
  endif()

  set(${PROJECT_NAME}_interface_files "")
  file(
    GLOB
    ${PROJECT_NAME}_interface_files
    RELATIVE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    CONFIGURE_DEPENDS
    "msg/*.msg"
    "srv/*.srv"
    "action/*.action"
  )

  if(NOT ${PROJECT_NAME}_interface_files)
    message(WARNING "ament_auto_generate_code: no message, service, or action files found")
    return()
  endif()

  # Only forward DEPENDENCIES if present to avoid passing empty args
  set(_rosidl_args
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_interface_files}
  )
  if(${PROJECT_NAME}_FOUND_BUILD_DEPENDS)
    list(APPEND _rosidl_args DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endif()
  rosidl_generate_interfaces(${_rosidl_args})
  ament_export_dependencies(rosidl_default_runtime)

  # Optionally wire interfaces into same-package targets
  if(_ARG_TARGETS)
    rosidl_get_typesupport_target(${PROJECT_NAME}_auto_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    foreach(target IN LISTS _ARG_TARGETS)
      target_link_libraries(${target} ${${PROJECT_NAME}_auto_typesupport_target})
    endforeach()
  endif()

  ament_execute_extensions(ament_auto_generate_code)
endmacro()
