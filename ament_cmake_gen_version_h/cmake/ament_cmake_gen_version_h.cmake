# Copyright 2019 Open Source Robotics Foundation, Inc.
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

################################################################################
# `ament_cmake_gen_version_h` function creates and installs a version header file.
# The version is taken from `package.xml` file's `<version>` tag
# The function uses a provided "version.h.in" template file to generate
# the destination `version.h` file and installs that generated file into `DESTINATION include`.
# The generated file is being (re-)created if:
# - the file does not exist
# - the file does exists but contains a version that differs from the version in `package.xml` file
################################################################################
function(ament_cmake_gen_version_h)
  if (NOT ${ARGC} EQUAL 0)
    message(FATAL_ERROR "ament_cmake_gen_version_h does not accept parameters but ${ARGC} parameters provided")
  endif()

  # Find myself so I can use my own cmake instalation folder `ament_cmake_gen_version_h_DIR`
  # I can't rely on a pervious find_package call because it could be a direct call
  find_package(ament_cmake_gen_version_h REQUIRED)
  set(TEMPLATE_FILE "${ament_cmake_gen_version_h_DIR}/version.h.in")
  if(NOT EXISTS "${TEMPLATE_FILE}")
    message(FATAL_ERROR "Can't find ${TEMPLATE_FILE}. Reinstall ament_cmake_gen_version_h package.")
  endif()

  include_directories(${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_gen_version_h/include)
  set(TMP_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_gen_version_h/include/${PROJECT_NAME})

  set(VERSION_FILE_NAME ${TMP_INCLUDE_DIR}/version.h)
  set(NEED_TO_CREATE_VERSION_FILE TRUE)

  # retrieve version information from <package>.xml file
  # call ament_package_xml() if it has not been called before
  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
  set(VERSION_STR ${${PROJECT_NAME}_VERSION})

  # parse version information from the version string
  string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" "" dummy ${VERSION_STR})
  set(VERSION_MAJOR ${CMAKE_MATCH_1})
  set(VERSION_MINOR ${CMAKE_MATCH_2})
  set(VERSION_PATCH ${CMAKE_MATCH_3})

  # Check if the version file exist
  if(EXISTS "${VERSION_FILE_NAME}")
    # The file exists
    # Check if it contains the same version
    set(LINE_PATTERN "#define[ \t]+${PROJECT_NAME_UPPER}_VERSION_STR")
    file(STRINGS ${VERSION_FILE_NAME} VERSION_FILE_STRINGS REGEX ${LINE_PATTERN})
    set(VERSION_PATTERN "^#define[ \t]+${PROJECT_NAME_UPPER}_VERSION_STR[ \t]+\"([0-9]+\.[0-9]+\.[0-9]+)\"")
    string(REGEX MATCH ${VERSION_PATTERN} dummy ${VERSION_FILE_STRINGS})
    if("${CMAKE_MATCH_1}" STREQUAL "${VERSION_STR}")
      message(STATUS "File and project versions match: \"${CMAKE_MATCH_1}\" == \"${VERSION_STR}\"")
      set(NEED_TO_CREATE_VERSION_FILE FALSE)
    endif()
  endif()

  if(${NEED_TO_CREATE_VERSION_FILE})
    message(STATUS "Create new version file for version ${${PROJECT_NAME}_VERSION}")
    file(MAKE_DIRECTORY ${TMP_INCLUDE_DIR})
    # create the version.h file
    configure_file(${TEMPLATE_FILE} ${VERSION_FILE_NAME})
  else()
    message(STATUS "Skip version file creation")
  endif()

  install(
    DIRECTORY ${TMP_INCLUDE_DIR}
    DESTINATION include)
endfunction()
