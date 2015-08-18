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

# copied from ament_cmake_gmock/ament_cmake_gmock-extras.cmake

# find gmock and create library targets once
macro(_ament_cmake_gmock_find_gmock)
  if(NOT DEFINED _AMENT_CMAKE_GMOCK_FIND_GMOCK)
    set(_AMENT_CMAKE_GMOCK_FIND_GMOCK TRUE)

    find_package(ament_cmake_test QUIET REQUIRED)

    if(NOT GMOCK_FOUND)
      # find gmock include and source folders
      # build the list of locations to search in reverse order
      # the last result is to use the internal GMock fork from this package
      set(_prefix "${ament_cmake_gmock_DIR}/../../..")
      set(_search_path_include "${_prefix}/src/ament_cmake_gmock/googlemock-1.7.0/include/gmock")
      set(_search_path_src "${_prefix}/src/ament_cmake_gmock/googlemock-1.7.0/src")
      # also fall back to system installed path (i.e. on Ubuntu)
      list(INSERT _search_path_include 0 "/usr/include/gmock")
      list(INSERT _search_path_src 0 "/usr/src/gmock/src")
      # option() consider environment variable to find gmock
      if(NOT "$ENV{GMOCK_DIR} " STREQUAL " ")
        list(INSERT _search_path_include 0 "$ENV{GMOCK_DIR}/include/gmock")
        list(INSERT _search_path_src 0 "$ENV{GMOCK_DIR}/src")
      endif()
      find_file(_gmock_header_file "gmock.h"
        PATHS ${_search_path_include}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )
      find_file(_gmock_src_file
        NAMES
          "gmock.cc"
          "gmock-gtest-all.cc"  # Alternative when using "fused" sources.
        PATHS ${_search_path_src}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )

      if(_gmock_header_file AND _gmock_src_file)
        get_filename_component(_gmock_base_dir "${_gmock_src_file}" PATH)
        get_filename_component(_gmock_base_dir "${_gmock_base_dir}" PATH)
        # add CMakeLists.txt from gmock dir
        set(_gmock_binary_dir "${CMAKE_BINARY_DIR}/gmock")
        add_subdirectory("${_gmock_base_dir}" ${_gmock_binary_dir})
        # mark gmock targets with EXCLUDE_FROM_ALL to only build
        # when tests are built which depend on them
        set_target_properties(gmock gmock_main PROPERTIES EXCLUDE_FROM_ALL 1)
        get_filename_component(_gmock_include_dir "${_gmock_header_file}" PATH)
        get_filename_component(_gmock_include_dir ${_gmock_include_dir} PATH)
        # set from-source variables
        set(GMOCK_FROM_SOURCE_FOUND TRUE CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_INCLUDE_DIRS "${_gmock_include_dir}"
          CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_LIBRARY_DIRS "${_gmock_binary_dir}"
          CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_LIBRARIES "gmock" CACHE INTERNAL "")
        set(GMOCK_FROM_SOURCE_MAIN_LIBRARIES "gmock_main" CACHE INTERNAL "")
        message(STATUS "Found gmock sources under '${_gmock_base_dir}': "
          "C++ tests using 'Google Mock' will be built")

        set(_gtest_base_dir "${_gmock_base_dir}/gtest")
        # set from-source variables for embedded gtest
        set(GTEST_FROM_SOURCE_FOUND TRUE CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_INCLUDE_DIRS "${_gmock_base_dir}/gtest/include"
          CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARY_DIRS "${_gmock_binary_dir}/gtest"
          CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARIES "gtest" CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_MAIN_LIBRARIES "gtest_main" CACHE INTERNAL "")
        message(STATUS "Found gtest sources under '${_gtest_base_dir}': "
          "C++ tests using 'Google Test' will be built")
      endif()
      if(GMOCK_FROM_SOURCE_FOUND)
        # set the same variables as find_package() would set
        # but do NOT set GMOCK_FOUND in the cache when using gmock from source
        # since the subdirectory must always be added to add the gmock targets
        set(GMOCK_FOUND ${GMOCK_FROM_SOURCE_FOUND})
        set(GMOCK_INCLUDE_DIRS ${GMOCK_FROM_SOURCE_INCLUDE_DIRS})
        set(GMOCK_LIBRARY_DIRS ${GMOCK_FROM_SOURCE_LIBRARY_DIRS})
        set(GMOCK_LIBRARIES ${GMOCK_FROM_SOURCE_LIBRARIES})
        set(GMOCK_MAIN_LIBRARIES ${GMOCK_FROM_SOURCE_MAIN_LIBRARIES})
        set(GMOCK_BOTH_LIBRARIES ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
        # also set the gtest variables
        # but do NOT set GTEST_FOUND in the cache when using gtest from source
        # since the subdirectory must always be added to add the gmock targets
        set(GTEST_FOUND ${GTEST_FROM_SOURCE_FOUND})
        set(GTEST_INCLUDE_DIRS ${GTEST_FROM_SOURCE_INCLUDE_DIRS})
        set(GTEST_LIBRARY_DIRS ${GTEST_FROM_SOURCE_LIBRARY_DIRS})
        set(GTEST_LIBRARIES ${GTEST_FROM_SOURCE_LIBRARIES})
        set(GTEST_MAIN_LIBRARIES ${GTEST_FROM_SOURCE_MAIN_LIBRARIES})
        set(GTEST_BOTH_LIBRARIES ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
        # gmock adds its own include dir first and then its embedded gtest one
        # but when gtest is installed beside gmock it will overlay the gtest
        # headers embedded in gmock
        # therefore prepending the include dir of the embedded gtest version
        # to ensure it is being used
        include_directories(BEFORE "${GTEST_INCLUDE_DIRS}")
      endif()
    endif()
    if(NOT GMOCK_FOUND)
      message(WARNING
        "'gmock' not found, C++ tests using 'Google Mock' can not be built. "
        "Please install the 'Google Mock' headers globally in your system to "
        "enable these tests (e.g. on Ubuntu/Debian install the package "
          "'google-mock')")
    endif()

    # for Visual 2012 we need to increase the fixed variadic template size to
    # build gtest (https://code.google.com/p/googletest/source/detail?r=675)
    if(GMOCK_FOUND AND MSVC AND MSVC_VERSION EQUAL 1700)
      add_definitions(/D _VARIADIC_MAX=10)
    endif()
  endif()
endmacro()

include("${ament_cmake_gmock_DIR}/ament_add_gmock.cmake")
include("${ament_cmake_gmock_DIR}/ament_find_gmock.cmake")

find_package(ament_cmake_core QUIET REQUIRED)
ament_register_extension("ament_cmake_gtest_find_gtest" "ament_cmake_gmock"
  "ament_cmake_gmock_find_gtest_hook.cmake")
