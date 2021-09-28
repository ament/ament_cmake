cmake_minimum_required(VERSION 3.5)
project(ament_cmake_gen_version_h)
find_package(ament_cmake_core REQUIRED)

# GTest needs it, Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# GTest needs it, Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include(CTest)
if(BUILD_TESTING)
  # Simulate pre-installed package
  set(ament_cmake_gen_version_h_DIR ${CMAKE_SOURCE_DIR}/cmake)
  include(cmake/ament_cmake_gen_version_h.cmake)
  find_package(ament_cmake_gtest REQUIRED)

  # Generate version heades using different scenarios
  ament_cmake_gen_version_h(NO_INSTALL)
  ament_cmake_gen_version_h(NO_INSTALL VERSION_FILE_NAME "version1.h")
  ament_cmake_gen_version_h(NO_INSTALL INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include VERSION_FILE_NAME "version2.h")
  ament_cmake_gen_version_h(
    NO_INSTALL TRUE
    INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include
    VERSION_FILE_NAME "version_custom.h"
    VERSION_MAJOR 1
    VERSION_MINOR 2
    VERSION_PATCH 3
  )

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_version_custom.cpp
    test/test_version_h.cpp
    test/test_version1_h.cpp
    test/test_version2_h.cpp
  )
endif()

ament_package(CONFIG_EXTRAS "ament_cmake_gen_version_h-extras.cmake")

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
