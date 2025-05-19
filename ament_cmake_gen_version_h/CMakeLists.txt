cmake_minimum_required(VERSION 3.5)
project(ament_cmake_gen_version_h)
find_package(ament_cmake_core REQUIRED)

# GTest needs it, Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# GTest needs it, Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

include(CTest)
if(BUILD_TESTING)
  # Simulate pre-installed package
  set(ament_cmake_gen_version_h_DIR ${CMAKE_SOURCE_DIR}/cmake)
  include(cmake/ament_generate_version_header.cmake)
  find_package(ament_cmake_gtest REQUIRED)

  # Generate version headers that don't conflict with existing tests
  add_library(some_lib INTERFACE)
  ament_generate_version_header(some_lib SKIP_INSTALL
    HEADER_PATH "ament_generate_version_header/version.hpp")
  ament_add_gtest(test_ament_generate_version_header
    test/test_version_hpp.cpp
  )
  target_link_libraries(test_ament_generate_version_header some_lib)
endif()

ament_package(CONFIG_EXTRAS "ament_cmake_gen_version_h-extras.cmake")

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
