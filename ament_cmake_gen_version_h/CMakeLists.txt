cmake_minimum_required(VERSION 3.20)
project(ament_cmake_gen_version_h)
find_package(ament_cmake_core REQUIRED)

ament_set_default_language_standards()

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
