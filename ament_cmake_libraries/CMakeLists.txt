cmake_minimum_required(VERSION 3.12)

project(ament_cmake_libraries NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_libraries-extras.cmake"
)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
