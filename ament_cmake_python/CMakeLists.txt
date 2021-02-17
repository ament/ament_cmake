cmake_minimum_required(VERSION 3.5)

project(ament_cmake_python NONE)

find_package(ament_cmake_core REQUIRED)

set(ament_cmake_python_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("ament_cmake_python-extras.cmake")
ament_python_install_package(${PROJECT_NAME} NO_DATA)

include(CTest)

if(BUILD_TESTING)
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -m py_compile
      foo/__init__.py foo/bar/__init__.py baz/__init__.py
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
  add_test(
    NAME test_find_packages_data
    COMMAND "${PYTHON_EXECUTABLE}" test_find_packages_data.py
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
  set_tests_properties(test_find_packages_data PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 30
  )
endif()

ament_package(
  CONFIG_EXTRAS "ament_cmake_python-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
