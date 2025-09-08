cmake_minimum_required(VERSION 3.12)

project(ament_cmake_python_test)

find_package(ament_cmake_core REQUIRED)

# Uncomment to debug CMake variables
#
#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)

  set(_pytest_tests
    test/build_with_colcon.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      ENV SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR} PYEGG_VERSION=py${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}
      TIMEOUT 360
      WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    )
  endforeach()
endif()
ament_package()
