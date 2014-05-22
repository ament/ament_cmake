# copied from ament_cmake_nose/ament_cmake_nose-extras.cmake

# find nosetests once
macro(_ament_cmake_nose_find_nosetests)
  if(NOT DEFINED _AMENT_CMAKE_NOSE_FIND_NOSETESTS)
    set(_AMENT_CMAKE_NOSE_FIND_NOSETESTS TRUE)

    find_package(ament_cmake_core REQUIRED)
    find_package(ament_cmake_test REQUIRED)

    find_program(NOSETESTS NAMES
      "nosetests${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
      "nosetests-${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
      "nosetests${PYTHON_VERSION_MAJOR}"
      "nosetests-${PYTHON_VERSION_MAJOR}"
      "nosetests")
    if(NOSETESTS)
      message(STATUS "Using Python nosetests: ${NOSETESTS}")
    else()
      if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
        set(_python_nosetests_package "python3-nose")
      else()
        set(_python_nosetests_package "python-nose")
      endif()
      message(WARNING "'nosetests' not found, Python nose tests can not be run (e.g. on Ubuntu/Debian install the package '${_python_nosetests_package}')")
    endif()
  endif()
endmacro()

include("${ament_cmake_nose_DIR}/ament_add_nose_test.cmake")
include("${ament_cmake_nose_DIR}/ament_find_nosetests.cmake")
