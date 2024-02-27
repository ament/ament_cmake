set(ament_cmake_libraries_DIR "${CMAKE_CURRENT_LIST_DIR}/../cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ament_cmake_libraries-extras.cmake")

macro(assert_equal EXPECTED ACTUAL)
  if(NOT "${EXPECTED}" STREQUAL "${ACTUAL}")
    message(SEND_ERROR "Assert failed: Expected '${EXPECTED}', got '${ACTUAL}'")
  endif()
endmacro()
