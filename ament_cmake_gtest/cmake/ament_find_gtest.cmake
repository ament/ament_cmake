#
# Find gtest.
#
# Set the variables ``GTEST_FOUND``, ``GTEST_INCLUDE_DIRS``,
# ``GTEST_LIBRARIES``, ``GTEST_MAIN_LIBRARIES`` and
# ``GTEST_BOTH_LIBRARIES``.
#
# @public
#
macro(ament_find_gtest)
  if(ARGN)
    message(FATAL_ERROR "ament_find_gtest() called with unused arguments: ${ARGN}")
  endif()
  _ament_cmake_gtest_find_gtest()
endmacro()
