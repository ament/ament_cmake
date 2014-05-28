#
# Find gmock.
#
# Set the variables ``GMOCK_FOUND``, ``GMOCK_INCLUDE_DIRS``,
# ``GMOCK_LIBRARIES``, ``GMOCK_MAIN_LIBRARIES`` and
# ``GMOCK_BOTH_LIBRARIES``.
#
# @public
#
macro(ament_find_gmock)
  if(ARGN)
    message(FATAL_ERROR "ament_find_gmock() called with unused arguments: ${ARGN}")
  endif()
  _ament_cmake_gmock_find_gmock()
endmacro()
