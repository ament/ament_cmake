# include CMake functions
include(CMakeParseArguments)

#
# Find nosetests.
#
# Set the variable ``NOSETESTS`` to the absolute path of the
# executable if found.
#
# @public
#
macro(ament_find_nose)
  if(ARGN)
    message(FATAL_ERROR "ament_find_nose() called with unused arguments: ${ARGN}")
  endif()
  _ament_cmake_nose_find_nosetests()
endmacro()
