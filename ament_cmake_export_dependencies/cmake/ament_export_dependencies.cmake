#
# Export dependencies to downstream packages.
#
# Each package name must be find_package()-able with the exact same case.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be INCLUDE_DIRS and LIBRARIES.
#
# :param ARGN: a list of package names
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_dependencies)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_dependencies() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_dependencies_register_package_hook()
    message(" - ament_export_dependencies(${ARGN})")
    foreach(_arg ${ARGN})
      # only pass package name
      # will be resolved by downstream packages
      # must be find_package()-able
      # and provide _INCLUDE_DIRS and _LIBRARIES
      list(APPEND _AMENT_CMAKE_EXPORT_DEPENDENCIES "${_arg}")
    endforeach()
  endif()
endmacro()
