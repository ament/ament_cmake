#
# Export definitions to downstream packages.
#
# Each package name must be find_package()-able with the exact same case.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be INCLUDE_DIRS and LIBRARIES.
#
# :param ARGN: a list of definitions
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_definitions)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "ament_export_definitions() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_definitions_register_package_hook()
    message(" - ament_export_definitions(${ARGN})")
    foreach(_arg ${ARGN})
      list(APPEND _AMENT_CMAKE_EXPORT_DEFINITIONS "${_arg}")
    endforeach()
  endif()
endmacro()
