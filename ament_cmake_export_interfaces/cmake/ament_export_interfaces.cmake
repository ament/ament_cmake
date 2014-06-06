#
# Export interfaces to downstream packages.
#
# Each interface name must have been used to install targets using
# ``install(TARGETS ... EXPORT name ...)``.
# The ``install(EXPORT ...)`` invocation is handled by this macros.
#
# :param ARGN: a list of export names
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_interfaces)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_interfaces() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_interfaces_register_package_hook()
    message(" - ament_export_interfaces(${ARGN})")
    foreach(_arg ${ARGN})
      list(APPEND _AMENT_CMAKE_EXPORT_INTERFACES "${_arg}")
    endforeach()
  endif()
endmacro()
