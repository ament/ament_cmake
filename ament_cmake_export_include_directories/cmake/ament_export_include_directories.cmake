#
# Export include directories to downstream packages.
#
# Relative paths will be exported before absolute paths.
# Non existing absolute paths will result in warning.
#
# :param ARGN: a list of include directories where each value might
#   be either an absolute path or path relative to the
#   CMAKE_INSTALL_PREFIX.
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_include_directories)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "ament_export_include_directories() must be called "
      "before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_include_directories_register_package_hook()
    message(" - ament_export_include_directories(${ARGN})")

    foreach(_arg ${ARGN})
      if(NOT IS_ABSOLUTE "${_arg}")
        # prefix relative paths with CMAKE_INSTALL_PREFIX
        # while avoiding to embed any absolute path
        set(_arg "\${${PROJECT_NAME}_DIR}/../../../${_arg}")
        list(APPEND _AMENT_EXPORT_RELATIVE_INCLUDE_DIRECTORIES "${_arg}")
      else()
        if(NOT IS_DIRECTORY "${_arg}")
          message(WARNING
            "ament_export_include_directories() package '${PROJECT_NAME}' "
            "exports the include directory '${_arg}' which doesn't exist")
        endif()
        list(APPEND _AMENT_EXPORT_ABSOLUTE_INCLUDE_DIRECTORIES "${_arg}")
      endif()
    endforeach()
  endif()
endmacro()
