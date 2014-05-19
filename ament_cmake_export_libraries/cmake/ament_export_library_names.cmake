#
# Export library names to downstream packages.
# The libraries are either searched in the paths passed in as
# LIBRARY_DIRS or if none are specified in the default locations.
#
# :param ARGN: a list of library names.
# :type ARGN: list of strings
# :param LIBRARY_DIRS: an optional list of search paths.
# :type LIBRARY_DIRS: list of paths
#
# @public
#
macro(ament_export_library_names)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "ament_export_library_names() must be called before ament_package()")
  endif()

  cmake_parse_arguments(_ARG "" "" "LIBRARY_DIRS" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    _ament_cmake_export_libraries_register_environment_hook()
    _ament_cmake_export_libraries_register_package_hook()
    message(" - ament_export_library_names(${ARGN})")

    # library directories need to be added as a suffix to each library name
    set(_library_dirs "")
    if(_ARG_LIBRARY_DIRS)
      string(REPLACE ";" ":" _library_dirs "${_ARG_LIBRARY_DIRS}")
    endif()

    foreach(_library_name ${_ARG_UNPARSED_ARGUMENTS})
      if("${_library_name}" MATCHES "^debug|optimized|general$")
        # keep build configuration keyword as-is
        list(APPEND _AMENT_EXPORT_LIBRARY_NAMES "${_library_name}")
      else()
        if(_library_dirs_suffix)
          # append library directories to library name
          set(_library_name "${_library_name}:${_library_dirs}")
        endif()
        list(APPEND _AMENT_EXPORT_LIBRARY_NAMES "${_library_name}")
      endif()
    endforeach()
  endif()
endmacro()
