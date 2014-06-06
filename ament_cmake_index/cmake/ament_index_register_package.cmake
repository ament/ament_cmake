# include CMake functions
include(CMakeParseArguments)

#
# Register a package name with the resource index.
#
# :param PACKAGE_NAME: the package name (default: ${PROJECT_NAME})
# :type PACKAGE_NAME: string
#
function(ament_index_register_package)
  cmake_parse_arguments(ARG "" "PACKAGE_NAME" "" ${ARGN})
  if(ARGN)
    message(FATAL_ERROR
      "ament_index_register_package() called with unused arguments: ${ARGN}")
  endif()

  # register package name with the resource type 'packages'
  ament_index_register_resource("packages" ${ARGN})
endfunction()
