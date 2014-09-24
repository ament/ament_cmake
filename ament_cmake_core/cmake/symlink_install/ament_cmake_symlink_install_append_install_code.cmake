# include CMake functions
include(CMakeParseArguments)

#
# Register a CMake script for execution at install time.
# The script is populated with th
#
# :param ARGN: the list of CMake code lines
# :type ARGN: list of strings
# :param COMMENTS: an optional list of comments
# :type COMMENTS: list of string
#
function(ament_cmake_symlink_install_append_install_code)
  cmake_parse_arguments(ARG "" "" "COMMENTS" ${ARGN})

  # append code to install script
  if(ARG_COMMENTS)
    file(APPEND "${AMENT_CMAKE_SYMLINK_INSTALL_INSTALL_SCRIPT}"
      "\n# ${ARG_COMMENTS}\n")
  endif()
  foreach(code ${ARG_UNPARSED_ARGUMENTS})
    file(APPEND "${AMENT_CMAKE_SYMLINK_INSTALL_INSTALL_SCRIPT}" "${code}\n")
  endforeach()
endfunction()
