#
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of copying resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_directory directory_keyword)
  if(NOT "${directory_keyword}" STREQUAL "DIRECTORY")
    message(FATAL_ERROR "ament_cmake_symlink_install_directory() first argument must be 'DIRECTORY', not '${directory_keyword}'")
  endif()

  set(unsupported_keywords
    "FILE_PERMISSIONS"
    "DIRECTORY_PERMISSIONS"
    "USE_SOURCE_PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "FILES_MATCHING"
    "PATTERN"
    "REGEX"
    "EXCLUDE"
    "PERMISSIONS"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(DIRECTORY ${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    message("   - using symlinks")
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")
    ament_cmake_symlink_install_append_install_code(
      "ament_cmake_symlink_install_directory(DIRECTORY ${argn_quoted})"
      COMMENTS "install(DIRECTORY ${argn_quoted})"
    )
  endif()
endfunction()
