#
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_programs programs_keyword)
  if(NOT "${programs_keyword} " STREQUAL "PROGRAMS ")
    message(FATAL_ERROR "ament_cmake_symlink_install_programs() first argument "
      "must be 'PROGRAMS', not '${programs_keyword}'")
  endif()

  set(unsupported_keywords
    "PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "RENAME"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(PROGRAMS ${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    message("   - using symlinks")
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")
    ament_cmake_symlink_install_append_install_code(
      "ament_cmake_symlink_install_programs(PROGRAMS ${argn_quoted})"
      COMMENTS "install(PROGRAMS ${argn_quoted})"
    )
  endif()
endfunction()
