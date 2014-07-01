#
# Overwrite CMake install command to use symlinks instead of copying resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(install signature)
  string(TOUPPER "${signature}" signature)
  message(" - install(${signature};${ARGN})")

  if("${signature}" STREQUAL "DIRECTORY")
    ament_cmake_symlink_install_directory(DIRECTORY ${ARGN})
    return()
  elseif("${signature}" STREQUAL "FILES")
    ament_cmake_symlink_install_files(FILES ${ARGN})
    return()
  elseif("${signature}" STREQUAL "PROGRAMS")
    ament_cmake_symlink_install_programs(PROGRAMS ${ARGN})
    return()
  elseif("${signature}" STREQUAL "TARGETS")
    ament_cmake_symlink_install_targets(TARGETS ${ARGN})
    return()
  endif()

  # fall back to CMake install() command
  # if the arguments haven't been handled before
  _install(${signature} ${ARGN})
endfunction()
