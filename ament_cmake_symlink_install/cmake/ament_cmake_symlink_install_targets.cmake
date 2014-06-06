#
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  if(NOT "${ARGV0}" STREQUAL "TARGETS")
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() first argument "
      "must be 'TARGETS', not '${ARGV0}'")
  endif()

  set(unsupported_keywords
    "EXPORT"
    "FRAMEWORK"
    "BUNDLE"
    "PRIVATE_HEADER"
    "PUBLIC_HEADER"
    "RESOURCE"
    "INCLUDES"
    "PERMISSIONS"
    "CONFIGURATIONS"
    "COMPONENT"
    "NAMELINK_ONLY"
    "NAMELINK_SKIP"
  )
  foreach(unsupported_keyword ${unsupported_keywords})
    list(FIND ARGN "${unsupported_keyword}" index)
    if(NOT index EQUAL -1)
      # fall back to CMake install() command
      # if the arguments can't be handled
      _install(${ARGN})
      break()
    endif()
  endforeach()

  if(index EQUAL -1)
    message("   - using symlinks")

    cmake_parse_arguments(ARG "ARCHIVE;LIBRARY;RUNTIME;OPTIONAL" "DESTINATION"
      "TARGETS" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
        "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    # convert target names into absolute files
    set(target_files "")
    foreach(target ${ARG_TARGETS})
      if(NOT TARGET ${target})
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() '${target}' is not a target")
      endif()
      get_target_property(is_imported "${target}" IMPORTED)
      if(is_imported)
        message(FATAL_ERROR "ament_cmake_symlink_install_targets() "
          "'${target}' is an imported target")
      endif()
      # TODO consider using a generator expression instead
      # $<TARGET_FILE:target>
      get_property(location TARGET ${target} PROPERTY LOCATION)
      list(APPEND target_files "${location}")
    endforeach()

    string(REPLACE ";" "\" \"" target_files_quoted
      "\"TARGET_FILES;${target_files}\"")
    string(REPLACE ";" "\" \"" argn_quoted "\"${ARGN}\"")
    ament_cmake_symlink_install_append_install_code(
      "ament_cmake_symlink_install_targets(${target_files_quoted};${argn_quoted})"
      COMMENTS "install(${argn_quoted})"
    )
  endif()
endfunction()
