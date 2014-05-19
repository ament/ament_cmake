#
# Generate setup files in the root of the install prefix.
#
# @public
#
function(ament_generate_environment)
  if(ARGN)
    message(FATAL_ERROR "ament_generate_environment() called with unused arguments: ${ARGN}")
  endif()

  # configure and install setup files
  foreach(extension ${AMENT_CMAKE_ENVIRONMENT_SUPPORTED_EXTENSIONS})
    _ament_generate_environment("setup" "${extension}")
    _ament_generate_environment("local_setup" "${extension}")
  endforeach()
endfunction()

function(_ament_generate_environment name extension)
  set(file "${ament_cmake_environment_DIR}/environment/${name}.${extension}")
  if(EXISTS "${file}.in")
    configure_file(
      "${file}.in"
      "${CMAKE_BINARY_DIR}/ament_cmake_environment/${name}.${extension}"
      @ONLY
    )
    set(file "${CMAKE_BINARY_DIR}/ament_cmake_environment/${name}.${extension}")
  elseif(NOT EXISTS "${file}")
    message(FATAL_ERROR "ament_generate_environment() could find neither find '${file}.in' nor '${file}'")
  endif()

  install(
    FILES "${file}"
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
  )
endfunction()
