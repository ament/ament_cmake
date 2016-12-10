#
# Generate setup files in share folder of the current package.
#
# @public
#
function(ament_generate_package_environment)
  if(ARGN)
    message(FATAL_ERROR "ament_generate_package_environment() called with unused arguments: ${ARGN}")
  endif()

  message(" - ament_generate_package_environment()")

  # mark that ament_generate_package_environment() was called in order to detect wrong order of calling
  set(_${PROJECT_NAME}_AMENT_GENERATE_PACKAGE_ENVIRONMENT TRUE PARENT_SCOPE)

  # generate setup files for the package
  foreach(extension ${AMENT_CMAKE_ENVIRONMENT_SUPPORTED_EXTENSIONS})
    set(SOURCE_HOOKS "")
    foreach(hook ${_AMENT_CMAKE_ENVIRONMENT_PACKAGE_HOOKS_${extension}})
      set(SOURCE_HOOKS "${SOURCE_HOOKS}${AMENT_CMAKE_ENVIRONMENT_SOURCE_COMMAND_${extension}} \"$AMENT_CURRENT_PREFIX/${hook}\"\n")
    endforeach()

    configure_file(
      "${ament_cmake_environment_DIR}/environment/package/local_setup.${extension}.in"
      "${CMAKE_BINARY_DIR}/ament_cmake_environment/package/local_setup.${extension}"
      @ONLY
    )

    install(
      FILES "${CMAKE_BINARY_DIR}/ament_cmake_environment/package/local_setup.${extension}"
      DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    )
  endforeach()
endfunction()
