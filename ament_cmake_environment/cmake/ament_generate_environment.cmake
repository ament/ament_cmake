#
# Generate setup files in the root of the install prefix.
#
# @public
#
function(ament_generate_environment)
  if(ARGN)
    message(FATAL_ERROR
      "ament_generate_environment() called with unused arguments: ${ARGN}")
  endif()

  find_package(ament_cmake_package_templates REQUIRED)

  # configure and install setup files
  foreach(file ${ament_cmake_package_templates_PREFIX_LEVEL})
    # check if the file is a template
    string_ends_with("${file}" ".in" is_template)
    if(is_template)
      # cut of .in extension
      string(LENGTH "${file}" length)
      math(EXPR offset "${length} - 3")
      string(SUBSTRING "${file}" 0 ${offset} name)
      # expand template
      get_filename_component(name "${name}" NAME)
      configure_file(
        "${file}"
        "${CMAKE_BINARY_DIR}/ament_cmake_environment/${name}"
        @ONLY
      )
      set(file "${CMAKE_BINARY_DIR}/ament_cmake_environment/${name}")
    endif()

    install(
      FILES "${file}"
      DESTINATION "${CMAKE_INSTALL_PREFIX}"
    )
  endforeach()
endfunction()
