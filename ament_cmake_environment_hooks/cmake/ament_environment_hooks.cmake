#
# Register environment hooks.
#
# Each file can either be a plain file (ending with a supported extensions)
# or a template which is expanded using configure_file() (ending in
# '.<ext>.in') with @ONLY.
#
# :param ARGN: a list of environment hook files
# :type ARGN: list of strings
#
# @public
#
function(ament_environment_hooks)
  if(_${PROJECT_NAME}_AMENT_GENERATE_PACKAGE_ENVIRONMENT)
    message(FATAL_ERROR "ament_environment_hooks() must be called before ament_generate_package_environment() (which is invoked by ament_package())")
  endif()

  foreach(hook ${ARGN})
    message(" - ament_environment_hooks() ${hook}")
    assert_file_exists("${hook}" "ament_environment_hooks() the passed hook file '${hook}' does not exist")
    stamp("${hook}")

    get_filename_component(hook_filename "${hook}" NAME)

    # check if the file is a template
    string_ends_with("${hook_filename}" ".in" is_template)
    if(is_template)
      # cut of .in extension
      string(LENGTH "${hook_filename}" length)
      math(EXPR offset "${length} - 3")
      string(SUBSTRING "${hook_filename}" 0 ${offset} hook_filename)
    endif()

    # ensure that the extension is supported
    string(FIND "${hook_filename}" "." index REVERSE)
    if(index EQUAL -1)
      message(FATAL_ERROR "ament_environment_hooks() called with the hook '${hook}' which doesn't have a file extension")
    endif()
    math(EXPR index "${index} + 1")
    string(SUBSTRING "${hook_filename}" ${index} -1 hook_extension)
    list(FIND AMENT_CMAKE_ENVIRONMENT_SUPPORTED_EXTENSIONS "${hook_extension}" supported)
    list(FIND AMENT_CMAKE_ENVIRONMENT_IGNORED_EXTENSIONS "${hook_extension}" ignored)
    if(supported EQUAL -1 AND ignored EQUAL -1)
      message(FATAL_ERROR "ament_environment_hooks() called with the hook '${hook}' with the unsupported file extension '${hook_extension}'")
    endif()

    if(NOT supported EQUAL -1)
      if(is_template)
        # expand template
        configure_file(
          "${hook}"
          "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment_hooks/${hook_filename}"
          @ONLY
        )
        set(hook "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment_hooks/${hook_filename}")
      endif()

      # install hook file
      install(
        FILES "${hook}"
        DESTINATION "share/${PROJECT_NAME}/environment"
      )

      # remember all environment hooks for generating the package environment files
      list(APPEND _AMENT_CMAKE_ENVIRONMENT_HOOKS_${hook_extension} "share/${PROJECT_NAME}/environment/${hook_filename}")
      set(_AMENT_CMAKE_ENVIRONMENT_HOOKS_${hook_extension} "${_AMENT_CMAKE_ENVIRONMENT_HOOKS_${hook_extension}}" PARENT_SCOPE)
    endif()
  endforeach()
endfunction()
