# include CMake functions
include(CMakeParseArguments)

#
# Register a package resource of a specific type with the index.
#
# :param resource_type: the type of the resource
# :type resource_type: string
# :param CONTENT: the content of the marker file being installed
#   as a result of the registration (default: empty string)
# :type CONTENT: string
# :param CONTENT_FILE: the path to a file which will be used to fill
#   the marker file being installed as a result of the registration.
#   The file can either be a plain file or a template (ending with
#   '.in') which is expanded using configure_file() with @ONLY.
#   (optional, conflicts with CONTENT)
# :type CONTENT_FILE: string
# :param PACKAGE_NAME: the package name (default: ${PROJECT_NAME})
# :type PACKAGE_NAME: string
#
# @public
#
function(ament_index_register_resource resource_type)
  if("${resource_type} " STREQUAL " ")
    message(FATAL_ERROR
      "ament_index_register_resource() called without a 'resource_type'")
  endif()

  cmake_parse_arguments(ARG "" "PACKAGE_NAME;CONTENT_FILE" "CONTENT" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_index_register_resource() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(ARG_CONTENT AND ARG_CONTENT_FILE)
    message(FATAL_ERROR "ament_index_register_resource() called with both "
      "'CONTENT' and 'CONTENT_FILE', only one is allowed")
  endif()

  if("${ARG_PACKAGE_NAME} " STREQUAL " ")
    set(ARG_PACKAGE_NAME "${PROJECT_NAME}")
  endif()

  if(ARG_CONTENT_FILE)
    if(NOT IS_ABSOLUTE "${ARG_CONTENT_FILE}")
      set(ARG_CONTENT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_CONTENT_FILE}")
    endif()
    if(NOT EXISTS "${ARG_CONTENT_FILE}")
      message(FATAL_ERROR "ament_index_register_resource() the content file "
        "'${ARG_CONTENT_FILE}' does not exist")
    endif()

    string_ends_with("${ARG_CONTENT_FILE}" ".in" is_template)
    if(NOT is_template)
      # read non-template file content
      file(READ "${ARG_CONTENT_FILE}" ARG_CONTENT)
    endif()
  endif()

  set(destination "share/ament_index/resource_index/${resource_type}")
  set(marker_file
    "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_index/${resource_type}/${ARG_PACKAGE_NAME}")

  if(ARG_CONTENT OR NOT ARG_CONTENT_FILE)
    # use the CONTENT argument to create the marker file
    file(WRITE "${marker_file}" "${ARG_CONTENT}")
  else()
    # expand template
    configure_file(
      "${ARG_CONTENT_FILE}"
      "${marker_file}"
      @ONLY
    )
  endif()

  install(
    FILES "${marker_file}"
    DESTINATION "${destination}"
  )
endfunction()
