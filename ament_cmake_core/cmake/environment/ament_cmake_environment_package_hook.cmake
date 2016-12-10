if(AMENT_CMAKE_ENVIRONMENT_GENERATION)
  ament_generate_environment()
endif()

function(ament_cmake_environment_generate_package_run_dependencies_marker)
  # use all run dependencies from the package.xml
  set(run_depends "")
  list_append_unique(run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS}
    ${${PROJECT_NAME}_TEST_DEPENDS})

  # register direct run dependencies
  ament_index_register_resource("package_run_dependencies" CONTENT "${run_depends}")
endfunction()

function(ament_cmake_environment_generate_parent_prefix_path_marker)
  if(NOT "$ENV{AMENT_PREFIX_PATH} " STREQUAL " ")
    set(marker_file
      "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment/parent_prefix_path")
    file(WRITE "${marker_file}" "$ENV{AMENT_PREFIX_PATH}")
    file(MD5 "${marker_file}" md5)
    install(
      FILES "${marker_file}"
      DESTINATION "share/ament_parent_prefix_path"
      RENAME "${md5}"
    )
  endif()
endfunction()

if(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION)
  ament_cmake_environment_generate_package_run_dependencies_marker()
  ament_cmake_environment_generate_parent_prefix_path_marker()
endif()
