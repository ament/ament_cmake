if(AMENT_CMAKE_ENVIRONMENT_GENERATION)
  ament_generate_environment()
endif()

function(ament_cmake_environment_generate_parent_prefix_path_marker)
  if(NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    set(marker_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment/parent_prefix_path")
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
  ament_cmake_environment_generate_parent_prefix_path_marker()
endif()
