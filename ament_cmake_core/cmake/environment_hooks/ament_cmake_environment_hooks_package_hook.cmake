if(WIN32)
    set(_ext ".bat")
else()
    set(_ext ".sh")
endif()
ament_environment_hooks(
  "${ament_cmake_core_DIR}/environment_hooks/environment/ament_prefix_path${_ext}"
  "${ament_cmake_core_DIR}/environment_hooks/environment/path${_ext}"
)

if(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION)
  ament_generate_package_environment()
endif()
