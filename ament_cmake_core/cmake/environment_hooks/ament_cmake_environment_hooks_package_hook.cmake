ament_environment_hooks(
  "${ament_cmake_core_DIR}/environment_hooks/environment/ament_prefix_path.sh"
  "${ament_cmake_core_DIR}/environment_hooks/environment/path.sh"
)

if(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION)
  ament_generate_package_environment()
endif()
