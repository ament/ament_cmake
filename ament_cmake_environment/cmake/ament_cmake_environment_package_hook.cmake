ament_environment_hooks(
  "${ament_cmake_environment_DIR}/environment/package/ament_prefix_path.sh.in"
  "${ament_cmake_environment_DIR}/environment/package/path.sh.in"
)

if(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION)
  ament_generate_package_environment()
endif()
