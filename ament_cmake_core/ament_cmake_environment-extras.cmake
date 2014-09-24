# copied from ament_cmake_core/ament_cmake_environment-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_GENERATION
  "Generate environment files in the CMAKE_INSTALL_PREFIX" ON)
option(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION
  "Generate marker file containing the parent prefix path" ON)

include("${ament_cmake_core_DIR}/environment/ament_generate_environment.cmake")

ament_register_extension("ament_package" "ament_cmake_core"
  "environment/ament_cmake_environment_package_hook.cmake")
