# copied from ament_cmake_environment/ament_cmake_environment-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_GENERATION
  "Generate environment files in the CMAKE_INSTALL_PREFIX" ON)
option(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION
  "Generate marker file containing the parent prefix path" ON)

include("${ament_cmake_environment_DIR}/ament_generate_environment.cmake")

find_package(ament_cmake_core REQUIRED)
ament_register_extension("ament_package" "ament_cmake_environment"
  "ament_cmake_environment_package_hook.cmake")
