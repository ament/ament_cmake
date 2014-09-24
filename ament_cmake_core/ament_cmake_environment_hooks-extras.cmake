# copied from ament_cmake_core/ament_cmake_environment_hooks-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION
  "Generate environment files in the package share folder" ON)

include(
  "${ament_cmake_core_DIR}/environment_hooks/ament_environment_hooks.cmake")
include(
  "${ament_cmake_core_DIR}/environment_hooks/ament_generate_package_environment.cmake")

ament_register_extension("ament_package" "ament_cmake_core"
  "environment_hooks/ament_cmake_environment_hooks_package_hook.cmake")
