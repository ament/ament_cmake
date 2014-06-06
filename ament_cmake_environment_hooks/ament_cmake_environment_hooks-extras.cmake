# copied from
# ament_cmake_environment_hooks/ament_cmake_environment_hooks-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION
  "Generate environment files in the package share folder" ON)

include("${ament_cmake_environment_hooks_DIR}/ament_environment_hooks.cmake")
include("${ament_cmake_environment_hooks_DIR}/ament_generate_package_environment.cmake")

find_package(ament_cmake_core REQUIRED)
ament_register_extension("ament_package" "ament_cmake_environment_hooks"
  "ament_cmake_environment_hooks_package_hook.cmake")

find_package(ament_cmake_environment REQUIRED)
find_package(ament_cmake_index REQUIRED)
