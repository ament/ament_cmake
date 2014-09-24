# copied from ament_core/ament_index-extras.cmake

include("${ament_cmake_core_DIR}/index/ament_index_get_resources.cmake")
include("${ament_cmake_core_DIR}/index/ament_index_register_package.cmake")
include("${ament_cmake_core_DIR}/index/ament_index_register_resource.cmake")

ament_register_extension("ament_package" "ament_cmake_core"
  "index/ament_cmake_index_package_hook.cmake")
