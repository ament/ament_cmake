# copied from ament_index/ament_index-extras.cmake

include("${ament_cmake_index_DIR}/ament_index_register_package.cmake")
include("${ament_cmake_index_DIR}/ament_index_register_resource.cmake")

find_package(ament_cmake_core REQUIRED)
ament_register_extension("ament_package" "ament_cmake_index"
  "ament_cmake_index_package_hook.cmake")
