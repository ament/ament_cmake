# copied from ament_cmake_auto/ament_cmake_auto-extras.cmake

find_package(ament_cmake REQUIRED)

include("${ament_cmake_auto_DIR}/ament_auto_add_executable.cmake")
include("${ament_cmake_auto_DIR}/ament_auto_add_library.cmake")
include("${ament_cmake_auto_DIR}/ament_auto_generate_code.cmake")
include("${ament_cmake_auto_DIR}/ament_auto_find_build_dependencies.cmake")
include("${ament_cmake_auto_DIR}/ament_auto_package.cmake")
