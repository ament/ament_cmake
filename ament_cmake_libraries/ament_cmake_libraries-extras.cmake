# copied from ament_cmake_libraries/ament_cmake_libraries-extras.cmake

set(AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR ":")

include("${ament_cmake_libraries_DIR}/ament_libraries_deduplicate.cmake")
include("${ament_cmake_libraries_DIR}/ament_libraries_pack_build_configuration.cmake")
include("${ament_cmake_libraries_DIR}/ament_libraries_unpack_build_configuration.cmake")
