# generated from ament_cmake_export_definitions/ament_cmake_export_definitions-extras.cmake.in

# register ament_package() hook for definitions once
macro(_ament_cmake_export_definitions_register_package_hook)
  if(NOT DEFINED _AMENT_CMAKE_EXPORT_DEFINITIONS_PACKAGE_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_DEFINITIONS_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    ament_register_extension("ament_package" "ament_cmake_export_definitions" "ament_cmake_export_definitions_package_hook.cmake")
  endif()
endmacro()

include("${ament_cmake_export_definitions_DIR}/ament_export_definitions.cmake")
