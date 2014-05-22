# copied from ament_cmake_export_interfaces/ament_cmake_export_interfaces-extras.cmake

# register ament_package() hook for interfaces once
macro(_ament_cmake_export_interfaces_register_package_hook)
  if(NOT DEFINED _AMENT_CMAKE_EXPORT_INTERFACES_PACKAGE_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_INTERFACES_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    ament_register_extension("ament_package" "ament_cmake_export_interfaces" "ament_cmake_export_interfaces_package_hook.cmake")
  endif()
endmacro()

include("${ament_cmake_export_interfaces_DIR}/ament_export_interfaces.cmake")
