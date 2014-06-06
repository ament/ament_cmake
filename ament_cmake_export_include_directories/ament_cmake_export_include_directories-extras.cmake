# copied from ament_cmake_export_include_directories
# /ament_cmake_export_include_directories-extras.cmake

# register ament_package() hook for include directories once
macro(_ament_cmake_export_include_directories_register_package_hook)
  if(NOT DEFINED
      _AMENT_CMAKE_EXPORT_INCLUDE_DIRECTORIES_PACKAGE_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_INCLUDE_DIRECTORIES_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    ament_register_extension("ament_package"
      "ament_cmake_export_include_directories"
      "ament_cmake_export_include_directories_package_hook.cmake")
  endif()
endmacro()

include(
  "${ament_cmake_export_include_directories_DIR}/ament_export_include_directories.cmake")
