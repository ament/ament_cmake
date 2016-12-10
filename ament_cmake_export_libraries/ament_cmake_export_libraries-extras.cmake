# copied from
# ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake

# register environment hook for libraries once
macro(_ament_cmake_export_libraries_register_environment_hook)
  if(NOT DEFINED _AMENT_CMAKE_EXPORT_LIBRARIES_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_LIBRARIES_ENVIRONMENT_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    if(WIN32)
      set(_ext .bat)
    else()
      set(_ext .sh)
    endif()
    ament_environment_hooks(
      "${ament_cmake_export_libraries_DIR}/environment/library_path.bat${_ext}")
    unset(_ext)
  endif()
endmacro()

# register ament_package() hook for libraries once
macro(_ament_cmake_export_libraries_register_package_hook)
  if(NOT DEFINED _AMENT_CMAKE_EXPORT_LIBRARIES_PACKAGE_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_LIBRARIES_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    ament_register_extension("ament_package" "ament_cmake_export_libraries"
      "ament_cmake_export_libraries_package_hook.cmake")
  endif()
endmacro()

include("${ament_cmake_export_libraries_DIR}/ament_export_libraries.cmake")
include("${ament_cmake_export_libraries_DIR}/ament_export_library_names.cmake")
