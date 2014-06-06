# copied from ament_cmake_python/ament_cmake_python-extras.cmake

# register environment hook for PYTHONPATH once
macro(_ament_cmake_python_register_environment_hook)
  if(NOT DEFINED _AMENT_CMAKE_PYTHON_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_PYTHON_ENVIRONMENT_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    find_package(ament_cmake_environment_hooks REQUIRED)
    ament_environment_hooks(
      "${ament_cmake_python_DIR}/environment/pythonpath.sh.in")
  endif()
endmacro()

include("${ament_cmake_python_DIR}/ament_python_install_module.cmake")
include("${ament_cmake_python_DIR}/ament_python_install_package.cmake")
