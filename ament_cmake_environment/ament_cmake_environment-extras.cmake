# copied from ament_cmake_environment/ament_cmake_environment-extras.cmake

option(AMENT_CMAKE_ENVIRONMENT_GENERATION "Generate environment files in the CMAKE_INSTALL_PREFIX" OFF)
option(AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION "Generate environment files in the package share folder" ON)
option(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION "Generate marker file containing the parent prefix path" ON)

# set supported extensions based on platform
set(_non_windows_extensions "bash" "sh")
set(_windows_extensions "bat")
if(NOT WIN32)
  # non-windows
  set(_supported_extensions "${_non_windows_extensions}")
  set(_ignored_extensions "${_windows_extensions}")
else()
  # windows
  set(_supported_extensions "${_windows_extensions}")
  set(_ignored_extensions "${_non_windows_extensions}")
endif()
# option()
set(
  AMENT_CMAKE_ENVIRONMENT_SUPPORTED_EXTENSIONS "${_supported_extensions}"
  CACHE STRING "Generate environment files for these extensions"
)
# option()
set(
  AMENT_CMAKE_ENVIRONMENT_IGNORED_EXTENSIONS "${_ignored_extensions}"
  CACHE STRING "Ignore environment files for these extensions"
)

# set commands to source files with a specific extension
set(AMENT_CMAKE_ENVIRONMENT_SOURCE_COMMAND_bash ".")
set(AMENT_CMAKE_ENVIRONMENT_SOURCE_COMMAND_sh ".")
set(AMENT_CMAKE_ENVIRONMENT_SOURCE_COMMAND_bat "call")

include("${ament_cmake_environment_DIR}/ament_environment_hooks.cmake")
include("${ament_cmake_environment_DIR}/ament_generate_environment.cmake")
include("${ament_cmake_environment_DIR}/ament_generate_package_environment.cmake")

if(AMENT_CMAKE_ENVIRONMENT_GENERATION)
  ament_generate_environment()
endif()

function(ament_cmake_environment_generate_parent_prefix_path_marker)
  if(NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    set(marker_file "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment/parent_prefix_path")
    file(WRITE "${marker_file}" "$ENV{AMENT_PREFIX_PATH}")
    file(MD5 "${marker_file}" md5)
    install(
      FILES "${marker_file}"
      DESTINATION "share/ament_parent_prefix_path"
      RENAME "${md5}"
    )
  endif()
endfunction()

if(AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION)
  ament_cmake_environment_generate_parent_prefix_path_marker()
endif()

find_package(ament_cmake_core REQUIRED)
ament_register_extension("ament_package" "ament_cmake_environment" "ament_cmake_environment_package_hook.cmake")

find_package(ament_cmake_index REQUIRED)
