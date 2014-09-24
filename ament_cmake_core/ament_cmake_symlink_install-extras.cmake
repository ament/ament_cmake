# copied from ament_cmake_core/ament_cmake_symlink_install-extras.cmake

option(AMENT_CMAKE_SYMLINK_INSTALL
  "Replace the CMake install command with a custom implementation using symlinks instead of copying resources"
  OFF)

if(AMENT_CMAKE_SYMLINK_INSTALL)
  message(STATUS "Override CMake install command with custom implementation "
    "using symlinks instead of copying resources")

  include(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install_append_install_code.cmake")
  include(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install_directory.cmake")
  include(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install_files.cmake")
  include(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install_programs.cmake")
  include(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install_targets.cmake")
  include("${ament_cmake_core_DIR}/symlink_install/install.cmake")

  # create the install script from the template
  # ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install-extras.cmake.in
  set(AMENT_CMAKE_SYMLINK_INSTALL_INSTALL_SCRIPT
    "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
  configure_file(
    "${ament_cmake_core_DIR}/symlink_install/ament_cmake_symlink_install.cmake.in"
    "${AMENT_CMAKE_SYMLINK_INSTALL_INSTALL_SCRIPT}"
    @ONLY
  )
  # register script for being executed at install time
  install(SCRIPT "${AMENT_CMAKE_SYMLINK_INSTALL_INSTALL_SCRIPT}")
endif()
