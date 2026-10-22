cmake_minimum_required(VERSION 3.12)

project(ament_cmake_export_dependencies NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_export_dependencies-extras.cmake"
)

ament_package_config_install_dir(_config_install_dir)
install(
  DIRECTORY cmake/
  DESTINATION ${_config_install_dir}
)
