cmake_minimum_required(VERSION 3.12)
project(ament_cmake_google_benchmark NONE)

# find dependencies
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(ament_cmake_test)

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)

ament_package_config_install_dir(_config_install_dir)
install(
  DIRECTORY cmake/
  DESTINATION ${_config_install_dir}
)
