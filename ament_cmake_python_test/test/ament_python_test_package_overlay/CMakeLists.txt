cmake_minimum_required(VERSION 3.12)

project(ament_python_test_package_overlay)

find_package(ament_cmake_core REQUIRED)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
    ament_python_test_package_overlay
    PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/ament_python_test_package"
)
ament_python_install_package(
    ament_python_test_package_overlay
    PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/ament_python_test_package_overlay"
)

ament_package()
