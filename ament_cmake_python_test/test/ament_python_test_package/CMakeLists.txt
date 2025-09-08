cmake_minimum_required(VERSION 3.12)

project(ament_python_test_package)

find_package(ament_cmake_core REQUIRED)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
