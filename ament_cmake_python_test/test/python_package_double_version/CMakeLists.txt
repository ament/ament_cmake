cmake_minimum_required(VERSION 3.20)
project(python_package_double_version)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} VERSION 1.2.34)
ament_python_install_package(some_other_name PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}" VERSION 5.6.78)

ament_package()
