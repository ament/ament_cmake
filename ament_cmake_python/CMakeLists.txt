cmake_minimum_required(VERSION 3.12)

project(ament_cmake_python NONE)

find_package(ament_cmake_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_python_DIR "cmake")
  include("ament_cmake_python-extras.cmake")

  ament_python_install_package(
      ament_python_test_package
      PACKAGE_DIR test/ament_python_test_package
  )
  ament_python_install_package(
      ament_python_test_package_overlay
      PACKAGE_DIR test/ament_python_test_package
  )
  ament_python_install_package(
      ament_python_test_package_overlay
      PACKAGE_DIR test/ament_python_test_package_overlay
  )

  ament_add_pytest_test(
    test_ament_python_install_package
    test/test_ament_python_install_package.py
    ENV TEST_PACKAGE_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
  )
endif()

ament_package(
  CONFIG_EXTRAS "ament_cmake_python-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
