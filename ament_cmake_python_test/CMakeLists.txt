cmake_minimum_required(VERSION 3.12)

project(ament_cmake_python_test)

find_package(ament_cmake_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_python REQUIRED)

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

  ament_python_install_package(
      ament_python_test_package_no_compile
      PACKAGE_DIR test/ament_python_test_package
      SKIP_COMPILE
  )

  ament_python_install_package(
      ament_python_test_package_versioned
      PACKAGE_DIR test/ament_python_test_package
      VERSION 1.2.3
  )

  set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_pkg")
  file(WRITE "${_generated_dir}/__init__.py" "generated = True\n")
  add_custom_target(generate_test_files
      COMMAND ${CMAKE_COMMAND} -E touch "${_generated_dir}/_generated_marker.py"
  )
  ament_python_install_package(
      ament_python_test_package_generated
      PACKAGE_DIR "${_generated_dir}"
      DEPENDS generate_test_files
  )

  ament_add_pytest_test(
    test_ament_python_install_package
    test/test_ament_python_install_package.py
    ENV TEST_PACKAGE_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
  )
endif()

ament_package()
