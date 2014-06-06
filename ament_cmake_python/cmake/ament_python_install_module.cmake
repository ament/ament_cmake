#
# Install a Python module.
#
# :param module_file: the Python module file
# :type MODULE_FILE: string
#
macro(ament_python_install_module)
  _ament_cmake_python_register_environment_hook()
  _ament_cmake_python_install_module(${ARGN})
endmacro()

function(_ament_cmake_python_install_module module_file)
  if(ARGN)
    message(FATAL_ERROR
      "ament_python_install_module() called with unused arguments: ${ARGN}")
  endif()

  if(NOT IS_ABSOLUTE "${module_file}")
    set(module_file "${CMAKE_CURRENT_LIST_DIR}/${module_file}")
  endif()
  if(NOT EXISTS "${module_file}")
    message(FATAL_ERROR "ament_python_install_module() the Python module file "
      "'${module_file}' doesn't exist")
  endif()

  if(NOT PYTHON_INSTALL_DIR)
    message(FATAL_ERROR "ament_python_install_module() variable "
      "'PYTHON_INSTALL_DIR' must not be empty")
  endif()
  install(
    FILES "${module_file}"
    DESTINATION "${PYTHON_INSTALL_DIR}"
  )
  # TODO optionally compile Python file

  get_filename_component(name "${module_file}" NAME_WE)
  list(FIND AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES "${name}" index)
  if(NOT index EQUAL -1)
    message(FATAL_ERROR "ament_python_install_module() a Python module file "
      "or package with the same name '${name}' has been installed before")
  endif()
  list(APPEND AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES "${name}")
  set(AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES
    "${AMENT_CMAKE_PYTHON_INSTALL_INSTALLED_NAMES}" PARENT_SCOPE)
endfunction()
