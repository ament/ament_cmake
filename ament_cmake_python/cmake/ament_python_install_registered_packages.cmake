
function(ament_cmake_python_install_registered_packages)
  get_property(_pkgs GLOBAL PROPERTY AMENT_CMAKE_PYTHON_PKGS)
  foreach(pkg IN LISTS _pkgs)
    _ament_cmake_python_install_package_impl(${pkg})
  endforeach()
endfunction()

function(_ament_cmake_python_install_package_impl package_name)
  foreach(_prop IN ITEMS SKIP_COMPILE VERSION SETUP_CFG DESTINATION SCRIPTS_DESTINATION PACKAGE_DIRS)
    get_property(_${_prop} GLOBAL PROPERTY AMENT_CMAKE_PYTHON_${package_name}_${_prop})
  endforeach()

  _ament_cmake_python_prepare_build(${package_name})
  _ament_cmake_python_copy_or_symlink(${package_name})

  # Technically, we should call find_package(Python3) first to ensure that Python3::Interpreter
  # is available.  But we skip this here because this macro requires ament_cmake, and ament_cmake
  # calls find_package(Python3) for us.
  get_executable_path(python_interpreter Python3::Interpreter BUILD)

  _ament_cmake_python_generate_egg(${package_name})

  if(_SCRIPTS_DESTINATION)
  _ament_cmake_python_install_scripts(${package_name})
  endif()

  _ament_cmake_python_install_sources(${package_name})

  if(NOT _SKIP_COMPILE)
    _ament_cmake_python_byte_compile(${package_name})
  endif()

endfunction()

macro(_ament_cmake_python_prepare_build package_name)
  set(_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${package_name}")

  string(CONFIGURE "\
from setuptools import find_packages
from setuptools import setup

setup(
  name='${package_name}',
  version='${_VERSION}',
  packages=find_packages(
      include=('${package_name}', '${package_name}.*')),
)
" setup_py_content)

  file(GENERATE
    OUTPUT "${_build_dir}/setup.py"
    CONTENT "${setup_py_content}"
  )

endmacro()

macro(_ament_cmake_python_copy_or_symlink package_name)
  set(_sync_target "ament_cmake_python_sync_${package_name}")

  set(_dsts  "")
  set(_srcs  "")
  foreach(_dir IN LISTS _PACKAGE_DIRS)
    file(GLOB_RECURSE _dir_files CONFIGURE_DEPENDS RELATIVE "${_dir}" "${_dir}/*")
    foreach(_rel IN LISTS _dir_files)
      set(_src "${_dir}/${_rel}")
      set(_dst "${_build_dir}/${package_name}/${_rel}")

      list(FIND _dsts "${_dst}" _idx)
      if(NOT _idx EQUAL -1)
        list(REMOVE_AT _dsts  ${_idx})
        list(REMOVE_AT _srcs  ${_idx})
      endif()
      list(APPEND _dsts "${_dst}")
      list(APPEND _srcs "${_src}")
    endforeach()
  endforeach()

  set(_sync_deps "")
  list(LENGTH _dsts _len)
  if(_len GREATER 0)
    math(EXPR _last "${_len} - 1")
    foreach(_file_idx RANGE 0 ${_last})
      list(GET _dsts ${_file_idx} _dst)
      list(GET _srcs ${_file_idx} _src)

      get_filename_component(_dst_parent "${_dst}" DIRECTORY)
      file(MAKE_DIRECTORY "${_dst_parent}")

      if(AMENT_CMAKE_SYMLINK_INSTALL)
        add_custom_command(
          OUTPUT  "${_dst}"
          COMMAND ${CMAKE_COMMAND} -E create_symlink "${_src}" "${_dst}"
          DEPENDS "${_src}"
          COMMENT "Symlinking ${_dst}"
          VERBATIM
        )
      else()
        add_custom_command(
          OUTPUT  "${_dst}"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_src}" "${_dst}"
          DEPENDS "${_src}"
          COMMENT "Copying    ${_dst}"
          VERBATIM
        )
      endif()
      list(APPEND _sync_deps "${_dst}")
    endforeach()
  endif()

  if(_SETUP_CFG)
    set(_cfg_dst "${_build_dir}/setup.cfg")
    if(AMENT_CMAKE_SYMLINK_INSTALL)
      set(_copy_cmd ${CMAKE_COMMAND} -E create_symlink "${_SETUP_CFG}" "${_cfg_dst}")
    else()
      set(_copy_cmd ${CMAKE_COMMAND} -E copy_if_different "${_SETUP_CFG}" "${_cfg_dst}")
    endif()

    add_custom_command(
      OUTPUT  "${_cfg_dst}"
      COMMAND ${_copy_cmd}
      DEPENDS "${_SETUP_CFG}"
      COMMENT "Synchronising setup.cfg"
      VERBATIM
    )
    list(APPEND _sync_deps "${_cfg_dst}")
  endif()

  add_custom_target(${_sync_target} DEPENDS ${_sync_deps})

endmacro()

macro(_ament_cmake_python_generate_egg package_name)
  add_custom_target(
    ament_cmake_python_build_${package_name}_egg ALL
    COMMAND ${python_interpreter} setup.py egg_info
    WORKING_DIRECTORY "${_build_dir}"
    DEPENDS ${_sync_target}
  )

  set(python_version "py${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

  set(egg_name "${package_name}")
  set(egg_install_name "${egg_name}-${_VERSION}")
  set(egg_install_name "${egg_install_name}-${python_version}")

  install(
    DIRECTORY "${_build_dir}/${egg_name}.egg-info/"
    DESTINATION "${_DESTINATION}/${egg_install_name}.egg-info"
  )
endmacro()

macro(_ament_cmake_python_install_scripts package_name)
  file(MAKE_DIRECTORY "${_build_dir}/scripts")  # setup.py may or may not create it

  add_custom_target(
    ament_cmake_python_build_${package_name}_scripts ALL
    COMMAND ${python_interpreter} setup.py install_scripts -d scripts
    WORKING_DIRECTORY "${_build_dir}"
    DEPENDS ${_sync_target}
  )

  if(NOT AMENT_CMAKE_SYMLINK_INSTALL)
    # Not needed for nor supported by symlink installs
    set(_extra_install_args USE_SOURCE_PERMISSIONS)
  endif()

  install(
    DIRECTORY "${_build_dir}/scripts/"
    DESTINATION "${_SCRIPTS_DESTINATION}/"
    ${_extra_install_args}
  )
endmacro()

macro(_ament_cmake_python_install_sources package_name)
  foreach(_dir IN LISTS _PACKAGE_DIRS)
    install(
      DIRECTORY "${_dir}/"
      DESTINATION "${_DESTINATION}/${package_name}"
      PATTERN "*.pyc"     EXCLUDE
      PATTERN "__pycache__" EXCLUDE
    )
  endforeach()
endmacro()

macro(_ament_cmake_python_byte_compile package_name)
  get_executable_path(python_interpreter_config Python3::Interpreter CONFIGURE)
  # compile Python files
  install(CODE
    "execute_process(
      COMMAND
      \"${python_interpreter_config}\" \"-m\" \"compileall\"
      \"${CMAKE_INSTALL_PREFIX}/${_DESTINATION}/${package_name}\"
    )"
  )
endmacro()

ament_cmake_python_install_registered_packages()