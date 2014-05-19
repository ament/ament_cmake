# prevent multiple inclusion
if(DEFINED _AMENT_CMAKE_CORE_INCLUDED)
  message(FATAL_ERROR "ament_cmake_core/cmake/CMakeLists.cmake included multiple times")
endif()
set(_AMENT_CMAKE_CORE_INCLUDED TRUE)

if(NOT DEFINED ament_cmake_core_DIR)
  message(FATAL_ERROR "ament_cmake_core_DIR is not set")
endif()

# enable all new policies (if available)
macro(_set_cmake_policy_to_new_if_available policy)
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endmacro()
_set_cmake_policy_to_new_if_available(CMP0000)
_set_cmake_policy_to_new_if_available(CMP0001)
_set_cmake_policy_to_new_if_available(CMP0002)
_set_cmake_policy_to_new_if_available(CMP0003)
_set_cmake_policy_to_new_if_available(CMP0004)
_set_cmake_policy_to_new_if_available(CMP0005)
_set_cmake_policy_to_new_if_available(CMP0006)
_set_cmake_policy_to_new_if_available(CMP0007)
_set_cmake_policy_to_new_if_available(CMP0008)
_set_cmake_policy_to_new_if_available(CMP0009)
_set_cmake_policy_to_new_if_available(CMP0010)
_set_cmake_policy_to_new_if_available(CMP0011)
_set_cmake_policy_to_new_if_available(CMP0012)
_set_cmake_policy_to_new_if_available(CMP0013)
_set_cmake_policy_to_new_if_available(CMP0014)
_set_cmake_policy_to_new_if_available(CMP0015)
_set_cmake_policy_to_new_if_available(CMP0016)
_set_cmake_policy_to_new_if_available(CMP0017)

# the following operations must be performed inside a project context
if(NOT PROJECT_NAME)
  project(ament_cmake_internal)
endif()

# include CMake functions
include(CMakeParseArguments)

# various functions / macros
foreach(filename
    ament_execute_extensions
    ament_package
    ament_package_xml
    ament_register_extension
    assert_file_exists
    list_append_unique
    python
    stamp
    string_ends_with
  )
  include(${ament_cmake_core_DIR}/${filename}.cmake)
endforeach()

# ensure that no current package name is set
unset(_AMENT_PACKAGE_NAME)
