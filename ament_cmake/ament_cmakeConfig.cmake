# mark as found
set(ament_cmake_FOUND)

# define set of default components if not specified
if(NOT ament_cmake_FIND_COMPONENTS)
  set(ament_cmake_FIND_COMPONENTS "core" "environment" "export_dependencies" "export_include_directories" "export_libraries" "index" "python" "symlink_install")
endif()

# find_package each component
set(ament_cmake_COMPONENTS "")
foreach(component ${ament_cmake_FIND_COMPONENTS})
  # find package component
  set(required_flag "")
  if(ament_cmake_FIND_REQUIRED)
    set(required_flag "REQUIRED")
  endif()

  set(quiet_flag "")
  if(ament_cmake_FIND_QUIETLY)
    set(quiet_flag "QUIET")
  endif()

  find_package(ament_cmake_${component} ${required_flag} ${quiet_flag} CONFIG)

  # export found components
  if(ament_cmake_${component}_FOUND)
    list(APPEND ament_cmake_COMPONENTS "ament_cmake_${component}")
  endif()
endforeach()
