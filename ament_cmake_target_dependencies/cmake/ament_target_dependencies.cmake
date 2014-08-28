#
# Add the definitions, include directories and libraries of packages to a
# target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be _DEFINITIONS, _INCLUDE_DIRS and _LIBRARIES.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names
# :type ARGN: list of strings
#
# @public
#
function(ament_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    message(" - ament_target_dependencies(${target} ${ARGN})")
    foreach(package_name ${ARGN})
      message(" - ament_target_dependencies(${target}) ${package_name}")
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      target_compile_definitions(${target}
        PUBLIC ${${package_name}_DEFINITIONS})
      target_include_directories(${target}
        PUBLIC ${${package_name}_INCLUDE_DIRS})
      target_link_libraries(${target}
        PUBLIC ${${package_name}_LIBRARIES})
    endforeach()
  endif()
endfunction()
