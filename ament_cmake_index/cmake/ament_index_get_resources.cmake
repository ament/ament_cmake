#
# Get all registered package resource of a specific type from the index.
#
# :param var: the output variable name
# :type var: list of resource names
# :param resource_type: the type of the resource
# :type resource_type: string
#
# @public
#
function(ament_index_get_resources var resource_type)
  if("${resource_type} " STREQUAL " ")
    message(FATAL_ERROR
      "ament_index_get_resources() called without a 'resource_type'")
  endif()

  if(NOT "${ARGN} " STREQUAL " ")
    message(FATAL_ERROR "ament_index_get_resources() called with unused "
      "arguments: ${ARGN}")
  endif()

  # TODO search in all workspaces
  file(GLOB resources RELATIVE "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/${resource_type}" "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/${resource_type}/*")
  set(${var} "${resources}" PARENT_SCOPE)
endfunction()
