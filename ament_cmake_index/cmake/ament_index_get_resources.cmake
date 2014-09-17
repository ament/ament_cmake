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
  if("${resource_type}" STREQUAL "")
    message(FATAL_ERROR
      "ament_index_get_resources() called without a 'resource_type'")
  endif()

  if(NOT "${ARGN}" STREQUAL "")
    message(FATAL_ERROR "ament_index_get_resources() called with unused "
      "arguments: ${ARGN}")
  endif()

  set(paths_to_search $ENV{AMENT_PREFIX_PATH})
  # Remove CMAKE_INSTALL_PREFIX if it is in the list of paths to search,
  # and add it to the list at the front
  list(REMOVE_ITEM paths_to_search "${CMAKE_INSTALL_PREFIX}")
  list(INSERT paths_to_search 0 "${CMAKE_INSTALL_PREFIX}")
  set(all_resources "")
  foreach(path IN LISTS paths_to_search)
    file(GLOB resources
      RELATIVE "${path}/share/ament_index/resource_index/${resource_type}"
      "${path}/share/ament_index/resource_index/${resource_type}/*")
    list(APPEND all_resources "${resources}")
  endforeach()
  set(${var} "${all_resources}" PARENT_SCOPE)
endfunction()
