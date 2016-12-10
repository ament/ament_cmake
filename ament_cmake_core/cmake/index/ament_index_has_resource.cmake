#
# Check if the index contains a specific resource.
#
# :param var: the flag if the requested resource exists
# :type var: boolean flag
# :param resource_type: the type of the resource
# :type resource_type: string
# :param resource_name: the name of the resource
# :type resource_name: string
#
# @public
#
function(ament_index_has_resource var resource_type resource_name)
  if("${resource_type} " STREQUAL " ")
    message(FATAL_ERROR
      "ament_index_has_resource() called without a 'resource_type'")
  endif()
  if("${resource_name} " STREQUAL " ")
    message(FATAL_ERROR
      "ament_index_has_resource() called without a 'resource_name'")
  endif()

  if(NOT "${ARGN} " STREQUAL " ")
    message(FATAL_ERROR "ament_index_has_resource() called with unused "
      "arguments: ${ARGN}")
  endif()

  string(REPLACE ":" ";" paths_to_search "$ENV{AMENT_PREFIX_PATH}")
  # Remove CMAKE_INSTALL_PREFIX if it is in the list of paths to search,
  # and add it to the list at the front
  list(REMOVE_ITEM paths_to_search "${CMAKE_INSTALL_PREFIX}")
  list(INSERT paths_to_search 0 "${CMAKE_INSTALL_PREFIX}")
  set(retval FALSE)
  foreach(path IN LISTS paths_to_search)
    if(EXISTS
        "${path}/share/ament_index/resource_index/${resource_type}/${resource_name}")
      set(retval TRUE)
      break()
    endif()
  endforeach()
  set(${var} "${retval}" PARENT_SCOPE)
endfunction()
