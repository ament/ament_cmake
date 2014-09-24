#
# Append elements to a list if they are not already in the list.
#
# :param list: the list
# :type list: list variable
# :param ARGN: the elements
# :type ARGN: list of strings
#
# .. note:: Using CMake's ``list(APPEND ..)`` and
#   ``list(REMOVE_DUPLICATES ..)`` is not sufficient since its
#   implementation uses a set internally which makes the operation
#   unstable.
#
function(list_append_unique list)
  foreach(element ${ARGN})
    list(FIND ${list} "${element}" index)
    if(index EQUAL -1)
      list(APPEND ${list} "${element}")
    endif()
  endforeach()
  set(${list} ${${list}} PARENT_SCOPE)
endfunction()
