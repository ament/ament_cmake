#
# Deduplicate libraries.
#
# If the list contains duplicates only the last value is kept.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries.
#   Each element might either a build configuration keyword or a library.
# :type ARGN: list of strings
#
# @public
#
macro(ament_libraries_deduplicate VAR)
  ament_libraries_pack_build_configuration(_packed ${ARGN})
  set(_unique "")
  foreach(_lib ${_packed})
    # remove existing value if it exists
    list(REMOVE_ITEM _unique ${_lib})
    # append value to the end
    list(APPEND _unique ${_lib})
  endforeach()
  ament_libraries_unpack_build_configuration(${VAR} ${_unique})
endmacro()
