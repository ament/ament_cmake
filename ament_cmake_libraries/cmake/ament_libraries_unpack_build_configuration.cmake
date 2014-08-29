#
# Unpack a list of libraries with optional build configuration keyword prefixes.
#
# Libraries prefixed with a keyword are split into the keyword and the library.
#
# :param VAR: the output variable name
# :type VAR: string
# :param ARGN: a list of libraries
# :type ARGN: list of strings
#
macro(ament_libraries_unpack_build_configuration VAR)
  set(${VAR} "")
  foreach(_lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" _lib "${_lib}")
    list(APPEND ${VAR} "${_lib}")
  endforeach()
endmacro()
