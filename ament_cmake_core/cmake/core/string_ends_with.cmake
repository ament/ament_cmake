#
# Check if a string ends with a specific suffix.
#
# :param str: the string
# :type str: string
# :param suffix: the suffix
# :type suffix: string
# :param var: the output variable name
# :type var: bool
#
function(string_ends_with str suffix var)
  string(LENGTH "${str}" str_length)
  string(LENGTH "${suffix}" suffix_length)
  set(value FALSE)
  if(NOT ${str_length} LESS ${suffix_length})
    math(EXPR str_offset "${str_length} - ${suffix_length}")
    string(SUBSTRING "${str}" ${str_offset} ${suffix_length} str_suffix)
    if("${str_suffix} " STREQUAL "${suffix} ")
      set(value TRUE)
    endif()
  endif()
  set(${var} ${value} PARENT_SCOPE)
endfunction()
