#
# Include all registered extensions.
#
# :param extension_point: the name of the extension point
# :type extension_point: string
#
# @public
#
macro(ament_execute_extensions extension_point)
  if(AMENT_EXTENSIONS_${extension_point})
    foreach(_extension ${AMENT_EXTENSIONS_${extension_point}})
      string(REPLACE ":" ";" _extension_list "${_extension}")
      list(LENGTH _extension_list _length)
      if(NOT _length EQUAL 2)
        message(FATAL_ERROR "ament_execute_extensions(${extension_point}) "
          "registered extension '${_extension}' can not be split into package "
          "name and cmake filename")
      endif()
      list(GET _extension_list 0 _pkg_name)
      list(GET _extension_list 1 _cmake_filename)
      set(_extension_file "${${_pkg_name}_DIR}/${_cmake_filename}")
      assert_file_exists("${_extension_file}"
        "ament_execute_extensions(${extension_point}) registered extension "
        "'${_extension_file}' does not exist")
      include("${_extension_file}")
    endforeach()
  endif()
endmacro()
