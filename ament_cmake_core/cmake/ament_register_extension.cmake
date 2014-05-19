#
# Register a CMake filename to be included as part of an extension
# point.
#
# :param extension_point: the name of the extension point
# :type extension_point: string
# :param package_name: the name of the package containing the CMake
#   file
# :type package_name: string
# :param cmake_filename: the path to a CMake file relative to the
#   ${package_name}_DIR folder
# :type cmake_filename: string
#
# @public
#
macro(ament_register_extension extension_point package_name cmake_filename)
  list(APPEND AMENT_EXTENSIONS_${extension_point} "${package_name}:${cmake_filename}")
endmacro()
