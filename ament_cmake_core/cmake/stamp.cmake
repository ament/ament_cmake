#
#   :param path:  file name
#
#   Uses ``configure_file`` to generate a file ``filepath.stamp`` hidden
#   somewhere in the build tree.  This will cause cmake to rebuild its
#   cache when ``filepath`` is modified.
#
function(stamp path)
  get_filename_component(filename "${path}" NAME)
  configure_file(
    "${path}"
    "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_core/stamps/${filename}.stamp"
    COPYONLY
  )
endfunction()
