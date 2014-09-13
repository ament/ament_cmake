# copied from ament_cmake_gtest/ament_cmake_gtest-extras.cmake

# find gtest and create library targets once
macro(_ament_cmake_gtest_find_gtest)
  if(NOT DEFINED _AMENT_CMAKE_GTEST_FIND_GTEST)
    set(_AMENT_CMAKE_GTEST_FIND_GTEST TRUE)

    find_package(ament_cmake_test REQUIRED)

    # allow other packages to find gtest instead
    ament_execute_extensions(ament_cmake_gtest_find_gtest)

    if(NOT GTEST_FOUND)
      find_package(GTest QUIET)
    endif()
    if(GTEST_FOUND)
      message(STATUS "Found gtest: C++ tests using 'Google Test' will be built")
      set(GTEST_FOUND ${GTEST_FOUND} CACHE INTERNAL "")
      set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS} CACHE INTERNAL "")
      set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")
      set(GTEST_MAIN_LIBRARIES ${GTEST_MAIN_LIBRARIES} CACHE INTERNAL "")
      set(GTEST_BOTH_LIBRARIES ${GTEST_BOTH_LIBRARIES} CACHE INTERNAL "")
    else()
      # find gtest include and source folders
      # fall back to system installed path (i.e. on Ubuntu)
      set(_search_path_include "/usr/include/gtest")
      set(_search_path_src "/usr/src/gtest/src")
      # option() consider environment variable to find gtest
      if(NOT "$ENV{GTEST_DIR} " STREQUAL " ")
        list(INSERT _search_path_include 0 "$ENV{GTEST_DIR}/include/gtest")
        list(INSERT _search_path_src 0 "$ENV{GTEST_DIR}/src")
      endif()
      find_file(_gtest_header_file "gtest.h"
        PATHS ${_search_path_include}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )
      find_file(_gtest_src_file "gtest.cc"
        PATHS ${_search_path_src}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )

      if(_gtest_header_file AND _gtest_src_file)
        get_filename_component(_gtest_base_dir "${_gtest_src_file}" PATH)
        get_filename_component(_gtest_base_dir "${_gtest_base_dir}" PATH)
        # add CMakeLists.txt from gtest dir
        set(_gtest_binary_dir "${CMAKE_BINARY_DIR}/gtest")
        add_subdirectory("${_gtest_base_dir}" ${_gtest_binary_dir})
        # mark gtest targets with EXCLUDE_FROM_ALL to only build
        # when tests are built which depend on them
        set_target_properties(gtest gtest_main PROPERTIES EXCLUDE_FROM_ALL 1)
        get_filename_component(_gtest_include_dir "${_gtest_header_file}" PATH)
        get_filename_component(_gtest_include_dir ${_gtest_include_dir} PATH)
        # set from-source variables
        set(GTEST_FROM_SOURCE_FOUND TRUE CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_INCLUDE_DIRS "${_gtest_include_dir}"
          CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARY_DIRS "${_gtest_binary_dir}"
          CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_LIBRARIES "gtest" CACHE INTERNAL "")
        set(GTEST_FROM_SOURCE_MAIN_LIBRARIES "gtest_main" CACHE INTERNAL "")
        message(STATUS "Found gtest sources under '${_gtest_base_dir}': "
          "C++ tests using 'Google Test' will be built")
      endif()
      if(GTEST_FROM_SOURCE_FOUND)
        # set the same variables as find_package()
        # but do NOT set GTEST_FOUND in the cache when using gtest from source
        # since the subdirectory must always be added to add the gtest targets
        set(GTEST_FOUND ${GTEST_FROM_SOURCE_FOUND})
        set(GTEST_INCLUDE_DIRS ${GTEST_FROM_SOURCE_INCLUDE_DIRS})
        set(GTEST_LIBRARY_DIRS ${GTEST_FROM_SOURCE_LIBRARY_DIRS})
        set(GTEST_LIBRARIES ${GTEST_FROM_SOURCE_LIBRARIES})
        set(GTEST_MAIN_LIBRARIES ${GTEST_FROM_SOURCE_MAIN_LIBRARIES})
        set(GTEST_BOTH_LIBRARIES ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
      endif()
      if(NOT GTEST_FOUND)
        message(WARNING
          "'gtest' not found, C++ tests using 'Google Test' can not be built. "
          "Please install the 'Google Test' headers globally in your system "
          "to enable these tests (e.g. on Ubuntu/Debian install the package "
          "'libgtest-dev')")
      endif()
    endif()

    # for Visual 2012 we need to increase the fixed variadic template size to
    # build gtest (https://code.google.com/p/googletest/source/detail?r=675)
    if(GTEST_FOUND AND MSVC AND MSVC_VERSION EQUAL 1700)
      add_definitions(/D _VARIADIC_MAX=10)
    endif()
  endif()
endmacro()

include("${ament_cmake_gtest_DIR}/ament_add_gtest.cmake")
include("${ament_cmake_gtest_DIR}/ament_find_gtest.cmake")
