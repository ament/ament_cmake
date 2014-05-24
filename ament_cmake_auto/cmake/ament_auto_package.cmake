#
# Export information, install files and targets, execute the
# extension point ``ament_auto_package`` and invoke
# ``ament_package()``.
#
# Export all found build dependencies which are also run
# dependencies.
# If the package has an include directory install all recursively
# found header files (ending in h, hh, hpp, hxx) and export the
# include directory.
# Export and install all library targets and install all executable
# targets.
#
# @public
#

macro(ament_auto_package)
  # export all found build dependencies which are also run dependencies
  set(_run_depends ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS} ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS} ${${PROJECT_NAME}_EXEC_DEPENDS})
  foreach(_dep ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS} ${${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS})
    list(FIND _run_depends "${_dep}" _index)
    if(NOT _index EQUAL -1)
      ament_export_dependencies("${_dep}")
    endif()
  endforeach()

  # export and install include directory of this package if it has one
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    ament_export_include_directories("include")
    install(DIRECTORY include/ DESTINATION include)
  endif()

  # export and install all libraries
  if(NOT "${${PROJECT_NAME}_LIBRARIES}" STREQUAL "")
    ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
    install(
      TARGETS ${${PROJECT_NAME}_LIBRARIES}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )
  endif()

  # install all executables
  if(NOT "${${PROJECT_NAME}_EXECUTABLES}" STREQUAL "")
    install(
      TARGETS ${${PROJECT_NAME}_EXECUTABLES}
      RUNTIME DESTINATION bin
    )
  endif()

  ament_execute_extensions(ament_auto_package)

  ament_package()
endmacro()
