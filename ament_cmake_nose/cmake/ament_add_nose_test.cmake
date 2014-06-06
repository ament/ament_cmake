# include CMake functions
include(CMakeParseArguments)

#
# Add a nose test.
#
# :param testname: the name of the test
# :type testname: string
# :param path: the path to a file or folder where ``nosetests``
#   should be invoked on
# :type path: string
# :param TIMEOUT: the test timeout in seconds, default: 60
# :type TIMEOUT: integer
#
# @public
#
macro(ament_add_nose_test testname path)
  _ament_cmake_nose_find_nosetests()
  if(NOSETESTS)
    _ament_add_nose_test("${testname}" "${path}" ${ARGN})
  endif()
endmacro()

function(_ament_add_nose_test testname path)
  cmake_parse_arguments(ARG "" "TIMEOUT" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_add_nose_test() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT IS_ABSOLUTE "${path}")
    set(path "${CMAKE_CURRENT_SOURCE_DIR}/${path}")
  endif()
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR
      "ament_add_nose_test() the path '${path}' does not exist")
  endif()

  set(cmd
    "${NOSETESTS} \"${path}\" --with-xunit \"--xunit-file=${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xml\"")
  if(ARG_TIMEOUT)
    set(ARG_TIMEOUT "TIMEOUT" "${ARG_TIMEOUT}")
  endif()

  ament_add_test(
    "${testname}"
    COMMAND ${cmd}
    ${ARG_TIMEOUT}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
endfunction()

# TODO provide function to register all found tests separately
