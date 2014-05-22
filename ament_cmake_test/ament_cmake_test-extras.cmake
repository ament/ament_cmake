# copied from ament_cmake_test/ament_cmake_test-extras.cmake

# register environment hook for libraries once
option(AMENT_ENABLE_TESTING "Enable testing" OFF)
# option()
set(
  AMENT_TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results"
  CACHE STRING "The path where test results are generated"
)

if(AMENT_ENABLE_TESTING)
  enable_testing()
endif()

find_package(ament_cmake_core)

include("${ament_cmake_test_DIR}/ament_add_test.cmake")
