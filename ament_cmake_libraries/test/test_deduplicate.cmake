include("${CMAKE_CURRENT_LIST_DIR}/utilities.cmake")

# Empty
set(TEST_IN "")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("" "${ACTUAL}")

# Noop
set(TEST_IN "foo;bar;baz")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("foo;bar;baz" "${ACTUAL}")

# Simple
set(TEST_IN "foo;bar;baz;bar")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("foo;baz;bar" "${ACTUAL}")

# With matching build configs
set(TEST_IN "debug;foo;debug;bar;debug;baz;debug;bar")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("debug;foo;debug;baz;debug;bar" "${ACTUAL}")

# With missing build configs
set(TEST_IN "debug;foo;debug;bar;debug;baz;bar")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("debug;foo;debug;bar;debug;baz;bar" "${ACTUAL}")

# With mismatched build configs
set(TEST_IN "debug;foo;debug;bar;debug;baz;release;bar")
ament_libraries_deduplicate(ACTUAL ${TEST_IN})
assert_equal("debug;foo;debug;bar;debug;baz;release;bar" "${ACTUAL}")
