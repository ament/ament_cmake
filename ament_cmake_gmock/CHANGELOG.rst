^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ament_cmake_gmock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.11 (2024-11-25)
-------------------

1.3.10 (2024-07-26)
-------------------

1.3.9 (2024-05-15)
------------------

1.3.8 (2024-02-16)
------------------

1.3.7 (2024-01-24)
------------------
* Fix compiler warnings related to gtest/gmock (`#408 <https://github.com/ament/ament_cmake/issues/408>`_) (`#495 <https://github.com/ament/ament_cmake/issues/495>`_)
* Contributors: atzaros

1.3.6 (2023-11-13)
------------------

1.3.5 (2023-06-22)
------------------

1.3.4 (2023-04-25)
------------------

1.3.3 (2022-11-07)
------------------

1.3.2 (2022-05-17)
------------------

1.3.1 (2022-03-28)
------------------

1.3.0 (2022-02-17)
------------------
* Update forthcoming version in changelog
* Contributors: Audrow Nash

1.2.1 (2022-01-14)
------------------
* Update maintainers to Michael Jeronimo and Michel Hidalgo (`#362 <https://github.com/ament/ament_cmake/issues/362>`_)
* Contributors: Audrow Nash

1.2.0 (2021-10-29)
------------------
* Use FindPython3 instead of FindPythonInterp (`#355 <https://github.com/ament/ament_cmake/issues/355>`_)
* Update maintainers (`#336 <https://github.com/ament/ament_cmake/issues/336>`_)
* Contributors: Chris Lalancette, Shane Loretz

1.1.4 (2021-05-06)
------------------

1.1.3 (2021-03-09)
------------------

1.1.2 (2021-02-26 22:59)
------------------------

1.1.1 (2021-02-26 19:12)
------------------------

1.1.0 (2021-02-24)
------------------

1.0.4 (2021-01-25)
------------------

1.0.3 (2020-12-10)
------------------

1.0.2 (2020-12-07)
------------------
* Update package maintainers. (`#286 <https://github.com/ament/ament_cmake/issues/286>`_)
* Contributors: Michel Hidalgo

1.0.1 (2020-09-10)
------------------

1.0.0 (2020-07-22)
------------------

0.9.6 (2020-06-23)
------------------

0.9.5 (2020-06-02)
------------------

0.9.4 (2020-05-26)
------------------

0.9.3 (2020-05-19)
------------------

0.9.2 (2020-05-07)
------------------

0.9.1 (2020-04-24 15:45)
------------------------

0.9.0 (2020-04-24 12:25)
------------------------
* Don't pass FALSE value to ament_add_test when SKIP_TEST is not provided (`#236 <https://github.com/ament/ament_cmake/issues/236>`_)
* Contributors: Emerson Knapp

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-10-04)
------------------
* Revert "Add gtest and gmock headers as system headers: (`#175 <https://github.com/ament/ament_cmake/issues/175>`_)" (`#184 <https://github.com/ament/ament_cmake/issues/184>`_)
  This reverts commit e1ff1c1a0a1e08d43e939cdb943a88be601808bd.
* Add gtest and gmock headers as system headers: (`#175 <https://github.com/ament/ament_cmake/issues/175>`_)
  Certain gtest and gmock header files contain constructs
  which generate warnings when certain compile flags are
  enabled. By including the header files as system headers,
  the compiler knows that it doesn't need to generate these
  warnings since they are coming from (third-party) system
  headers
* Add 'runner' option to ament_add_gmock / nose (`#177 <https://github.com/ament/ament_cmake/issues/177>`_)
  * Add 'runner' option to ament_add_gmock
  * Give ament_add_nose ability to specify a different runner, too
* Contributors: Peter Baughman, Shane Loretz, jpsamper2009

0.7.3 (2019-05-29)
------------------

0.7.2 (2019-05-20)
------------------

0.7.1 (2019-05-07)
------------------
* Fix unused-arg check in ament_cmake packages: (`#167 <https://github.com/ament/ament_cmake/issues/167>`_)
  Arguments to a macro are not variables, so it's not
  possible to do 'if(ARGN)' to check for arguments;
  however, copying ARGN to a variable works.
* Contributors: jpsamper2009

0.7.0 (2019-04-08)
------------------

0.6.0 (2018-11-13)
------------------

0.5.1 (2018-07-17)
------------------
* Add SKIP_TEST flag to ament_add_gmock() (`#143 <https://github.com/ament/ament_cmake/issues/143>`_)
* Contributors: Andreas Greimel

0.5.0 (2018-06-13)
------------------

0.4.0 (2017-12-08)
------------------
* 0.0.3
* Merge pull request `#104 <https://github.com/ament/ament_cmake/issues/104>`_ from ament/googletest
  update to googletest 1.8
* update to googletest 1.8
* 0.0.2
* Merge pull request `#94 <https://github.com/ament/ament_cmake/issues/94>`_ from ament/logging
  suppress compiler warnings within gmock/gtest
* suppress missing-field-initializers warnings
* prevent adding the gmock subdirectory twice (`#91 <https://github.com/ament/ament_cmake/issues/91>`_)
* Merge pull request `#86 <https://github.com/ament/ament_cmake/issues/86>`_ from ament/remove_include
  remove unnecessary include
* remove unnecessary include
* update schema url
* add schema to manifest files
* Merge pull request `#72 <https://github.com/ament/ament_cmake/issues/72>`_ from ament/cmake35
  require CMake 3.5
* remove trailing spaces from comparisons, obsolete quotes and explicit variable expansion
* remove obsolete policies
* require CMake 3.5
* Merge pull request `#54 <https://github.com/ament/ament_cmake/issues/54>`_ from ament/test_working_dir
  support WORKING_DIRECTORY in ament_add_nose_test
* fix WORKING_DIRECTORY for ament_add_gtest/gmock
* follow fixes from `#52 <https://github.com/ament/ament_cmake/issues/52>`_
* Merge pull request `#52 <https://github.com/ament/ament_cmake/issues/52>`_ from ament/add_test_append_env_option
  add APPEND_ENV and APPEND_LIBRARY_DIRS options to ament_add\_*test macros
* add APPEND_ENV and APPEND_LIBRARY_DIRS options to ament_add\_*test macros
* Merge pull request `#50 <https://github.com/ament/ament_cmake/issues/50>`_ from ament/pass_extra_env_to_tests
  add option to pass extra env to ament_add\_*test
* addressing comments
* Merge pull request `#37 <https://github.com/ament/ament_cmake/issues/37>`_ from ament/test_labels
  add labels to tests
* add labels to tests
* Merge pull request `#34 <https://github.com/ament/ament_cmake/issues/34>`_ from ament/prevent_gtest_in_cache
  refactor finding GTest / GMock
* refactor finding GTest / GMock
* Merge pull request `#29 <https://github.com/ament/ament_cmake/issues/29>`_ from ament/suppress_cmp0026
  set cmp0026 to OLD until we can migrate to use $<TARGET_FILE:...>
* update comment and set the policy in two other places
* Merge pull request `#28 <https://github.com/ament/ament_cmake/issues/28>`_ from ament/gtest_location
  fix location of gtest / gmock executables on Windows
* add type as extension to test result files
* fix location of gtest executable on Windows
* Merge pull request `#25 <https://github.com/ament/ament_cmake/issues/25>`_ from ament/use_gmock_vendor
  optionally use gmock_vendor
* optionally use gtest/gmock_vendor
* update ament_add_gmock to support SKIP_LINKING_MAIN_LIBRARIES
* add explicit build type
* disable debug output
* add missing copyright / license information, update format of existing license information
* update quoting of additional ament_add_test() arguments
* use project(.. NONE)
* refactor several low-level packages into ament_cmake_core (environment, environment_hooks, index, package_templates, symlink_install)
* invert dependency between ament_cmake_environment and ament_cmake_environment_hooks, add dependency on ament_cmake_environment
* deal with CMake double expansion
* update cmake code style
* add ament_cmake_gmock
* Contributors: Dirk Thomas, William Woodall
