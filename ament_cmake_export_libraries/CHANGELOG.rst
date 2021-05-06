^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ament_cmake_export_libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Fix variable name in ament_export_libraries.cmake (`#314 <https://github.com/ament/ament_cmake/issues/314>`_)
* Contributors: Alejandro Hernández Cordero

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
* use OUTPUT_NAME of exported library if set (`#239 <https://github.com/ament/ament_cmake/issues/239>`_)
* Contributors: Dirk Thomas

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-10-04)
------------------
* _library_dirs -> _library_dirs_suffix (`#179 <https://github.com/ament/ament_cmake/issues/179>`_)
* Contributors: Shane Loretz

0.7.3 (2019-05-29)
------------------

0.7.2 (2019-05-20)
------------------

0.7.1 (2019-05-07)
------------------

0.7.0 (2019-04-08)
------------------

0.6.0 (2018-11-13)
------------------
* fix regex for build configuration keywords (`#148 <https://github.com/ament/ament_cmake/issues/148>`_)
* Contributors: Dirk Thomas

0.5.1 (2018-07-17)
------------------

0.5.0 (2018-06-13)
------------------

0.4.0 (2017-12-08)
------------------
* 0.0.3
* Merge pull request `#103 <https://github.com/ament/ament_cmake/issues/103>`_ from ament/resolve_some_todos
  Resolve some todos
* move todo to line with comment
* 0.0.2
* Revert "consider LOCATION property if IMPORTED_LOCATION is not set" (`#83 <https://github.com/ament/ament_cmake/issues/83>`_)
* Merge pull request `#81 <https://github.com/ament/ament_cmake/issues/81>`_ from ament/consider_location_property
  consider LOCATION property if IMPORTED_LOCATION is not set
* consider LOCATION property if IMPORTED_LOCATION is not set
* Merge pull request `#75 <https://github.com/ament/ament_cmake/issues/75>`_ from ament/refactor_library_export
  keep order of exported libraries and allow linker flags
* keep order of exported libraries and allow linker flags
* update schema url
* add schema to manifest files
* Merge pull request `#72 <https://github.com/ament/ament_cmake/issues/72>`_ from ament/cmake35
  require CMake 3.5
* remove trailing spaces from comparisons, obsolete quotes and explicit variable expansion
* require CMake 3.5
* Merge pull request `#42 <https://github.com/ament/ament_cmake/issues/42>`_ from ament/reuse_hook_from_ament_package
  reuse environment hook provided by ament_package
* reuse environment hook provided by ament_package
* Merge pull request `#39 <https://github.com/ament/ament_cmake/issues/39>`_ from ament/remove_lib_from_path
  remove the lib folder from the PATH on Windows
* remove the lib folder from the PATH on Windows
* add explicit build type
* disable debug output
* add missing copyright / license information, update format of existing license information
* Merge pull request `#3 <https://github.com/ament/ament_cmake/issues/3>`_ from ament/windows
  Windows Support
* escalating missing library to FATAL_ERROR
  It was previously a WARNING in CMake, but that
  leads to missing symbol errors, which can be
  misleading since the library was actually not
  found but the first inclination is to check the
  library which contains the symbols for errors.
  We might consider the need to change this back
  in the future for cases where having the library
  is not critical.
* addressing review comments
* addressing review comments
* [windows] add missing file ext
* [windows] remove redundant .bat
* [windows] compact file extension logic
* [windows] fix bug in prepend unique bat function
* [windows] add batch version of env hooks
* use project(.. NONE)
* refactor several low-level packages into ament_cmake_core (environment, environment_hooks, index, package_templates, symlink_install)
* invert dependency between ament_cmake_environment and ament_cmake_environment_hooks, add dependency on ament_cmake_environment
* refactor to use templates provided by ament_package
* deal with CMake double expansion
* fix exported library names
* fix exporting absolute libraries
* update cmake code style
* add ament_cmake_gmock
* add ament_cmake_auto
* add ament_cmake_environment_hooks
* minor
* add ament_cmake_export_libraries
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
