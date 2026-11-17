# Express Version Compatibility in Dependencies

`ament_export_dependencies` shall propagate the version requested of dependencies to a call to `find_dependency <version>`.

For example:
`find_package(foo_core CONFIG 4)` results in
`add_dependency(foo_core CONFIG 4)`. 

`find_package(foo_core CONFIG 4.8.3-dev2)` results in
`add_dependency(foo_core CONFIG 4.8.3-dev2)`. 

`find_package(foo_core MODULE)` results in
`add_dependency(foo_core MODULE)`. 

I think this can be done automatically with the use of `foo_core_FIND_VERSION`.

Adding in the find method of MODULE/CONFIG may be out of scope for this discussion.

I want to avoid a user manually maintaining versions in multiple places.
It's hard enough as it is to remember when you add a find_package call to also add it to `ament_export_dependencies`.
