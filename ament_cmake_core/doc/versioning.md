## Express Version Compatibility in the Installation

Add a COMPATIBILITY argument to `ament_package`. Default to AnyNewerVersion if not supplied which matches current behavior.

User call in `foo's` `CMakeLists.txt`:
```cmake
ament_package(COMPATIBILITY SameMajorVersion)
```

The compatibility would be forwarded to `write_basic_package_version_file`.

Then, you can use it in `bar`
```cmake
find_package(foo_core 4 CONFIG REQUIRED)
```

If you forgot to update `foo_core` from 3 to 4 and rebuild your workspace, then colcon 
will fail at the configure stage of `bar` with a nice error message.

