# ament_cmake_python_test

This package exists solely to test the ament_cmake_python package.

It runs `colcon build` on some test packages, with the working directory `build/ament_cmake_package_test`.
That means that the normal `build`, `install`, and `log` directories are subdirectories of `build/ament_cmake_package_test`
