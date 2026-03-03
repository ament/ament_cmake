# ament_cmake_python_test

This package exists solely to test the ament_cmake_python package.

Packages to test are prepared as typical ros-style packages (e.g. with packages.xml). Some of these packages
exist in subdirectory `test/packages`, others are generated dynamically. The dynamically generated packages, as well
as the `build` and `install` subdirectories, are generated in a user's pytest temporary directories (which is typically
at /tmp/pytest-of-{username}/pytest-NN)

Testing can be initiated using normal ROS test commmands, that is `colcon test --packages-select ament_cmake_python_test`
