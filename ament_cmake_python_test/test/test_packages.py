# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Test of ament_python_install_package."""

# Not used until PR #587
# import filecmp
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import subprocess

from jinja2 import Template
from options import get_options
import pytest


SOURCE_DIR = Path(__file__).parent.parent
PYTHON_INSTALL_DIR = Path(os.environ.get('PYTHON_INSTALL_DIR'))
PYEGG_VERSION = os.environ.get('PYEGG_VERSION')
CMAKE_COMMAND = os.environ.get('CMAKE_COMMAND', 'cmake')

AMENT_PYTHON_TEST_PACKAGE = 'ament_python_test_package'
AMENT_PYTHON_TEST_PACKAGE_OVERLAY = AMENT_PYTHON_TEST_PACKAGE + '_overlay'


@pytest.fixture(scope='module')
def module_dir(tmp_path_factory):
    """Create a temporary directory for the module."""
    return tmp_path_factory.getbasetemp()


def generate_package(options, package_dir):
    """Generate a package from the template using the given options."""
    template_dir = Path(__file__).parent / 'pkg_template'
    python_subdir = options['python_subdir'] or options['name']

    package_dir.mkdir(parents=True)

    install_options = ''
    if options['has_msg']:
        shutil.copytree(template_dir / 'msg', package_dir / 'msg')

    if options['has_python'] or options['has_python_before']:
        ignore_patterns = shutil.ignore_patterns('*.jinja')
        shutil.copytree(template_dir / 'package_directory',
                        package_dir / python_subdir, ignore=ignore_patterns)
        template = Template(
            Path.read_text(template_dir / 'package_directory' / '__init__.py.jinja'))
        Path.write_text(package_dir / python_subdir / '__init__.py', template.render(options))

    if options['version']:
        install_options += f' VERSION {options["version"]}'

    if options['setup_cfg']:
        (package_dir / options['setup_cfg']).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(template_dir / options['setup_cfg'],
                    package_dir / (Path(options['setup_cfg']).parent))
        install_options += f' SETUP_CFG {options["setup_cfg"]}'

    if options['scripts_destination']:
        scripts_dir = template_dir / 'python_scripts'
        shutil.copytree(scripts_dir, package_dir / python_subdir, dirs_exist_ok=True)
        template = Template(Path.read_text(template_dir / 'setup.cfg.jinja'))
        Path.write_text(package_dir / 'setup.cfg', template.render(options))
        install_options += f' SCRIPTS_DESTINATION {options["scripts_destination"]}'

    if options['destination']:
        install_options += f' DESTINATION {options["destination"]}'

    # The python package has a name that differs from the name in package.xml
    if options['python_subdir']:
        install_options += f' PACKAGE_DIR {options["python_subdir"]}'

    options['install_options'] = install_options
    template = Template(Path.read_text(template_dir / 'package.xml.jinja'))
    Path.write_text(package_dir / 'package.xml', template.render(options))
    template = Template(Path.read_text(template_dir / 'CMakeLists.txt.jinja'))
    Path.write_text(package_dir / 'CMakeLists.txt', template.render(options))


def do_build_package(package_name, packages_dir, module_dir, symlink_install=False):
    """Build the package using cmake."""
    build_dir = module_dir / 'build' / package_name
    build_dir.mkdir(parents=True, exist_ok=True)
    package_dir = packages_dir / package_name

    configure_command = [CMAKE_COMMAND,
                         '-S', package_dir,
                         '-B', build_dir, '--install-prefix',
                         str(module_dir / 'install' / package_name)]
    if symlink_install:
        configure_command.append('-DAMENT_CMAKE_SYMLINK_INSTALL=1')
    print(f'Configuring package {package_name} with command: {configure_command}')
    result = subprocess.run(configure_command, text=True)
    assert result.returncode == 0, f'cmake configure failed for package {package_name}'

    build_command = ['cmake', '--build', str(build_dir)]
    print(f'Building package {package_name} with command: {build_command}')
    result = subprocess.run(build_command, text=True)
    assert result.returncode == 0, f'cmake build failed for package {package_name}'

    install_command = ['cmake', '--install', str(build_dir)]
    print(f'Installing package {package_name} with command: {install_command}')
    result = subprocess.run(install_command, text=True)
    assert result.returncode == 0, f'cmake install failed for package {package_name}'


def do_test_package(options, module_dir):
    """Test a package that was generated using the template."""
    install_base = module_dir / 'install'
    package_name = options['name']
    if options['destination']:
        install_path = install_base / package_name / options['destination'] / package_name
    else:
        install_path = install_base / package_name / PYTHON_INSTALL_DIR / package_name
    print(f'install_path for package {package_name}: {install_path}')
    assert install_path.exists(), f'install path does not exist for {package_name}: {install_path}'
    assert (install_path / '__init__.py').exists(), f'missing __init__.py in {install_path}'

    if options['has_python'] or options['has_python_before']:
        assert Path.read_text(
            install_path / '__init__.py').startswith(f'# This is {package_name}'), \
            f'__init__.py should be from {package_name} python package'

    if options['has_msg']:
        assert (install_path / 'msg').is_dir(), \
            f'There should be a msg directory in {install_path}'

    if options['symlink_install']:
        print(f'Testing symlink install in package {package_name}')
        assert (install_path / '__init__.py').is_symlink(), '__init__.py should be a symlink'

    if options['version']:
        print(f'Testing version: {options["version"]} IN EGG-INFO in package {package_name}')
        version = options['version']
        egg_info_dir = install_base / package_name / PYTHON_INSTALL_DIR / \
            f'{package_name}-{version}-{PYEGG_VERSION}.egg-info'
        assert egg_info_dir.exists(), \
            f'egg-info dir does not exist for {package_name}: {egg_info_dir}'
        egg_info_file = egg_info_dir / 'PKG-INFO'
        assert Path.read_text(egg_info_file).find(f'Version: {version}') != -1, \
            f'egg-info file should contain "Version: {version}"'

    if options['setup_cfg']:
        print(f'Testing setup.cfg metadata in package {package_name}')
        print(f'  options: {options}')
        version = options['version'] or '0.0.0'
        egg_info_dir = install_base / package_name / PYTHON_INSTALL_DIR / \
            f'{package_name}-{version}-{PYEGG_VERSION}.egg-info'
        assert egg_info_dir.exists(), \
            f'egg-info dir does not exist for {package_name}: {egg_info_dir}'
        egg_info_file = egg_info_dir / 'PKG-INFO'
        assert Path.read_text(egg_info_file).find('Keywords: test_of_ament_cmake_python') != -1, \
            'egg-info file should contain "Keywords: test_of_ament_cmake_python"'

    if options['scripts_destination']:
        print(f'Testing script installed in package {package_name}')
        script_path = install_base / package_name / \
            options['scripts_destination'] / 'do_something'
        assert script_path.exists(), \
            f'script do_something does not exist for {package_name}: {script_path}'


def do_package(test_spec):
    (options, module_dir) = test_spec
    generated_packages_dir = module_dir / 'packages'
    message = 'Unknown error'
    return_value = -1

    try:
        # Create test package from template
        package_dir = generated_packages_dir / options['name']
        print(f'Generating package {options["name"]}')
        generate_package(options, package_dir)

        # Build each package using cmake
        print(f'Building package {options["name"]}')
        do_build_package(
            options['name'], generated_packages_dir, module_dir, options['symlink_install'])

        # Test each generated package
        print(f'Testing package {options["name"]}')
        do_test_package(options, module_dir)
    except AssertionError as e:
        message = str(e)
        return_value = 2
    except BaseException as e:  # noqa: B902
        message = str(e)
        return_value = 3
    else:
        return_value = 0
    finally:
        print(f'Package {options["name"]} finished with return code {return_value}: {message}')
    return (options['name'], return_value, message)


def test_from_options(module_dir):
    """Generate, build, and test packages from the options."""
    # Uncomment to debug environment issues
    # print('Environment: ')
    # for name, value in os.environ.items():
    #     print(f'{name}={value}')
    # assert False

    pool = mp.Pool()
    pool_results = pool.imap_unordered(
        do_package, [(options, module_dir) for options in get_options()])
    while True:
        try:
            (name, returns, message) = pool_results.next()
            print(f'Package {name} returned {returns}: {message}')
            assert returns == 0, f'Package {name} failed with code {returns}: {message}'
        except StopIteration:
            break
    # I'd prefer close() then join() but that seems to sometimes hang.
    pool.terminate()


'''Disable tests until PR #587 is merged
def test_ament_python_test_package(module_dir) -> None:
    """Test installing a known package and comparing to the source files."""
    do_build_package(AMENT_PYTHON_TEST_PACKAGE, SOURCE_DIR / 'test' / 'packages', module_dir)
    print(f'Checking installed package files for {AMENT_PYTHON_TEST_PACKAGE}')
    test_dircmp = filecmp.dircmp(
        SOURCE_DIR / 'test' / 'packages' / AMENT_PYTHON_TEST_PACKAGE / AMENT_PYTHON_TEST_PACKAGE,
        module_dir / 'install' / AMENT_PYTHON_TEST_PACKAGE
                   / PYTHON_INSTALL_DIR / AMENT_PYTHON_TEST_PACKAGE
    )
    assert not test_dircmp.left_only, \
        f'Files only in source package: {test_dircmp.left_only}'
    assert not test_dircmp.right_only, \
        f'Files only in installed package: {test_dircmp.right_only}'
    assert not test_dircmp.diff_files, \
        f'Two python packages should match after install: {test_dircmp.diff_files}'


def test_ament_python_test_package_with_overlay(module_dir) -> None:
    """Test installing two python packages in the same install space."""
    compare_dir = module_dir / 'compare'
    do_build_package(AMENT_PYTHON_TEST_PACKAGE_OVERLAY,
                     SOURCE_DIR / 'test' / 'packages', module_dir)

    INSTALL_DIR = module_dir / 'install' / AMENT_PYTHON_TEST_PACKAGE_OVERLAY / PYTHON_INSTALL_DIR
    shutil.copytree(SOURCE_DIR / 'test' / 'packages' / AMENT_PYTHON_TEST_PACKAGE /
                    AMENT_PYTHON_TEST_PACKAGE, compare_dir, dirs_exist_ok=True)
    shutil.copytree(SOURCE_DIR / 'test' / 'packages' / AMENT_PYTHON_TEST_PACKAGE_OVERLAY /
                    AMENT_PYTHON_TEST_PACKAGE_OVERLAY, compare_dir, dirs_exist_ok=True)
    test_dircmp = filecmp.dircmp(compare_dir, INSTALL_DIR / AMENT_PYTHON_TEST_PACKAGE_OVERLAY)
    assert not test_dircmp.left_only, \
        'Files only in source package overlay'
    assert not test_dircmp.right_only, \
        'Files only in installed package overlay'
    assert not test_dircmp.diff_files, \
        'Two overlaid python packages should match after install'


def test_python_double_version(module_dir) -> None:
    """Test installing two versions of the same package with different names."""
    package_name = 'python_package_double_version'
    do_build_package(package_name, SOURCE_DIR / 'test' / 'packages', module_dir)

    # This package installs two versions of a package with different names and version numbers.
    install_base = module_dir / 'install' / package_name / PYTHON_INSTALL_DIR
    for additional_name in [package_name, 'some_other_name']:
        install_path = install_base / additional_name
        print(f'install_path for package {additional_name}: {install_path}')
        assert install_path.exists(), \
            f'install path should exist for {package_name}: {install_path}'
        assert (install_path / '__init__.py').exists(), \
            f'missing __init__.py in {install_path}'
        print(f'Testing version IN EGG-INFO in package {package_name}')
        version = '1.2.34' if additional_name == package_name else '5.6.78'
        egg_info_dir = module_dir / 'install' / package_name / PYTHON_INSTALL_DIR / \
            f'{additional_name}-{version}-{PYEGG_VERSION}.egg-info'
        assert egg_info_dir.exists(), \
            f'egg-info dir should exist for {package_name}: {egg_info_dir}'
        egg_info_file = egg_info_dir / 'PKG-INFO'
        assert Path.read_text(egg_info_file).find(f'Version: {version}') != -1, \
            f'egg-info file should contain "Version: {version}"'
'''
