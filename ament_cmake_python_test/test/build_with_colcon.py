"""
Test building python packages with colcon
"""

import os
from pathlib import Path
import shutil
import sys
import subprocess

from jinja2 import Template

PWD = Path(os.environ.get('PWD'))
SOURCE_DIR = Path(os.environ.get('SOURCE_DIR'))
PYTHON_INSTALL_DIR = Path(os.environ.get('PYTHON_INSTALL_DIR'))
PYEGG_VERSION = os.environ.get('PYEGG_VERSION')

DEFAULT_OPTIONS = {
  'name': 'SET_ME',
  'version': None,
  'description': 'SET_ME',
  'setup_cfg': None,
  'destination': None,
  'symlink_install': False,
  'package_subdir': None,
  'has_python': True,
  'has_python_before': False, # This will only make sense when both python and msg are in the same package
  'has_msg': False,
  'scripts_destination': None
}

TESTS_OPTIONS = [
  {
    'name': 'python_package',
    'description': 'Package with python code',
  },
  {
    'name': 'msg_package',
    'description': 'Package with only msg files',
    'has_msg': True,
    'has_python': False,
  },
  {
    'name': 'python_package_symlink',
    'description': 'Package with python code, installed with symlink in build',
    'symlink_install': True,
  },
  {
    'name': 'python_package_rename',
    'description': 'Package with python code, installed from alternate directory name',
    'package_subdir': 'renamed_dir',
  },
  {
    'name': 'python_package_version',
    'description': 'Package with python code, specifying version in ament_python_install_package',
    'version': '6.7.89',
  },
  {
    'name': 'python_package_setup',
    'description': 'Package with python code, using setup.cfg for metadata',
    'setup_cfg': Path('config') / Path('setup.cfg'),
  },
  {
    'name': 'python_package_destination',
    'description': 'Package with python code, installed to alternate destination',
    'destination': 'new_destination',
  },
  {
    'name': 'python_package_with_scripts',
    'description': 'Package with python code',
    'scripts_destination': 'lib/python_package_with_scripts',
  },
]


def test_from_template():
  print(f"PWD: {PWD}")

  # delete any existing package directory
  packages_dir = PWD / 'packages'
  shutil.rmtree(packages_dir, ignore_errors=True)

  # extend the tests to include combinations of msg and python
  additional_options = TESTS_OPTIONS.copy()
  for options in TESTS_OPTIONS:
    if options['name'].startswith('python_package'):
      msg_options = options.copy()
      msg_options['name'] += '_with_msg'
      msg_options['description'] += 'and msg files'
      msg_options['has_msg'] = True
      additional_options.append(msg_options)
    elif options['name'].startswith('msg_package'):
      py_options = options.copy()
      py_options['name'] += '_with_python_before'
      py_options['description'] += ' and python code before msg'
      py_options['has_python_before'] = True
      additional_options.append(py_options)

  template_dir = SOURCE_DIR / 'test' / 'pkg_template'
  # Create test packages from template
  for options in additional_options:
    options = DEFAULT_OPTIONS | options
    print(f"Generating package {options['name']}")
    print(f"  options: {options}")
    package_dir = packages_dir / options['name']
    shutil.rmtree(package_dir, ignore_errors=True)
    package_subdir = options['package_subdir'] or options['name']

    package_dir.mkdir(parents=True)

    install_options = ''
    if options['has_msg']:
      shutil.copytree(template_dir / 'msg', package_dir / 'msg')

    if options['has_python'] or options['has_python_before']:
      ignore_patterns = shutil.ignore_patterns('*.jinja')
      shutil.copytree(template_dir / 'package_directory', package_dir / package_subdir, ignore=ignore_patterns)
      template = Template(Path.read_text(template_dir / 'package_directory' / '__init__.py.jinja'))
      Path.write_text(package_dir / package_subdir / '__init__.py', template.render(options))

    if options['version']:
      install_options += f' VERSION {options["version"]}'

    if options['setup_cfg']:
      (package_dir / options['setup_cfg']).parent.mkdir(parents=True, exist_ok=True)
      shutil.copy(template_dir / options['setup_cfg'], package_dir / options['setup_cfg'].parent)
      install_options += f' SETUP_CFG {options["setup_cfg"]}'

    if options['scripts_destination']:
      scripts_dir = template_dir / 'python_scripts'
      shutil.copytree(scripts_dir, package_dir / package_subdir, dirs_exist_ok=True)
      template = Template(Path.read_text(template_dir / 'setup.cfg.jinja'))
      Path.write_text(package_dir / 'setup.cfg', template.render(options))
      install_options += f' SCRIPTS_DESTINATION {options["scripts_destination"]}'

    if options['destination']:
      install_options += f' DESTINATION {options["destination"]}'

    if options['package_subdir']:
      install_options += f' PACKAGE_DIR {options["package_subdir"]}'

    options['install_options'] = install_options
    template = Template(Path.read_text(template_dir / 'package.xml.jinja'))
    Path.write_text(package_dir / 'package.xml', template.render(options))
    template = Template(Path.read_text(template_dir / 'CMakeLists.txt.jinja'))
    Path.write_text(package_dir / 'CMakeLists.txt', template.render(options))

    do_build_package(options['name'], options, base_prefix=PWD)
    do_test_package(options['name'], options)


def do_build_package(package_name, options=None, base_prefix=SOURCE_DIR / 'test'):
  if options and 'build' in options:
    build_options = options['build']
  elif options and 'symlink_install' in options and options['symlink_install']:
    build_options = '--symlink-install'
  else:
    build_options = None
  
  print(f"Building package {package_name} with colcon options: {build_options}")
  build_command = ['colcon', 'build',
    '--base-paths', base_prefix / 'packages' / package_name]
  if build_options:
    build_command.append(build_options)
  result = subprocess.run(build_command, capture_output=True, text=True)

  print("\nCOLCON stdout:\n\n" + result.stdout + '\n---(end stdout)', file=sys.stdout)
  print("\nCOLCON stderr:\n\n" + result.stderr + '\n---(end stderr)', file=sys.stderr)


def do_test_package(package_name, options):
  if options['destination']:
    install_path = PWD / 'install' / package_name / options['destination'] / package_name
  else:
    install_path = PWD / 'install' / package_name / PYTHON_INSTALL_DIR / package_name
  print(f"install_path for package {package_name}: {install_path}")
  assert install_path.exists(), f"install path does not exist for {package_name}: {install_path}"
  assert (install_path / '__init__.py').exists(), f"missing __init__.py in {install_path}"

  if options['has_python'] or options['has_python_before']:
    assert Path.read_text (install_path / '__init__.py').startswith(f"# This is {package_name}"), \
          f"__init__.py should be from {package_name} python package"

  if options['has_msg']:
    assert(install_path/ 'msg').is_dir(), f"There should be a msg directory in {install_path}"

  if options['symlink_install']:
      print(f"Testing symlink install in package {package_name}")
      assert (install_path / '__init__.py').is_symlink(), "__init__.py should be a symlink"

  if options['version']:
      print(f"Testing version: {options['version']} IN EGG-INFO in package {package_name}")
      version = options['version']
      egg_info_dir = PWD / 'install' / package_name / PYTHON_INSTALL_DIR / f'{package_name}-{version}-{PYEGG_VERSION}.egg-info'
      assert egg_info_dir.exists(), f"egg-info dir does not exist for {package_name}: {egg_info_dir}"
      egg_info_file = egg_info_dir / 'PKG-INFO'
      assert Path.read_text(egg_info_file).find(f"Version: {version}") != -1, \
        f"egg-info file should contain 'Version: {version}'"

  if options['setup_cfg']:
      print(f"Testing setup.cfg metadata in package {package_name}")
      print(f"  options: {options}")
      version = options['version'] or "0.0.0"
      egg_info_dir = PWD / 'install' / package_name / PYTHON_INSTALL_DIR / f'{package_name}-{version}-{PYEGG_VERSION}.egg-info'
      assert egg_info_dir.exists(), f"egg-info dir does not exist for {package_name}: {egg_info_dir}"
      egg_info_file = egg_info_dir / 'PKG-INFO'
      assert Path.read_text(egg_info_file).find(f"Keywords: test_of_ament_cmake_python") != -1, \
        f"egg-info file should contain 'Keywords: test_of_ament_cmake_python' specified in setup.cfg"

  if options['scripts_destination']:
      print(f"Testing script installed in package {package_name}")
      script_path = PWD / 'install' / package_name / options['scripts_destination'] / 'do_something'
      assert script_path.exists(), f"script do_something does not exist for {package_name}: {script_path}"
