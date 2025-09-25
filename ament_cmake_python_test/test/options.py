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

"""Options used to generate test packages from a template."""

# Set True to generate both a python package and a msg package for each python package
# This is useful to test that python and msg packages can coexist in the same package
# See issue #514 and PR #587
COMBINE_PYTHON_WITH_MSG = False

# Set True to test ordering sensitivity of python and msg generation in CMakeLists.txt
TEST_ORDERING_SENSITIVITY = False

DEFAULT_OPTIONS = {
    'name': 'SET_ME',
    'version': None,
    'description': 'SET_ME',
    'setup_cfg': None,
    'destination': None,
    'symlink_install': False,
    'python_subdir': None,
    'has_python': True,
    'has_python_before': False,  # if both python and msg are in the same package
    'has_msg': False,
    'scripts_destination': None,
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
        'python_subdir': 'renamed_dir',
    },
    {
        'name': 'python_package_version',
        'description': 'Package with python code, specifying version',
        'version': '6.7.89',
    },
    {
        'name': 'python_package_setup',
        'description': 'Package with python code, using setup.cfg for metadata',
        'setup_cfg': 'config/setup.cfg',
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


def get_options():
    """Return a list of options dictionaries for generating test packages."""
    tests_options = []
    for options in TESTS_OPTIONS:
        options = DEFAULT_OPTIONS | options
        tests_options.append(options)

        if COMBINE_PYTHON_WITH_MSG:
            if options['name'].startswith('python_package'):
                msg_options = options.copy()
                msg_options['name'] += '_with_msg'
                msg_options['description'] += 'and msg files'
                msg_options['has_msg'] = True
                tests_options.append(msg_options)

            # This only makes sense when both python and msg are in the same package
            if TEST_ORDERING_SENSITIVITY:
                if options['name'].startswith('msg_package'):
                    py_options = options.copy()
                    py_options['name'] += '_with_python_before'
                    py_options['description'] += ' and python code before msg'
                    py_options['has_python_before'] = True
                    tests_options.append(py_options)

    return tests_options
