#!/usr/bin/env python3

# Apache License 2.0
#
# Copyright (c) 2014, Open Source Robotics Foundation, Inc.

from __future__ import print_function
import argparse
import sys

from ament_package.templates import TEMPLATE_DIRECTORY
from ament_package.templates import get_environment_hook_template_path
from ament_package.templates import get_package_level_template_names
from ament_package.templates import get_package_level_template_path
from ament_package.templates import get_prefix_level_template_names
from ament_package.templates import get_prefix_level_template_path


def main(argv=sys.argv[1:]):
    """
    Extract the information about templates provided by ament_package.

    Call the API provided by ament_package and
    print CMake code defining several variables containing information about
    the available templates.
    """
    parser = argparse.ArgumentParser(
        description='Extract information about templates provided by '
                    'ament_package and print CMake code defining several '
                    'variables',
    )
    parser.add_argument(
        'outfile',
        nargs='?',
        help='The filename where the output should be written to',
    )
    args = parser.parse_args(argv)

    lines = generate_cmake_code()
    if args.outfile:
        with open(args.outfile, 'w') as f:
            for line in lines:
                f.write('%s\n' % line)
    else:
        for line in lines:
            print(line)


def generate_cmake_code():
    """
    Return a list of CMake set() commands containing the template information.

    :returns: list of str
    """
    variables = []
    variables.append(('TEMPLATE_DIR', '"%s"' % TEMPLATE_DIRECTORY))

    variables.append((
        'ENVIRONMENT_HOOK_PYTHONPATH',
        '"%s"' % get_environment_hook_template_path('pythonpath.sh.in')))

    templates = []
    for name in get_package_level_template_names():
        templates.append('"%s"' % get_package_level_template_path(name))
    variables.append((
        'PACKAGE_LEVEL',
        templates))

    templates = []
    for name in get_prefix_level_template_names():
        templates.append('"%s"' % get_prefix_level_template_path(name))
    variables.append((
        'PREFIX_LEVEL',
        templates))

    lines = []
    for (k, v) in variables:
        if isinstance(v, list):
            lines.append('set(ament_cmake_package_templates_%s "")' % k)
            for vv in v:
                lines.append('list(APPEND ament_cmake_package_templates_%s %s)'
                             % (k, vv))
        else:
            lines.append('set(ament_cmake_package_templates_%s %s)' % (k, v))
    return lines


if __name__ == '__main__':
    main()
