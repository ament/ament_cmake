#!/usr/bin/env python3

# Apache License 2.0
#
# Copyright (c) 2014, Open Source Robotics Foundation

from __future__ import print_function
import argparse
import sys

try:
    from ament_package import parse_package_string
except ImportError as e:
    sys.exit("ImportError: 'from ament_package import parse_package_string' failed: %s\nMake sure that you have installed 'ament_package', it is up to date and on the PYTHONPATH." % e)


def main(argv=sys.argv[1:]):
    """
    Parses the given package.xml file and
    prints CMake code defining several variables containing the content.
    """
    parser = argparse.ArgumentParser(
        description='Parse package.xml file and print CMake code defining several variables',
    )
    parser.add_argument(
        'package_xml',
        type=argparse.FileType('r'),
        help='The path to a package.xml file',
    )
    parser.add_argument(
        'outfile',
        nargs='?',
        help='The filename where the output should be written to',
    )
    args = parser.parse_args(argv)

    try:
        package = parse_package_string(args.package_xml.read())
    except Exception as e:
        print("Error parsing '%s':" % args.package_xml.name, file=sys.stderr)
        raise e

    lines = generate_cmake_code(package)
    if args.outfile:
        with open(args.outfile, 'w') as f:
            for line in lines:
                f.write('%s\n' % line)
    else:
        for line in lines:
            print(line)


def get_dependency_values(key, depends):
    dependencies = []
    dependencies.append((key, ' '.join(['"%s"' % str(d) for d in depends])))
    for d in depends:
        comparisons = ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']
        for comp in comparisons:
            value = getattr(d, comp, None)
            if value is not None:
                dependencies.append(('%s_%s_%s' % (key, str(d), comp.upper()), '"%s"' % value))
    return dependencies


def generate_cmake_code(package):
    """
    Returns a list of CMake commands
    defining variables containing the content of the passed package.

    :param package: ament_package.Package
    :returns: list of str
    """
    variables = []
    variables.append(('VERSION', '"%s"' % package.version))

    variables.append(('MAINTAINER',  '"%s"' % (', '.join([str(m) for m in package.maintainers]))))

    variables.extend(get_dependency_values('BUILD_DEPENDS', package.build_depends))
    variables.extend(get_dependency_values('BUILDTOOL_DEPENDS', package.buildtool_depends))
    variables.extend(get_dependency_values('BUILD_EXPORT_DEPENDS', package.build_export_depends))
    variables.extend(get_dependency_values('BUILDTOOL_EXPORT_DEPENDS', package.buildtool_export_depends))
    variables.extend(get_dependency_values('EXEC_DEPENDS', package.exec_depends))
    variables.extend(get_dependency_values('TEST_DEPENDS', package.test_depends))

    deprecated = [e.content for e in package.exports if e.tagname == 'deprecated']
    variables.append(('DEPRECATED', '"%s"' % ((deprecated[0] if deprecated[0] else 'TRUE') if deprecated else '')))

    lines = []
    lines.append('set(_AMENT_PACKAGE_NAME "%s")' % package.name)
    for (k, v) in variables:
        lines.append('set(%s_%s %s)' % (package.name, k, v))
    return lines


if __name__ == '__main__':
    main()
