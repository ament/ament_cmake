#!/usr/bin/env python3

from __future__ import print_function
from __future__ import unicode_literals

import argparse
import codecs
import errno
import os
import re
import sys
import subprocess
from xml.etree.ElementTree import ElementTree
from xml.etree.ElementTree import ParseError


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Run the test command passed as an argument and ensures'
                    'that the expected result file is generated.')
    parser.add_argument(
        'result_file', help='The path to the xunit result file')
    parser.add_argument(
        '--command',
        nargs='+',
        help='The test command to execute. '
             'It must be passed after other arguments since it collects all '
             'following options.')
    parser.add_argument(
        '--output-file',
        help='The path to the output log file')
    parser.add_argument(
        '--generate-result-on-success',
        action='store_true',
        default=False,
        help='Generate a result file if the command returns with code zero')

    if '--command' in argv:
        index = argv.index('--command')
        argv, command = argv[0:index + 1] + ['dummy'], argv[index + 1:]
    args = parser.parse_args(argv)
    args.command = command

    # if result file exists remove it before test execution
    if os.path.exists(args.result_file):
        os.remove(args.result_file)

    # create folder if necessary
    if not os.path.exists(os.path.dirname(args.result_file)):
        try:
            os.makedirs(os.path.dirname(args.result_file))
        except OSError as e:
            # catch case where folder has been created in the mean time
            if e.errno != errno.EEXIST:
                raise

    # generate result file with one failed test
    # in case the command segfaults or timeouts and does not generate one
    failure_result_file = _generate_result(
        args.result_file,
        'The test did not generate a result file.'
    )
    with open(args.result_file, 'w') as f:
        f.write(failure_result_file)

    print("-- run_test.py: invoke following command in '%s':\n - %s" %
          (os.getcwd(), ' '.join(args.command)))

    if args.output_file:
        with open(args.output_file, 'w') as h:
            rc = subprocess.call(args.command, stdout=h, stderr=subprocess.STDOUT)
    else:
        rc = subprocess.call(args.command)

    print("-- run_test.py: verify result file '%s'" % args.result_file)

    if os.path.exists(args.result_file):
        # if result file exists ensure that it contains valid xml
        # unit test suites are not good about screening out
        # illegal unicode characters
        tree = None
        try:
            tree = ElementTree(None, args.result_file)
        except ParseError as e:
            modified = _tidy_xml(args.result_file)
            if not modified:
                print("Invalid XML in result file '%s': %s" %
                      (args.result_file, str(e)), file=sys.stderr)
            else:
                try:
                    tree = ElementTree(None, args.result_file)
                except ParseError as e:
                    print("Invalid XML in result file '%s' "
                          "(even after trying to tidy it): %s" %
                          (args.result_file, str(e)), file=sys.stderr)

        if not tree:
            # set error code when result file is not parsable
            rc = 1
        else:
            # set error code when result file contains errors or failures
            root = tree.getroot()
            num_errors = int(root.attrib.get('errors', 0))
            num_failures = int(root.attrib.get('failures', 0))
            if num_errors or num_failures:
                rc = 1

    elif not rc and args.generate_result_on_success:
        # generate result file with one passed test
        # if it was expected that no result file was generated
        # and the command returned with code zero
        success_result_file = _generate_result(args.result_file)
        with open(args.result_file, 'w') as f:
            f.write(success_result_file)

    # ensure that a result file exists at the end
    if not rc and not os.path.exists(args.result_file):
        print('-- run_test.py: override return code since no result file was '
              'generated', file=sys.stderr)
        rc = 1

    return rc


def _generate_result(result_file, failure_message=None):
    pkgname = os.path.basename(os.path.dirname(result_file))
    testname = os.path.splitext(os.path.basename(result_file))[0]
    name = '%s__%s' % (pkgname, testname)
    failure_message = '<failure message="%s" type=""/>' % failure_message \
        if failure_message else ''
    return '''<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="%d" time="1" errors="0" name="%s">
  <testcase name="%s" status="run" time="1" classname="Results">
    %s
  </testcase>
</testsuite>''' % (1 if failure_message else 0,
                   name,
                   name,
                   failure_message)


def _tidy_xml(filename):
    assert os.path.isfile(filename)

    # try reading utf-8 firth then iso
    # this is ugly but the files in question do not declare a unicode type
    data = None
    for encoding in ['utf-8', 'iso8859-1']:
        f = None
        try:
            f = codecs.open(filename, 'r', encoding)
            data = f.read()
            break
        except ValueError:
            continue
        finally:
            if f:
                f.close()

    if data is None:
        return False

    try:
        char = unichr
    except NameError:
        char = chr
    RE_XML_ILLEGAL = (
        '([%s-%s%s-%s%s-%s%s-%s])' +
        '|' +
        '([%s-%s][^%s-%s])|([^%s-%s][%s-%s])|([%s-%s]$)|(^[%s-%s])') % \
        (char(0x0000), char(0x0008), char(0x000b), char(0x000c),
         char(0x000e), char(0x001f), char(0xfffe), char(0xffff),
         char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
         char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
         char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff))
    SAFE_XML_REGEX = re.compile(RE_XML_ILLEGAL)

    for match in SAFE_XML_REGEX.finditer(data):
        data = data[:match.start()] + '?' + data[match.end():]

    with open(filename, 'w') as f:
        f.write(data)
    return True


if __name__ == '__main__':
    sys.exit(main())
