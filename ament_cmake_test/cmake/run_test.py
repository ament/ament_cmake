#!/usr/bin/env python3

# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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
from xml.sax.saxutils import quoteattr


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
    with open(args.result_file, 'w') as h:
        h.write(failure_result_file)

    # collect output / exception to generate more detailed result file
    # if the command fails to generate it
    output = ''
    output_handle = None
    if args.output_file:
        output_path = os.path.dirname(args.output_file)
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        output_handle = open(args.output_file, 'wb')

    def log(msg, **kwargs):
        print(msg, **kwargs)
        if output_handle:
            output_handle.write((msg + '\n').encode())
            output_handle.flush()

    log("-- run_test.py: invoking following command in '%s':\n - %s" %
        (os.getcwd(), ' '.join(args.command)))
    if output_handle:
        output_handle.write('\n'.encode())
        output_handle.flush()

    try:
        proc = subprocess.Popen(args.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while True:
            line = proc.stdout.readline()
            if not line:
                break
            decoded_line = line.decode()
            print(decoded_line, end='')
            output += decoded_line
            if output_handle:
                output_handle.write(line)
                output_handle.flush()
        proc.wait()
        rc = proc.returncode
        if output_handle:
            # separate progress of this script from subprocess output
            output_handle.write('\n\n'.encode())
        log('-- run_test.py: return code ' + str(rc), file=sys.stderr if rc else sys.stdout)
    except Exception as e:
        if output_handle:
            # separate subprocess output from progress of this script
            output_handle.write('\n\n'.encode())
        log('-- run_test.py: invocation failed: ' + str(e), file=sys.stderr)
        output += str(e)
        rc = 1

    if not rc and args.generate_result_on_success:
        # generate result file with one passed test
        # if it was expected that no result file was generated
        # and the command returned with code zero
        log("-- run_test.py: generate result file '%s' with successful test" % args.result_file)
        success_result_file = _generate_result(args.result_file)
        with open(args.result_file, 'w') as h:
            h.write(success_result_file)

    elif os.path.exists(args.result_file):
        # check if content of result file has actually changed
        with open(args.result_file, 'r') as h:
            not_changed = h.read() == failure_result_file

        if not_changed:
            log("-- run_test.py: generate result file '%s' with failed test" % args.result_file,
                file=sys.stderr)
            # regenerate result file to include output / exception of the invoked command
            failure_result_file = _generate_result(
                args.result_file,
                'The test did not generate a result file:\n\n' + output
            )
            with open(args.result_file, 'w') as h:
                h.write(failure_result_file)

        else:
            log("-- run_test.py: verify result file '%s'" % args.result_file)
            # if result file exists ensure that it contains valid xml
            # unit test suites are not good about screening out
            # illegal unicode characters
            tree = None
            try:
                tree = ElementTree(None, args.result_file)
            except ParseError as e:
                modified = _tidy_xml(args.result_file)
                if not modified:
                    log("Invalid XML in result file '%s': %s" %
                        (args.result_file, str(e)), file=sys.stderr)
                else:
                    try:
                        tree = ElementTree(None, args.result_file)
                    except ParseError as e:
                        log("Invalid XML in result file '%s' (even after trying to tidy it): %s" %
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

    # ensure that a result file exists at the end
    if not rc and not os.path.exists(args.result_file):
        log('-- run_test.py: override return code since no result file was '
            'generated', file=sys.stderr)
        rc = 1

    return rc


def _generate_result(result_file, failure_message=None):
    pkgname = os.path.basename(os.path.dirname(result_file))
    testname = os.path.splitext(os.path.basename(result_file))[0]
    name = '%s.%s' % (pkgname, testname)
    failure_message = '<failure message=%s/>' % quoteattr(failure_message) \
        if failure_message else ''
    return '''<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="%d" time="1" errors="0" name="%s">
  <testcase name="missing_result" status="run" time="1" classname="%s">
    %s
  </testcase>
</testsuite>\n''' % \
        (1 if failure_message else 0, name, name, failure_message)


def _tidy_xml(filename):
    assert os.path.isfile(filename)

    # try reading utf-8 first then iso
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

    with open(filename, 'w') as h:
        h.write(data)
    return True


if __name__ == '__main__':
    sys.exit(main())
