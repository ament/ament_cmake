# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import argparse
import os
import subprocess
import sys


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Run a Google Benchmark test and convert the results to '
                    'a common format.')
    parser.add_argument(
        'result_file_in', help='The path to the Google Benchmark result file')
    parser.add_argument(
        'result_file_out',
        help='The path to where the common result file should be written')
    parser.add_argument(
        '--command',
        nargs='+',
        help='The test command to execute. '
             'It must be passed after other arguments since it collects all '
             'following options.')
    if '--command' in argv:
        index = argv.index('--command')
        argv, command = argv[0:index + 1] + ['dummy'], argv[index + 1:]
    args = parser.parse_args(argv)
    args.command = command

    if os.environ.get('AMENT_RUN_PERFORMANCE_TESTS') not in (
        '1', 'true', 'True', 'TRUE', 'yes', 'Yes', 'YES', 'y', 'Y'
    ):
        return 127

    res = subprocess.run(args.command)

    with open(args.result_file_in, 'r') as in_file:
        with open(args.result_file_out, 'w') as out_file:
            # TODO(cottsay): Convert the results file
            pass

    return res.returncode if res.returncode != 127 else 1
