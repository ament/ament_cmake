# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import os
import unittest

from ament_cmake_python import find_packages_data


class TestFindPackagesData(unittest.TestCase):

    def test_all_packages_data_is_found(self):
        data = find_packages_data()
        assert set(data) == {'foo', 'foo.bar', 'baz'}
        assert set(data['foo']) == {'data', 'data.txt'}
        assert set(data['foo.bar']) == {
            'data.txt',
            os.path.join('resources', 'fizz.txt'),
            os.path.join('resources', 'buzz.txt')
        }
        assert set(data['baz']) == {'data.bin', 'data'}

    def test_whole_package_data_is_included(self):
        data = find_packages_data(
            include=('foo', 'foo.*'))
        assert set(data) == {'foo', 'foo.bar'}
        assert set(data['foo']) == {'data', 'data.txt'}
        assert set(data['foo.bar']) == {
            'data.txt',
            os.path.join('resources', 'fizz.txt'),
            os.path.join('resources', 'buzz.txt')
        }

    def test_whole_package_data_is_excluded(self):
        data = find_packages_data(
            include=('foo', 'foo.*'),
            exclude=('foo.bar',))
        assert set(data) == {'foo'}
        assert set(data['foo']) == {'data', 'data.txt'}

    def test_partial_package_data_is_excluded(self):
        data = find_packages_data(
            include=('foo', 'foo.*'),
            exclude={'foo.bar': ['resources/*']})
        assert set(data) == {'foo', 'foo.bar'}
        assert set(data['foo']) == {'data', 'data.txt'}
        assert set(data['foo.bar']) == {'data.txt'}

    def test_partial_package_data_is_included(self):
        data = find_packages_data(
            include={
                'foo': ['*.txt'],
                'foo.*': ['resources/*.txt']
            },
        )
        assert set(data) == {'foo', 'foo.bar'}
        assert set(data['foo']) == {'data.txt'}
        assert set(data['foo.bar']) == {
            os.path.join('resources', 'fizz.txt'),
            os.path.join('resources', 'buzz.txt')
        }


if __name__ == '__main__':
    unittest.main()
