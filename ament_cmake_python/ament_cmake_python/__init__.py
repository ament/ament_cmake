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

import fnmatch
import pathlib
from urllib.parse import urlparse

from setuptools import find_packages


def fuzzy_lookup(key, mapping):
    """Lookup key in a mapping where keys may contain wildcards ('*')."""
    for pattern, value in mapping.items():
        if fnmatch.fnmatch(key, pattern):
            yield value


def find_packages_data(where='.', exclude=(), include=('*',)):
    """
    Find data in Python packages found within directory 'where'.

    Similar to `setuptools.find_packages`.

    :param where: a cross-platform (i.e. URL-style) path
    :param exclude: a dictionary that maps from package names to
      lists of glob patterns to be excluded from the search. Wildcards
      ('*') may be used in package names. A collection of package names
      may be provided instead of a dictionary, in which case whole packages
      will be excluded.
    :param include: a dictionary that maps from package names to
      lists of glob patterns to be included in the search. Wildcards
      ('*') may be used in package names. A collection of package names
      may be provided instead of a dictionary, in which case whole packages
      will be included but excluding Python sources and byte-compiled code.
    :returns: a dictionary suitable to be used as 'package_data' when calling
      `setuptools.setup`
    """
    packages = find_packages(
        where=where, include=set(include),
        # Defer package exclusion (may be partial)
    )
    where = pathlib.Path(urlparse(where).path)
    if not isinstance(exclude, dict):
        # Exclude whole packages
        exclude = {name: ['**/*'] for name in exclude}
    if not isinstance(include, dict):
        # Include whole packages
        include = {name: ['**/*'] for name in include}
        # But
        for name in include:
            # Exclude Python sources and byte-compiled code
            if name not in exclude:
                exclude[name] = []
            exclude[name].extend([
                '**/*.py', '**/*.pyc',
                '**/__pycache__/**/*'
            ])

    packages_data = {}
    processed_data = set()
    # Bottom-up search for packages' data
    for name in sorted(packages, reverse=True):
        rootpath = where / name.replace('.', '/')

        # Exclude nested packages' content too
        excluded_data = set(processed_data)
        for patterns in fuzzy_lookup(name, exclude):
            excluded_data.update(
                path for pattern in patterns
                for path in rootpath.glob(pattern)
            )

        included_data = set()
        for patterns in fuzzy_lookup(name, include):
            included_data.update(
                path for pattern in patterns
                for path in rootpath.glob(pattern)
                if not path.is_dir() and path not in excluded_data
            )

        if included_data:
            packages_data[name] = [
                str(path.relative_to(rootpath))
                for path in included_data
            ]

        # Keep track of packages processed
        processed_data.update(rootpath.glob('**/*'))
        processed_data.add(rootpath)

    return packages_data
