# Copyright 2026 Tanay Baisware
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

"""
pytest setup shared by this package's tests.

The generated lint tests (test_copyright.py, test_flake8.py, test_pep257.py) check the current
directory. colcon test runs them from the package directory; the fixture below does the same,
so ``python3 -m pytest src/warehouse_layout/test`` from the workspace root lints this package
only, not the whole workspace.
"""

import os

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def _lint_from_the_package_directory(request, monkeypatch):
    if request.node.get_closest_marker('linter'):
        monkeypatch.chdir(PACKAGE_DIR)
