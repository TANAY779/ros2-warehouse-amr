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
The generated files are committed (plan P2-D1), so check that they are current and stable.

- Current: regenerating from the installed layout.yaml and robot.yaml gives the installed
  world, registry and inventory byte for byte. After editing layout.yaml (or robot.yaml), run
  ``ros2 run warehouse_layout generate`` from the workspace root and rebuild.
- Deterministic: generating again, in this process or in fresh ones, gives the same bytes. A
  fresh Python process orders a set of strings differently (hash randomisation), so two
  processes with different PYTHONHASHSEED values catch output that depends on such an order.

With ``colcon build --symlink-install`` the installed files are links to the source tree, so
this checks the files in src/ directly.
"""

import hashlib
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
import pytest
from warehouse_layout import cli
from warehouse_layout import model

# Prints a digest of every generated file, made from the installed layout.
DIGEST = """
import hashlib
from warehouse_layout import cli, model
print(hashlib.sha256(''.join(o.text for o in cli.outputs(model.load())).encode()).hexdigest())
"""


@pytest.fixture(scope='module')
def outputs():
    return cli.outputs(model.load())


def test_generated_files_are_current(outputs):
    stale = []
    for output in outputs:
        path = os.path.join(get_package_share_directory(output.package), output.path)
        try:
            with open(path, 'rb') as stream:
                current = stream.read() == output.text.encode('utf-8')
        except FileNotFoundError:
            current = False
        if not current:
            stale.append(path)
    assert not stale, ('out of date: ' + ', '.join(stale) + '. Run `ros2 run warehouse_layout '
                       'generate` from the workspace root, then rebuild.')


def test_generating_twice_gives_the_same_bytes(outputs):
    assert cli.outputs(model.load()) == outputs


def test_output_does_not_depend_on_the_process(outputs):
    expected = hashlib.sha256(''.join(o.text for o in outputs).encode()).hexdigest()
    for seed in ('1', '2'):
        result = subprocess.run([sys.executable, '-c', DIGEST], capture_output=True, text=True,
                                env=dict(os.environ, PYTHONHASHSEED=seed), check=False)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == expected, f'PYTHONHASHSEED={seed}'
