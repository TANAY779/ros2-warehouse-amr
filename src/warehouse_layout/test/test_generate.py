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
The generate command, run on a copy of the source tree in a temporary folder.

It writes the three files, ``--check`` passes right after, and a hand edit or a missing file
makes ``--check`` exit 1 with the file's name.
"""

import shutil

import pytest
from warehouse_description import robot_config
from warehouse_layout import cli
from warehouse_layout import model

OUTPUTS = ('warehouse_gazebo/worlds/warehouse.sdf',
           'warehouse_layout/generated/locations.yaml',
           'warehouse_layout/generated/expected_inventory.yaml')


@pytest.fixture
def src(tmp_path):
    """Return a source tree with only the two inputs: layout.yaml and robot.yaml."""
    for package, source in (('warehouse_layout', model.default_path()),
                            ('warehouse_description', robot_config.default_path())):
        folder = tmp_path / 'src' / package / 'config'
        folder.mkdir(parents=True)
        shutil.copy(source, folder)
    return tmp_path / 'src'


def test_generate_writes_the_files_and_check_passes(src, capsys):
    assert cli.main(['--src', str(src)]) == 0
    printed = capsys.readouterr().out
    for path in OUTPUTS:
        assert (src / path).is_file()
        assert f'wrote      {src / path}' in printed
    assert '36 picks' in printed and '207 totes' in printed

    assert cli.main(['--src', str(src), '--check']) == 0
    assert 'all 3 generated files are current' in capsys.readouterr().out

    # A second run finds nothing to do and leaves the files alone.
    assert cli.main(['--src', str(src)]) == 0
    assert capsys.readouterr().out.count('unchanged') == 3


@pytest.mark.parametrize('path', OUTPUTS)
def test_check_fails_after_a_hand_edit(src, capsys, path):
    assert cli.main(['--src', str(src)]) == 0
    with open(src / path, 'a') as stream:
        stream.write('\n')     # even one extra newline
    capsys.readouterr()
    assert cli.main(['--src', str(src), '--check']) == 1
    error = capsys.readouterr().err
    assert f'stale: {src / path}' in error
    assert '1 of 3 generated files are out of date' in error


def test_check_fails_when_a_file_is_missing(src, capsys):
    assert cli.main(['--src', str(src), '--check']) == 1
    assert capsys.readouterr().err.count('missing: ') == 3


def test_a_missing_source_tree_exits_2(tmp_path, capsys):
    assert cli.main(['--src', str(tmp_path / 'nowhere')]) == 2
    assert 'Run this from the workspace root' in capsys.readouterr().err


def test_a_schema_error_exits_2_and_writes_nothing(src, capsys):
    layout = src / 'warehouse_layout' / 'config' / 'layout.yaml'
    line = '  wall_height: 2.50\n'
    text = layout.read_text()
    assert line in text
    layout.write_text(text.replace(line, line + '  wall_hieght: 2.50\n'))   # a typo
    assert cli.main(['--src', str(src)]) == 2
    assert 'hall.wall_hieght: unknown key' in capsys.readouterr().err
    assert not any((src / path).exists() for path in OUTPUTS)
