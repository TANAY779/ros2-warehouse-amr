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

"""The preview command: it draws the layout, prints the rule results and sets its exit code."""

import copy
import struct

import pytest
from warehouse_layout import model
from warehouse_layout import preview
import yaml

PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


@pytest.fixture(scope='module')
def layout_yaml():
    with open(model.default_path()) as stream:
        return yaml.safe_load(stream)


def png_size(path):
    """Return (width, height) of a PNG file, read from its header."""
    with open(path, 'rb') as stream:
        header = stream.read(24)
    assert header[:8] == PNG_SIGNATURE
    return struct.unpack('>II', header[16:24])


def test_preview_draws_the_installed_layout(tmp_path, capsys):
    out = tmp_path / 'layout.png'
    assert preview.main(['--out', str(out)]) == 0
    assert png_size(out) == (2090, 1276)       # 19 x 11.6 inches at 110 dpi
    printed = capsys.readouterr().out
    assert printed.count('[PASS]') == 9
    assert '9 of 9 fit rules pass' in printed


def test_preview_exits_1_when_a_rule_fails(layout_yaml, tmp_path, capsys):
    data = copy.deepcopy(layout_yaml)
    data['inventory']['fill_ratio'] = 0.58          # 240 totes: more than the tag IDs
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(data))
    out = tmp_path / 'layout.png'
    assert preview.main(['--out', str(out), '--layout', str(layout)]) == 1
    assert png_size(out) == (2090, 1276)       # drawn anyway, to show what is wrong
    assert '[FAIL] at most 230 totes' in capsys.readouterr().out


def test_preview_exits_2_on_a_schema_error(layout_yaml, tmp_path, capsys):
    data = copy.deepcopy(layout_yaml)
    data['hall']['wall_thikness'] = 0.2
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(data))
    out = tmp_path / 'layout.png'
    assert preview.main(['--out', str(out), '--layout', str(layout)]) == 2
    assert not out.exists()
    assert 'hall.wall_thikness: unknown key' in capsys.readouterr().err
