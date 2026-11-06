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

"""Map files: written as map_saver_cli writes them, and read as map_server reads them."""

import math

import numpy as np
from PIL import Image
import pytest
from warehouse_layout import occupancy
from warehouse_layout.occupancy import FREE
from warehouse_layout.occupancy import MapError
from warehouse_layout.occupancy import OccupancyMap
from warehouse_layout.occupancy import OCCUPIED
from warehouse_layout.occupancy import UNKNOWN
import yaml

# Three rows of four cells; row 0 is the south edge.
CELLS = np.array([[FREE, FREE, OCCUPIED, UNKNOWN],       # south
                  [OCCUPIED, FREE, FREE, FREE],
                  [UNKNOWN, UNKNOWN, OCCUPIED, FREE]],   # north
                 dtype=np.int8)

GOOD_YAML = """\
image: small.pgm
mode: trinary
resolution: 0.05
origin: [1.0, 2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""


@pytest.fixture
def small():
    return OccupancyMap(CELLS.copy(), 0.05, (1.0, 2.0, 0.0))


def test_a_map_reads_back_as_it_was_written(small, tmp_path):
    path = tmp_path / 'small.yaml'
    assert occupancy.save(small, str(path)) == str(tmp_path / 'small.pgm')
    loaded = occupancy.load(str(path))
    assert np.array_equal(loaded.cells, small.cells)
    assert loaded.resolution == 0.05
    assert loaded.origin == (1.0, 2.0, 0.0)


def test_the_image_is_a_pgm_with_the_north_row_first(small, tmp_path):
    occupancy.save(small, str(tmp_path / 'small.yaml'))
    data = (tmp_path / 'small.pgm').read_bytes()
    header = b'P5\n4 3\n255\n'
    assert data[:len(header)] == header
    rows = np.frombuffer(data[len(header):], dtype=np.uint8).reshape(3, 4)
    assert rows.tolist() == [[205, 205, 0, 254],      # north row: the image's top
                             [0, 254, 254, 254],
                             [254, 254, 0, 205]]      # south row: the image's bottom


def test_the_yaml_has_map_saver_keys_and_thresholds(small, tmp_path):
    path = tmp_path / 'small.yaml'
    occupancy.save(small, str(path), comment=('made by a test',))
    text = path.read_text()
    assert text.startswith('# made by a test\n')
    data = yaml.safe_load(text)
    assert list(data) == ['image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh',
                          'free_thresh']     # map_saver_cli's order
    assert data == {'image': 'small.pgm', 'mode': 'trinary', 'resolution': 0.05,
                    'origin': [1.0, 2.0, 0.0], 'negate': 0, 'occupied_thresh': 0.65,
                    'free_thresh': 0.196}


def test_unknown_pixels_read_as_free_with_a_free_threshold_of_025(small, tmp_path):
    # The trap that free_thresh 0.196 avoids: pixel 205 is an occupancy of 50 / 255 = 0.196,
    # below 0.25, so map_server would read the unknown cells as free.
    occupancy.save(small, str(tmp_path / 'small.yaml'))
    (tmp_path / 'small.yaml').write_text(GOOD_YAML.replace('free_thresh: 0.196',
                                                           'free_thresh: 0.25'))
    loaded = occupancy.load(str(tmp_path / 'small.yaml'))
    assert np.count_nonzero(loaded.cells == UNKNOWN) == 0
    assert np.array_equal(loaded.cells == OCCUPIED, CELLS == OCCUPIED)


def test_negate_makes_white_occupied(tmp_path):
    # Black (0) and white (255) pixels, top row first; with negate 1, white is occupied.
    Image.fromarray(np.array([[0, 255], [255, 0]], dtype=np.uint8)).save(tmp_path / 'n.pgm')
    (tmp_path / 'n.yaml').write_text(GOOD_YAML.replace('small.pgm', 'n.pgm')
                                     .replace('negate: 0', 'negate: 1'))
    loaded = occupancy.load(str(tmp_path / 'n.yaml'))
    assert loaded.cells.tolist() == [[OCCUPIED, FREE], [FREE, OCCUPIED]]


def test_an_rgb_image_counts_the_mean_of_its_channels(tmp_path):
    # (255, 255, 0) averages to 170 (occupancy 0.33: unknown); (0, 0, 60) to 20 (occupied).
    pixels = np.array([[[255, 255, 0], [0, 0, 60]]], dtype=np.uint8)
    Image.fromarray(pixels).save(tmp_path / 'rgb.png')
    (tmp_path / 'rgb.yaml').write_text(GOOD_YAML.replace('small.pgm', 'rgb.png'))
    assert occupancy.load(str(tmp_path / 'rgb.yaml')).cells.tolist() == [[UNKNOWN, OCCUPIED]]


def test_cell_centres_and_indices_follow_the_origin(small):
    centres = small.centres(OCCUPIED)
    assert np.allclose(centres, [(1.125, 2.025), (1.025, 2.075), (1.125, 2.125)])
    rows, columns = small.cell_index(centres)
    assert np.array_equal(small.cells[rows, columns], [OCCUPIED] * 3)
    assert small.value_at(1.18, 2.01) == UNKNOWN
    with pytest.raises(IndexError):
        small.value_at(0.0, 0.0)


def test_the_origin_yaw_turns_the_map_about_its_corner():
    turned = OccupancyMap(CELLS.copy(), 0.05, (1.0, 2.0, math.pi / 2))
    # Cell [0, 2] is 0.125 m along the map's x axis and 0.025 m along its y axis; turned by
    # 90 degrees, its x axis points north and its y axis west.
    assert np.allclose(turned.centres(OCCUPIED)[0], (1.0 - 0.025, 2.0 + 0.125))
    assert turned.value_at(1.0 - 0.025, 2.0 + 0.125) == OCCUPIED


@pytest.mark.parametrize('text, message', [
    (GOOD_YAML.replace('resolution: 0.05\n', ''), 'missing key resolution'),
    ('- just\n- a list\n', 'expected the keys of a map file'),
    ('image: [small.pgm\n', 'not valid YAML'),
    (GOOD_YAML.replace('[1.0, 2.0, 0.0]', '[1.0, 2.0]'), 'origin must be [x, y, yaw]'),
    (GOOD_YAML.replace('resolution: 0.05', 'resolution: 0'), 'resolution must be a number'),
    (GOOD_YAML.replace('negate: 0', 'negate: 2'), 'negate must be 0 or 1'),
    (GOOD_YAML.replace('free_thresh: 0.196', 'free_thresh: 0.7'), 'must be below'),
    (GOOD_YAML.replace('mode: trinary', 'mode: raw'), "mode 'raw' is not supported"),
    (GOOD_YAML.replace('small.pgm', 'missing.pgm'), 'does not exist'),
    (GOOD_YAML.replace('small.pgm', 'small.yaml'), 'cannot read its image'),
])
def test_a_malformed_map_file_names_the_problem(small, tmp_path, text, message):
    occupancy.save(small, str(tmp_path / 'small.yaml'))     # the image the YAML may name
    (tmp_path / 'small.yaml').write_text(text)
    with pytest.raises(MapError) as error:
        occupancy.load(str(tmp_path / 'small.yaml'))
    assert message in str(error.value)
    assert str(tmp_path / 'small.yaml') in str(error.value)


def test_a_missing_map_file_names_it(tmp_path):
    with pytest.raises(MapError, match='No such file'):
        occupancy.load(str(tmp_path / 'nothing.yaml'))
