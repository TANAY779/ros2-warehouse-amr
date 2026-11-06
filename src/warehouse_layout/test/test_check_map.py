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
check_map on maps with known faults, made from the rasterized layout.

The rasterized layout is the map an ideal SLAM run would make, so it must pass with no offset.
Each fault (a shift, a turn, ghost walls, a turned origin) must be measured as made and fail
its criterion, and only that one.
"""

import dataclasses
import math
import struct

import numpy as np
import pytest
from warehouse_layout import check_map
from warehouse_layout import model
from warehouse_layout import occupancy
from warehouse_layout import rasterize
from warehouse_layout.occupancy import FREE
from warehouse_layout.occupancy import OccupancyMap
from warehouse_layout.occupancy import OCCUPIED

CRITERIA = ('offset', 'rotation', 'near_faces', 'ghost_walls', 'origin_yaw')


@pytest.fixture(scope='module')
def layout():
    return model.load()


@pytest.fixture(scope='module')
def raster(layout):
    return rasterize.rasterize(layout)


@pytest.fixture(scope='module')
def faces(layout):
    return rasterize.surfaces(layout)


def failed(map_check):
    """Return the names of the criteria that fail."""
    return [result.rule for result in map_check.results if not result.passed]


def with_occupied(raster, points):
    """Return a copy of the map whose occupied cells are the cells holding these points."""
    cells = raster.cells.copy()
    cells[cells == OCCUPIED] = FREE
    rows, columns = raster.cell_index(points)
    inside = raster.contains(rows, columns)
    cells[rows[inside], columns[inside]] = OCCUPIED
    return OccupancyMap(cells, raster.resolution, raster.origin)


def with_ghost_wall(raster, length):
    """Return a copy of the map with a wall of ``length`` m across the open floor at y 5."""
    cells = raster.cells.copy()
    xs = np.arange(14.025, 14.025 + length, raster.resolution)   # 1.8 m from row H, 2.2 m
    rows, columns = raster.cell_index(np.column_stack((xs, np.full(len(xs), 5.025))))
    cells[rows, columns] = OCCUPIED
    return OccupancyMap(cells, raster.resolution, raster.origin), len(xs)


def png_size(path):
    """Return (width, height) of a PNG file, read from its header."""
    with open(path, 'rb') as stream:
        header = stream.read(24)
    assert header[:8] == b'\x89PNG\r\n\x1a\n'
    return struct.unpack('>II', header[16:24])


def test_the_criteria_are_the_plan_f5_acceptance(raster, faces):
    result = check_map.check(raster, faces)
    assert [r.rule for r in result.results] == list(CRITERIA)
    assert (check_map.MAX_OFFSET, check_map.MAX_ROTATION) == (0.05, 0.2)
    assert (check_map.NEAR, check_map.MIN_NEAR_FRACTION) == (0.10, 0.95)
    assert (check_map.GHOST, check_map.MAX_GHOST_FRACTION) == (0.25, 0.01)


def test_the_rasterized_layout_passes_with_no_offset(raster, faces):
    result = check_map.check(raster, faces)
    assert result.passed, [str(r) for r in result.results]
    assert result.fit.offset < 0.005
    assert abs(math.degrees(result.fit.yaw)) < 0.01
    assert result.fit.settled
    assert result.fit.pairs == len(result.cells)
    # Each occupied cell's centre is half a cell (0.025 m) behind the face it holds.
    assert np.percentile(result.distances, 95) < 0.03


def test_a_map_shifted_by_015_m_is_measured_and_fails(raster, faces):
    x0, y0, yaw = raster.origin
    shifted = dataclasses.replace(raster, origin=(x0 + 0.15, y0, yaw))   # every cell 0.15 m east
    result = check_map.check(shifted, faces)
    assert result.fit.x == pytest.approx(-0.15, abs=0.005)     # move it back west
    assert result.fit.y == pytest.approx(0.0, abs=0.005)
    assert result.fit.offset == pytest.approx(0.15, abs=0.005)
    assert failed(result) == ['offset']


def test_a_map_turned_by_1_degree_is_measured_and_fails(raster, faces):
    # Turn every occupied cell 1 degree about the origin (robot1's spawn, where mapping starts).
    turn = math.radians(1.0)
    rotation = np.array([[math.cos(turn), math.sin(turn)], [-math.sin(turn), math.cos(turn)]])
    turned = with_occupied(raster, raster.centres(OCCUPIED) @ rotation)
    result = check_map.check(turned, faces)
    assert math.degrees(result.fit.yaw) == pytest.approx(-1.0, abs=0.05)   # turn it back
    assert result.fit.offset < 0.01
    assert failed(result) == ['rotation']


def test_ghost_walls_beyond_1_percent_fail(raster, faces):
    ghosted, added = with_ghost_wall(raster, 6.0)
    result = check_map.check(ghosted, faces)
    assert added / len(result.cells) > 0.015
    assert np.count_nonzero(result.distances > check_map.GHOST) == added
    assert result.fit.offset < 0.005          # the ghost cells take no part in the fit
    assert failed(result) == ['ghost_walls']


def test_a_few_stray_cells_pass(raster, faces):
    ghosted, added = with_ghost_wall(raster, 1.5)
    result = check_map.check(ghosted, faces)
    assert added / len(result.cells) < 0.005
    assert result.passed


def test_an_origin_with_a_yaw_fails(raster, faces):
    x0, y0, _ = raster.origin
    result = check_map.check(dataclasses.replace(raster, origin=(x0, y0, 0.01)), faces)
    assert 'origin_yaw' in failed(result)
    # The yaw turns the whole map about its lower-left corner, and the fit sees that turn.
    assert math.degrees(result.fit.yaw) == pytest.approx(-math.degrees(0.01), abs=0.05)


def test_a_map_of_somewhere_else_cannot_be_fitted(raster, faces):
    x0, y0, yaw = raster.origin
    with pytest.raises(check_map.FitError, match='is this a map of this warehouse'):
        check_map.check(dataclasses.replace(raster, origin=(x0 + 100.0, y0, yaw)), faces)


def test_the_command_passes_the_rasterized_layout_and_draws_the_overlay(raster, tmp_path,
                                                                        capsys):
    occupancy.save(raster, str(tmp_path / 'layout_map.yaml'))
    overlay = tmp_path / 'overlay.png'
    assert check_map.main([str(tmp_path / 'layout_map.yaml'), '--overlay', str(overlay)]) == 0
    printed = capsys.readouterr().out
    assert printed.count('[PASS]') == 5
    assert '5 of 5 map checks pass' in printed
    assert png_size(overlay) == (1920, 1080)       # 16 x 9 inches at 120 dpi


def test_the_command_exits_1_on_a_failed_criterion_and_still_draws(raster, tmp_path, capsys):
    x0, y0, yaw = raster.origin
    occupancy.save(dataclasses.replace(raster, origin=(x0 + 0.15, y0, yaw)),
                   str(tmp_path / 'shifted.yaml'))
    overlay = tmp_path / 'overlay.png'
    assert check_map.main([str(tmp_path / 'shifted.yaml'), '--overlay', str(overlay)]) == 1
    printed = capsys.readouterr().out
    assert '[FAIL] best-fit offset <= 0.05 m: 0.150 m (move the map x -0.150' in printed
    assert '4 of 5 map checks pass' in printed
    assert overlay.exists()


def test_the_command_exits_1_on_an_empty_map_and_still_draws(raster, tmp_path, capsys):
    empty = OccupancyMap(np.full(raster.cells.shape, FREE, dtype=np.int8), raster.resolution,
                         raster.origin)
    occupancy.save(empty, str(tmp_path / 'empty.yaml'))
    overlay = tmp_path / 'overlay.png'
    assert check_map.main([str(tmp_path / 'empty.yaml'), '--overlay', str(overlay)]) == 1
    assert '[FAIL] no fit: the map has no occupied cells' in capsys.readouterr().out
    assert png_size(overlay) == (1920, 1080)     # the layout's faces over the empty map


@pytest.mark.parametrize('text, message', [
    ('image: map.pgm\nmode: trinary\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n'
     'occupied_thresh: 0.65\nfree_thresh: 0.196\n', 'missing key resolution'),
    ('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0]\nnegate: 0\n'
     'occupied_thresh: 0.65\nfree_thresh: 0.196\n', 'origin must be [x, y, yaw]'),
    ('image: [map.pgm\n', 'not valid YAML'),
    ('image: nothing.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n'
     'occupied_thresh: 0.65\nfree_thresh: 0.196\n', 'does not exist'),
])
def test_a_malformed_map_yaml_exits_2_with_a_clear_error(tmp_path, capsys, text, message):
    (tmp_path / 'map.yaml').write_text(text)
    overlay = tmp_path / 'overlay.png'
    assert check_map.main([str(tmp_path / 'map.yaml'), '--overlay', str(overlay)]) == 2
    error = capsys.readouterr().err
    assert error.startswith(f'error: {tmp_path / "map.yaml"}: ')
    assert message in error
    assert not overlay.exists()
