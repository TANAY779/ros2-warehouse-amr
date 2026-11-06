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

"""What the lidar sees at its scan plane: the faces, and the map drawn from them."""

import numpy as np
import pytest
from scipy.spatial import cKDTree
from warehouse_layout import model
from warehouse_layout import occupancy
from warehouse_layout import rasterize
from warehouse_layout.occupancy import FREE
from warehouse_layout.occupancy import OCCUPIED
from warehouse_layout.occupancy import UNKNOWN
import yaml


@pytest.fixture(scope='module')
def layout():
    return model.load()


@pytest.fixture(scope='module')
def raster(layout):
    return rasterize.rasterize(layout)


@pytest.fixture(scope='module')
def faces(layout):
    return rasterize.surfaces(layout)


def box(layout, name):
    return next(solid.box for solid in layout.solids if solid.name == name)


def test_only_what_the_scan_plane_cuts_is_seen(layout):
    kinds = {solid.kind for solid in rasterize.seen_solids(layout)}
    assert {'wall', 'pilaster', 'shelf', 'guard', 'station', 'dock_pedestal', 'fence'} <= kinds
    assert not kinds & {'contact_bar', 'dock_plate'}       # 0.12 m and 0.01 m high


def test_the_map_frame_is_the_world_frame(layout, raster):
    x0, y0, yaw = raster.origin
    assert yaw == 0.0
    assert raster.resolution == rasterize.RESOLUTION
    for corner in (x0, y0):    # cell edges fall on the layout's round numbers
        assert abs(corner / raster.resolution - round(corner / raster.resolution)) < 1e-9
    spawn = layout.spawn('robot1')
    assert raster.value_at(spawn.x + 0.01, spawn.y + 0.01) == FREE
    assert set(np.unique(raster.cells)) == {FREE, OCCUPIED, UNKNOWN}


def test_a_wall_is_seen_only_on_its_inner_face(layout, raster):
    hall, x = layout.hall, 3.0     # the south wall, clear of pilasters, docks and the counter
    assert raster.value_at(x, hall.y0 + 0.02) == FREE
    assert raster.value_at(x, hall.y0 - 0.02) == OCCUPIED    # the cell behind the inner face
    assert raster.value_at(x, hall.y0 - 0.12) == UNKNOWN     # inside the wall
    assert raster.value_at(x, hall.y0 - hall.wall_thickness - 0.1) == UNKNOWN   # outside


def test_a_shelf_plinth_is_seen_on_its_faces(layout, raster):
    shelf = box(layout, 'shelf_A_south')
    y = (shelf.y0 + shelf.y1) / 2
    assert raster.value_at(shelf.x0 - 0.02, y) == FREE
    assert raster.value_at(shelf.x0 + 0.02, y) == OCCUPIED
    assert raster.value_at((shelf.x0 + shelf.x1) / 2, y) == UNKNOWN
    assert raster.value_at(shelf.x1 - 0.02, y) == OCCUPIED


def test_what_lies_below_the_scan_plane_is_free_floor(layout, raster):
    for dock in layout.docks:
        for low in (dock.contact_bar, dock.plate):
            x, y, _ = low.centre
            assert raster.value_at(x, y) == FREE, dock.name
        x = (dock.pedestal.x0 + dock.pedestal.x1) / 2
        assert raster.value_at(x, dock.pedestal.y1 - 0.02) == OCCUPIED, dock.name


def test_the_fence_is_seen_and_its_gate_is_open(layout, raster):
    fence = layout.fences[0]
    (x0, y0), (x1, y1) = fence.gates[0]
    assert raster.value_at((x0 + x1) / 2, y0 + 0.01) == FREE
    piece = box(layout, f'{fence.name}_3')     # 0.05 m thick, centred on y 11.8
    x = (piece.x0 + piece.x1) / 2
    assert [raster.value_at(x, piece.y0 + dy) for dy in (-0.04, 0.01, 0.04, 0.09)] == [
        FREE, OCCUPIED, OCCUPIED, FREE]      # two cells: the fence lies across a cell edge


def test_the_faces_and_the_occupied_cells_agree(raster, faces):
    cells = raster.centres(OCCUPIED)
    # Every occupied cell holds a face (its centre is at most half a cell from it) ...
    distance, _ = cKDTree(faces.points).query(cells)
    assert distance.max() < 0.026
    # ... and every face point lies in or beside an occupied cell: within half a cell's
    # diagonal (0.035 m) of one, except where two fence pieces meet off the cell grid. There
    # the corner cell is partly covered by both pieces, so none of its edge neighbours is
    # free and it stays unknown; the nearest occupied cell is 0.045 m away.
    distance, _ = cKDTree(cells).query(faces.points)
    assert np.mean(distance < 0.036) > 0.9999
    assert distance.max() < 0.05
    assert np.allclose(np.linalg.norm(faces.normals, axis=1), 1.0)


def test_hidden_faces_are_left_out(layout, faces):
    hall = layout.hall
    # Behind a pilaster: neither the wall's face nor the pilaster's back is visible.
    pilaster = box(layout, 'pilaster_south_1')
    hidden = ((np.abs(faces.points[:, 1] - hall.y0) < 1e-9)
              & (faces.points[:, 0] > pilaster.x0 + 0.01)
              & (faces.points[:, 0] < pilaster.x1 - 0.01))
    assert not hidden.any()
    # Nothing on the outer faces of the walls.
    assert faces.points[:, 1].min() >= hall.y0 - 1e-9
    assert faces.points[:, 0].min() >= hall.x0 - 1e-9
    # Every face looks into the hall: one step in front of it is free floor.
    probes = faces.points + 0.01 * faces.normals
    floor = rasterize._box_array(rasterize.floor_boxes(layout))
    solids = rasterize._box_array(solid.box for solid in rasterize.seen_solids(layout))
    assert rasterize._inside(probes, floor).all()
    assert not rasterize._inside(probes, solids).any()


def test_the_command_writes_a_map_server_map(raster, tmp_path, capsys):
    assert rasterize.main(['--out', str(tmp_path / 'layout_map')]) == 0
    assert 'below the scan plane, so not drawn: contact_bar, dock_plate' in capsys.readouterr().out
    data = yaml.safe_load((tmp_path / 'layout_map.yaml').read_text())
    assert data == {'image': 'layout_map.pgm', 'mode': 'trinary', 'resolution': 0.05,
                    'origin': [raster.origin[0], raster.origin[1], 0.0], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.196}
    image = (tmp_path / 'layout_map.pgm').read_bytes()
    header = f'P5\n{raster.width} {raster.height}\n255\n'.encode()
    assert image.startswith(header)
    assert set(image[len(header):]) == {0, 205, 254}
    loaded = occupancy.load(str(tmp_path / 'layout_map.yaml'))
    assert np.array_equal(loaded.cells, raster.cells)


def test_the_same_layout_gives_the_same_bytes(tmp_path):
    (tmp_path / 'a').mkdir()
    (tmp_path / 'b').mkdir()
    assert rasterize.main(['--out', str(tmp_path / 'a' / 'map')]) == 0
    assert rasterize.main(['--out', str(tmp_path / 'b' / 'map.yaml')]) == 0   # same file names
    for name in ('map.yaml', 'map.pgm'):
        assert (tmp_path / 'a' / name).read_bytes() == (tmp_path / 'b' / name).read_bytes()


def test_a_layout_that_cannot_be_read_exits_2(tmp_path, capsys):
    out = tmp_path / 'map'
    assert rasterize.main(['--out', str(out), '--layout', str(tmp_path / 'none.yaml')]) == 2
    assert 'error:' in capsys.readouterr().err
    assert not (tmp_path / 'map.yaml').exists()
