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
The layout model: what model.py derives from layout.yaml, and how it rejects a bad file.

Expected positions are computed from layout.yaml's own numbers where that keeps the test
readable; the counts in test_documented_numbers are the ones the plan and the README quote.
"""

import copy
import math
import re

import pytest
from warehouse_description import robot_config
from warehouse_layout import geometry
from warehouse_layout import model
import yaml


@pytest.fixture(scope='module')
def robot():
    return robot_config.load()


@pytest.fixture(scope='module')
def layout_yaml():
    with open(model.default_path()) as stream:
        return yaml.safe_load(stream)


@pytest.fixture(scope='module')
def layout(layout_yaml, robot):
    return model.from_dict(layout_yaml, robot=robot)


def named(items, name):
    """Return the object with this name from a tuple of the layout."""
    return next(entry for entry in items if entry.name == name)


def test_documented_numbers(layout):
    # The plan (section 3) and the package README quote these numbers: update them together.
    assert len({row.row for row in layout.rows}) == 8
    assert len(layout.rows) == 16                       # 8 rows x 2 blocks
    assert sum(len(row.bays) for row in layout.rows) == 69
    assert len(layout.slots) == 414                     # 69 bays x 3 levels x 2 positions
    assert layout.inventory.tote_count == 207           # fill ratio 0.5
    assert len(layout.aisles) == 8                      # 4 aisles x 2 blocks
    assert len(layout.picks) == 36
    assert len(layout.stations) == 1
    assert len(layout.docks) == 3
    assert len(layout.spawns) == 3
    assert [z.generated for z in layout.keepouts].count(False) == 3
    assert [z.generated for z in layout.keepouts].count(True) == 3          # one per dock
    assert [z.generated for z in layout.speed_zones].count(False) == 2
    assert [z.generated for z in layout.speed_zones].count(True) == 8       # one per aisle
    assert [r.name for r in layout.routes] == ['survey', 'smoke']


def test_load_reads_the_installed_file(robot):
    layout = model.load(robot=robot)
    assert layout.name == 'warehouse'
    assert layout.style.show_totes is False     # bare shelves until Phase 7


def test_slot_c07_2_1(layout):
    # Row C, bay 07, level 2, position 1 (the south one).
    slot = named(layout.slots, 'C07-2-1')
    row = named(layout.rows, 'C_north')
    bay = next(b for b in row.bays if b.number == 7)
    tote = layout.inventory.tote
    assert (slot.row, slot.bay, slot.level, slot.position) == ('C', 7, 2, 1)
    assert row.face == '+x'
    assert slot.tote.x1 == pytest.approx(row.x_face)                 # flush with the face
    assert slot.tote.x1 - slot.tote.x0 == pytest.approx(tote.length)  # long side into the shelf
    assert slot.tote.z0 == pytest.approx(layout.shelving.board_heights[0])
    pitch = (bay.y1 - bay.y0) / layout.shelving.positions_per_level
    assert slot.tote.centre[1] == pytest.approx(bay.y0 + pitch / 2)


def test_slot_ids_are_unique_and_follow_the_pattern(layout):
    names = [slot.name for slot in layout.slots]
    assert len(set(names)) == len(names)
    assert all(model.SLOT_ID_PATTERN.match(name) for name in names)
    assert model.SLOT_ID_PATTERN.match('C07-2-1')
    assert not model.SLOT_ID_PATTERN.match('I07-2-1')     # no I or O
    assert not model.SLOT_ID_PATTERN.match('C7-2-1')      # two-digit bay


def test_bay_numbers_continue_across_blocks(layout):
    assert [b.number for b in named(layout.rows, 'A_south').bays] == [1, 2, 3, 4, 5]
    assert [b.number for b in named(layout.rows, 'A_north').bays] == [6, 7, 8]
    assert [b.number for b in named(layout.rows, 'B_north').bays] == [6, 7, 8, 9]


def test_bays_are_the_openings_between_uprights(layout):
    shelving = layout.shelving
    row = named(layout.rows, 'B_south')
    for bay in row.bays:
        assert bay.y1 - bay.y0 == pytest.approx(shelving.bay_width - shelving.upright)
    # Uprights at both ends: the segment is one upright longer than its bays.
    assert row.box.y1 - row.box.y0 == pytest.approx(
        len(row.bays) * shelving.bay_width + shelving.upright)


def test_aisles_run_between_facing_rows(layout):
    aisle = named(layout.aisles, 'aisle_1_south')
    assert aisle.rows == ('A', 'B')
    assert aisle.box.x0 == pytest.approx(named(layout.rows, 'A_south').x_face)
    assert aisle.box.x1 == pytest.approx(named(layout.rows, 'B_south').x_face)
    # From guard to guard: the longer of the two rows plus an end guard at each end.
    guard = layout.shelving.guard_depth
    assert aisle.box.y0 == pytest.approx(named(layout.rows, 'A_south').box.y0 - guard)
    assert aisle.box.y1 == pytest.approx(named(layout.rows, 'B_south').box.y1 + guard)


def test_pick_poses_on_the_centreline(layout):
    pick = named(layout.picks, 'pick_1_02')
    aisle = named(layout.aisles, 'aisle_1_south')
    bay = named(layout.rows, 'B_south').bays[1]
    assert (pick.x, pick.y, pick.yaw) == pytest.approx((aisle.centre_x, bay.centre_y, math.pi / 2))
    # Aisle 1 north: row A has 3 bays, row B 4, so there are 4 pick poses (06-09).
    north = [p.name for p in layout.picks if p.name.startswith('pick_1_')][5:]
    assert north == ['pick_1_06', 'pick_1_07', 'pick_1_08', 'pick_1_09']


def test_dock_poses(layout, layout_yaml):
    docks = layout_yaml['docks']
    dock = named(layout.docks, 'dock_1')
    hall = layout.hall
    contact_y = hall.y0 + docks['pedestal']['depth'] + docks['tag_setback']
    assert dock.contact == pytest.approx((10.0, contact_y))
    assert dock.tag == pytest.approx((10.0, hall.y0 + docks['pedestal']['depth']))
    # The robot docks forwards, facing the south wall: base_link half its length out.
    half_length = layout.robot.half_length
    assert (dock.docked.x, dock.docked.y, dock.docked.yaw) == pytest.approx(
        (10.0, contact_y + half_length, -math.pi / 2))
    assert (dock.staging.x, dock.staging.y) == pytest.approx(
        (10.0, contact_y + half_length + docks['staging_distance']))
    # The plan's numbers (section 3.2): contact face y -1.45, docked -1.13, staging -0.43.
    assert (contact_y, dock.docked.y, dock.staging.y) == pytest.approx((-1.45, -1.13, -0.43))


def test_dock_poses_follow_the_robot(layout_yaml, robot):
    # One source of truth: a longer robot docks further out, with no edit to layout.yaml.
    longer = copy.deepcopy(robot.raw)
    longer['footprint']['length'] = robot.length + 0.2
    layout = model.from_dict(layout_yaml, robot=robot_config.from_dict(longer))
    dock = named(layout.docks, 'dock_1')
    assert dock.docked.y - dock.contact[1] == pytest.approx(robot.half_length + 0.1)


def test_dock_keepout_covers_the_parts_below_the_scan_plane(layout):
    for dock in layout.docks:
        for part in (dock.contact_bar, dock.plate):
            assert geometry.polygon_contains_box(dock.keepout.polygon, part)
        assert dock.keepout in layout.keepouts


def test_walls_step_round_the_recess(layout):
    hall = layout.hall
    walls = {s.name: s.box for s in layout.solids if s.kind == 'wall'}
    assert set(walls) == {'wall_south', 'wall_north', 'wall_west', 'wall_east_1', 'wall_east_2',
                          'loading_door_back', 'loading_door_side_1', 'loading_door_side_2'}
    assert (walls['wall_east_1'].y0, walls['wall_east_1'].y1) == pytest.approx((hall.y0, 4.5))
    assert (walls['wall_east_2'].y0, walls['wall_east_2'].y1) == pytest.approx((8.5, hall.y1))
    back = walls['loading_door_back']
    assert back.x0 == pytest.approx(hall.x1 + 0.40)          # the recess depth
    assert back.x1 - back.x0 == pytest.approx(hall.wall_thickness)
    # The south and north walls run past the corners, so the hall is closed.
    assert walls['wall_south'].x0 == pytest.approx(hall.x0 - hall.wall_thickness)
    assert walls['wall_south'].x1 == pytest.approx(hall.x1 + hall.wall_thickness)


def test_fence_leaves_the_gate_open(layout):
    fence = named(layout.fences, 'machinery_fence')
    assert fence.gates == (((23.5, 11.8), (24.9, 11.8)),)
    assert len(fence.pieces) == 3
    pieces = [s for s in layout.solids if s.group == 'machinery_fence']
    assert [p.name for p in pieces] == ['machinery_fence_1', 'machinery_fence_2',
                                        'machinery_fence_3']
    assert all(p.solid_to == pytest.approx(fence.kick_plate_height) for p in pieces)
    assert not any(p.box.x0 < 24.9 and p.box.x1 > 23.5 and p.box.y0 < 11.8 < p.box.y1
                   for p in pieces)


def test_generated_speed_zones_cover_the_aisles(layout, layout_yaml):
    limit = layout_yaml['zones']['pick_aisle_speed']
    zones = {z.name: z for z in layout.speed_zones if z.generated}
    for aisle in layout.aisles:
        assert zones[aisle.name].limit == pytest.approx(limit)
        assert zones[aisle.name].polygon == aisle.box.corners()


def test_markings(layout, layout_yaml, robot):
    margin = layout_yaml['markings']['bay_margin']
    bay = named(layout.markings, 'robot1_parking_bay').box
    assert bay.x1 - bay.x0 == pytest.approx(robot.length + 2 * margin)
    assert bay.y1 - bay.y0 == pytest.approx(robot.width + 2 * margin)
    zebra = [m for m in layout.markings if m.kind == 'zebra_stripe']
    assert len(zebra) == 10                 # 5 m crossing, a 0.25 m band every 0.5 m
    assert all(m.box.y1 - m.box.y0 == pytest.approx(0.25) for m in zebra)
    stripes = [m for m in layout.markings if m.kind == 'aisle_stripe']
    assert len(stripes) == 2 * len(layout.aisles)


def test_solids_have_unique_names_and_stand_on_the_floor(layout):
    names = [s.name for s in layout.solids]
    assert len(set(names)) == len(names)
    assert all(s.box.z0 == 0.0 for s in layout.solids)
    shelf = named(layout.solids, 'shelf_A_south')
    assert shelf.solid_to == pytest.approx(layout.shelving.plinth_height)   # open above
    assert shelf.box.z1 == pytest.approx(layout.shelving.top_height)


def test_goals_and_lookups(layout):
    goals = layout.goals()
    assert len(goals) == 3 + 1 + 3 + 36     # spawns, drop, dock staging poses, picks
    assert layout.spawn('robot1') == model.Pose('spawn_robot1', 0.0, 0.0, 0.0)
    assert len(layout.route('survey').waypoints) == 19
    with pytest.raises(KeyError):
        layout.spawn('robot9')


# ----------------------------------------------------------------------------- schema errors


def _set(data, path, value):
    """Set a value at a dotted path; list items are picked by id: 'rows[B].x_face'."""
    keys = re.findall(r'[^.\[\]]+|\[[^\]]+\]', path)
    node = data
    for key in keys[:-1]:
        node = _step(node, key)
    last = keys[-1]
    if value is _DELETE:
        del node[last]
    else:
        node[last] = value


def _step(node, key):
    if key.startswith('['):
        return next(entry for entry in node if str(entry['id']) == key[1:-1])
    return node[key]


_DELETE = object()


@pytest.mark.parametrize('path, value, message', [
    ('hall.wall_height', _DELETE, 'hall.wall_height: missing'),
    ('hall.wall_thikness', 0.2, 'hall.wall_thikness: unknown key'),
    ('seed', 'two', "seed: expected a whole number, got 'two'"),
    ('schema_version', 2, 'schema_version: 2 is not supported'),
    ('style.floor', 'grey', "style.floor: expected a colour '#rrggbb', got 'grey'"),
    ('style.fixtures.machine', _DELETE, "style.fixtures: no colour for kind 'machine'"),
    ('hall.x', [28.5, -1.5], 'hall.x: expected [low, high] with low < high'),
    ('rows[B].x_face', '3.6', "rows[B].x_face: expected a number, got '3.6'"),
    ('rows[B].face', 'north', 'rows[B].face: expected one of +x, -x'),
    ('rows[B].bays', {'north': 5}, 'rows[B].bays.north: 5 bays, but the north block has only 4'),
    ('rows[B].bays', {'east': 3}, "rows[B].bays.east: no block 'east'"),
    ('aisles[1].rows', ['A', 'Z'], "aisles[1].rows: no row 'Z'"),
    ('aisles[1].rows', ['B', 'C'], 'aisles[1].rows: rows B and C do not face each other'),
    ('shelving.board_heights', [1.10, 0.75], 'shelving: plinth_height, board_heights and '
                                             'top_height must rise'),
    ('docks.units[dock_1].wall', 'roof', 'docks.units[dock_1].wall: expected one of south, north'),
    ('hall.recesses[loading_door].span', [19.0, 21.0], 'is not on the east wall'),
    ('fences[machinery_fence].path', [[22.5, 18.0], [23.0, 11.8], [28.5, 11.8]],
     'fences[machinery_fence].path: segment 0 is not parallel to x or y'),
    ('zones.keepout[machinery].polygon', [[0, 0], [1, 1]],
     'zones.keepout[machinery].polygon: expected a list of at least 3 [x, y] points'),
    ('fixtures[office].id', 'pallet_1', "object names: 'pallet_1' is used twice"),
    ('inventory.fill_ratio', 1.5, 'inventory.fill_ratio: must be at most 1'),
    ('style.show_totes', 'no', "style.show_totes: expected true or false, got 'no'"),
])
def test_schema_errors_name_the_key(layout_yaml, robot, path, value, message):
    data = copy.deepcopy(layout_yaml)
    _set(data, path, value)
    with pytest.raises(model.LayoutError, match=re.escape(message)):
        model.from_dict(data, robot=robot)


def test_every_row_needs_an_aisle(layout_yaml, robot):
    data = copy.deepcopy(layout_yaml)
    data['aisles'] = data['aisles'][:3]          # rows G and H lose aisle 4
    with pytest.raises(model.LayoutError, match=re.escape('rows[G]: not in any aisle')):
        model.from_dict(data, robot=robot)


def test_load_names_the_file(layout_yaml, robot, tmp_path):
    data = copy.deepcopy(layout_yaml)
    del data['hall']['wall_height']
    path = tmp_path / 'broken.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(model.LayoutError, match=re.escape(f'{path}: hall.wall_height: missing')):
        model.load(str(path), robot=robot)
