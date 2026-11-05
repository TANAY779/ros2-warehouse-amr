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
Check the layout fit rules (plan section 3.6) on layout.yaml and on broken copies of it.

layout.yaml passes every rule, and every rule fails on a copy of layout.yaml with one
deliberate mistake, with a message that names the object that breaks it. A rule that cannot
fail proves nothing, so each negative test is the evidence that its rule works. Plain pytest,
no ROS graph: python3 -m pytest src/warehouse_layout/test -q
"""

import copy
import dataclasses

import pytest
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import rules
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


@pytest.fixture
def data(layout_yaml):
    """Return a fresh copy of layout.yaml (a dict) to put one mistake into."""
    return copy.deepcopy(layout_yaml)


def item(items, name):
    """Return the entry with this id from a list in layout.yaml."""
    return next(entry for entry in items if str(entry['id']) == name)


def violations(rule, layout):
    """Run one rule that must fail; return its violations as one text."""
    result = rule(layout)
    assert not result.passed, f'{rule.__name__} passed, but the layout breaks it'
    return '\n'.join(result.violations)


def broken(rule, data, robot):
    """Build the layout from a modified copy and return the rule's violations."""
    return violations(rule, model.from_dict(data, robot=robot))


# ----------------------------------------------------------------------------- layout.yaml


@pytest.mark.parametrize('rule', rules.RULES, ids=lambda rule: rule.__name__)
def test_layout_yaml_passes(rule, layout):
    result = rule(layout)
    assert result.passed, str(result)


def test_check_runs_every_rule(layout):
    results = rules.check(layout)
    assert [r.rule for r in results] == [
        'pick_aisle_width', 'corridor_width', 'goal_clearance', 'visibility', 'docks',
        'spawns', 'totes', 'containment', 'routes']
    assert all(r.passed for r in results)


def test_derived_limits_come_from_the_robot(robot):
    # plan section 3.6, for the Rev B robot: 0.456 + 0.10, 0.34 + 0.06, 0.24 + 0.16
    assert rules.goal_clearance(robot) == pytest.approx(robot.r_circumscribed_padded + 0.10)
    assert rules.visible_height(robot) == pytest.approx(robot.scan_plane_height + 0.06)
    assert rules.route_clearance(robot) == pytest.approx(robot.half_width + 0.16)


# ----------------------------------------------------------------------------- one rule each


def test_aisle_of_1_5_m_fails(data, robot):
    item(data['rows'], 'B')['x_face'] = 3.5
    text = broken(rules.check_pick_aisle_width, data, robot)
    assert 'aisle_1_south: 1.500 m between the faces of rows A and B' in text
    assert 'aisle_1_north' in text


def test_a_wider_robot_no_longer_fits_the_aisles(layout_yaml, robot):
    # The rules read the robot from robot.yaml: a robot 0.9 m wide would need 1.66 m to
    # turn in place, so the 1.60 m aisle rule itself has to be revisited.
    wide = copy.deepcopy(robot.raw)
    wide['footprint'].update(length=1.0, width=0.9)
    layout = model.from_dict(layout_yaml, robot=robot_config.from_dict(wide))
    text = violations(rules.check_pick_aisle_width, layout)
    assert 'robot: turning in place needs 1.658 m' in text


def test_narrow_corridor_fails(data, robot):
    # The forklift keepout grows south to y 3.0: 1.5 m left between it and the office.
    item(data['zones']['keepout'], 'forklift_area')['polygon'] = [
        [21.5, 3.0], [28.5, 3.0], [28.5, 9.0], [21.5, 9.0]]
    text = broken(rules.check_corridor_width, data, robot)
    assert 'office_corridor: 1.50 m clear between office and forklift_area' in text


def test_pick_pose_0_4_m_from_a_shelf_fails(data, robot):
    # Row B moved next to row A: aisle 1 is 0.80 m wide, its pick poses 0.40 m from the faces.
    item(data['rows'], 'B')['x_face'] = 2.8
    text = broken(rules.check_goal_clearance, data, robot)
    assert 'pick_1_01: 0.400 m from shelf_' in text     # rows A and B are equally close


def test_goal_inside_a_keepout_fails(data, robot):
    data['spawns']['robot3'] = {'x': 27.0, 'y': 5.0, 'yaw': 0.0}
    text = broken(rules.check_goal_clearance, data, robot)
    assert 'spawn_robot3: inside keepout forklift_area' in text


def test_plinth_of_0_30_m_fails(data, robot):
    data['shelving']['plinth_height'] = 0.30
    text = broken(rules.check_visibility, data, robot)
    assert 'shelf_A_south: solid only to 0.30 m; the lidar needs 0.40 m' in text
    assert text.count('shelf_') == 16      # every row segment


def test_low_fixture_fails(data, robot):
    item(data['fixtures'], 'pallet_1')['height'] = 0.30
    text = broken(rules.check_visibility, data, robot)
    assert 'pallet_1: solid only to 0.30 m' in text


def test_totes_sticking_out_fail(data, robot):
    data['inventory']['tote']['setback'] = -0.05
    text = broken(rules.check_visibility, data, robot)
    assert 'shelf_A_south: 30 totes stick out past the face, up to 0.050 m' in text


def test_contact_bar_outside_every_keepout_fails(layout):
    # The model always puts a keepout box round each bar; without them the rule must object.
    text = violations(rules.check_visibility, dataclasses.replace(layout, keepouts=()))
    assert 'dock_1_bar: 0.12 m high, below the scan plane, and not inside a keepout zone' in text
    assert 'dock_1_plate' in text


@pytest.mark.parametrize('section, key, value, message', [
    ('contact_bar', 'height', 0.20, 'dock_1: contact bar 0.20 m high; at most 0.15 m'),
    (None, 'tag_setback', 0.30, 'dock_1: pedestal face 0.30 m behind the contact face'),
    (None, 'staging_distance', 0.50, 'dock_1: staging pose 0.50 m from the docked pose'),
])
def test_wrong_dock_geometry_fails(data, robot, section, key, value, message):
    (data['docks'][section] if section else data['docks'])[key] = value
    assert message in broken(rules.check_docks, data, robot)


def test_docked_pose_away_from_the_contact_face_fails(layout):
    dock = layout.docks[0]
    docked = dataclasses.replace(dock.docked, y=dock.docked.y + 0.1)
    moved = dataclasses.replace(dock, docked=docked)
    text = violations(rules.check_docks, dataclasses.replace(layout, docks=(moved,)))
    assert 'dock_1: docked pose 0.420 m from the contact face; must be 0.320 m' in text


def test_robot1_not_at_the_origin_fails(data, robot):
    data['spawns']['robot1']['x'] = 0.5
    text = broken(rules.check_spawns, data, robot)
    assert 'robot1: spawns at (0.5, 0, yaw 0); must be (0, 0, yaw 0)' in text


def test_spawns_too_close_fail(data, robot):
    data['spawns']['robot3']['y'] = 0.9
    text = broken(rules.check_spawns, data, robot)
    assert 'robot1 and robot3: spawns 0.90 m apart; need 1.11 m' in text


def test_240_totes_fail(data, robot):
    data['inventory']['fill_ratio'] = 0.58       # 414 x 0.58 = 240.1
    text = broken(rules.check_totes, data, robot)
    assert 'inventory: 240 totes' in text
    assert 'only 230 tote tag IDs (20-249)' in text


def test_totes_set_back_from_the_face_fail(data, robot):
    data['inventory']['tote']['setback'] = 0.02
    text = broken(rules.check_totes, data, robot)
    assert 'inventory.tote: setback 0.02 m' in text


def test_two_objects_overlapping_fail(data, robot):
    item(data['fixtures'], 'column_1').update(x=[10.0, 10.5], y=[5.0, 5.5])
    text = broken(rules.check_containment, data, robot)
    assert 'shelf_G_south: overlaps column_1' in text


def test_object_outside_the_hall_fails(data, robot):
    item(data['fixtures'], 'compressor')['x'] = [27.0, 29.0]
    text = broken(rules.check_containment, data, robot)
    assert 'compressor: not inside the hall' in text


def test_route_outside_the_hall_fails(data, robot):
    data['routes']['smoke'][2] = [30.0, 1.5]
    text = broken(rules.check_containment, data, robot)
    assert 'route smoke: (30, 1.5) is not inside the hall' in text


def test_row_letter_i_makes_invalid_slot_ids(data, robot):
    # Warehouses skip I and O: on a label they read like 1 and 0.
    item(data['rows'], 'A')['id'] = 'I'
    item(data['aisles'], '1')['rows'] = ['I', 'B']
    text = broken(rules.check_containment, data, robot)
    assert 'row I: slot IDs such as I01-1-1 are not valid' in text


def test_duplicate_slot_ids_fail(layout):
    doubled = dataclasses.replace(layout, slots=layout.slots + layout.slots[:1])
    text = violations(rules.check_containment, doubled)
    assert 'A01-1-1: slot ID used 2 times' in text


def test_waypoint_too_close_fails(data, robot):
    data['routes']['survey'][3] = [20.0, 17.6]    # 0.40 m from the north wall
    text = broken(rules.check_routes, data, robot)
    assert 'route survey: waypoint 3 (20, 17.6) is 0.40 m from wall_north' in text


def test_segment_too_close_fails(data, robot):
    smoke = data['routes']['smoke']
    smoke[2], smoke[3] = [2.3, 1.5], [2.3, 9.7]    # 0.30 m from row A's face
    text = broken(rules.check_routes, data, robot)
    assert 'route smoke: segment 2 [2.3, 1.5] -> [2.3, 9.7] passes 0.30 m from' in text


def test_route_through_a_keepout_fails(data, robot):
    survey = data['routes']['survey']
    survey[2], survey[3] = [24.0, 1.5], [24.0, 16.6]
    text = broken(rules.check_routes, data, robot)
    assert 'route survey: segment 2 [24.0, 1.5] -> [24.0, 16.6] runs through keepout ' \
        'forklift_area' in text
