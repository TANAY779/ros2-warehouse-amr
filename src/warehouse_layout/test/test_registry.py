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

"""The location registry (locations.yaml), read back with PyYAML as its users will read it."""

import math

import pytest
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import registry
import yaml


@pytest.fixture(scope='module')
def layout():
    return model.load(robot=robot_config.load())


@pytest.fixture(scope='module')
def text(layout):
    return registry.render(layout)


@pytest.fixture(scope='module')
def locations(text):
    return yaml.safe_load(text)


def of_kind(locations, kind):
    return {name: pose for name, pose in locations['poses'].items() if pose['kind'] == kind}


def test_documented_numbers(locations):
    # The plan's F3 acceptance: 3 spawns, 36 pick poses, 1 drop, 3 docks.
    assert len(of_kind(locations, 'spawn')) == 3
    assert len(of_kind(locations, 'pick')) == 36
    assert len(of_kind(locations, 'drop')) == 1
    assert len(locations['docks']) == 3
    assert len(of_kind(locations, 'dock_docked')) == len(of_kind(locations, 'dock_staging')) == 3
    assert len(locations['aisles']) == 8
    assert len(locations['zones']['keepout']) == 6
    assert len(locations['zones']['speed']) == 10
    assert list(locations['routes']) == ['survey', 'smoke']


def test_header(locations, layout):
    assert locations['schema_version'] == registry.SCHEMA_VERSION
    assert locations['world'] == layout.name
    assert locations['frame_id'] == 'map'


def test_location_ids_are_unique(text, locations, layout):
    # A YAML mapping keeps the last of two equal keys without a word, so count the lines.
    lines = [line for line in text.splitlines() if line.startswith('  ') and ': {kind: ' in line]
    assert len(lines) == len(locations['poses'])
    expected = (len(layout.spawns) + len(layout.stations) + 2 * len(layout.docks)
                + len(layout.picks))
    assert len(locations['poses']) == expected


def test_robot1_spawns_at_the_origin(locations):
    # The map frame is the world frame only if mapping starts at the origin (plan 3.1).
    spawn = locations['poses']['spawn_robot1']
    assert (spawn['robot'], spawn['x'], spawn['y'], spawn['yaw']) == ('robot1', 0.0, 0.0, 0.0)


def test_poses_match_the_layout(locations, layout):
    for pose in layout.goals() + tuple(dock.docked for dock in layout.docks):
        entry = locations['poses'][pose.name]
        assert entry['x'] == pytest.approx(pose.x, abs=1e-6), pose.name
        assert entry['y'] == pytest.approx(pose.y, abs=1e-6), pose.name
        assert entry['yaw'] == pytest.approx(pose.yaw, abs=1e-6), pose.name


def test_pick_poses_name_their_aisle_and_bays(locations):
    poses = locations['poses']
    assert poses['pick_1_02']['aisle'] == '1'      # an ID, not the number 1
    assert poses['pick_1_02']['bay'] == 2
    assert poses['pick_1_02']['bays'] == ['A02', 'B02']
    assert poses['pick_1_02']['yaw'] == pytest.approx(math.pi / 2, abs=1e-6)
    # Row A has only three bays in the north block: pick_1_09 faces row B alone.
    assert poses['pick_1_09']['bays'] == ['B09']


def test_every_pick_belongs_to_one_aisle_segment(locations):
    picks = [pick for aisle in locations['aisles'].values() for pick in aisle['picks']]
    assert sorted(picks) == sorted(of_kind(locations, 'pick'))
    aisle = locations['aisles']['aisle_1_south']
    assert aisle['rows'] == ['A', 'B']
    assert aisle['width'] == pytest.approx(1.6)
    assert aisle['picks'] == [f'pick_1_0{bay}' for bay in range(1, 6)]


def test_docks_refer_to_their_poses_and_zone(locations):
    for name, dock in locations['docks'].items():
        assert locations['poses'][dock['docked']]['dock'] == name
        assert locations['poses'][dock['staging']]['dock'] == name
        assert dock['keepout'] in locations['zones']['keepout']
    dock = locations['docks']['dock_1']
    assert dock['contact'] == pytest.approx([10.0, -1.45])
    assert dock['tag'] == pytest.approx([10.0, -1.70])


def test_zones_and_routes(locations, layout):
    zones = locations['zones']
    assert zones['speed']['staff_crossing'] == {
        'limit': 0.3, 'generated': False,
        'polygon': [[14.0, -2.0], [18.0, -2.0], [18.0, 3.0], [14.0, 3.0]]}
    assert zones['speed']['aisle_1_south']['generated'] is True
    assert zones['keepout']['dock_1_bar_keepout']['generated'] is True
    assert 'limit' not in zones['keepout']['machinery']
    assert locations['routes']['smoke'] == [list(p) for p in layout.route('smoke').waypoints]
