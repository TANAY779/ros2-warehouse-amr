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
Check warehouse_description.robot_config: robot.yaml's values and what is derived from them.

The expected values are computed here from robot.yaml (or from a made-up robot), so the
tests keep passing when the robot changes, and they fail if the module gets the arithmetic
wrong.
"""

import ast
import copy
import dataclasses
import math

import pytest
from warehouse_description import robot_config
import yaml


@pytest.fixture(scope='module')
def robot_yaml():
    """Return robot.yaml as a plain dict, read without robot_config."""
    with open(robot_config.default_path()) as stream:
        return yaml.safe_load(stream)


def test_values_match_robot_yaml(robot_yaml):
    robot = robot_config.load()
    assert robot.revision == str(robot_yaml['revision'])
    assert robot.length == robot_yaml['footprint']['length']
    assert robot.width == robot_yaml['footprint']['width']
    assert robot.padding == robot_yaml['footprint']['padding']
    assert robot.clearance == robot_yaml['body']['clearance']
    assert robot.deck_height == robot_yaml['body']['deck_height']
    for name, value in robot_yaml['lidar_front'].items():
        assert getattr(robot.lidar, name) == value, name
    for name, value in robot_yaml['imu'].items():
        assert getattr(robot.imu, name) == value, name
    assert robot.raw == robot_yaml


def test_derived_values_of_the_installed_robot(robot_yaml):
    footprint = robot_yaml['footprint']
    half_length = footprint['length'] / 2
    half_width = footprint['width'] / 2
    pad = footprint['padding']

    robot = robot_config.load()
    assert robot.half_length == pytest.approx(half_length)
    assert robot.half_width == pytest.approx(half_width)
    assert robot.r_inscribed == pytest.approx(min(half_length, half_width))
    assert robot.r_circumscribed == pytest.approx(math.hypot(half_length, half_width))
    assert robot.padded_half_length == pytest.approx(half_length + pad)
    assert robot.padded_half_width == pytest.approx(half_width + pad)
    assert robot.r_inscribed_padded == pytest.approx(min(half_length, half_width) + pad)
    assert robot.r_circumscribed_padded == pytest.approx(
        math.hypot(half_length + pad, half_width + pad))
    assert robot.scan_plane_height == robot_yaml['lidar_front']['height']


def test_derived_values_of_a_made_up_robot(robot_yaml):
    # A robot whose answers are easy to check by hand: 1.0 x 0.6 m, padding 0.1 m.
    data = copy.deepcopy(robot_yaml)
    data['footprint'].update(length=1.0, width=0.6, padding=0.1)
    data['lidar_front']['height'] = 0.5

    robot = robot_config.from_dict(data)
    assert robot.half_length == pytest.approx(0.5)
    assert robot.half_width == pytest.approx(0.3)
    assert robot.r_inscribed == pytest.approx(0.3)
    assert robot.r_circumscribed == pytest.approx(math.sqrt(0.5**2 + 0.3**2))
    assert robot.padded_half_length == pytest.approx(0.6)
    assert robot.padded_half_width == pytest.approx(0.4)
    assert robot.r_inscribed_padded == pytest.approx(0.4)
    assert robot.r_circumscribed_padded == pytest.approx(math.sqrt(0.6**2 + 0.4**2))
    assert robot.scan_plane_height == pytest.approx(0.5)


def test_footprint_polygon():
    robot = robot_config.load()
    for padded in (False, True):
        polygon = robot.footprint_polygon(padded=padded)
        half_length = robot.padded_half_length if padded else robot.half_length
        half_width = robot.padded_half_width if padded else robot.half_width
        assert polygon[0] == pytest.approx((half_length, half_width))  # front-left
        assert sorted(abs(x) for x, _ in polygon) == pytest.approx([half_length] * 4)
        assert sorted(abs(y) for _, y in polygon) == pytest.approx([half_width] * 4)
        # Shoelace formula: positive area means counter-clockwise.
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                   in zip(polygon, polygon[1:] + polygon[:1])) / 2
        assert area == pytest.approx(4 * half_length * half_width)


def test_nav2_footprint_reads_back_as_the_unpadded_outline():
    robot = robot_config.load()
    points = yaml.safe_load(robot.nav2_footprint())
    corners = robot.footprint_polygon()
    assert len(points) == len(corners)
    for point, corner in zip(points, corners):
        assert point == pytest.approx(list(corner))


def test_missing_key_is_named(robot_yaml):
    data = copy.deepcopy(robot_yaml)
    del data['footprint']['width']
    with pytest.raises(ValueError, match='footprint.width'):
        robot_config.from_dict(data)


def test_config_is_read_only():
    robot = robot_config.load()
    with pytest.raises(dataclasses.FrozenInstanceError):
        robot.length = 1.0


def test_module_repeats_no_number_from_robot_yaml(robot_yaml):
    # One source of truth: the module may compute with 0, 1 and 2 (halving a length), but
    # any other number that also appears in robot.yaml would be a copied value.
    def yaml_numbers(node):
        if isinstance(node, dict):
            for value in node.values():
                yield from yaml_numbers(value)
        elif isinstance(node, (int, float)) and not isinstance(node, bool):
            yield node

    with open(robot_config.__file__) as stream:
        tree = ast.parse(stream.read())
    module_numbers = {
        node.value for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float))
        and not isinstance(node.value, bool)}
    copied = (module_numbers - {0, 1, 2}) & set(yaml_numbers(robot_yaml))
    assert not copied, f'robot_config.py repeats robot.yaml values: {sorted(copied)}'
