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
Check the SLAM, AMCL and map server parameter files (plan F6; risks R2-5, R2-6, R2-14).

Reads the installed YAML files and starts nothing, so it runs in about a second:

    python3 -m pytest src/warehouse_navigation/test/test_nav_params.py

ROS 2 ignores a parameter that matches no node or no declared name, without a warning, and a
topic name resolves wherever the node runs. So these mistakes do not fail at start-up; the
robot just never localizes. The tests catch them: keys that match no node in /robot1, keys
slam_toolbox does not know, topics in the wrong namespace, frames that disagree with the EKF,
laser ranges that disagree with robot.yaml, and ground truth leaking into the robot's
configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
import pytest
from warehouse_description import robot_config
import yaml

CONFIG = os.path.join(get_package_share_directory('warehouse_navigation'), 'config')
CONFIG_FILES = sorted(name for name in os.listdir(CONFIG) if name.endswith('.yaml'))

# The sample slam_toolbox.yaml starts from (see its header).
SLAM_SAMPLE = os.path.join(get_package_share_directory('slam_toolbox'), 'config',
                           'mapper_params_online_async.yaml')
# Keys slam_toolbox 2.8.5 reads that its sample does not list ...
SLAM_ADDED = {'map_name', 'use_lifecycle_manager'}
# ... and the sample's keys it never reads.
SLAM_NOT_READ = {'min_laser_range'}


def load(name):
    """Return one of this package's config files as a dict."""
    with open(os.path.join(CONFIG, name)) as stream:
        return yaml.safe_load(stream)


def parameters(name, node):
    """Return the parameters of one node in a config file."""
    return load(name)[f'/**/{node}']['ros__parameters']


def strings_in(value):
    """Yield every string in a parsed YAML value, at any depth."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from strings_in(item)
    elif isinstance(value, list):
        for item in value:
            yield from strings_in(item)


# Fixtures: each test names the parameters it needs. A file whose key lacks /**/ then fails
# only the tests that use it (with a KeyError), next to the clear message of
# test_node_keys_match_in_any_namespace.
@pytest.fixture
def slam():
    return parameters('slam_toolbox.yaml', 'slam_toolbox')


@pytest.fixture
def amcl():
    return parameters('amcl.yaml', 'amcl')


@pytest.fixture
def map_server():
    return parameters('map_server.yaml', 'map_server')


@pytest.fixture
def ekf():
    return parameters('ekf.yaml', 'ekf_filter_node')


@pytest.fixture
def lidar():
    """Return the lidar's numbers from robot.yaml (warehouse_description.robot_config)."""
    return robot_config.load().lidar


@pytest.mark.parametrize('name', CONFIG_FILES)
def test_node_keys_match_in_any_namespace(name):
    # The robot's nodes run in /robot1. A plain `amcl:` key matches only /amcl, and a key
    # that matches no node is ignored (plan risk R2-5).
    for key in load(name):
        assert key.startswith('/**/'), (
            f'{name}: the key {key} matches only a node in /; write /**/{key}')


def test_scan_topics_are_relative(slam, amcl):
    # Relative names resolve in the node's namespace: scan is /robot1/scan.
    assert slam['scan_topic'] == 'scan'
    assert amcl['scan_topic'] == 'scan'


def test_one_shared_map_topic(slam, amcl, map_server):
    # One map for every robot, in the system scope (plan §2.3): slam_toolbox publishes it
    # while mapping, map_server while localizing, and AMCL reads it. AMCL's default, the
    # relative `map`, would be /robot1/map, where nothing publishes (plan risk R2-6).
    assert slam['map_name'] == '/map'
    assert map_server['topic_name'] == '/map'
    assert amcl['map_topic'] == '/map'


def test_frames_agree_with_the_ekf(slam, amcl, map_server, ekf):
    frames = (ekf['map_frame'], ekf['odom_frame'], ekf['base_link_frame'])
    assert frames == ('map', 'odom', 'base_footprint')
    assert (slam['map_frame'], slam['odom_frame'], slam['base_frame']) == frames
    assert (amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']) == frames
    assert map_server['frame_id'] == frames[0]


def test_laser_ranges_agree_with_robot_yaml(slam, amcl, lidar):
    assert slam['max_laser_range'] == lidar.range_max
    # slam_toolbox takes the shortest range from each scan's range_min (from robot.yaml).
    assert 'min_laser_range' not in slam, 'slam_toolbox 2.8.5 never reads min_laser_range'
    # AMCL: -1 means the scan's own range_min and range_max, which come from robot.yaml.
    assert amcl['laser_min_range'] in (-1.0, lidar.range_min)
    assert amcl['laser_max_range'] in (-1.0, lidar.range_max)


def test_slam_keys_are_the_samples_keys(slam):
    # A misspelled key would be ignored; every key must be one slam_toolbox reads.
    with open(SLAM_SAMPLE) as stream:
        sample = yaml.safe_load(stream)['slam_toolbox']['ros__parameters']
    assert set(slam) - set(sample) == SLAM_ADDED, 'a key the sample does not have'
    assert set(sample) - set(slam) == SLAM_NOT_READ, 'a key of the sample is missing'
    # A node refuses a value of the wrong type: 20 for a double fails the whole configure.
    for key in set(slam) & set(sample):
        assert type(slam[key]) is type(sample[key]), (
            f'{key}: {slam[key]!r} is a {type(slam[key]).__name__}, the sample has '
            f'{sample[key]!r}')


def test_lifecycle(slam):
    # slam_toolbox is activated by launch events, without a manager and bond (plan P2-D8).
    assert slam['use_lifecycle_manager'] is False
    for name, manager, nodes in (('amcl.yaml', 'lifecycle_manager_localization', ['amcl']),
                                 ('map_server.yaml', 'lifecycle_manager_map', ['map_server'])):
        settings = parameters(name, manager)
        assert settings['node_names'] == nodes
        assert settings['autostart'] is True


def test_amcl_starts_at_the_pose_from_the_launch(amcl):
    assert amcl['set_initial_pose'] is True
    # localization.launch.py sets initial_pose.x, .y and .yaw (the spawn pose); a value here
    # would always be overwritten.
    assert 'initial_pose' not in amcl


@pytest.mark.parametrize('name', CONFIG_FILES)
def test_no_ground_truth(name):
    # Ground truth is for tests and evaluation only: the robot never sees it (plan §2.3,
    # risk R2-14).
    for text in strings_in(load(name)):
        assert 'ground_truth' not in text, f'{name} uses {text}'
