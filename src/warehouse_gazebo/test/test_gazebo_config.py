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
Check the Gazebo-side files without starting Gazebo, so PR CI runs these tests too.

CI cannot render the scanner, so it skips the Gazebo launch test (warehouse_system_tests).
Gazebo itself is lenient: `gz sdf -k` accepts a world with a misspelt system name, a bridge
entry for a gz topic that nothing publishes bridges nothing without an error, and the bridge
skips an entry with a misspelt ROS type after only a warning. These tests read the installed
files Gazebo would load and check them against the robot description:

- every world is valid SDF (`gz sdf -k`), is named after its file, steps physics like
  empty.sdf, and loads the systems the robot needs (a world with any <plugin> loads only those);
- every bridged gz topic is one the robot publishes, with the message type it carries, and
  reaches ROS under the same relative name, as the matching ROS type;
- after Gazebo's own URDF-to-SDF conversion, the casters are still frictionless.
"""

import functools
import math
import os
import subprocess
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
from rclpy.expand_topic_name import expand_topic_name
import xacro
import yaml

SHARE = get_package_share_directory('warehouse_gazebo')
DESCRIPTION_SHARE = get_package_share_directory('warehouse_description')
NAMESPACE = '/robot1'
# The bridge's node name in spawn_robot.launch.py (it only matters for names that start with ~).
BRIDGE_NODE = 'gz_bridge'
# Every installed world, so a world added later is checked too.
WORLDS = sorted(name[:-len('.sdf')] for name in os.listdir(os.path.join(SHARE, 'worlds'))
                if name.endswith('.sdf'))

# The systems the robot relies on, as <plugin filename> -> name (README, "Systems in the
# world"). A typo in either attribute shows up only when Gazebo loads the world.
REQUIRED_SYSTEMS = {
    'gz-sim-physics-system': 'gz::sim::systems::Physics',
    'gz-sim-user-commands-system': 'gz::sim::systems::UserCommands',
    'gz-sim-scene-broadcaster-system': 'gz::sim::systems::SceneBroadcaster',
    'gz-sim-sensors-system': 'gz::sim::systems::Sensors',
    'gz-sim-imu-system': 'gz::sim::systems::Imu',
}

# The gz message type that each kind of gz sensor publishes on its <topic>.
SENSOR_MESSAGE_TYPES = {'gpu_lidar': 'gz.msgs.LaserScan', 'imu': 'gz.msgs.IMU'}

# The ROS message type that each gz message type of the robot is bridged to. A wrong
# ros_type_name never stops the bridge: it skips a pair it cannot convert with a warning, and
# turns a scan into sensor_msgs/msg/Range without one.
ROS_MESSAGE_TYPES = {
    'gz.msgs.LaserScan': 'sensor_msgs/msg/LaserScan',
    'gz.msgs.IMU': 'sensor_msgs/msg/Imu',
    'gz.msgs.Odometry': 'nav_msgs/msg/Odometry',
}


def load_yaml(path):
    with open(path) as stream:
        return yaml.safe_load(stream)


ROBOT = load_yaml(os.path.join(DESCRIPTION_SHARE, 'config', 'robot.yaml'))
BRIDGE = load_yaml(os.path.join(SHARE, 'config', 'robot_bridge.yaml'))


@functools.lru_cache(maxsize=None)
def robot_urdf():
    """Return the URDF (XML string) of a Gazebo robot in NAMESPACE."""
    document = xacro.process_file(
        os.path.join(DESCRIPTION_SHARE, 'urdf', 'robot.urdf.xacro'),
        mappings={'namespace': NAMESPACE, 'hardware': 'gz'})
    return document.toxml()


def robot_gz_topics():
    """Return {gz topic: gz message type} for the topics the robot publishes in Gazebo."""
    root = ElementTree.fromstring(robot_urdf())
    # gz sensors only: the <sensor> in the <ros2_control> block is not a gz sensor.
    topics = {sensor.findtext('topic'): SENSOR_MESSAGE_TYPES[sensor.get('type')]
              for sensor in root.findall('gazebo/sensor')}
    ground_truth = root.find("gazebo/plugin[@name='gz::sim::systems::OdometryPublisher']")
    # Gazebo loads a system by its filename and class name. With a typo in either it logs an
    # error and runs on without it, so the robot would publish no ground truth.
    assert ground_truth is not None, 'the URDF loads no gz::sim::systems::OdometryPublisher'
    assert ground_truth.get('filename') == 'gz-sim-odometry-publisher-system'
    topics[ground_truth.findtext('odom_topic')] = 'gz.msgs.Odometry'
    return topics


def is_caster(collision):
    """Return True for a sphere collision with the caster radius of robot.yaml."""
    radius = collision.findtext('geometry/sphere/radius')
    return radius is not None and math.isclose(float(radius), ROBOT['casters']['radius'])


def world_root(world):
    return ElementTree.parse(os.path.join(SHARE, 'worlds', f'{world}.sdf')).getroot()


@pytest.mark.parametrize('world', WORLDS)
def test_world_is_valid_sdf(world):
    # sdformat's own check. It catches what Gazebo would refuse to load, such as a visual and
    # a collision with the same name in one link (worlds/warehouse.sdf is generated, plan
    # R2-12), but not a misspelt system name (the next tests).
    result = subprocess.run(['gz', 'sdf', '-k', os.path.join(SHARE, 'worlds', f'{world}.sdf')],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0 and 'Valid.' in result.stdout, result.stdout + result.stderr


@pytest.mark.parametrize('world', WORLDS)
def test_world_is_named_after_its_file(world):
    # world:=<name> picks the file; tools address the running world by its name
    # (/world/<name>/...), so the two must agree.
    assert world_root(world).find('world').get('name') == world


@pytest.mark.parametrize('world', WORLDS)
def test_world_steps_physics_every_millisecond_in_real_time(world):
    # The controller manager runs at 100 Hz: each control cycle covers 10 physics steps.
    physics = world_root(world).find('world/physics')
    assert float(physics.findtext('max_step_size')) == 0.001
    assert float(physics.findtext('real_time_factor')) == 1.0


@pytest.mark.parametrize('world', WORLDS)
def test_world_loads_the_systems_the_robot_needs(world):
    systems = {plugin.get('filename'): plugin
               for plugin in world_root(world).findall('world/plugin')}
    for filename, name in REQUIRED_SYSTEMS.items():
        assert filename in systems, f'{world}.sdf does not load {filename}'
        assert systems[filename].get('name') == name, f'{world}.sdf, {filename}'
    # The engine the scanner (gpu_lidar) was verified with.
    assert systems['gz-sim-sensors-system'].findtext('render_engine') == 'ogre2'


def test_bridged_gz_topics_are_published_by_the_robot():
    published = robot_gz_topics()
    for entry in BRIDGE:
        name = entry['gz_topic_name']
        # spawn_robot.launch.py runs the bridge in the robot namespace with
        # expand_gz_topic_names: true. The bridge then expands a relative gz name exactly
        # like a ROS name (rclcpp::expand_topic_or_service_name), the rule rclpy uses too.
        gz_topic = expand_topic_name(name, BRIDGE_NODE, NAMESPACE)
        assert gz_topic in published, (
            f'gz_topic_name {name} becomes {gz_topic}, which the robot does not publish. '
            f'It publishes {sorted(published)}')
        assert entry['gz_type_name'] == published[gz_topic], gz_topic
        assert entry['ros_type_name'] == ROS_MESSAGE_TYPES[entry['gz_type_name']], gz_topic
        assert entry['direction'] == 'GZ_TO_ROS', gz_topic
        # Relative, so the ROS side lands in the robot namespace too.
        assert not entry['ros_topic_name'].startswith('/'), entry['ros_topic_name']
        # One relative name on both sides: scan is /robot1/scan in gz and in ROS.
        assert entry['ros_topic_name'] == name, entry['ros_topic_name']


def test_scan_is_bridged_with_sensor_data_qos():
    # Best effort, the QoS of high-rate sensor streams (plan section 2.3).
    scans = [entry for entry in BRIDGE if entry['ros_topic_name'] == 'scan']
    assert len(scans) == 1
    assert scans[0].get('qos_profile') == 'SENSOR_DATA'


def test_casters_stay_frictionless_after_gazebo_converts_the_urdf(tmp_path):
    # Spawning converts the URDF to SDF with sdformat, and `gz sdf -p` runs the same
    # conversion. It drops <gazebo reference> friction from a named <collision> without an
    # error (Phase 1 report, problem 3), so check its output, not the URDF.
    urdf_file = tmp_path / 'robot.urdf'
    urdf_file.write_text(robot_urdf())
    result = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf_file)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    model = ElementTree.fromstring(result.stdout)
    # Gazebo merges the casters into base_footprint and renames their collisions, so find
    # them by their shape.
    casters = [collision for collision in model.iter('collision') if is_caster(collision)]
    assert len(casters) == 2, [collision.get('name') for collision in model.iter('collision')]
    for collision in casters:
        for coefficient in ('mu', 'mu2'):
            value = collision.findtext(f'surface/friction/ode/{coefficient}')
            # Unset means Gazebo's default friction, 1.
            assert value is not None and float(value) == 0.0, (
                f"{collision.get('name')}: {coefficient} is {value}")
