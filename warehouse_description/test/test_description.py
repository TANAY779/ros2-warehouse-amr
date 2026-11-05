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
Check the robot description against robot.yaml and the plan (sections 2.3, 2.4, 2.6).

The xacro generates the geometry from robot.yaml. These tests check what the URDF must
guarantee (frames, mass, namespaced gz topics) and the values tuned by hand in
controllers.yaml, which must agree with robot.yaml ("geometry is generated, tuning is
validated").
"""

import functools
import math
import os
import subprocess
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
from urdf_parser_py import urdf
import xacro
import yaml

SHARE = get_package_share_directory('warehouse_description')
XACRO_FILE = os.path.join(SHARE, 'urdf', 'robot.urdf.xacro')
NAMESPACE = 'robot1'

# The Phase 1 part of the TF tree (plan section 2.4) as child -> parent. The EKF adds
# odom -> base_footprint at run time.
EXPECTED_PARENTS = {
    'base_link': 'base_footprint',
    'lidar_front_link': 'base_link',
    'imu_link': 'base_link',
    'left_wheel_link': 'base_link',
    'right_wheel_link': 'base_link',
    'front_caster_link': 'base_link',
    'rear_caster_link': 'base_link',
}


def load_config(name):
    with open(os.path.join(SHARE, 'config', name)) as stream:
        return yaml.safe_load(stream)


ROBOT = load_config('robot.yaml')


@functools.lru_cache(maxsize=None)
def build(hardware, namespace=NAMESPACE):
    """Return the URDF (XML string) for one hardware option and namespace."""
    document = xacro.process_file(
        XACRO_FILE, mappings={'hardware': hardware, 'namespace': namespace})
    return document.toxml()


def parse(hardware, namespace=NAMESPACE):
    return urdf.URDF.from_xml_string(build(hardware, namespace))


def rotation_matrix(roll, pitch, yaw):
    """Return the rotation matrix of URDF roll-pitch-yaw angles, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def apply(transform, point):
    rot, offset = transform
    return [sum(rot[i][j] * point[j] for j in range(3)) + offset[i] for i in range(3)]


def link_poses(model):
    """Return {link: (rotation, position)} in the root frame, with every joint at zero."""
    identity = rotation_matrix(0.0, 0.0, 0.0)
    poses = {model.get_root(): (identity, [0.0, 0.0, 0.0])}
    pending = list(model.joints)
    while pending:
        joint = next(j for j in pending if j.parent in poses)
        pending.remove(joint)
        parent_rot, parent_pos = poses[joint.parent]
        joint_rot = rotation_matrix(*joint.origin.rpy)
        rot = [[sum(parent_rot[i][k] * joint_rot[k][j] for k in range(3)) for j in range(3)]
               for i in range(3)]
        poses[joint.child] = (rot, apply(poses[joint.parent], joint.origin.xyz))
    return poses


@pytest.mark.parametrize('hardware', ['gz', 'mock'])
def test_check_urdf_accepts_the_model(hardware, tmp_path):
    urdf_file = tmp_path / f'robot_{hardware}.urdf'
    urdf_file.write_text(build(hardware))
    result = subprocess.run(
        ['check_urdf', str(urdf_file)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr


def test_real_hardware_stops_with_a_clear_message():
    with pytest.raises(xacro.XacroException) as error:
        build('real')
    assert 'hardware:=real is reserved' in str(error.value)


def test_unknown_hardware_is_refused():
    with pytest.raises(xacro.XacroException) as error:
        build('gazebo')
    assert 'Unknown hardware:=gazebo' in str(error.value)


@pytest.mark.parametrize('hardware', ['gz', 'mock'])
def test_frames_match_the_plan(hardware):
    model = parse(hardware)
    assert model.get_root() == 'base_footprint'
    assert {joint.child: joint.parent for joint in model.joints} == EXPECTED_PARENTS


def test_base_link_sits_on_the_drive_axle():
    model = parse('mock')
    base_joint = model.joint_map['base_joint']
    assert base_joint.origin.xyz == pytest.approx([0.0, 0.0, ROBOT['drive']['wheel_radius']])


def test_mass_and_centre_of_mass_match_the_design():
    model = parse('mock')
    poses = link_poses(model)
    total_mass = 0.0
    moment = [0.0, 0.0, 0.0]
    for link in model.links:
        if link.inertial is None:
            continue
        com = apply(poses[link.name], link.inertial.origin.xyz)
        total_mass += link.inertial.mass
        moment = [m + link.inertial.mass * c for m, c in zip(moment, com)]
    com = [m / total_mass for m in moment]
    # Plan section 2.6: about 35 kg with the centre of mass 0.14 m above the floor.
    assert total_mass == pytest.approx(35.0, abs=0.5)
    assert com[2] == pytest.approx(0.14, abs=0.005)
    # Left-right and front-back symmetric, so the robot drives straight.
    assert com[0] == pytest.approx(0.0, abs=0.005)
    assert com[1] == pytest.approx(0.0, abs=0.005)


def test_wheels_collide_as_spheres():
    # Plan R13: cylinder collisions skew in-place yaw odometry; spheres do not.
    model = parse('gz')
    for side in ('left', 'right'):
        link = model.link_map[f'{side}_wheel_link']
        assert isinstance(link.collision.geometry, urdf.Sphere)
        assert link.collision.geometry.radius == pytest.approx(ROBOT['drive']['wheel_radius'])
        assert link.visual.geometry.radius == pytest.approx(ROBOT['drive']['wheel_radius'])


def test_casters_are_frictionless_in_gazebo():
    root = ElementTree.fromstring(build('gz'))
    for position in ('front', 'rear'):
        gazebo = root.find(f"gazebo[@reference='{position}_caster_link']")
        assert float(gazebo.findtext('mu1')) == 0.0
        assert float(gazebo.findtext('mu2')) == 0.0
    # Gazebo applies <gazebo reference> friction only to the collision names it generates
    # itself; a <collision name="..."> silently loses it.
    assert all(collision.get('name') is None for collision in root.iter('collision'))


def test_gz_topics_and_sensor_frames_are_namespaced():
    root = ElementTree.fromstring(build('gz'))
    sensors = {}
    for gazebo in root.findall('gazebo'):
        sensor = gazebo.find('sensor')
        if sensor is not None:
            sensors[sensor.get('name')] = (gazebo.get('reference'), sensor)
    assert set(sensors) == {'lidar_front', 'imu_sensor'}
    for reference, sensor in sensors.values():
        # gz topics are global: without the prefix all robots would share one topic (R12).
        assert sensor.findtext('topic').startswith(f'/{NAMESPACE}/')
        # The message header carries the URDF frame, not gz's scoped name (plan section 2.4).
        assert sensor.findtext('gz_frame_id') == reference
    assert sensors['lidar_front'][1].findtext('topic') == f'/{NAMESPACE}/scan'

    control = root.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    # Gazebo loads a system by its filename: a typo only logs an error, and nothing drives.
    assert control.get('filename') == 'gz_ros2_control-system'
    assert control.findtext('ros/namespace') == f'/{NAMESPACE}'
    assert control.findtext('parameters').endswith('config/controllers.yaml')

    ground_truth = root.find("gazebo/plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert ground_truth.findtext('odom_topic') == f'/{NAMESPACE}/ground_truth/odom'
    for tag in ('odom_covariance_topic', 'tf_topic'):
        assert ground_truth.findtext(tag).startswith(f'/{NAMESPACE}/')


def test_namespace_slashes_are_ignored_and_empty_namespace_works():
    assert build('gz', '/robot1/') == build('gz', 'robot1')
    root = ElementTree.fromstring(build('gz', ''))
    lidar = root.find("gazebo[@reference='lidar_front_link']/sensor")
    assert lidar.findtext('topic') == '/scan'
    control = root.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert control.find('ros') is None


def test_mock_hardware_has_no_gazebo_plugins():
    root = ElementTree.fromstring(build('mock'))
    hardware = root.find('ros2_control/hardware')
    assert hardware.findtext('plugin') == 'mock_components/GenericSystem'
    assert hardware.find("param[@name='calculate_dynamics']").text == 'true'
    assert root.find('gazebo/plugin') is None


def test_ros2_control_exports_wheels_and_imu():
    root = ElementTree.fromstring(build('gz'))
    control = root.find('ros2_control')
    for joint in ('left_wheel_joint', 'right_wheel_joint'):
        element = control.find(f"joint[@name='{joint}']")
        assert [c.get('name') for c in element.findall('command_interface')] == ['velocity']
        assert [s.get('name') for s in element.findall('state_interface')] == [
            'position', 'velocity']
    imu = control.find("sensor[@name='imu_sensor']")
    assert len(imu.findall('state_interface')) == 10


def test_scanner_field_of_view_clears_the_body():
    # gpu_lidar renders visuals, so the body in the groove must stay in the scanner's blind
    # sector (plan section 2.6: 0 self-hits). Check every corner of the set-back body.
    lidar = ROBOT['lidar_front']
    half_length = ROBOT['footprint']['length'] / 2.0 - ROBOT['body']['groove']['inset']
    half_width = ROBOT['footprint']['width'] / 2.0 - ROBOT['body']['groove']['inset']
    half_fov = lidar['fov_deg'] / 2.0
    for corner_x in (-half_length, half_length):
        for corner_y in (-half_width, half_width):
            dx, dy = corner_x - lidar['x'], corner_y - lidar['y']
            if math.hypot(dx, dy) < 1e-9:
                continue  # the scanner sits on this corner
            bearing = math.degrees(math.atan2(dy, dx)) - lidar['yaw_deg']
            bearing = (bearing + 180.0) % 360.0 - 180.0  # in the scanner frame, +-180
            assert abs(bearing) > half_fov + 1.0, (corner_x, corner_y, bearing)
    assert lidar['height'] > ROBOT['body']['groove']['bottom']
    assert lidar['height'] < ROBOT['body']['groove']['top']


def test_controllers_match_robot_yaml():
    controllers = load_config('controllers.yaml')
    # Plain keys are ignored once the controller manager runs in a namespace (plan R2).
    assert all(key.startswith('/**/') for key in controllers)
    manager = controllers['/**/controller_manager']['ros__parameters']
    drive = controllers['/**/diff_drive_controller']['ros__parameters']
    imu = controllers['/**/imu_sensor_broadcaster']['ros__parameters']
    limits = ROBOT['limits']

    assert drive['wheel_radius'] == pytest.approx(ROBOT['drive']['wheel_radius'])
    assert drive['wheel_separation'] == pytest.approx(ROBOT['drive']['wheel_separation'])

    # Unprefixed frames, and the EKF owns odom -> base_footprint (plan section 2.3, R5).
    assert drive['tf_frame_prefix_enable'] is False
    assert drive['enable_odom_tf'] is False
    assert drive['odom_frame_id'] == 'odom'
    assert drive['base_frame_id'] == 'base_footprint'
    assert min(drive['pose_covariance_diagonal']) > 0.0
    assert min(drive['twist_covariance_diagonal']) > 0.0

    linear = drive['linear']['x']
    for key in ('max_velocity', 'min_velocity', 'max_acceleration', 'max_deceleration'):
        assert linear[key] == pytest.approx(limits['linear'][key]), key
    # The same limits in reverse, with the opposite sign. An unset max_deceleration_reverse
    # would brake from reverse at max_acceleration instead (see controllers.yaml).
    assert linear['max_acceleration_reverse'] == pytest.approx(-linear['max_acceleration'])
    assert linear['max_deceleration_reverse'] == pytest.approx(-linear['max_deceleration'])
    angular = drive['angular']['z']
    assert angular['max_velocity'] == pytest.approx(limits['angular']['max_velocity'])
    assert angular['min_velocity'] == pytest.approx(-limits['angular']['max_velocity'])
    assert angular['max_acceleration'] == pytest.approx(limits['angular']['max_acceleration'])
    assert angular['max_deceleration'] == pytest.approx(-limits['angular']['max_acceleration'])
    assert angular['max_acceleration_reverse'] == pytest.approx(-angular['max_acceleration'])
    assert angular['max_deceleration_reverse'] == pytest.approx(-angular['max_deceleration'])
    # Driving at full speed while turning at full rate must stay within the wheel limit,
    # which the controller manager enforces.
    fastest_wheel = (limits['linear']['max_velocity'] + limits['angular']['max_velocity']
                     * ROBOT['drive']['wheel_separation'] / 2.0) / ROBOT['drive']['wheel_radius']
    assert fastest_wheel <= limits['wheel_max_velocity']
    assert manager['enforce_command_limits'] is True

    # Plan acceptance rates: imu 100 Hz (one message per control cycle), odom/wheel 50 Hz.
    assert manager['update_rate'] == pytest.approx(ROBOT['imu']['update_rate'])
    assert drive['publish_rate'] == pytest.approx(50.0)

    # IMU covariances = sigma^2 of the simulated noise (plan section 2.6).
    assert imu['sensor_name'] == 'imu_sensor'
    assert imu['frame_id'] == 'imu_link'
    gyro = ROBOT['imu']['gyro_noise_stddev'] ** 2
    accel = ROBOT['imu']['accel_noise_stddev'] ** 2
    assert imu['static_covariance_angular_velocity'] == pytest.approx(
        [gyro, 0.0, 0.0, 0.0, gyro, 0.0, 0.0, 0.0, gyro])
    assert imu['static_covariance_linear_acceleration'] == pytest.approx(
        [accel, 0.0, 0.0, 0.0, accel, 0.0, 0.0, 0.0, accel])
    assert min(imu['static_covariance_orientation'][0::4]) >= 100.0
