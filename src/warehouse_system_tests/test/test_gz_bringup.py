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
Gazebo bring-up: the Phase 1 acceptance checks that need the simulator.

Launches sim.launch.py headless with hardware:=gz in /robot1, plus a static box for the
scanner to see, and checks the controllers, sensor rates, frames, the namespaced TF tree, that
the scanner sees the box where it stands and nothing inside the robot's outline, wheel and EKF
odometry against ground truth (10 m straight, a 360 degree turn in place) and that the robot
stops after cmd_vel_timeout.

Gazebo's sensors render (gpu_lidar), which needs a GPU or EGL. The CI runners have neither,
so CI sets WAREHOUSE_SKIP_GZ_TESTS=1 and these tests are reported as skipped. Run them on a
machine with a GPU: colcon test --packages-select warehouse_system_tests.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from warehouse_system_tests import ros_helpers as rh

NAMESPACE = 'robot1'
NS = f'/{NAMESPACE}'
SKIP_GZ_TESTS = os.environ.get('WAREHOUSE_SKIP_GZ_TESTS', '') == '1'
# A private gz transport partition: this test's Gazebo never exchanges messages with
# another Gazebo on the same machine, and leftover processes can be found by it.
GZ_PARTITION = f'warehouse_test_{os.getpid()}'

# The robot's numbers come from warehouse_description, never repeated here.
ROBOT = rh.description_config('robot.yaml')
DRIVE = rh.diff_drive_parameters()
LIDAR = ROBOT['lidar_front']
OUTLINE_HALF_LENGTH = ROBOT['footprint']['length'] / 2.0  # m
OUTLINE_HALF_WIDTH = ROBOT['footprint']['width'] / 2.0  # m
CMD_VEL_TIMEOUT = DRIVE['cmd_vel_timeout']  # s

# A static box for the scanner to see (test_06). In the empty world every ray would miss,
# and a blind, turned or misplaced scanner would pass the self-hit check. The box stands
# 2 m to the left of the spawn pose, clear of the drive along +x in test_07.
TARGET_X = 0.0  # m, world position of its centre
TARGET_Y = 2.0
TARGET_SIZE = 0.4  # m, square footprint
TARGET_HEIGHT = 0.5  # m, from the floor; must reach above the scan plane (checked in test_06)
_TARGET_BOX = (f'<geometry><box><size>{TARGET_SIZE} {TARGET_SIZE} {TARGET_HEIGHT}</size></box>'
               '</geometry>')
TARGET_SDF = ('<sdf version="1.9"><model name="scan_target"><static>true</static>'
              f'<link name="link"><collision name="collision">{_TARGET_BOX}</collision>'
              f'<visual name="visual">{_TARGET_BOX}</visual></link></model></sdf>')


@pytest.mark.launch_test
@unittest.skipIf(SKIP_GZ_TESTS, 'WAREHOUSE_SKIP_GZ_TESTS=1: Gazebo tests need GPU or EGL '
                                'rendering, which CI runners lack')
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('warehouse_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'hardware': 'gz', 'namespace': NAMESPACE, 'world': 'empty', 'headless': 'true',
        }.items(),
    )
    # Waits for the world, spawns the box and exits (with code 0, checked after shutdown).
    scan_target = Node(
        package='ros_gz_sim', executable='create', name='spawn_scan_target', output='screen',
        arguments=['-string', TARGET_SDF, '-name', 'scan_target', '-x', str(TARGET_X),
                   '-y', str(TARGET_Y), '-z', str(TARGET_HEIGHT / 2.0)])
    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION', GZ_PARTITION),
        sim,
        scan_target,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGazeboBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Sim time: commands must be stamped with Gazebo's clock, or cmd_vel_timeout never
        # fires (plan R1).
        cls.node = rclpy.create_node(
            'gz_bringup_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.cmd_vel = cls.node.create_publisher(TwistStamped, f'{NS}/cmd_vel', 10)
        cls.wheel = rh.Collector(cls.node, Odometry, f'{NS}/odom/wheel')
        cls.fused = rh.Collector(cls.node, Odometry, f'{NS}/odom')
        cls.truth = rh.Collector(cls.node, Odometry, f'{NS}/ground_truth/odom')
        cls.tf_static = rh.Collector(cls.node, TFMessage, f'{NS}/tf_static', rh.TF_STATIC_QOS)
        cls.last_command_stamp = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def collect(self, msg_type, topic, qos=10):
        """Return a Collector on ``topic`` that unsubscribes when the current test ends."""
        collector = rh.Collector(self.node, msg_type, topic, qos)
        self.addCleanup(collector.close)
        return collector

    def sources(self):
        return {'truth': self.truth, 'wheel': self.wheel, 'fused': self.fused}

    def wait_for_sources(self):
        """Wait until ground truth, wheel and EKF odometry have each sent a message."""
        self.assertTrue(rh.spin_until(self.node, lambda: all(
            c.last is not None for c in self.sources().values()), 15.0), 'no odometry')

    def drive(self, linear, angular, until, timeout_s):
        """Publish stamped commands every COMMAND_PERIOD until ``until()`` is true (else False)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.header.frame_id = 'base_footprint'
            command.twist.linear.x = linear
            command.twist.angular.z = angular
            self.cmd_vel.publish(command)
            self.last_command_stamp = rh.stamp_to_seconds(command.header.stamp)
            rh.spin_for(self.node, rh.COMMAND_PERIOD)
            if until():
                return True
        return False

    def brake_and_settle(self):
        """Command zero for 1 s, then wait until ground truth says the robot stands still."""
        self.drive(0.0, 0.0, until=lambda: False, timeout_s=1.0)
        self.wait_for_sources()

        def still():
            twist = self.truth.last.twist.twist
            return abs(twist.linear.x) < 0.005 and abs(twist.angular.z) < 0.005

        self.assertTrue(rh.spin_until(self.node, still, 15.0), 'the robot did not stop')
        # Every odometry source must have reported the standstill: under load the EKF's
        # messages can arrive later than ground truth's.
        stopped_at = rh.stamp_to_seconds(self.truth.last.header.stamp)
        self.assertTrue(rh.spin_until(self.node, lambda: all(
            rh.stamp_to_seconds(c.last.header.stamp) > stopped_at + 0.3
            for c in self.sources().values()), 15.0))

    def test_01_controllers_become_active(self):
        states = rh.wait_for_active_controllers(
            self.node, f'{NS}/controller_manager', rh.CONTROLLERS, timeout_s=150.0)
        for name in rh.CONTROLLERS:
            self.assertEqual(states.get(name), 'active', f'{name}: {states}')
        # Sim time is flowing (the /clock bridge works).
        self.assertTrue(rh.spin_until(
            self.node, lambda: self.node.get_clock().now().nanoseconds > 0, 10.0))

    def test_02_cmd_vel_is_stamped_and_only_the_controller_reads_it(self):
        topic = f'{NS}/cmd_vel'
        rh.spin_until(self.node, lambda: self.node.get_subscriptions_info_by_topic(topic), 10.0)
        subscribers = self.node.get_subscriptions_info_by_topic(topic)
        self.assertEqual(rh.endpoint_names(subscribers), {'diff_drive_controller'})
        self.assertEqual({s.topic_type for s in subscribers}, {'geometry_msgs/msg/TwistStamped'})

    def test_03_sensor_rates(self):
        scan = self.collect(LaserScan, f'{NS}/scan', qos_profile_sensor_data)
        imu = self.collect(Imu, f'{NS}/imu', 100)
        self.wheel.clear()
        rh.spin_for(self.node, 5.0)
        # Rates in sim time (header stamps), within 10 % (plan Phase 1 acceptance).
        rates = {'scan': rh.rate_from_stamps(scan.messages),
                 'imu': rh.rate_from_stamps(imu.messages),
                 'odom/wheel': rh.rate_from_stamps(self.wheel.messages)}
        print(f'Measured rates (sim time): {rates}')
        expected = {'scan': LIDAR['update_rate'],
                    'imu': ROBOT['imu']['update_rate'],
                    'odom/wheel': DRIVE['publish_rate']}
        for topic, rate in expected.items():
            self.assertAlmostEqual(rates[topic], rate, delta=0.1 * rate, msg=topic)

    def test_04_frames_are_not_prefixed(self):
        scan = self.collect(LaserScan, f'{NS}/scan', qos_profile_sensor_data)
        imu = self.collect(Imu, f'{NS}/imu', 100)
        self.assertTrue(rh.spin_until(
            self.node, lambda: scan.last is not None and imu.last is not None, 10.0))
        self.wait_for_sources()
        self.assertEqual(scan.last.header.frame_id, 'lidar_front_link')
        self.assertEqual(imu.last.header.frame_id, 'imu_link')
        for odometry in (self.wheel.last, self.fused.last):
            self.assertEqual(odometry.header.frame_id, 'odom')
            self.assertEqual(odometry.child_frame_id, 'base_footprint')
        # Ground truth is the world pose; the Gazebo world frame is the map frame.
        self.assertEqual(self.truth.last.header.frame_id, 'map')
        self.assertEqual(self.truth.last.child_frame_id, 'base_footprint')

    def test_05_tf_stays_in_the_namespace(self):
        tf = self.collect(TFMessage, f'{NS}/tf', rh.TF_QOS)
        global_tf = self.collect(TFMessage, '/tf', rh.TF_QOS)
        global_tf_static = self.collect(TFMessage, '/tf_static', rh.TF_STATIC_QOS)

        def tree():
            return rh.tf_parents(tf.messages, self.tf_static.messages)

        rh.spin_until(self.node, lambda: tree() == rh.EXPECTED_TF_PARENTS, 20.0)
        self.assertEqual(tree(), rh.EXPECTED_TF_PARENTS)
        rh.spin_for(self.node, 2.0)
        # diff_drive_controller owns an idle /tf publisher (enable_odom_tf: false), so the
        # topic exists, but nothing may arrive on it.
        self.assertEqual(len(global_tf.messages), 0, 'a transform arrived on the global /tf')
        self.assertEqual(len(global_tf_static.messages), 0, 'a transform arrived on /tf_static')
        publishers = self.node.get_publishers_info_by_topic(f'{NS}/tf')
        self.assertEqual(rh.endpoint_names(publishers),
                         {'robot_state_publisher', 'ekf_filter_node'})

    def test_06_scanner_sees_the_target_and_not_itself(self):
        # A box lower than the scan plane would be invisible to the scanner.
        self.assertGreater(TARGET_HEIGHT, LIDAR['height'])
        # Scanner pose in base_footprint from the static transforms (a planar chain).
        parents = rh.tf_parents(self.tf_static.messages)
        self.assertEqual(parents.get('lidar_front_link'), 'base_link')
        transforms = {t.child_frame_id: t.transform
                      for m in self.tf_static.messages for t in m.transforms}
        lidar = transforms['lidar_front_link']  # in base_link
        base = transforms['base_link']  # in base_footprint
        base_yaw = rh.yaw_of(base.rotation)
        lidar_x = (base.translation.x + math.cos(base_yaw) * lidar.translation.x
                   - math.sin(base_yaw) * lidar.translation.y)
        lidar_y = (base.translation.y + math.sin(base_yaw) * lidar.translation.x
                   + math.cos(base_yaw) * lidar.translation.y)
        lidar_yaw = base_yaw + rh.yaw_of(lidar.rotation)

        # The robot's world pose from ground truth; it has not moved yet.
        self.wait_for_sources()
        robot = self.truth.last.pose.pose
        robot_yaw = rh.yaw_of(robot.orientation)

        def returns(message):
            return sum(math.isfinite(distance) for distance in message.ranges)

        scan = self.collect(LaserScan, f'{NS}/scan', qos_profile_sensor_data)
        # The target is spawned with the launch; wait until the scanner sees it.
        self.assertTrue(rh.spin_until(
            self.node, lambda: scan.last is not None and returns(scan.last) > 0, 30.0),
            'the scanner sees nothing: is the target missing, or the scanner blind?')
        scan.clear()
        self.assertTrue(rh.spin_until(self.node, lambda: len(scan.messages) >= 20, 30.0))
        self.assertEqual(len(scan.last.ranges), LIDAR['samples'])
        # The rays cover fov_deg (a full circle): each covers one angle_increment.
        span = scan.last.angle_max - scan.last.angle_min + scan.last.angle_increment
        self.assertAlmostEqual(span, math.radians(LIDAR['fov_deg']), places=4)

        # A return counts as on the target within 5 sigma of the range noise. With hundreds of
        # returns per run, a 3 sigma bound would fail regularly by chance; 5 sigma never does.
        tolerance = 5.0 * LIDAR['noise_stddev']
        on_target = 0
        self_hits = 0
        elsewhere = []
        for message in scan.messages:
            for i, distance in enumerate(message.ranges):
                if not math.isfinite(distance):
                    continue
                # The return in base_footprint ...
                bearing = lidar_yaw + message.angle_min + i * message.angle_increment
                x = lidar_x + distance * math.cos(bearing)
                y = lidar_y + distance * math.sin(bearing)
                if abs(x) <= OUTLINE_HALF_LENGTH and abs(y) <= OUTLINE_HALF_WIDTH:
                    self_hits += 1
                # ... and in the world.
                world_x = robot.position.x + math.cos(robot_yaw) * x - math.sin(robot_yaw) * y
                world_y = robot.position.y + math.sin(robot_yaw) * x + math.cos(robot_yaw) * y
                if (abs(world_x - TARGET_X) <= TARGET_SIZE / 2.0 + tolerance
                        and abs(world_y - TARGET_Y) <= TARGET_SIZE / 2.0 + tolerance):
                    on_target += 1
                else:
                    elsewhere.append((round(world_x, 3), round(world_y, 3)))
        print(f'Scan check: {len(scan.messages)} scans, {on_target} returns on the target, '
              f'{self_hits} inside the outline, {len(elsewhere)} elsewhere')
        self.assertGreater(on_target, 0)
        self.assertEqual(self_hits, 0)
        # Any other return means the scan is not where the URDF says the scanner is (turned,
        # moved or tilted towards the floor).
        self.assertEqual(len(elsewhere), 0, f'returns off the target (x, y): {elsewhere[:10]}')

    def test_07_odometry_over_10_m_straight(self):
        self.brake_and_settle()
        start = {name: c.last for name, c in self.sources().items()}
        reached = self.drive(1.0, 0.0, timeout_s=120.0, until=lambda: math.hypot(
            *rh.planar_displacement(start['truth'], self.truth.last)) >= 10.0)
        self.assertTrue(reached, 'did not drive 10 m')
        self.brake_and_settle()
        end = {name: c.last for name, c in self.sources().items()}

        truth_forward, truth_left = rh.planar_displacement(start['truth'], end['truth'])
        distance = math.hypot(truth_forward, truth_left)
        for name in ('wheel', 'fused'):
            forward, left = rh.planar_displacement(start[name], end[name])
            error = math.hypot(forward - truth_forward, left - truth_left) / distance
            print(f'{name} odometry: {forward:.3f} m forward, {left:.3f} m left, truth '
                  f'{truth_forward:.3f} / {truth_left:.3f} m, error {100.0 * error:.2f} %')
            self.assertLess(error, 0.01, f'{name} odometry error over {distance:.2f} m')

    def test_08_odometry_over_a_360_degree_turn(self):
        self.brake_and_settle()
        first = {name: len(c.messages) - 1 for name, c in self.sources().items()}

        def turned(name):
            return rh.total_yaw_change(self.sources()[name].messages[first[name]:])

        reached = self.drive(0.0, 1.0, timeout_s=90.0,
                             until=lambda: turned('truth') >= 2.0 * math.pi)
        self.assertTrue(reached, 'did not turn 360 degrees')
        self.brake_and_settle()
        truth = turned('truth')
        for name in ('wheel', 'fused'):
            error = abs(turned(name) - truth) / truth
            print(f'{name} odometry: turned {math.degrees(turned(name)):.2f} deg, truth '
                  f'{math.degrees(truth):.2f} deg, error {100.0 * error:.2f} %')
            self.assertLess(error, 0.02, f'{name} yaw error over {math.degrees(truth):.0f} deg')

    def test_09_stops_after_cmd_vel_timeout(self):
        speed = 0.5  # m/s
        self.brake_and_settle()
        sim_start = self.node.get_clock().now().nanoseconds * 1e-9
        self.drive(speed, 0.0, timeout_s=30.0, until=lambda: (
            self.node.get_clock().now().nanoseconds * 1e-9 - sim_start >= 2.0))
        last = self.last_command_stamp
        # No zero command from here on: only cmd_vel_timeout can stop the robot. The windows
        # are derived from controllers.yaml (see cmd_vel_timeout_windows).
        (moving_from, moving_to), stop_by = rh.cmd_vel_timeout_windows(last, speed)
        self.assertTrue(rh.spin_until(
            self.node,
            lambda: rh.stamp_to_seconds(self.truth.last.header.stamp) > stop_by + 0.5, 30.0))

        def speeds(source, begin, end):
            return [m.twist.twist.linear.x for m in source.messages
                    if begin <= rh.stamp_to_seconds(m.header.stamp) <= end]

        still_moving = speeds(self.wheel, moving_from, moving_to)
        stopped_wheel = speeds(self.wheel, stop_by, stop_by + 10.0)
        stopped_truth = speeds(self.truth, stop_by, stop_by + 10.0)
        print(f'cmd_vel_timeout {CMD_VEL_TIMEOUT} s: {len(still_moving)} odom/wheel samples from '
              f'+{moving_from - last:.2f} to +{moving_to - last:.2f} s after the last command, '
              f'lowest {min(still_moving, default=float("nan")):.3f} m/s; standing still from '
              f'+{stop_by - last:.2f} s')
        # odom/wheel arrives at publish_rate: the window holds all its samples but perhaps one
        # at an edge (5 at 0.15 s and 50 Hz), and at least 3, or the check would say little.
        expected = round((moving_to - moving_from) * DRIVE['publish_rate'])
        self.assertGreaterEqual(len(still_moving), max(3, expected - 1), still_moving)
        # odom/wheel is a 0.1 s rolling mean, so braking shows in it gradually: 95 % fails a
        # robot that starts braking more than about 0.06 s before the timeout, such as one
        # that brakes as soon as one command is missing. The wheels hold the commanded speed
        # within 1 % in Gazebo and exactly on mock hardware.
        self.assertGreater(min(still_moving), 0.95 * speed, still_moving)
        self.assertTrue(stopped_wheel and max(abs(v) for v in stopped_wheel) < 0.02, stopped_wheel)
        self.assertTrue(stopped_truth and max(abs(v) for v in stopped_truth) < 0.02, stopped_truth)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        # One SIGINT from the launch must stop every process with exit code 0.
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_no_gazebo_process_is_left_behind(self):
        # Gazebo started through a shell outlives the launch's SIGINT (see
        # warehouse_gazebo/launch/gazebo.launch.py). Every process this test started carries
        # its private GZ_PARTITION in its environment; none may still be running.
        marker = f'GZ_PARTITION={GZ_PARTITION}'.encode()

        def leftovers():
            found = []
            for pid in filter(str.isdigit, os.listdir('/proc')):
                try:
                    with open(f'/proc/{pid}/environ', 'rb') as environ:
                        if marker in environ.read().split(b'\0'):
                            found.append(pid)
                except OSError:
                    continue  # exited meanwhile, or not ours
            return found

        deadline = time.monotonic() + 10.0
        while leftovers() and time.monotonic() < deadline:
            time.sleep(0.5)
        self.assertEqual(leftovers(), [], 'processes of this test are still running')
