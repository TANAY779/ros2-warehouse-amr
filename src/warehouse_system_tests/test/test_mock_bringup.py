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
Mock-hardware bring-up: the Phase 1 checks that need no simulator, so they run in PR CI.

Launches sim.launch.py with hardware:=mock in /robot1, the same launch path Gazebo and real
hardware use, and checks the wiring: active controllers, the stamped cmd_vel interface,
unprefixed frames, the namespaced TF tree, that a command moves the mock wheels until
cmd_vel_timeout brakes them, and that braking keeps to the hard limit in both directions.
Mock hardware has no physics: the wheels follow the command exactly and the IMU keeps its
initial values.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from warehouse_system_tests import ros_helpers as rh

NAMESPACE = 'robot1'
NS = f'/{NAMESPACE}'

# The robot's numbers come from warehouse_description, never repeated here.
DRIVE = rh.diff_drive_parameters()
CMD_VEL_TIMEOUT = DRIVE['cmd_vel_timeout']  # s
MAX_DECELERATION = -DRIVE['linear']['x']['max_deceleration']  # m/s^2, as a positive number
FORWARD_SPEED = 0.5  # m/s, a test speed well inside the limits
REVERSE_SPEED = DRIVE['linear']['x']['min_velocity']  # m/s, full reverse (negative)


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('warehouse_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={'hardware': 'mock', 'namespace': NAMESPACE, 'rviz': 'false'}.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestMockBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mock_bringup_test')
        cls.cmd_vel = cls.node.create_publisher(TwistStamped, f'{NS}/cmd_vel', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def collect(self, msg_type, topic, qos=10):
        """Return a Collector on ``topic`` that unsubscribes when the current test ends."""
        collector = rh.Collector(self.node, msg_type, topic, qos)
        self.addCleanup(collector.close)
        return collector

    def drive_then_time_out(self, speed, collectors):
        """
        Command ``speed`` for 2 s, then stop publishing, so only cmd_vel_timeout stops the robot.

        Waits until every collector has messages from after the stop. Returns the stamp of the
        last command and the time by which the robot must stand still.
        """
        # Mock hardware runs on the wall clock (use_sim_time false), so the node's clock
        # stamps the commands correctly.
        last_stamp = None
        for _ in range(40):
            command = TwistStamped()
            command.header.stamp = self.node.get_clock().now().to_msg()
            command.header.frame_id = 'base_footprint'
            command.twist.linear.x = speed
            self.cmd_vel.publish(command)
            last_stamp = rh.stamp_to_seconds(command.header.stamp)
            rh.spin_for(self.node, 0.05)

        # No zero command: the controller brakes once the last command is older than the
        # timeout. 0.3 s of margin for the update and publish periods.
        stop_by = last_stamp + CMD_VEL_TIMEOUT + abs(speed) / MAX_DECELERATION + 0.3
        # Under load the EKF's messages can arrive later than the wheel odometry's.
        self.assertTrue(rh.spin_until(self.node, lambda: all(
            rh.stamp_to_seconds(c.last.header.stamp) > stop_by + 0.5 for c in collectors),
            20.0))
        return last_stamp, stop_by

    def test_01_controllers_become_active(self):
        states = rh.wait_for_active_controllers(
            self.node, f'{NS}/controller_manager', rh.CONTROLLERS, timeout_s=90.0)
        for name in rh.CONTROLLERS:
            self.assertEqual(states.get(name), 'active', f'{name}: {states}')

    def test_02_cmd_vel_is_stamped_and_only_the_controller_reads_it(self):
        topic = f'{NS}/cmd_vel'
        rh.spin_until(self.node, lambda: self.node.get_subscriptions_info_by_topic(topic), 10.0)
        subscribers = self.node.get_subscriptions_info_by_topic(topic)
        self.assertEqual(rh.endpoint_names(subscribers), {'diff_drive_controller'})
        self.assertEqual({s.topic_type for s in subscribers}, {'geometry_msgs/msg/TwistStamped'})

    def test_03_frames_are_not_prefixed(self):
        wheel = self.collect(Odometry, f'{NS}/odom/wheel')
        fused = self.collect(Odometry, f'{NS}/odom')
        imu = self.collect(Imu, f'{NS}/imu')
        joints = self.collect(JointState, f'{NS}/joint_states')
        self.assertTrue(rh.spin_until(self.node, lambda: all(
            c.last is not None for c in (wheel, fused, imu, joints)), 20.0))
        for odometry in (wheel.last, fused.last):
            self.assertEqual(odometry.header.frame_id, 'odom')
            self.assertEqual(odometry.child_frame_id, 'base_footprint')
        self.assertEqual(imu.last.header.frame_id, 'imu_link')
        self.assertAlmostEqual(imu.last.orientation.w, 1.0)  # the mock initial value
        self.assertEqual(set(joints.last.name), {'left_wheel_joint', 'right_wheel_joint'})

    def test_04_tf_stays_in_the_namespace(self):
        tf = self.collect(TFMessage, f'{NS}/tf', rh.TF_QOS)
        tf_static = self.collect(TFMessage, f'{NS}/tf_static', rh.TF_STATIC_QOS)
        global_tf = self.collect(TFMessage, '/tf', rh.TF_QOS)
        global_tf_static = self.collect(TFMessage, '/tf_static', rh.TF_STATIC_QOS)

        def tree():
            return rh.tf_parents(tf.messages, tf_static.messages)

        rh.spin_until(self.node, lambda: tree() == rh.EXPECTED_TF_PARENTS, 20.0)
        self.assertEqual(tree(), rh.EXPECTED_TF_PARENTS)
        rh.spin_for(self.node, 2.0)
        self.assertEqual(len(global_tf.messages), 0, 'a transform arrived on the global /tf')
        self.assertEqual(len(global_tf_static.messages), 0, 'a transform arrived on /tf_static')
        publishers = self.node.get_publishers_info_by_topic(f'{NS}/tf')
        self.assertEqual(rh.endpoint_names(publishers),
                         {'robot_state_publisher', 'ekf_filter_node'})

    def test_05_command_moves_the_robot_until_cmd_vel_timeout(self):
        wheel = self.collect(Odometry, f'{NS}/odom/wheel')
        fused = self.collect(Odometry, f'{NS}/odom')
        self.assertTrue(rh.spin_until(
            self.node, lambda: wheel.last is not None and fused.last is not None, 10.0))
        start_wheel, start_fused = wheel.last, fused.last

        last_stamp, stop_by = self.drive_then_time_out(FORWARD_SPEED, (wheel, fused))

        def speeds(begin, end):
            return [m.twist.twist.linear.x for m in wheel.messages
                    if begin <= rh.stamp_to_seconds(m.header.stamp) <= end]

        still_moving = speeds(last_stamp + 0.1, last_stamp + 0.3)
        stopped = speeds(stop_by, stop_by + 10.0)
        self.assertTrue(still_moving and min(still_moving) > 0.8 * FORWARD_SPEED, still_moving)
        self.assertTrue(stopped and max(abs(v) for v in stopped) < 0.01, stopped)

        travelled_wheel = rh.planar_displacement(start_wheel, wheel.last)[0]
        travelled_fused = rh.planar_displacement(start_fused, fused.last)[0]
        self.assertGreater(travelled_wheel, 0.5)
        # The EKF fuses odom/wheel and its output moves with it. It trails a sudden start by a
        # few centimetres, which matters on this 1 m drive; its accuracy is tested against
        # ground truth in the Gazebo test.
        self.assertGreater(travelled_fused, 0.75 * travelled_wheel)
        self.assertLess(travelled_fused, 1.25 * travelled_wheel)

    def test_06_brakes_at_the_hard_limit_in_both_directions(self):
        # Plan section 2.6: the controller brakes at max_deceleration from either direction;
        # the Phase 3 safety fields assume it. Without max_deceleration_reverse in
        # controllers.yaml, braking from reverse used max_acceleration instead (0.8 m/s^2).
        wheel = self.collect(Odometry, f'{NS}/odom/wheel')
        self.assertTrue(rh.spin_until(self.node, lambda: wheel.last is not None, 10.0))
        for speed in (FORWARD_SPEED, REVERSE_SPEED):
            last_stamp, _ = self.drive_then_time_out(speed, (wheel,))
            deceleration = rh.braking_deceleration(wheel.messages, speed, after=last_stamp)
            print(f'Braking from {speed:+.2f} m/s: {deceleration:.2f} m/s^2')
            # 20 %: enough for timing jitter on a busy CI runner, far from the 0.8 fallback.
            self.assertAlmostEqual(deceleration, MAX_DECELERATION,
                                   delta=0.2 * MAX_DECELERATION, msg=f'from {speed} m/s')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        # One SIGINT from the launch must stop every process with exit code 0.
        launch_testing.asserts.assertExitCodes(proc_info)
