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
Tests of the teleop window's ROS node: its namespace, and the clock it stamps commands with.

No window and no robot: nodes in this process play the robot (a cmd_vel subscriber) and the
simulation (a /clock publisher). Automatic discovery is off, so no other process sees these
nodes, and no robot that happens to run on this machine can receive a test command.
"""

import time

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from warehouse_bringup.connection import DISCOVERY_TIME
from warehouse_bringup.connection import OK
from warehouse_bringup.connection import PROBLEM
from warehouse_bringup.connection import SIM_TIME
from warehouse_bringup.connection import WAIT
from warehouse_bringup.connection import WALL_TIME
from warehouse_bringup.hold_to_drive import Command
from warehouse_bringup.teleop_window import TeleopNode

FORWARD = Command(0.3, 0.0)
SIM_SECONDS = 100  # the time on the fake simulation's clock


class Ros:
    """ROS in this test process: the teleop node, the nodes that play its peers, an executor."""

    def __init__(self):
        self.nodes = []
        self.executor = None

    def start(self, *ros_args):
        """Start ROS as `ros2 run warehouse_bringup teleop_window --ros-args ...` does."""
        rclpy.init(args=['teleop_window', '--ros-args', *ros_args])
        self.executor = SingleThreadedExecutor()
        return self.add(TeleopNode())

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def robot(self, topic):
        """Play a robot that listens on ``topic``; return the list its commands arrive in."""
        received = []
        robot = self.add(rclpy.create_node(f'fake_robot_{len(self.nodes)}'))
        robot.create_subscription(TwistStamped, topic, received.append, 10)
        return received

    def simulation(self):
        """Play a simulation: publish /clock, standing at SIM_SECONDS."""
        simulation = self.add(rclpy.create_node('fake_simulation'))
        clock = simulation.create_publisher(Clock, '/clock', 10)
        message = Clock(clock=Time(seconds=SIM_SECONDS).to_msg())
        simulation.create_timer(0.01, lambda: clock.publish(message))

    def spin(self, seconds, until=lambda: False):
        """Spin for ``seconds``, or until ``until()`` is true; return ``until()``."""
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            self.executor.spin_once(timeout_sec=0.02)
        return until()

    def stop(self):
        for node in self.nodes:
            node.destroy_node()
        if self.executor is not None:
            self.executor.shutdown()
        rclpy.try_shutdown()


@pytest.fixture
def ros(monkeypatch):
    # Read when ROS starts: this process then discovers no other process, and none finds it.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)
    ros = Ros()
    yield ros
    ros.stop()


def test_robot1_by_default(ros):
    node = ros.start()
    assert node.get_namespace() == '/robot1'
    assert node.topic == '/robot1/cmd_vel'


@pytest.mark.parametrize('namespace, topic', [('/robot2', '/robot2/cmd_vel'), ('/', '/cmd_vel')])
def test_a_namespace_remap_wins(ros, namespace, topic):
    node = ros.start('-r', f'__ns:={namespace}')
    assert node.get_namespace() == namespace
    assert node.topic == topic


def test_use_sim_time_true_means_sim_time_from_the_start(ros):
    assert ros.start('-p', 'use_sim_time:=true').chosen_clock == SIM_TIME


def test_with_nothing_in_the_graph_the_keys_send_nothing(ros):
    node = ros.start()
    assert node.status().level == WAIT  # looking for the robot
    assert not node.publish(FORWARD)
    assert ros.spin(DISCOVERY_TIME + 1.0, lambda: node.status().level == PROBLEM)
    assert node.status().text.startswith('Nothing listens on /robot1/cmd_vel')
    assert node.chosen_clock is None
    assert not node.publish(FORWARD)


def test_a_robot_in_another_namespace_is_named(ros):
    node = ros.start()
    ros.robot('/robot2/cmd_vel')
    assert ros.spin(DISCOVERY_TIME + 1.0, lambda: node.status().level == PROBLEM)
    assert '--ros-args -r __ns:=/robot2 to drive that robot' in node.status().text


def test_a_clock_publisher_means_sim_time(ros):
    node = ros.start()
    received = ros.robot('/robot1/cmd_vel')
    ros.simulation()
    assert ros.spin(DISCOVERY_TIME, lambda: node.status().level == OK)
    assert node.chosen_clock == SIM_TIME
    assert node.get_parameter('use_sim_time').value
    assert node.publish(FORWARD)
    assert ros.spin(1.0, lambda: received)
    assert received[0].header.stamp.sec == SIM_SECONDS  # sim time, not the wall clock
    assert received[0].twist.linear.x == FORWARD.linear


def test_a_robot_without_clock_means_wall_time_after_a_wait(ros):
    node = ros.start()
    received = ros.robot('/robot1/cmd_vel')
    ros.spin(0.5)
    assert node.status().level == WAIT  # choosing the clock: /clock may still turn up
    assert not node.publish(FORWARD)
    assert ros.spin(DISCOVERY_TIME + 1.0, lambda: node.status().level == OK)
    assert node.chosen_clock == WALL_TIME
    assert not node.get_parameter('use_sim_time').value
    assert node.publish(FORWARD)
    assert ros.spin(1.0, lambda: received)
    stamp = Time.from_msg(received[0].header.stamp).nanoseconds * 1e-9
    assert abs(stamp - time.time()) < 1.0
