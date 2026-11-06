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

"""Tests of the teleop window's clock choice and of the line at its top: no Qt, no ROS."""

import pytest
from warehouse_bringup.connection import choose_clock
from warehouse_bringup.connection import connection_status
from warehouse_bringup.connection import DISCOVERY_TIME
from warehouse_bringup.connection import namespace_of
from warehouse_bringup.connection import OK
from warehouse_bringup.connection import PROBLEM
from warehouse_bringup.connection import SIM_TIME
from warehouse_bringup.connection import WAIT
from warehouse_bringup.connection import WALL_TIME

TOPIC = '/robot1/cmd_vel'


def status(**changes):
    """Return the status for a robot that listens, on sim time that runs, with ``changes``."""
    arguments = {'topic': TOPIC, 'clock': SIM_TIME, 'listeners': 1, 'clock_publishers': 1,
                 'sim_time_started': True, 'domain_id': 0}
    arguments.update(changes)
    return connection_status(**arguments)


# choose_clock
def test_a_clock_publisher_means_sim_time():
    assert choose_clock(None, True, None) == SIM_TIME  # even before the robot listens
    assert choose_clock(None, True, 0.0) == SIM_TIME


def test_use_sim_time_true_stays_sim_time():
    # Chosen at the start with use_sim_time:=true, or earlier on: Gazebo has stopped since.
    assert choose_clock(SIM_TIME, False, None) == SIM_TIME
    assert choose_clock(SIM_TIME, False, 10.0) == SIM_TIME


def test_wall_time_only_once_the_robot_has_listened_long_enough_without_clock():
    assert choose_clock(None, False, None) is None  # nothing listens: keep waiting
    assert choose_clock(None, False, DISCOVERY_TIME - 0.1) is None  # /clock may still appear
    assert choose_clock(None, False, DISCOVERY_TIME) == WALL_TIME


def test_wall_time_switches_to_sim_time_when_clock_appears_later():
    assert choose_clock(WALL_TIME, False, None) == WALL_TIME  # the robot left: no change
    assert choose_clock(WALL_TIME, True, 5.0) == SIM_TIME


# connection_status
def test_ready_on_sim_time():
    ready = status()
    assert (ready.level, ready.can_send) == (OK, True)
    assert ready.text == ('Ready: the robot listens on /robot1/cmd_vel. Commands are stamped '
                          'with sim time (from /clock).')


def test_ready_on_wall_time():
    ready = status(clock=WALL_TIME, clock_publishers=0)
    assert (ready.level, ready.can_send) == (OK, True)
    assert 'wall time' in ready.text and 'mock hardware' in ready.text


def test_looking_for_the_robot_at_the_start():
    looking = status(listeners=0, clock=None, searching=True)
    assert (looking.level, looking.can_send) == (WAIT, False)
    assert looking.text == 'Looking for the robot on /robot1/cmd_vel. The keys send nothing yet.'


@pytest.mark.parametrize('clock', [None, SIM_TIME, WALL_TIME])
def test_nothing_listens_names_the_topic_and_the_likely_causes(clock):
    nothing = status(listeners=0, clock=clock, domain_id=7)
    assert (nothing.level, nothing.can_send) == (PROBLEM, False)
    assert nothing.text.startswith('Nothing listens on /robot1/cmd_vel, so the keys send '
                                   'nothing.')
    assert 'Is the simulation running?' in nothing.text
    assert 'ROS_DOMAIN_ID here: 7' in nothing.text
    assert '--ros-args -r __ns:=/robotN' in nothing.text


def test_nothing_listens_names_a_robot_in_another_namespace():
    nothing = status(listeners=0, others=('/robot2/cmd_vel',))
    assert nothing.level == PROBLEM
    assert ('/robot2/cmd_vel has a listener: start this window with --ros-args -r '
            '__ns:=/robot2 to drive that robot.') in nothing.text
    assert 'Is the simulation running?' not in nothing.text


def test_choosing_the_clock_sends_nothing():
    choosing = status(clock=None, clock_publishers=0)
    assert (choosing.level, choosing.can_send) == (WAIT, False)
    assert 'Choosing the clock' in choosing.text and 'within 2 s' in choosing.text


def test_waiting_for_the_first_clock_message_sends_nothing():
    waiting = status(sim_time_started=False)
    assert (waiting.level, waiting.can_send) == (WAIT, False)
    assert 'waiting for the first /clock message' in waiting.text


def test_sim_time_without_a_clock_publisher_sends_nothing():
    # use_sim_time:=true with mock hardware, or Gazebo stopped after the window chose sim time.
    stale = status(clock_publishers=0)
    assert (stale.level, stale.can_send) == (PROBLEM, False)
    assert 'nothing publishes /clock' in stale.text
    assert 'without use_sim_time:=true' in stale.text


def test_only_ready_sends():
    # Every combination: the keys send exactly when the line is green.
    for listeners in (0, 1):
        for clock in (None, SIM_TIME, WALL_TIME):
            for clock_publishers in (0, 1):
                for started in (False, True):
                    line = status(listeners=listeners, clock=clock,
                                  clock_publishers=clock_publishers, sim_time_started=started)
                    assert line.can_send == (line.level == OK)


def test_namespace_of():
    assert namespace_of('/robot2/cmd_vel') == '/robot2'
    assert namespace_of('/fleet/robot2/cmd_vel') == '/fleet/robot2'
    assert namespace_of('/cmd_vel') == '/'
