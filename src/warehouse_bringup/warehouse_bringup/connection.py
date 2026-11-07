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
Which clock the teleop window stamps its commands with, and the coloured line at its top.

Plain Python with no Qt and no ROS, like hold_to_drive.py, so it can be called on its own.
TeleopNode (teleop_window.py) looks at the ROS graph five times a second and passes what it
sees to choose_clock() and connection_status().

Why the clock matters: diff_drive_controller judges a command's age from its header stamp, on
its own clock, and brakes once the newest command is cmd_vel_timeout (0.15 s) old (plan R1).
In Gazebo the controller runs on sim time; with hardware:=mock, on wall time. A wall-clock stamp
is decades ahead of sim time, so in Gazebo a command never gets old and the controller never
brakes by itself. A sim-time stamp is decades behind wall time, so mock hardware ignores every
command as too old.

The rule (choose_clock):

- use_sim_time:=true: sim time from the start.
- Otherwise, as soon as /clock has a publisher (Gazebo is running): sim time, for good.
- Otherwise, once the robot has listened for DISCOVERY_TIME and nothing publishes /clock: wall
  time (hardware:=mock). If /clock appears later after all, sim time.
- use_sim_time:=false counts as not given. False is ROS's default, so the window cannot tell
  the two apart, and with Gazebo running it switches to sim time anyway.

The keys drive only while the line is green (Status.can_send): the robot listens, the clock is
chosen and, on sim time, /clock has a publisher and its first message has arrived. Otherwise
the window publishes nothing.
"""

from dataclasses import dataclass

SIM_TIME = 'sim time'
WALL_TIME = 'wall time'

# How long the window gives DDS discovery to find the robot and /clock. It reports that nothing
# listens only after this, and chooses wall time only once the robot has listened this long
# with no /clock publisher in sight.
DISCOVERY_TIME = 2.0  # s

# The levels of the line at the top of the window: green, amber and red.
OK = 'ok'
WAIT = 'wait'
PROBLEM = 'problem'


@dataclass(frozen=True)
class Status:
    """The line at the top of the window, and whether the keys may publish."""

    level: str
    text: str
    can_send: bool = False


def choose_clock(clock, clock_published, listening_for):
    """
    Return the clock to stamp commands with: SIM_TIME, WALL_TIME, or None (not chosen yet).

    clock: the clock chosen so far (None at the start; SIM_TIME with use_sim_time:=true).
    clock_published: True if /clock has a publisher.
    listening_for: seconds since something started to listen on the command topic; None while
    nothing listens.
    """
    if clock == SIM_TIME or clock_published:
        return SIM_TIME
    if listening_for is not None and listening_for >= DISCOVERY_TIME:
        return WALL_TIME
    return clock


def connection_status(*, topic, clock, listeners, clock_publishers, sim_time_started,
                      domain_id, others=(), searching=False):
    """
    Return the Status for the line at the top of the window.

    topic: the command topic, such as /robot1/cmd_vel. clock: what choose_clock() returned.
    listeners: subscriptions on the topic. clock_publishers: publishers on /clock.
    sim_time_started: the node's clock has received a /clock message (it reads 0 until then).
    domain_id: the window's ROS_DOMAIN_ID. others: other .../cmd_vel topics that something
    listens on, such as /robot2/cmd_vel. searching: the window started less than
    DISCOVERY_TIME ago, so discovery may not have found the robot yet.
    """
    if listeners == 0:
        if searching:
            return Status(WAIT, f'Looking for the robot on {topic}. The keys send nothing yet.')
        return Status(PROBLEM, _nothing_listens(topic, domain_id, others))
    if clock is None:
        return Status(WAIT, (
            f'The robot listens on {topic}. Choosing the clock: wall time (hardware:=mock) '
            f'if nothing publishes /clock within {DISCOVERY_TIME:g} s. The keys send nothing '
            'until then.'))
    if clock == SIM_TIME and clock_publishers == 0:
        return Status(PROBLEM, (
            'Sim time, but nothing publishes /clock, so the keys send nothing. Is the '
            'simulation running? For hardware:=mock, restart this window without '
            'use_sim_time:=true.'))
    if clock == SIM_TIME and not sim_time_started:
        return Status(WAIT, (
            'Sim time: waiting for the first /clock message. The keys send nothing until '
            'then.'))
    if clock == SIM_TIME:
        stamps = 'sim time (from /clock)'
    else:
        stamps = 'wall time (nothing publishes /clock: mock hardware)'
    return Status(OK, f'Ready: the robot listens on {topic}. Commands are stamped with {stamps}.',
                  can_send=True)


def _nothing_listens(topic, domain_id, others):
    # The commands stand inside the sentences: a full stop right after one would be copied too.
    text = f'Nothing listens on {topic}, so the keys send nothing.'
    if others:
        # Found a robot in another namespace: say how to drive it.
        return text + ''.join(
            f' {other} has a listener: start this window with --ros-args -r '
            f'__ns:={namespace_of(other)} to drive that robot.' for other in others)
    return text + (
        ' Is the simulation running? Was it started from a terminal set up like this one '
        f'(ROS_DOMAIN_ID here: {domain_id})? Is the robot in another namespace? Then give '
        'it with --ros-args -r __ns:=/robotN when you start this window.')


def namespace_of(topic):
    """Return the namespace of a topic name: /robot2 for /robot2/cmd_vel, / for /cmd_vel."""
    return topic.rsplit('/', 1)[0] or '/'
