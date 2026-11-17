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
Evaluate a navigation benchmark from its bag: goals, command age, holds, events, zones, load.

Usage::

    ros2 run warehouse_system_tests evaluate_navigation BAG [--plot OUT.png]
        [--namespace robot1]

BAG is the folder nav_benchmark wrote: the bag, and next to it results.json and measure_load's
load_summary.json (each used when present). Everything below is computed from the bag, in sim
time: the recorder ran with --use-sim-time, so a message's receive time is the sim time at
which it arrived, the clock diff_drive_controller judges command age by.

1. **Goals.** Each goal's window runs from its first status EXECUTING (or ACCEPTED) on the
   navigate_to_pose action's status topic to its first final status (SUCCEEDED, ABORTED,
   CANCELED); without that topic, from results.json's times. results.json adds the goal's index
   in the goal set, its pose, Nav2's error code, the recoveries (Nav2's own count, from the
   feedback), the straight distance and the timeout. Per goal: time, the ground-truth path
   length, and the end error: ground truth at the window's end against the goal pose.
2. **Held command age** (plan P3-D33): for consecutive messages i and i+1 on cmd_vel, the
   receive time of i+1 minus the stamp of i, judged while message i is not zero. That is how
   old the newest command is just before the next one replaces it, as the base sees it: it
   brakes at any update at which that age passes cmd_vel_timeout (0.15 s).
3. **The monitor's delay**: the receive time on cmd_vel minus the receive time on the
   collision monitor's input, for the same stamp (the monitor copies the header). The input is
   cmd_vel_mux from stage S5; until then it is cmd_vel_smoothed, the velocity smoother's output.
4. **Monitor holds**, for the stall fix that follows S3b: an input message that is not zero is
   held when the monitor sends zero for it on cmd_vel (STOP) or nothing at all (after
   stop_pub_timeout of STOP, or while inactive). Matched by stamp, each message counts for the
   time until the next input (at most MAX_INTERVAL). Per goal: the total hold time and the
   longest hold. Each hold over HOLD_REPORT seconds is listed with where the robot stood
   (ground truth), the command held, the velocity band of the collision monitor that command
   selects (from results.json's copy of the run's parameters) and the monitor's state from
   collision_monitor_state (published only when it changes, so the state is that of the last
   message before the hold, and "unknown" before the first).
5. **Events from /rosout**: the base's "Velocity command timed out. Braking." while the held
   command is not zero (judged; the same warning comes once a second whenever the robot stands
   without commands, and those are counted apart), "Ignoring the received message", the
   controller server's "Control loop missed its desired rate", "KeepoutFilter: Pose is in
   keepout zone", "Start occupied", "Velocity is not covered by any of the velocity polygons",
   and, for the stalls, the progress checker's "Failed to make progress", the controller's
   "detected collision ahead" and the recoveries the behavior server ran. The monitor's STOP and
   "invalid source" changes come from collision_monitor_state. /rosout is transient local: the
   recorder also gets the last 10 s of log lines from before it started, all with its start as
   receive time; events are counted per goal window, so those fall outside.
6. **Zones**, from the zones' true polygons in layout.yaml, as information ("no masks loaded"
   until stage S6b judges them): keepout entries (ground-truth poses, while a goal runs, at
   which the robot's unpadded outline overlaps a keepout zone) and the smallest clearance per
   zone; per goal the smallest distance between the ground-truth centre and a planning core
   (the zone grown 0.65 m); per speed zone, while the ground-truth centre is inside it, the
   largest commanded speed on cmd_vel (cap + 0.005 m/s allowed) and the largest ground-truth
   speed over 0.1 s (cap + 0.02 m/s); and the speed_limit messages at zone entries.
7. **Load**, from measure_load's samples: the real-time factor and the cores per group.

Then a [PASS] or [FAIL] line per limit of the Phase 3a acceptance (plan section 1, 3a-runs): at
least 96 % of the goals succeed (48 of 50); held command age under 0.12 s for 99.9 % and never
over 0.15 s; no base timeout while the held command is not zero; RTF mean at least 0.95; and
the run rules (no suspend, MemAvailable above 1.5 GiB, Gazebo statistics). Everything else is
information.

Exit codes: 0 every criterion passes; 1 one fails; 2 the bag cannot be read, lacks ground truth
or cmd_vel, results.json is malformed, or the plot cannot be written.
"""

import argparse
from dataclasses import dataclass
from dataclasses import field
import json
import math
import os
import sys
import uuid

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import matplotlib
from matplotlib.figure import Figure
from nav2_msgs.msg import CollisionMonitorState
import numpy as np
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import occupancy
from warehouse_layout.rules import RuleResult
from warehouse_system_tests import bags
from warehouse_system_tests import measure_load
from warehouse_system_tests import ros_helpers as rh
from warehouse_system_tests import zones

# The Phase 3a acceptance (plan section 1 and stage 3a-runs).
SUCCESS_SHARE = 48 / 50          # at least 48 of 50 goals: 96 % of the goals sent
AGE_P999_LIMIT = 0.12            # s: held command age under this for 99.9 %
AGE_MAX_LIMIT = 0.15             # s: and never over this (diff_drive_controller's timeout)
MIN_RTF = 0.95                   # mean real-time factor

# Zone checks of plan section 3.6 (information until stage S6b).
COMMAND_SPEED_MARGIN = 0.005     # m/s over a zone's cap for the commanded speed
TRUTH_SPEED_MARGIN = 0.02        # m/s over the cap for the ground-truth speed
SPEED_WINDOW = 0.1               # s: ground-truth speed from poses this far apart
NEAR_ZONE = 1.5                  # m: outline clearances are worked out within this (centre)

MOVING = 1e-6                    # a command with |v| or |w| above this is not zero
MAX_INTERVAL = 0.15              # s: the longest time one monitor input counts for (3 periods)
HOLD_REPORT = 2.0                # s: holds longer than this are listed one by one
HOLD_PLACE = 0.5                 # m: on the plot, holds this close share one label
PLOT_GAP = 0.25                  # s without a message: the plot's line breaks there

STATUS_FINAL = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'FAILED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}
STATUS_STARTED = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
MONITOR_ACTIONS = {CollisionMonitorState.DO_NOTHING: 'DO_NOTHING',
                   CollisionMonitorState.STOP: 'STOP',
                   CollisionMonitorState.SLOWDOWN: 'SLOWDOWN',
                   CollisionMonitorState.APPROACH: 'APPROACH',
                   CollisionMonitorState.LIMIT: 'LIMIT'}

# The /rosout events: (key, text in the message, the node that logs it, title in the report).
# The node is the end of the logger name: robot1.controller_server -> controller_server.
EVENTS = (
    ('base_timeout', 'Velocity command timed out', 'diff_drive_controller',
     "diff_drive_controller: 'Velocity command timed out. Braking.'"),
    ('ignored', 'Ignoring the received message', 'diff_drive_controller',
     "diff_drive_controller: 'Ignoring the received message' (older than the timeout)"),
    ('missed_rate', 'Control loop missed its desired rate', 'controller_server',
     "controller_server: 'Control loop missed its desired rate'"),
    ('keepout', 'KeepoutFilter: Pose is in keepout zone', None,
     "costmap: 'KeepoutFilter: Pose is in keepout zone'"),
    ('start_occupied', 'Start occupied', None, "planner: 'Start occupied'"),
    ('not_covered', 'Velocity is not covered by any of the velocity polygons',
     'collision_monitor',
     "collision_monitor: 'Velocity is not covered by any of the velocity polygons'"),
    ('no_progress', 'Failed to make progress', 'controller_server',
     "controller_server: 'Failed to make progress' (progress checker)"),
    ('collision_ahead', 'detected collision ahead', 'controller_server',
     "controller: 'detected collision ahead' (RPP or the Rotation Shim)"),
    ('spin', 'Running spin', 'behavior_server', 'behavior_server: recovery spin run'),
    ('wait', 'Running wait', 'behavior_server', 'behavior_server: recovery wait run'),
)

# Plot colours from the dataviz reference palette (light mode), as in evaluate_localization.
# Categorical slot 1 (blue) is the robot (its path, the commands it gets), slot 2 (orange) the
# monitor's holds; the pair passes the palette validator. A goal's result is a status: the
# fixed good and critical colours, always with a marker shape and a legend entry. The map and
# the keepout outlines stay grey, and every text wears an ink colour, never a series colour.
PATH_COLOUR = '#2a78d6'
HOLD_COLOUR = '#eb6834'
GOOD = '#0ca30c'
CRITICAL = '#d03b3b'
INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
DPI = 120


class BagError(ValueError):
    """The bag cannot be evaluated (the message says why)."""


# ----------------------------------------------------------------------------- reading


@dataclass(frozen=True, eq=False)
class Commands:
    """The messages of one TwistStamped topic: receive times, stamps (s, sim time) and twist."""

    recv: np.ndarray
    stamp: np.ndarray
    stamp_key: tuple             # per message (sec, nanosec): exact, for matching stamps
    v: np.ndarray                # linear.x [m/s]
    w: np.ndarray                # angular.z [rad/s]

    @staticmethod
    def from_rows(rows):
        rows = sorted(rows, key=lambda row: row[0])
        return Commands(np.array([r[0] for r in rows], dtype=float),
                        np.array([r[1][0] + r[1][1] * 1e-9 for r in rows], dtype=float),
                        tuple(r[1] for r in rows),
                        np.array([r[2] for r in rows], dtype=float),
                        np.array([r[3] for r in rows], dtype=float))

    def __len__(self):
        return len(self.recv)

    @property
    def moving(self):
        """Return, per message, True if it is not zero."""
        return (np.abs(self.v) > MOVING) | (np.abs(self.w) > MOVING)


@dataclass(frozen=True, eq=False)
class Truth:
    """Ground truth: header stamps (sim time), positions, yaw, and the speed over 0.1 s."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    def at(self, t):
        """Return (x, y, yaw) interpolated at the times t."""
        yaw = np.interp(t, self.t, np.unwrap(self.yaw))
        return (np.interp(t, self.t, self.x), np.interp(t, self.t, self.y),
                (yaw + math.pi) % (2.0 * math.pi) - math.pi)

    def window(self, start, end):
        """Return a boolean mask of the poses from ``start`` to ``end``."""
        return (self.t >= start) & (self.t <= end)

    def speed(self):
        """Return, per pose, the speed from the poses SPEED_WINDOW apart (m/s)."""
        x0, y0, _ = self.at(self.t - SPEED_WINDOW)
        speed = np.hypot(self.x - x0, self.y - y0) / SPEED_WINDOW
        speed[self.t - SPEED_WINDOW < self.t[0]] = 0.0
        return speed


@dataclass(frozen=True, eq=False)
class Recording:
    """What a benchmark bag holds (the topics are those of topics_for)."""

    path: str
    topics: dict
    available: frozenset
    truth: Truth
    commands: dict               # role ('cmd_vel', 'smoothed', 'mux', 'nav') -> Commands
    states: tuple                # (recv, action_type, polygon_name) of collision_monitor_state
    speed_limits: tuple          # (recv, percentage, speed_limit)
    rosout: tuple                # (recv, level, logger name, message)
    status: dict                 # goal id -> {'start': recv, 'end': recv, 'status': int}


def topics_for(namespace):
    """Return {role: topic} of the robot in ``namespace``."""
    ns = '/' + namespace.strip('/')
    return {'truth': f'{ns}/ground_truth/odom', 'cmd_vel': f'{ns}/cmd_vel',
            'smoothed': f'{ns}/cmd_vel_smoothed', 'mux': f'{ns}/cmd_vel_mux',
            'nav': f'{ns}/cmd_vel_nav', 'state': f'{ns}/collision_monitor_state',
            'speed_limit': f'{ns}/speed_limit', 'status': f'{ns}/navigate_to_pose/_action/status',
            'rosout': '/rosout'}


def bag_path(path):
    """Return the bag to open: the folder, or the one .mcap file given."""
    if not os.path.exists(path):
        raise BagError(f'{path} does not exist')
    return path


def read_recording(path, namespace='robot1'):
    """Read the bag at ``path``; raise BagError without ground truth or cmd_vel."""
    topics = topics_for(namespace)
    try:
        available = bags.topic_types(path)
    except RuntimeError as error:
        raise BagError(f'{path}: cannot read the bag: {error}') from None
    missing = [topics[role] for role in ('truth', 'cmd_vel') if topics[role] not in available]
    if missing:
        raise BagError(f'{path} has no {" and no ".join(missing)} (it has: '
                       f'{", ".join(sorted(available)) or "no topics"})')
    role_of = {topic: role for role, topic in topics.items()}
    truth, commands = [], {'cmd_vel': [], 'smoothed': [], 'mux': [], 'nav': []}
    states, limits, rosout, status = [], [], [], {}
    for topic, message, time_ns in bags.read(path, set(topics.values())):
        role = role_of[topic]
        recv = time_ns * 1e-9
        if role == 'truth':
            pose = message.pose.pose
            truth.append((rh.stamp_to_seconds(message.header.stamp), pose.position.x,
                          pose.position.y, rh.yaw_of(pose.orientation)))
        elif role in commands:
            stamp = message.header.stamp
            commands[role].append((recv, (stamp.sec, stamp.nanosec), message.twist.linear.x,
                                   message.twist.angular.z))
        elif role == 'state':
            states.append((recv, int(message.action_type), message.polygon_name))
        elif role == 'speed_limit':
            limits.append((recv, bool(message.percentage), float(message.speed_limit)))
        elif role == 'rosout':
            rosout.append((recv, int(message.level), message.name, message.msg))
        elif role == 'status':
            for item in message.status_list:
                goal_id = str(uuid.UUID(bytes=bytes(item.goal_info.goal_id.uuid)))
                entry = status.setdefault(goal_id, {'start': None, 'end': None, 'status': None})
                if item.status in STATUS_STARTED and entry['start'] is None:
                    entry['start'] = recv
                if item.status in STATUS_FINAL and entry['end'] is None:
                    entry['end'], entry['status'] = recv, int(item.status)
    if not truth or not commands['cmd_vel']:
        raise BagError(f'{path}: {topics["truth"]} or {topics["cmd_vel"]} has no messages')
    rows = np.asarray(sorted(truth), dtype=float)
    keep = np.append(np.diff(rows[:, 0]) > 0.0, True)       # one pose per stamp
    return Recording(
        path=path, topics=topics, available=frozenset(available),
        truth=Truth(*rows[keep].T),
        commands={role: Commands.from_rows(rows) for role, rows in commands.items() if rows},
        states=tuple(sorted(states)), speed_limits=tuple(sorted(limits)),
        rosout=tuple(sorted(rosout)), status=status)


def read_json(path):
    """Return a JSON file's content, None if there is no such file; BagError if malformed."""
    try:
        with open(path, encoding='utf-8') as stream:
            return json.load(stream)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        raise BagError(f'{path}: cannot read it: {error}') from None


def run_folder(path):
    """Return the folder that holds results.json and the load samples (the bag folder)."""
    return path if os.path.isdir(path) else os.path.dirname(os.path.abspath(path))


# ----------------------------------------------------------------------------- goals


@dataclass
class GoalRun:
    """One goal of the run and what the evaluation found for it."""

    number: int                  # its place in the run, from 1
    index: object                # its index in the goal set (None without results.json)
    target: object               # (x, y, yaw), or None
    start: float                 # s, sim time
    end: float
    result: str
    error: str = '-'
    recoveries: object = None
    straight: object = None
    timeout: object = None
    window_from: str = 'action status'
    path_length: float = 0.0
    end_error: object = None     # (metres, radians) from ground truth, or None
    hold_total: float = 0.0
    hold_longest: float = 0.0
    holds: list = field(default_factory=list)
    events: dict = field(default_factory=dict)
    stops: int = 0               # changes of the monitor's state to STOP for an obstacle
    invalid_source: int = 0      # changes to STOP for an invalid source
    core: object = None          # (smallest centre-to-core distance, zone name)

    @property
    def duration(self):
        return self.end - self.start

    @property
    def label(self):
        return f'goal {self.index}' if self.index is not None else f'goal #{self.number}'


def goal_runs(recording, results):
    """
    Return the goals of the run, in order, as GoalRun (empty if nothing says what they were).

    results.json gives the list; the action status in the bag gives each window (results.json's
    own times without it). Without results.json, every goal the status topic saw start is one.
    """
    status = recording.status
    runs = []
    if results is not None:
        for number, entry in enumerate(results.get('goals', []), 1):
            seen = status.get(entry.get('goal_id') or '', {})
            if seen.get('start') is not None and seen.get('end') is not None:
                start, end, source = seen['start'], seen['end'], 'action status'
            else:
                start, end, source = entry['sim_start'], entry['sim_end'], 'results.json'
            code = int(entry.get('error_code') or 0)
            error = f'{code} {entry.get("error_name", "")}'.strip() if code else '-'
            runs.append(GoalRun(
                number=number, index=entry['index'],
                target=(entry['x'], entry['y'], entry['yaw']), start=float(start),
                end=float(end), result=entry['result'], error=error,
                recoveries=entry.get('recoveries'), straight=entry.get('straight_distance_m'),
                timeout=entry.get('timeout_s'), window_from=source))
        return runs
    started = sorted((entry['start'], goal_id) for goal_id, entry in status.items()
                     if entry['start'] is not None)
    for number, (start, goal_id) in enumerate(started, 1):
        entry = status[goal_id]
        end = entry['end'] if entry['end'] is not None else float(recording.truth.t[-1])
        result = STATUS_FINAL.get(entry['status'], 'NO RESULT IN THE BAG')
        runs.append(GoalRun(number=number, index=None, target=None, start=start, end=end,
                            result=result))
    return runs


def measure_goals(recording, runs):
    """Fill each GoalRun's path length and end error from ground truth."""
    truth = recording.truth
    steps = np.concatenate(([0.0], np.hypot(np.diff(truth.x), np.diff(truth.y))))
    travelled = np.cumsum(steps)
    for run in runs:
        start, end = np.interp([run.start, run.end], truth.t, travelled)
        run.path_length = float(end - start)
        if run.target is not None:
            x, y, yaw = truth.at(run.end)
            run.end_error = (float(math.hypot(x - run.target[0], y - run.target[1])),
                             abs(rh.wrap_angle(float(yaw) - run.target[2])))


def window_of(runs, t):
    """Return the GoalRun whose window holds time t, or None."""
    for run in runs:
        if run.start <= t <= run.end:
            return run
    return None


# ----------------------------------------------------------------------------- commands


def held_ages(commands):
    """
    Return the held command ages of cmd_vel (plan P3-D33) as (times, ages).

    For consecutive messages i and i+1: receive(i+1) - stamp(i), while message i is not zero.
    ``times`` are the receive times of message i+1.
    """
    if len(commands) < 2:
        return np.zeros(0), np.zeros(0)
    judged = commands.moving[:-1]
    ages = commands.recv[1:] - commands.stamp[:-1]
    return commands.recv[1:][judged], ages[judged]


def monitor_input(recording):
    """Return (role, Commands) of the collision monitor's input: cmd_vel_mux from S5 on."""
    if 'mux' in recording.commands:
        return 'mux', recording.commands['mux']
    return 'smoothed', recording.commands.get('smoothed')


def monitor_delays(recording):
    """Return the delays receive(cmd_vel) - receive(input) for equal stamps, and the input role."""
    role, inputs = monitor_input(recording)
    if inputs is None:
        return role, np.zeros(0)
    received = dict(zip(inputs.stamp_key, inputs.recv))
    out = recording.commands['cmd_vel']
    delays = [recv - received[key] for key, recv in zip(out.stamp_key, out.recv)
              if key in received]
    return role, np.asarray(delays, dtype=float)


@dataclass
class Hold:
    """One stretch of inputs the collision monitor held (see the module docstring, step 4)."""

    start: float                 # s, sim time: the first held input's receive time
    duration: float = 0.0
    zero: float = 0.0            # of which the monitor sent zeros
    nothing: float = 0.0         # of which it sent nothing
    v: list = field(default_factory=list)
    w: list = field(default_factory=list)


def find_holds(recording, runs):
    """Return every Hold while a goal runs, and add them, and their totals, to each GoalRun."""
    _, inputs = monitor_input(recording)
    if inputs is None or len(inputs) == 0:
        return []
    out = recording.commands['cmd_vel']
    passed = dict(zip(out.stamp_key, out.moving))
    holds, current, current_run = [], None, None
    for j in range(len(inputs)):
        t = inputs.recv[j]
        run = window_of(runs, t)
        held = run is not None and inputs.moving[j] and not passed.get(inputs.stamp_key[j], False)
        if not held:
            current = None
            continue
        next_t = inputs.recv[j + 1] if j + 1 < len(inputs) else t + MAX_INTERVAL
        interval = min(next_t - t, MAX_INTERVAL, run.end - t)
        if current is None or run is not current_run:
            current, current_run = Hold(start=t), run
            holds.append(current)
            run.holds.append(current)
        current.duration += interval
        if inputs.stamp_key[j] in passed:
            current.zero += interval
        else:
            current.nothing += interval
        current.v.append(float(inputs.v[j]))
        current.w.append(float(inputs.w[j]))
    for run in runs:
        run.hold_total = sum(hold.duration for hold in run.holds)
        run.hold_longest = max((hold.duration for hold in run.holds), default=0.0)
    return holds


def band_of(bands, v, w):
    """Return the name of the first velocity band that holds (v, w), bounds included."""
    for band in bands:
        if (band['linear_min'] <= v <= band['linear_max']
                and band['theta_min'] <= w <= band['theta_max']):
            return band['band']
    return 'no band'


def state_text(state):
    """Return a collision_monitor_state as text, for example "STOP 'velocity_fields'"."""
    _, action, polygon = state
    name = MONITOR_ACTIONS.get(action, f'action {action}')
    return f'{name} {polygon!r}' if polygon else name


def hold_states(states, hold):
    """
    Return the monitor's state during a hold as text: its state at the start, then any changes.

    The monitor publishes a state message in the same step as its output, so the message that
    goes with the hold's first input arrives up to the monitor's delay after that input:
    messages up to MAX_INTERVAL after the start count as the state at the start.
    """
    start = hold.start + MAX_INTERVAL
    before = [state for state in states if state[0] <= start]
    during = [state for state in states if start < state[0] <= hold.start + hold.duration]
    text = (state_text(before[-1]) if before
            else 'unknown (no collision_monitor_state message before it)')
    if during:
        text += ', then ' + ', then '.join(f'{state_text(state)} at {state[0]:.2f} s'
                                           for state in during)
    return text


# ----------------------------------------------------------------------------- events


def count_events(recording, runs):
    """
    Count the /rosout events per goal window; return what falls outside any goal.

    The base timeout is split: while the held command is not zero (the newest cmd_vel received
    before the log line), or while standing without commands.
    """
    out = recording.commands['cmd_vel']
    outside = {}
    for run in runs:
        run.events = {}
    for recv, level, name, text in recording.rosout:
        node = name.rsplit('.', 1)[-1]
        for key, needle, logger, _ in EVENTS:
            if needle not in text or (logger is not None and node != logger):
                continue
            if key == 'base_timeout':
                newest = np.searchsorted(out.recv, recv, side='right') - 1
                moving = newest >= 0 and bool(out.moving[newest])
                key = 'base_timeout' if moving else 'base_timeout_standing'
            run = window_of(runs, recv)
            counts = run.events if run is not None else outside
            counts[key] = counts.get(key, 0) + 1
    # The monitor's changes of state.
    previous = None
    for recv, action, polygon in recording.states:
        changed = (action, polygon) != previous
        previous = (action, polygon)
        if not changed or action != CollisionMonitorState.STOP:
            continue
        run = window_of(runs, recv)
        if run is None:
            continue
        if polygon == 'invalid source':
            run.invalid_source += 1
        else:
            run.stops += 1
    return outside


# ----------------------------------------------------------------------------- zones


@dataclass
class ZoneReport:
    """The zone metrics (information until stage S6b)."""

    entries: int = 0                         # poses overlapping a keepout zone during goals
    entry_zones: dict = field(default_factory=dict)
    clearance: dict = field(default_factory=dict)       # zone -> smallest outline clearance
    speeds: list = field(default_factory=list)          # one dict per speed zone
    speed_limit_messages: int = 0
    zone_entries: int = 0                    # entries of the centre into a speed zone


def zone_metrics(recording, runs, layout, robot):
    """Compute the zone metrics of plan section 3.6 from the zones' true polygons."""
    report = ZoneReport()
    truth = recording.truth
    during = np.zeros(len(truth.t), dtype=bool)
    for run in runs:
        during |= truth.window(run.start, run.end)
    xs, ys, yaws = truth.x[during], truth.y[during], truth.yaw[during]
    for zone in layout.keepouts:
        centre = zones.polygon_distance(xs, ys, zone.polygon)
        near = centre < NEAR_ZONE
        if near.any():
            clearance = zones.outline_clearance(xs[near], ys[near], yaws[near],
                                                robot.half_length, robot.half_width,
                                                zone.polygon)
            report.clearance[zone.name] = float(clearance.min())
            overlapping = int(np.count_nonzero(clearance <= 0.0))
            if overlapping:
                report.entries += overlapping
                report.entry_zones[zone.name] = overlapping
        else:
            report.clearance[zone.name] = None
    for run in runs:
        mask = truth.window(run.start, run.end)
        best = None
        for zone in layout.keepouts:
            if not mask.any():
                break
            distance = float(zones.polygon_distance(truth.x[mask], truth.y[mask],
                                                    zone.polygon).min()) - zones.PLANNING_CORE
            if best is None or distance < best[0]:
                best = (distance, zone.name)
        run.core = best
    # Speed zones.
    speed = truth.speed()
    out = recording.commands['cmd_vel']
    out_x, out_y, _ = truth.at(out.recv)
    out_during = np.zeros(len(out), dtype=bool)
    for run in runs:
        out_during |= (out.recv >= run.start) & (out.recv <= run.end)
    report.speed_limit_messages = len(recording.speed_limits)
    for zone in layout.speed_zones:
        inside = zones.inside_polygon(truth.x, truth.y, zone.polygon) & during
        entries = int(np.count_nonzero(np.diff(inside.astype(int)) == 1))
        report.zone_entries += entries
        commanded = zones.inside_polygon(out_x, out_y, zone.polygon) & out_during
        report.speeds.append({
            'zone': zone.name, 'cap': zone.limit, 'entries': entries,
            'samples': int(np.count_nonzero(inside)),
            'truth_max': float(speed[inside].max()) if inside.any() else None,
            'truth_over': int(np.count_nonzero(speed[inside] > zone.limit + TRUTH_SPEED_MARGIN)),
            'command_max': float(np.abs(out.v[commanded]).max()) if commanded.any() else None,
            'command_over': int(np.count_nonzero(
                np.abs(out.v[commanded]) > zone.limit + COMMAND_SPEED_MARGIN)),
        })
    return report


# ----------------------------------------------------------------------------- load


def load_numbers(folder):
    """Return measure_load's summary of the run (a dict), or None without load samples."""
    summary = read_json(os.path.join(folder, 'load_summary.json'))
    if summary is not None:
        return summary
    # nav_benchmark stopped before measure_load wrote its summary: rebuild the RTF from the
    # samples.
    path = os.path.join(folder, 'load.jsonl')
    try:
        with open(path, encoding='utf-8') as stream:
            records = [json.loads(line) for line in stream if line.strip()]
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        raise BagError(f'{path}: cannot read it: {error}') from None
    timed = [record for record in records if record.get('rtf') is not None]
    if not timed:
        return {'samples': len(records), 'warnings': ['no Gazebo statistics in load.jsonl']}
    return {'samples': len(records),
            'rtf_mean': sum(r['sim_s'] for r in timed) / sum(r['real_s'] for r in timed),
            'rtf_worst_window': measure_load.worst_rtf(records, measure_load.RTF_WINDOW),
            'warnings': ['load_summary.json is missing: RTF rebuilt from load.jsonl']}


# ----------------------------------------------------------------------------- the evaluation


@dataclass(eq=False)
class Evaluation:
    """Everything evaluate() found."""

    recording: Recording
    results: object              # results.json's content, or None
    runs: list
    ages: tuple                  # (times, ages) of the held command age
    delay_input: str
    delays: np.ndarray
    holds: list
    outside_events: dict
    zones: ZoneReport
    load: object                 # measure_load's summary, or None
    bands: list
    layout: object               # the warehouse layout (zones, map name)


def evaluate(path, namespace='robot1'):
    """Read the bag at ``path`` and what lies next to it; return an Evaluation."""
    recording = read_recording(bag_path(path), namespace)
    folder = run_folder(path)
    results = read_json(os.path.join(folder, 'results.json'))
    runs = goal_runs(recording, results)
    measure_goals(recording, runs)
    holds = find_holds(recording, runs)
    outside = count_events(recording, runs)
    robot = robot_config.load()
    layout = model.load(robot=robot)
    delay_input, delays = monitor_delays(recording)
    return Evaluation(
        recording=recording, results=results, runs=runs,
        ages=held_ages(recording.commands['cmd_vel']), delay_input=delay_input,
        delays=delays, holds=holds, outside_events=outside,
        zones=zone_metrics(recording, runs, layout, robot), load=load_numbers(folder),
        bands=(results or {}).get('collision_monitor_bands') or [], layout=layout)


def _criterion(rule, title, passed, summary, advice):
    """Return a RuleResult: the same [PASS] / [FAIL] lines as evaluate_localization."""
    return RuleResult(rule, title, () if passed else (advice,), summary)


def criteria(evaluation):
    """Return one RuleResult per limit of the Phase 3a acceptance (module docstring)."""
    runs = evaluation.runs
    results = []
    sent = len(runs)
    needed = math.ceil(SUCCESS_SHARE * sent - 1e-9)
    succeeded = sum(run.result == 'SUCCEEDED' for run in runs)
    title = (f'goals succeed: at least {100 * SUCCESS_SHARE:.0f} % of those sent '
             '(Phase 3a: 48 of 50)')
    if not sent:
        results.append(_criterion('goals', title, False, 'no goals in the bag or results.json',
                                  'run nav_benchmark, which records the goals'))
    else:
        failed = ', '.join(f'{run.label} {run.result}' for run in runs
                           if run.result != 'SUCCEEDED')
        results.append(_criterion(
            'goals', title, succeeded >= needed,
            f'{succeeded} of {sent} ({100 * succeeded / sent:.0f} %; {needed} needed)',
            f'not succeeded: {failed}: see the per-goal table and the holds'))

    _, ages = evaluation.ages
    if len(ages) == 0:
        for rule, title in (('age_p999', 'held command age'), ('age_max', 'held command age')):
            results.append(_criterion(rule, title, False, 'no moving command on cmd_vel',
                                      'the robot never moved: is this a benchmark bag?'))
    else:
        p999 = float(np.percentile(ages, 99.9))
        worst = float(ages.max())
        over = int(np.count_nonzero(ages > AGE_MAX_LIMIT))
        results.append(_criterion(
            'age_p999', f'held command age under {AGE_P999_LIMIT:.2f} s for 99.9 % of the '
            'moving commands', p999 < AGE_P999_LIMIT,
            f'p99.9 {p999:.3f} s over {len(ages)} commands',
            'commands reach the base late: check the smoother stamps and rates (plan 3.1), '
            'then the RTF'))
        results.append(_criterion(
            'age_max', f'held command age never over {AGE_MAX_LIMIT:.2f} s (cmd_vel_timeout) '
            'while the command is not zero', worst <= AGE_MAX_LIMIT,
            f'max {worst:.3f} s, {over} over {AGE_MAX_LIMIT:.2f} s',
            f'{over} commands were held longer than the timeout: the base braked between them'))

    timeouts = sum(run.events.get('base_timeout', 0) for run in runs) + \
        evaluation.outside_events.get('base_timeout', 0)
    standing = sum(run.events.get('base_timeout_standing', 0) for run in runs) + \
        evaluation.outside_events.get('base_timeout_standing', 0)
    results.append(_criterion(
        'base_timeout', "no 'Velocity command timed out' while the held command is not zero",
        timeouts == 0, f'{timeouts} while moving ({standing} more while standing, not counted)',
        'the base braked on a stale command: see the held command age'))

    load = evaluation.load
    title = f'RTF mean at least {MIN_RTF:.2f} (measure_load)'
    if load is None:
        results.append(_criterion('rtf', title, False, 'no load samples next to the bag',
                                  'run the benchmark with nav_benchmark, which runs measure_load'))
    else:
        if load.get('rtf_mean') is None:
            results.append(_criterion('rtf', title, False, 'no Gazebo statistics in the samples',
                                      'measure_load must see the same GZ_PARTITION as Gazebo'))
        else:
            worst = load.get('rtf_worst_window')
            worst_text = '' if worst is None else f', worst 10 s {worst:.3f}'
            results.append(_criterion(
                'rtf', title, load['rtf_mean'] >= MIN_RTF,
                f'{load["rtf_mean"]:.3f}{worst_text}',
                'the simulation ran slow: close other programs, check the power profile'))
        warnings = load.get('warnings') or []
        results.append(_criterion(
            'run_rules', 'the run counts: Gazebo statistics, no suspend, MemAvailable above '
            '1.5 GiB (measure_load)', not warnings, '; '.join(warnings) or 'yes',
            'repeat the run (plan section 3.6, run rules)'))
    return tuple(results)


# ----------------------------------------------------------------------------- the report


def report(evaluation, results):
    """Return the text report: information, then the [PASS] / [FAIL] lines."""
    rec, runs = evaluation.recording, evaluation.runs
    topics = rec.topics
    lines = [f'bag {rec.path}: {len(rec.truth.t)} ground-truth poses '
             f'({rec.truth.t[-1] - rec.truth.t[0]:.1f} s of sim time), '
             f'{len(rec.commands["cmd_vel"])} commands on {topics["cmd_vel"]}, '
             f'{len(rec.rosout)} /rosout lines']
    lines += _source_lines(evaluation)
    lines += _goal_lines(evaluation)
    lines += _hold_lines(evaluation)
    lines += _command_lines(evaluation)
    lines += _event_lines(evaluation)
    lines += _zone_lines(evaluation)
    lines += _load_lines(evaluation)
    lines += [str(result) for result in results]
    passed = sum(result.passed for result in results)
    lines.append(f'{passed} of {len(results)} navigation checks pass')
    return '\n'.join(lines)


def _source_lines(evaluation):
    results = evaluation.results
    if results is None:
        return ['results.json: none next to the bag: goal indices, poses and error codes are '
                'unknown; goals and results come from the action status in the bag']
    goal_set = results.get('goal_set', {})
    lines = [f'results.json: {results.get("status")} run of {results.get("requested")} goals '
             f'from goal {results.get("first")} of {os.path.basename(goal_set.get("file", "?"))}'
             f' (seed {goal_set.get("seed")}), started {results.get("started")}']
    parameters = results.get('nav2_parameters') or {}
    smoother = parameters.get('velocity_smoother') or {}
    monitor = parameters.get('collision_monitor') or {}
    progress = parameters.get('controller_server') or {}
    if parameters:
        bands = ', '.join(f'{band["band"]} {band["linear_min"]:g}..{band["linear_max"]:g} m/s'
                          for band in evaluation.bands)
        lines.append(f'configuration (read from the nodes): smoother feedback '
                     f'{smoother.get("feedback")}, velocity_timeout '
                     f'{smoother.get("velocity_timeout")} s; monitor input '
                     f'{monitor.get("cmd_vel_in_topic")}, stop_pub_timeout '
                     f'{monitor.get("stop_pub_timeout")} s; progress checker '
                     f'{progress.get("progress_checker.required_movement_radius")} m in '
                     f'{progress.get("progress_checker.movement_time_allowance")} s')
        lines.append(f'  velocity bands: {bands or "none read"}')
    return lines


def _goal_lines(evaluation):
    runs = evaluation.runs
    if not runs:
        return ['Goals: none (no results.json and no navigate_to_pose status in the bag)']
    lines = ['', 'Goals (time from the action status in the bag; path and end error from '
                 'ground truth; hold = time the collision monitor held a moving command):',
             '  goal  result      error                        time [s]  path [m]  straight '
             '[m]  recoveries  end error [m] [rad]  hold [s]  longest [s]  holds > '
             f'{HOLD_REPORT:g} s']
    for run in runs:
        end = ('       -      -' if run.end_error is None
               else f'{run.end_error[0]:8.3f} {run.end_error[1]:6.3f}')
        straight = '-' if run.straight is None else f'{run.straight:.2f}'
        recoveries = '-' if run.recoveries is None else str(run.recoveries)
        index = run.index if run.index is not None else f'#{run.number}'
        long_holds = sum(hold.duration > HOLD_REPORT for hold in run.holds)
        lines.append(f'  {index!s:>4}  {run.result:<10s}  {run.error:<27s}  '
                     f'{run.duration:8.1f}  {run.path_length:8.2f}  {straight:>12}  '
                     f'{recoveries:>10}     {end}  {run.hold_total:8.1f}  '
                     f'{run.hold_longest:11.1f}  {long_holds:8d}')
    succeeded = [run for run in runs if run.result == 'SUCCEEDED']
    total_time = sum(run.duration for run in runs)
    lines.append(f'  {len(succeeded)} of {len(runs)} succeeded; {total_time:.1f} s and '
                 f'{sum(run.path_length for run in runs):.1f} m in all; held '
                 f'{sum(run.hold_total for run in runs):.1f} s '
                 f'({100 * sum(run.hold_total for run in runs) / max(total_time, 1e-9):.0f} % of '
                 'the goal time)')
    if succeeded:
        errors = [run.end_error[0] for run in succeeded if run.end_error is not None]
        if errors:
            lines.append(f'  end error of the goals that succeeded: mean {np.mean(errors):.3f} '
                         f'm, max {max(errors):.3f} m')
    windows = {run.window_from for run in runs}
    if windows != {'action status'}:
        lines.append('  some windows come from results.json: the bag has no action status '
                     'for those goals')
    return lines


def _hold_lines(evaluation):
    rec = evaluation.recording
    role, inputs = monitor_input(rec)
    if inputs is None:
        return ['', f'Monitor holds: no {rec.topics["smoothed"]} in the bag, so none measured']
    lines = ['', f'Monitor holds over {HOLD_REPORT:g} s (input {rec.topics[role]}: moving, '
                 f'output {rec.topics["cmd_vel"]}: zero or nothing), with where the robot stood '
                 '(ground truth), the command held and the band it selects:']
    listed = 0
    for run in evaluation.runs:
        for hold in run.holds:
            if hold.duration <= HOLD_REPORT:
                continue
            listed += 1
            x, y, yaw = rec.truth.at(hold.start)
            x_end, y_end, _ = rec.truth.at(hold.start + hold.duration)
            moved = math.hypot(float(x_end - x), float(y_end - y))
            v, w = np.median(hold.v), np.median(hold.w)
            band = band_of(evaluation.bands, v, w) if evaluation.bands else 'bands unknown'
            lines.append(
                f'  {run.label}: {hold.duration:5.1f} s from {hold.start:8.2f} s at x '
                f'{float(x):6.2f}, y {float(y):6.2f}, yaw {float(yaw):5.2f} (moved '
                f'{moved:.2f} m); held v {min(hold.v):.2f}..{max(hold.v):.2f} m/s, w '
                f'{min(hold.w):+.2f}..{max(hold.w):+.2f} rad/s (median {v:.2f}, {w:+.2f}: band '
                f'{band}); zero {hold.zero:.1f} s, nothing {hold.nothing:.1f} s; monitor state: '
                f'{hold_states(rec.states, hold)}')
    if not listed:
        lines.append('  none')
    total = sum(hold.duration for hold in evaluation.holds)
    longest = max((hold.duration for hold in evaluation.holds), default=0.0)
    lines.append(f'  all holds while goals ran: {len(evaluation.holds)}, {total:.1f} s in all, '
                 f'the longest {longest:.1f} s')
    return lines


def _command_lines(evaluation):
    rec = evaluation.recording
    _, ages = evaluation.ages
    lines = ['', 'Commands:']
    if len(ages):
        lines.append(f'  held command age on {rec.topics["cmd_vel"]} (receive of the next minus '
                     f'stamp, while not zero): {len(ages)} commands, median '
                     f'{np.median(ages):.3f} s, p99 {np.percentile(ages, 99):.3f} s, p99.9 '
                     f'{np.percentile(ages, 99.9):.3f} s, max {ages.max():.3f} s, '
                     f'{int(np.count_nonzero(ages > AGE_MAX_LIMIT))} over {AGE_MAX_LIMIT:.2f} s')
    role, delays = evaluation.delay_input, evaluation.delays
    note = ('' if role == 'mux' else
            f' ({rec.topics["mux"]} does not exist until stage S5, so measured from '
            f'{rec.topics["smoothed"]})')
    if len(delays):
        lines.append(f'  monitor delay, receive on cmd_vel minus receive on its input for the '
                     f'same stamp{note}: {len(delays)} messages, median '
                     f'{np.median(delays) * 1000:.0f} ms, p99 '
                     f'{np.percentile(delays, 99) * 1000:.0f} ms, max '
                     f'{delays.max() * 1000:.0f} ms')
    else:
        lines.append(f'  monitor delay: no messages with equal stamps{note}')
    return lines


def _event_lines(evaluation):
    runs = evaluation.runs
    lines = ['', 'Events from /rosout while goals ran (per goal where any), and the monitor\'s '
                 'changes to STOP (collision_monitor_state):']
    titles = {key: title for key, _, _, title in EVENTS}
    titles['base_timeout'] = titles['base_timeout'] + ' while the held command is not zero'
    titles['base_timeout_standing'] = ("diff_drive_controller: 'Velocity command timed out' "
                                       'while standing without commands (not counted)')
    for key in [key for key, _, _, _ in EVENTS] + ['base_timeout_standing']:
        total = sum(run.events.get(key, 0) for run in runs)
        where = ', '.join(f'{run.label} {run.events[key]}' for run in runs
                          if run.events.get(key))
        lines.append(f'  {total:5d}  {titles[key]}{f" ({where})" if where else ""}')
    stops = sum(run.stops for run in runs)
    invalid = sum(run.invalid_source for run in runs)
    where = ', '.join(f'{run.label} {run.stops}' for run in runs if run.stops)
    lines.append(f'  {stops:5d}  collision monitor: STOP for an obstacle'
                 f'{f" ({where})" if where else ""}')
    lines.append(f'  {invalid:5d}  collision monitor: STOP for an invalid source')
    outside = evaluation.outside_events
    if outside:
        text = ', '.join(f'{key} {count}' for key, count in sorted(outside.items()))
        lines.append(f'  outside the goals (before the first, between goals, after the last; '
                     f'the recorder also gets the last 10 s of /rosout from before it '
                     f'started): {text}')
    return lines


def _zone_lines(evaluation):
    report_ = evaluation.zones
    lines = ['', 'Zones from the true polygons of layout.yaml: information only, no masks '
                 'loaded (judged from stage S6b on):']
    clearances = ', '.join(f'{name} {value:.2f} m' if value is not None
                           else f'{name} > {NEAR_ZONE - 0.4:g} m'
                           for name, value in report_.clearance.items())
    entries = ', '.join(f'{name} {count}' for name, count in report_.entry_zones.items())
    lines.append(f'  keepout entries (outline over a zone while a goal ran): {report_.entries} '
                 f'poses{f" ({entries})" if entries else ""}')
    lines.append(f'  smallest outline clearance per keepout zone: {clearances}')
    cores = [(run, run.core) for run in evaluation.runs if run.core is not None]
    if cores:
        text = ', '.join(f'{run.label} {core[0]:+.2f} ({core[1]})' for run, core in cores)
        lines.append(f'  smallest distance from the centre to a planning core '
                     f'({zones.PLANNING_CORE:.2f} m round a zone) per goal [m]: {text}')
    for zone in report_.speeds:
        if not zone['samples']:
            lines.append(f'  speed zone {zone["zone"]} (cap {zone["cap"]:g} m/s): not entered')
            continue
        lines.append(
            f'  speed zone {zone["zone"]} (cap {zone["cap"]:g} m/s): {zone["entries"]} entries; '
            f'commanded max {zone["command_max"] or 0.0:.2f} m/s ({zone["command_over"]} '
            f'commands over cap + {COMMAND_SPEED_MARGIN:g}), ground truth max '
            f'{zone["truth_max"]:.2f} m/s ({zone["truth_over"]} poses over cap + '
            f'{TRUTH_SPEED_MARGIN:g})')
    limit_note = ('' if report_.speed_limit_messages
                  else ' (no speed filter until stage S6b)')
    lines.append(f'  speed_limit: {report_.speed_limit_messages} messages for '
                 f'{report_.zone_entries} zone entries{limit_note}')
    return lines


def _load_lines(evaluation):
    load = evaluation.load
    if load is None:
        return ['', 'Load: no load_summary.json or load.jsonl next to the bag']
    lines = ['', f'Load (measure_load, {load.get("samples", 0)} samples over '
                 f'{load.get("duration", 0.0):.0f} s):']
    if load.get('rtf_mean') is not None:
        worst = load.get('rtf_worst_window')
        lines.append(f'  RTF {load["rtf_mean"]:.3f} mean'
                     + ('' if worst is None else f', {worst:.3f} worst 10 s'))
    groups = load.get('groups') or {}
    if groups:
        cores = ', '.join(f'{values["label"]} {values["cores_mean"]:.2f}'
                          for key, values in groups.items() if key != measure_load.SELF)
        lines.append(f'  cores (mean): {cores}; run total {load.get("run_cores_mean", 0):.2f}, '
                     f'machine {load.get("machine_cores_mean", 0):.2f}')
    if load.get('power'):
        lines.append(f'  power: {load["power"]}')
    return lines


# ----------------------------------------------------------------------------- the plot


def draw(path, evaluation, title):
    """
    Draw the run and save it as a PNG.

    Left: the SLAM map seen from above with the keepout zones, the ground-truth path, the goals
    (numbered, coloured by result) and the holds over HOLD_REPORT seconds (circles sized by
    their time). Right, over sim time: the commanded speed in and out of the collision monitor
    with the goal windows and holds, and the held command age with its two limits.
    """
    with matplotlib.rc_context({'font.size': 8, 'axes.edgecolor': AXIS,
                                'axes.labelcolor': INK_SECONDARY, 'xtick.color': INK_MUTED,
                                'ytick.color': INK_MUTED, 'axes.titlecolor': INK}):
        figure = Figure(figsize=(13.0, 7.0), dpi=DPI)
        grid = figure.add_gridspec(2, 2, width_ratios=[1.25, 1.0], hspace=0.3, wspace=0.16,
                                   left=0.05, right=0.985, top=0.9, bottom=0.17)
        _draw_map(figure.add_subplot(grid[:, 0]), evaluation.layout, evaluation)
        speed_ax = figure.add_subplot(grid[0, 1])
        age_ax = figure.add_subplot(grid[1, 1], sharex=speed_ax)
        _draw_speed(speed_ax, evaluation)
        _draw_ages(age_ax, evaluation)
        figure.suptitle(title, x=0.05, y=0.975, ha='left', fontsize=11, color=INK)
        figure.savefig(path, dpi=DPI, facecolor='white')


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)


def _draw_map(ax, layout, evaluation):
    rec = evaluation.recording
    try:
        share = get_package_share_directory('warehouse_navigation')
        grid_map = occupancy.load(os.path.join(share, 'maps', f'{layout.name}.yaml'))
        # The map stays in the background: free white, unknown a light grey, walls mid grey.
        shades = np.full(grid_map.cells.shape, 1.0)
        shades[grid_map.cells == occupancy.UNKNOWN] = 0.93
        shades[grid_map.cells == occupancy.OCCUPIED] = 0.55
        x0, y0, _ = grid_map.origin
        ax.imshow(shades, cmap='gray', vmin=0.0, vmax=1.0, origin='lower',
                  extent=(x0, x0 + grid_map.width * grid_map.resolution,
                          y0, y0 + grid_map.height * grid_map.resolution),
                  interpolation='nearest')
    except (OSError, ValueError, LookupError):
        pass                      # the plot works without the map
    for number, zone in enumerate(layout.keepouts):
        corners = np.array(zone.polygon + (zone.polygon[0],), dtype=float)
        ax.plot(corners[:, 0], corners[:, 1], color=INK_SECONDARY, linewidth=0.8,
                label='keepout zone (layout.yaml)' if number == 0 else None)
    truth = rec.truth
    during = np.zeros(len(truth.t), dtype=bool)
    for run in evaluation.runs:
        during |= truth.window(run.start, run.end)
    ax.plot(np.where(during, truth.x, np.nan), np.where(during, truth.y, np.nan),
            color=PATH_COLOUR, linewidth=1.2, solid_capstyle='round',
            label='ground-truth path while goals ran')
    long_holds = [hold for hold in evaluation.holds if hold.duration > HOLD_REPORT]
    if long_holds:
        xs, ys, _ = truth.at(np.array([hold.start for hold in long_holds]))
        sizes = [30.0 + 8.0 * hold.duration for hold in long_holds]
        ax.scatter(xs, ys, s=sizes, facecolor=HOLD_COLOUR, alpha=0.25, linewidth=0)
        ax.scatter(xs, ys, s=sizes, facecolor='none', edgecolor=HOLD_COLOUR, linewidth=1.2,
                   label=f'monitor hold over {HOLD_REPORT:g} s (area: its time)')
        # One label per place: holds within HOLD_PLACE of each other share it ("9 + 10 s").
        places = []
        for x, y, hold in zip(xs, ys, long_holds):
            place = next((place for place in places
                          if math.hypot(x - place[0], y - place[1]) < HOLD_PLACE), None)
            if place is None:
                places.append((x, y, [hold.duration]))
            else:
                place[2].append(hold.duration)
        for x, y, durations in places:
            ax.annotate(' + '.join(f'{d:.0f}' for d in durations) + ' s', (x, y),
                        xytext=(8, -12), textcoords='offset points', color=INK_SECONDARY,
                        fontsize=7)
    # A goal's result is a status: colour, marker shape and legend entry together.
    for run in evaluation.runs:
        if run.target is None:
            continue
        good = run.result == 'SUCCEEDED'
        ax.plot(run.target[0], run.target[1], 'o' if good else 'X', markersize=7,
                markerfacecolor=GOOD if good else CRITICAL, markeredgecolor='white',
                markeredgewidth=1.0)
        ax.annotate(str(run.index), (run.target[0], run.target[1]), xytext=(5, 5),
                    textcoords='offset points', color=INK, fontsize=8)
    ax.plot([], [], 'o', markersize=7, markerfacecolor=GOOD, markeredgecolor='white',
            label='goal (its index), succeeded')
    ax.plot([], [], 'X', markersize=7, markerfacecolor=CRITICAL, markeredgecolor='white',
            label='goal, did not succeed')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m] (map frame)')
    ax.set_ylabel('y [m]')
    ax.set_title('Paths, goals and holds, seen from above', loc='left')
    # Under the axes: inside, it would cover part of the warehouse.
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=2, frameon=False,
              labelcolor=INK)


def _draw_speed(ax, evaluation):
    _style(ax)
    rec = evaluation.recording
    role, inputs = monitor_input(rec)
    out = rec.commands['cmd_vel']
    for number, run in enumerate(evaluation.runs):
        if number % 2 == 0:
            ax.axvspan(run.start, run.end, color=GRID, alpha=0.6, linewidth=0)
    for hold in evaluation.holds:
        ax.axvspan(hold.start, hold.start + hold.duration, color=HOLD_COLOUR, alpha=0.25,
                   linewidth=0)
    if inputs is not None:
        ax.plot(*_with_gaps(inputs.recv, inputs.v), color=INK_MUTED, linewidth=1.0,
                label=f'into the monitor ({role})')
    ax.plot(*_with_gaps(out.recv, out.v), color=PATH_COLOUR, linewidth=1.2,
            label='out of it, to the base (cmd_vel)')
    # Head room for the legend above the fastest command.
    top = max(1.0, float(np.max(out.v)) if len(out) else 1.0)
    ax.set_ylim(min(-0.05, float(np.min(out.v)) - 0.05 if len(out) else -0.05), top * 1.3)
    ax.set_ylabel('linear velocity [m/s]')
    ax.set_title('Commanded speed (every other goal shaded grey, monitor holds orange)',
                 loc='left')
    ax.legend(loc='upper right', ncol=2, frameon=False, labelcolor=INK)
    ax.tick_params(labelbottom=False)


def _with_gaps(t, values, gap=PLOT_GAP):
    """Return (t, values) with a NaN wherever no message came for ``gap`` s: no line there."""
    breaks = np.flatnonzero(np.diff(t) > gap) + 1
    return (np.insert(np.asarray(t, dtype=float), breaks, np.nan),
            np.insert(np.asarray(values, dtype=float), breaks, np.nan))


def _draw_ages(ax, evaluation):
    _style(ax)
    times, ages = evaluation.ages
    ax.plot(times, ages, '.', color=PATH_COLOUR, markersize=2.0)
    for limit, text in ((AGE_P999_LIMIT, f'{AGE_P999_LIMIT:.2f} s (99.9 %)'),
                        (AGE_MAX_LIMIT, f'{AGE_MAX_LIMIT:.2f} s (timeout)')):
        ax.axhline(limit, color=INK_SECONDARY, linewidth=0.9, linestyle=(0, (4, 3)))
        ax.annotate(text, (1.0, limit), xycoords=('axes fraction', 'data'), xytext=(-4, 3),
                    textcoords='offset points', ha='right', color=INK_SECONDARY)
    ax.set_ylim(0.0, max(0.2, float(ages.max()) * 1.1 if len(ages) else 0.2))
    ax.set_ylabel('held command age [s]')
    ax.set_xlabel('sim time [s]')
    ax.set_title('Held command age on cmd_vel (while not zero)', loc='left')


# ----------------------------------------------------------------------------- the command


def main(argv=None):
    """Run the evaluate_navigation command; return its exit code."""
    parser = argparse.ArgumentParser(
        prog='evaluate_navigation',
        description='Evaluate a navigation benchmark from its bag (made by nav_benchmark): the '
                    "goals, the command age the base holds, the collision monitor's holds and "
                    'delay, events from /rosout, the zones and the load, against the Phase 3a '
                    'limits.')
    parser.add_argument('bag', help="the bag folder nav_benchmark wrote (or one .mcap file)")
    parser.add_argument('--namespace', default='robot1',
                        help='the robot namespace (default %(default)s)')
    parser.add_argument('--plot', metavar='OUT.png', help='draw the run into this PNG file')
    args = parser.parse_args(argv)

    try:
        evaluation = evaluate(args.bag, args.namespace)
    except (OSError, RuntimeError, ValueError, KeyError) as error:   # BagError is a ValueError
        print(f'error: {error}', file=sys.stderr)
        return 2
    results = criteria(evaluation)
    print(report(evaluation, results))
    failed = [result for result in results if not result.passed]
    if args.plot:
        verdict = ('all checks pass' if not failed else
                   f'{len(failed)} of {len(results)} checks fail')
        name = os.path.basename(os.path.normpath(args.bag))
        try:
            draw(args.plot, evaluation, f'evaluate_navigation: {name}: {verdict}')
        except OSError as error:
            print(f'error: cannot write the plot {args.plot}: {error.strerror}', file=sys.stderr)
            return 2
        print(f'wrote {args.plot}')
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
