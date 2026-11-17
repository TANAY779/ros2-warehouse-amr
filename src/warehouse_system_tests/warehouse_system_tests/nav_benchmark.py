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
Drive goals of the frozen goal set with Nav2, record the run and measure what it costs.

Usage::

    ros2 run warehouse_system_tests nav_benchmark --goals N [--from K] [--out DIR]
        [--goal-set FILE] [--with-scans] [--namespace robot1]

With ``sim.launch.py world:=warehouse navigation:=true`` running (normally right after the
launch, with the robot at its spawn) it does this, in order:

1. **Waits for Nav2 as warehouse_ops navigate does** (S3a): AMCL active and its amcl_pose
   received, bt_navigator active, the navigate_to_pose action server there; then
   BasicNavigator.waitUntilNav2Active(localizer='robot_localization'). With its default
   localizer, BasicNavigator would publish an initial pose at the origin; nav_benchmark never
   publishes on initialpose.
2. **Reads the configuration** the run uses from the running nodes (PARAMETERS, and the
   collision monitor's velocity bands) for results.json.
3. **Starts the recorder**, ``ros2 bag record`` into DIR, under the recording rules of plan
   section 3.6: after Nav2 is active (so it takes each publisher's QoS and gets tf_static) and
   before the robot moves; ``--use-sim-time`` (receive times in sim time, the base's clock);
   no /clock (1000 messages a second); no scans unless --with-scans. Topics: RECORDED in the
   robot's namespace and /rosout, plus the navigate_to_pose action's status, a hidden topic
   (``--include-hidden-topics``), so that the bag itself holds when each goal ran and how it
   ended. Topics that do not exist yet (cmd_vel_mux and cmd_vel_teleop before stage S5,
   speed_limit before S6b) stay in the list: the recorder subscribes once they appear. It
   waits until the recorder has subscribed to every listed topic that has a publisher, then
   DISCOVERY_SETTLE seconds more.
4. **Starts measure_load** (``--duration 0 --out DIR``), which samples CPU, memory and
   Gazebo's real-time factor every 2 s until it is stopped.
5. **Sends goals K to K+N-1** of the goal set, one NavigateToPose goal at a time, each once the
   one before has ended, whatever its result. Each goal has a timeout of TIMEOUT_BASE s plus
   TIMEOUT_PER_METRE s per metre of straight distance from where the robot stands (ground
   truth) to the goal, in sim time. A goal still running then is cancelled, counted as
   TIMEOUT, and the next one is sent once the robot stands.
6. **Stops measure_load and the recorder** (SIGTERM, which both handle as a clean stop), writes
   DIR/results.json and prints a table. ``evaluate_navigation DIR`` judges the run.

DIR is the bag folder itself: rosbag2 reads only the files its metadata.yaml lists, so
results.json, measure_load's load.jsonl, load_summary.json and load_summary.txt, and the logs
of the recorder and measure_load sit next to the MCAP file. results.json is written again
after every goal, so a run that stops early keeps its results.

**Clocks.** The node runs on the wall clock: a Python node on sim time pays for every /clock
message, about 0.27 of a core at the 1 ms step (PC assessment, section 5.4). Sim time comes
from the stamps of the ground truth (/<namespace>/ground_truth/odom, 50 Hz) instead: goal
times and timeouts are in sim time, so they do not depend on the real-time factor. Without new
ground truth for CLOCK_TIMEOUT seconds of wall time the simulation has stopped, and the run
ends with exit code 1.

**Children.** The recorder and measure_load run in sessions of their own: a Ctrl-C in the
terminal reaches only nav_benchmark, which stops them in a fixed order, after the robot has
stopped (up to STOP_WAIT seconds after a cancelled goal), so the bag shows the end of the run.
Their console output goes to DIR/recorder.log and DIR/measure_load.log.

**Ctrl-C.** rclpy's signal handlers are off (as in navigate). At the first Ctrl-C (or
SIGTERM), nav_benchmark cancels the active goal, waits until Nav2 confirms it and the robot
stands, records the goal as INTERRUPTED, stops measure_load and the recorder, writes
results.json and exits with 130. A second Ctrl-C ends any wait at once, but the children are
still stopped, so the bag stays readable.

Exit codes: 0 the run completed, whatever the goals' results; 1 it could not run (Nav2 not
ready, the recorder or measure_load did not start, the simulation stopped, Nav2 did not end a
cancelled goal); 2 a usage error (bad arguments, an unreadable goal set, goals outside the
set, DIR exists already); 130 Ctrl-C.
"""

import argparse
import datetime
import functools
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowPath
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.logging import LoggingSeverity
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from warehouse_system_tests import goal_set
from warehouse_system_tests import ros_helpers as rh

SCHEMA_VERSION = 1               # of results.json

# The timeout of a goal, in sim time (plan S3b).
TIMEOUT_BASE = 60.0              # s
TIMEOUT_PER_METRE = 4.0          # s per metre of straight distance

# The recording rules (plan section 3.6): topics relative to the robot's namespace.
RECORDED = ('ground_truth/odom', 'odom', 'odom/wheel', 'amcl_pose', 'cmd_vel', 'cmd_vel_nav',
            'cmd_vel_smoothed', 'cmd_vel_mux', 'cmd_vel_teleop', 'speed_limit',
            'collision_monitor_state', 'plan', 'tf', 'tf_static')
ACTION_STATUS = 'navigate_to_pose/_action/status'   # hidden: needs --include-hidden-topics
ROSOUT = '/rosout'
SCAN = 'scan'                    # only with --with-scans
RECORDER_NODE = 'nav_benchmark_recorder'

# Waiting, in wall-clock seconds.
NAV2_WAIT = 60.0                 # for AMCL and Nav2 to be ready
REPORT_PERIOD = 15.0             # while waiting: how often to repeat what is still missing
CHECK_PERIOD = 0.5               # between two rounds of the readiness checks
ANSWER_WAIT = 5.0                # for one service, action server, answer or amcl_pose
RECORDER_WAIT = 30.0             # for the recorder's bag folder and subscriptions
# After the recorder has subscribed: DDS matches the publisher's end separately, a moment later
# (drive_route's wait_for_controller explains it). The robot stands meanwhile.
DISCOVERY_SETTLE = 3.0
CLOCK_TIMEOUT = 10.0             # no new ground truth for this long: the simulation stopped
CANCEL_WAIT = 5.0                # for Nav2 to answer a cancel request
CANCELED_WAIT = 10.0             # for the goal to end once the cancel was accepted
STOP_WAIT = 3.0                  # after a cancel, for the robot to stand (ground truth)
STOPPED = 0.05                   # m/s and rad/s: below both, the robot stands
CHILD_STOP_WAIT = 30.0           # for the recorder or measure_load to exit after SIGTERM
SPIN_STEP = 0.1                  # the longest the tool waits before it looks for Ctrl-C again
FEEDBACK_PERIOD = 5.0            # a progress line this often while a goal runs
PAUSE_BETWEEN_GOALS = 1.0        # s of wall time: each goal's end stays apart in the bag
END_SETTLE = 2.0                 # s after the last goal, so the bag shows the robot stopped

EXIT_COMPLETED = 0
EXIT_FAILED = 1
EXIT_USAGE = 2
EXIT_CTRL_C = 130

# AMCL publishes amcl_pose reliable and transient local: a subscription with that QoS still
# gets the last estimate when it joins late.
AMCL_POSE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

# A goal's end, named as BasicNavigator names it: FAILED is the action status ABORTED.
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'FAILED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}

# Where Nav2's error codes come from. bt_navigator's NavigateToPose result carries the code of
# the controller (FollowPath, 100-107) or the planner (ComputePathToPose, 200-208) that failed.
ERROR_SOURCES = (('controller', FollowPath.Result), ('planner', ComputePathToPose.Result))

# The configuration a run used, read from the running nodes and kept in results.json. The
# collision monitor's velocity bands are added to it (see Benchmark.read_parameters).
PARAMETERS = {
    'collision_monitor': ('cmd_vel_in_topic', 'stop_pub_timeout', 'source_timeout',
                          'transform_tolerance', 'polygons'),
    'velocity_smoother': ('feedback', 'max_velocity', 'min_velocity', 'max_accel', 'max_decel',
                          'velocity_timeout', 'smoothing_frequency',
                          'stamp_smoothed_velocity_with_smoothing_time'),
    'controller_server': ('controller_frequency', 'failure_tolerance',
                          'progress_checker.required_movement_radius',
                          'progress_checker.movement_time_allowance',
                          'goal_checker.xy_goal_tolerance', 'goal_checker.yaw_goal_tolerance',
                          'FollowPath.desired_linear_vel'),
}
BAND_LIMITS = ('linear_min', 'linear_max', 'theta_min', 'theta_max')


class Interrupted(Exception):
    """The first Ctrl-C: the run stops after the active goal is cancelled."""


class NotReady(Exception):
    """AMCL or Nav2 was not ready in time; the message says what was missing."""


class RunError(Exception):
    """The run cannot go on (the recorder, measure_load, the simulation or Nav2 failed)."""


def describe_error(code):
    """Return (name, source) of one of Nav2's error codes, for example ('NO_VALID_PATH', ...)."""
    if code == 0:
        return 'NONE', ''
    for source, result_type in ERROR_SOURCES:
        # The generated message classes keep their constants as class attributes.
        for name, value in vars(result_type).items():
            if name.isupper() and isinstance(value, int) and value == code:
                return name, source
    return 'UNKNOWN_CODE', ''


def goal_timeout(straight):
    """Return the timeout of a goal ``straight`` metres away, in seconds of sim time."""
    return TIMEOUT_BASE + TIMEOUT_PER_METRE * straight


def recorded_topics(namespace, with_scans):
    """Return the topics the recorder records, as absolute names."""
    prefix = '/' + namespace.strip('/') + '/'
    names = [prefix + name for name in RECORDED + (ACTION_STATUS,)]
    if with_scans:
        names.append(prefix + SCAN)
    return names + [ROSOUT]


def pose_message(goal):
    """Return the goal as a PoseStamped in the map frame (the stamp stays zero)."""
    pose = PoseStamped()
    pose.header.frame_id = goal_set.FRAME_ID
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
    pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
    return pose


def now_text():
    """Return the wall-clock time as ISO text, to the second."""
    return datetime.datetime.now().isoformat(timespec='seconds')


class CtrlC:
    """
    Counts Ctrl-C (SIGINT) and SIGTERM, which nav_benchmark handles itself.

    The same scheme as warehouse_ops navigate: the first press only sets ``pressed``, and the
    tool acts on it between its own steps, at least every SPIN_STEP (an exception raised at
    whatever line runs could stop rclpy halfway through a call). The second press raises
    KeyboardInterrupt, which ends any wait at once, even one inside BasicNavigator.
    """

    def __init__(self):
        self.count = 0

    @property
    def pressed(self):
        return self.count > 0

    def install(self):
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def ignore(self):
        """
        Ignore SIGINT and SIGTERM from now on: the tool is finishing with the code it chose.

        While it stops its children, a further Ctrl-C must not cut that short; and when Python
        exits it puts the default handlers back, which would let a late Ctrl-C kill it.
        """
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    def _on_signal(self, signum, frame):
        self.count += 1
        if self.count == 2:
            raise KeyboardInterrupt


class Child:
    """
    A process the benchmark starts and stops: the recorder, or measure_load.

    It runs in a session of its own, so the terminal's Ctrl-C does not reach it, and its output
    is collected by a thread and written to a log file when it stops. SIGTERM stops either
    child cleanly: ``ros2 bag record`` then closes the bag, measure_load writes its summary.
    """

    def __init__(self, name, command):
        self.name = name
        self.command = command
        self.lines = []
        self.process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.process.stdout:
            self.lines.append(line.rstrip('\n'))

    def running(self):
        return self.process.poll() is None

    def tail(self, count=5):
        """Return the last lines of its output, as one string."""
        return '\n'.join(self.lines[-count:]) or '(no output)'

    def stop(self, wait=CHILD_STOP_WAIT):
        """Send SIGTERM and wait for the exit (SIGKILL after ``wait`` s); return the exit code."""
        if self.running():
            os.killpg(self.process.pid, signal.SIGTERM)
        try:
            code = self.process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            print(f'{self.name} did not stop within {wait:g} s of SIGTERM: killed',
                  file=sys.stderr)
            os.killpg(self.process.pid, signal.SIGKILL)
            code = self.process.wait()
        self._reader.join(timeout=2.0)
        return code

    def write_log(self, path):
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write(' '.join(self.command) + '\n\n' + '\n'.join(self.lines) + '\n')


class Benchmark:
    """The ROS side of nav_benchmark: readiness, recorder, goals, results."""

    def __init__(self, navigator, ctrl_c, goals, args, out):
        self.nav = navigator
        self.ctrl_c = ctrl_c
        self.goals = goals                       # the GoalSet
        self.args = args
        self.out = out
        self.namespace = navigator.get_namespace()          # '/robot1'
        self.prefix = self.namespace.rstrip('/') + '/'
        self.selected = goals.goals[args.first - 1:args.first - 1 + args.goals]
        self.topics = recorded_topics(self.namespace, args.with_scans)
        self.results = []
        self.status = 'running'
        self.started = now_text()
        self.parameters = {}
        self.bands = []
        self.recorder = None
        self.load = None
        self.goal_active = False                 # Nav2 accepted a goal that has not ended
        self.cancel_sent = False
        self.amcl_pose = None
        self.truth = None                        # the newest ground truth (Odometry)
        self.truth_arrived = time.monotonic()    # when it arrived (wall clock)
        self._state_clients = {}
        self._parameter_clients = {}
        navigator.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                      self._on_amcl_pose, AMCL_POSE_QOS)
        # Sensor-data QoS (best effort): only the newest pose matters, and it matches the
        # bridge's reliable publisher.
        navigator.create_subscription(Odometry, 'ground_truth/odom', self._on_truth,
                                      qos_profile_sensor_data)
        # BasicNavigator waits for some answers without a timeout; with rclpy's signal handlers
        # off, nothing would wake such a wait at a Ctrl-C. This timer makes every wait return
        # at least every SPIN_STEP, on the steady clock.
        navigator.create_timer(SPIN_STEP, lambda: None,
                               clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_amcl_pose(self, message):
        self.amcl_pose = message

    def _on_truth(self, message):
        self.truth = message
        self.truth_arrived = time.monotonic()

    # Time and the robot's pose, from ground truth.

    def sim_now(self):
        """Return the newest ground truth's stamp: sim time, at most 20 ms behind."""
        return rh.stamp_to_seconds(self.truth.header.stamp)

    def truth_pose(self):
        """Return the newest ground-truth pose as (x, y, yaw)."""
        pose = self.truth.pose.pose
        return pose.position.x, pose.position.y, rh.yaw_of(pose.orientation)

    def check_clock(self):
        """Raise RunError if ground truth stopped arriving: the simulation stopped."""
        if time.monotonic() - self.truth_arrived > CLOCK_TIMEOUT:
            raise RunError(f'no ground truth on {self.prefix}ground_truth/odom for '
                           f'{CLOCK_TIMEOUT:g} s: is Gazebo paused or gone?')

    # Waiting.

    def spin_until(self, condition, timeout, interruptible=True):
        """
        Spin until condition() is true; return False once ``timeout`` seconds have passed.

        With ``interruptible``, a first Ctrl-C ends the wait with Interrupted.
        """
        end = time.monotonic() + timeout
        while not condition():
            if interruptible and self.ctrl_c.pressed:
                raise Interrupted
            left = end - time.monotonic()
            if left <= 0.0:
                return False
            rclpy.spin_once(self.nav, timeout_sec=min(SPIN_STEP, left))
        return True

    def wait_until_ready(self):
        """
        Wait until AMCL is active and localized, and the navigator and its action are there.

        The checks of warehouse_ops navigate, with ground truth added. Raises NotReady after
        NAV2_WAIT seconds, naming the first check that has not passed.
        """
        print(f'Checking AMCL and Nav2 in {self.namespace} (for up to {NAV2_WAIT:g} s)')
        checks = (self._ground_truth, functools.partial(self._active, 'amcl'), self._localized,
                  functools.partial(self._active, 'bt_navigator'), self._action_server)
        started = time.monotonic()
        shown, shown_at = None, started
        while True:
            missing = next((text for text in (check() for check in checks) if text), None)
            if missing is None:
                break
            now = time.monotonic()
            if now - started >= NAV2_WAIT:
                raise NotReady(f'not ready after {NAV2_WAIT:g} s: {missing}')
            if missing != shown or now - shown_at >= REPORT_PERIOD:
                print(f'  waiting ({now - started:.0f} s): {missing}')
                shown, shown_at = missing, now
            self.spin_until(lambda: False, CHECK_PERIOD)
        x, y, yaw = self.truth_pose()
        pose = self.amcl_pose.pose.pose
        print(f'Ready: AMCL active, last estimate x {pose.position.x:.2f}, y '
              f'{pose.position.y:.2f}; ground truth x {x:.2f}, y {y:.2f}, yaw {yaw:.2f}; '
              f'{self.prefix}navigate_to_pose answers')
        # BasicNavigator's own check, now that ours passed: with this localizer it asks only
        # the navigator (and sleeps 2 s), and it never touches initialpose.
        self.nav.waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')
        if self.ctrl_c.pressed:
            raise Interrupted

    def _ground_truth(self):
        if not self.spin_until(lambda: self.truth is not None, ANSWER_WAIT):
            return (f'no ground truth on {self.prefix}ground_truth/odom: is the simulation '
                    "running in this shell's ROS domain "
                    f"({os.environ.get('ROS_DOMAIN_ID', '0')})?")
        return None

    def _active(self, node):
        client = self._state_clients.get(node)
        if client is None:
            client = self.nav.create_client(GetState, f'{node}/get_state')
            self._state_clients[node] = client
        name = f'{self.prefix}{node}'
        if not self.spin_until(client.service_is_ready, ANSWER_WAIT):
            return (f'no answer from {name}/get_state: is the simulation running with '
                    'navigation:=true?')
        future = client.call_async(GetState.Request())
        if not self.spin_until(future.done, ANSWER_WAIT):
            client.remove_pending_request(future)
            return f'{name} did not answer get_state within {ANSWER_WAIT:g} s'
        state = future.result().current_state
        if state.id != State.PRIMARY_STATE_ACTIVE:
            return f'{name} is {state.label}, not active'
        return None

    def _localized(self):
        if not self.spin_until(lambda: self.amcl_pose is not None, ANSWER_WAIT):
            return (f'{self.prefix}amcl is active but has not published amcl_pose yet (the '
                    "launch sets an initial pose; nav_benchmark never sets one)")
        return None

    def _action_server(self):
        if not self.spin_until(self.nav.nav_to_pose_client.server_is_ready, ANSWER_WAIT):
            return f'no action server {self.prefix}navigate_to_pose'
        return None

    # The configuration of the run.

    def read_parameters(self):
        """
        Read the parameters of PARAMETERS, and the collision monitor's velocity bands.

        They go into results.json, so a run says which configuration it measured, and
        evaluate_navigation names the band of each command the monitor held. A node that does
        not answer is noted, not fatal.
        """
        for node, names in PARAMETERS.items():
            self.parameters[node] = self._get(node, names) or {}
        monitor = 'collision_monitor'
        for polygon in self.parameters[monitor].get('polygons') or []:
            info = self._get(monitor, [f'{polygon}.type', f'{polygon}.velocity_polygons'])
            if not info or info.get(f'{polygon}.type') != 'velocity_polygon':
                continue
            for band in info.get(f'{polygon}.velocity_polygons') or []:
                limits = self._get(monitor, [f'{polygon}.{band}.{key}' for key in BAND_LIMITS])
                if limits:
                    self.bands.append({'polygon': polygon, 'band': band, **{
                        key: limits.get(f'{polygon}.{band}.{key}') for key in BAND_LIMITS}})
        print(f'Configuration read from the nodes: {len(self.bands)} velocity bands of the '
              f'collision monitor, smoother feedback '
              f'{self.parameters["velocity_smoother"].get("feedback")}')

    def _get(self, node, names):
        """Return {name: value} of a node's parameters, or None if it does not answer."""
        client = self._parameter_clients.get(node)
        if client is None:
            client = AsyncParameterClient(self.nav, f'{self.prefix}{node}')
            self._parameter_clients[node] = client
        if not client.wait_for_services(timeout_sec=ANSWER_WAIT):
            print(f'warning: {self.prefix}{node} has no parameter services: its parameters are '
                  'not in results.json', file=sys.stderr)
            return None
        future = client.get_parameters(list(names))
        if not self.spin_until(future.done, ANSWER_WAIT) or future.result() is None:
            print(f'warning: {self.prefix}{node} did not answer get_parameters',
                  file=sys.stderr)
            return None
        values = [parameter_value_to_python(value) for value in future.result().values]
        return dict(zip(names, values))

    # The recorder and measure_load.

    def start_recorder(self):
        """Start ros2 bag record into the bag folder; wait until it records (see the docstring)."""
        ros2 = shutil.which('ros2')
        if ros2 is None:
            raise RunError('no ros2 command on PATH: source ROS 2 first')
        custom = ['tool=nav_benchmark', f'goal_set={os.path.basename(self.goals.path)}',
                  f'first_goal={self.args.first}', f'goals={self.args.goals}']
        command = [ros2, 'bag', 'record', '--output', self.out, '--use-sim-time',
                   '--include-hidden-topics', '--disable-keyboard-controls',
                   '--node-name', RECORDER_NODE, '--custom-data', *custom,
                   '--topics', *self.topics]
        scans = 'with scans' if self.args.with_scans else 'no scans'
        print(f'Recording {len(self.topics)} topics into {self.out} (sim time, no /clock, '
              f'{scans})')
        self.recorder = Child('recorder', command)
        started = time.monotonic()

        def recording():
            if not self.recorder.running():
                raise RunError(f'the recorder exited with code {self.recorder.process.returncode}:'
                               f'\n{self.recorder.tail()}')
            return os.path.isdir(self.out) and not self._not_subscribed()

        if not self.spin_until(recording, RECORDER_WAIT):
            raise RunError(f'the recorder is not recording after {RECORDER_WAIT:g} s; not '
                           f'subscribed: {", ".join(self._not_subscribed()) or "-"}\n'
                           f'{self.recorder.tail()}')
        absent = [topic for topic in self.topics if not self.nav.get_publishers_info_by_topic(
            topic)]
        self.spin_until(lambda: False, DISCOVERY_SETTLE)
        print(f'  recording after {time.monotonic() - started:.1f} s; no publisher yet (they '
              f'are recorded once they appear): {", ".join(absent) or "none"}')

    def _not_subscribed(self):
        """Return the listed topics that have a publisher but no subscription of the recorder."""
        missing = []
        for topic in self.topics:
            if not self.nav.get_publishers_info_by_topic(topic):
                continue
            if not any(endpoint.node_name == RECORDER_NODE
                       for endpoint in self.nav.get_subscriptions_info_by_topic(topic)):
                missing.append(topic)
        return missing

    def start_load(self):
        """Start measure_load --duration 0 into the bag folder."""
        try:
            prefix = get_package_prefix('warehouse_system_tests')
        except PackageNotFoundError:
            raise RunError('warehouse_system_tests is not installed: source the workspace') \
                from None
        command = [os.path.join(prefix, 'lib', 'warehouse_system_tests', 'measure_load'),
                   '--world', self.goals.world, '--duration', '0', '--out', self.out, '--quiet']
        self.load = Child('measure_load', command)
        if not self.spin_until(lambda: os.path.exists(os.path.join(self.out, 'load.jsonl'))
                               or not self.load.running(), ANSWER_WAIT * 2):
            raise RunError(f'measure_load has not started sampling:\n{self.load.tail()}')
        if not self.load.running():
            raise RunError(f'measure_load exited with code {self.load.process.returncode}:\n'
                           f'{self.load.tail()}')
        print(f'measure_load samples every 2 s into {os.path.join(self.out, "load.jsonl")}')

    def stop_children(self):
        """Stop measure_load, then the recorder; keep their logs in the bag folder."""
        for child in (self.load, self.recorder):
            if child is None:
                continue
            code = child.stop()
            if os.path.isdir(self.out):
                child.write_log(os.path.join(self.out, f'{child.name}.log'))
            if code != 0:
                print(f'warning: {child.name} exited with code {code} (see {child.name}.log)',
                      file=sys.stderr)

    # The goals.

    def run(self):
        """Wait for Nav2, start recording, run the goals; return the exit code."""
        self.wait_until_ready()
        self.read_parameters()
        self.start_recorder()
        self.start_load()
        self.write_results()
        count = len(self.selected)
        for number, goal in enumerate(self.selected, 1):
            self.results.append(self.run_goal(goal, number, count))
            self.write_results()
            if number < count:
                self.spin_until(lambda: False, PAUSE_BETWEEN_GOALS)
        self.spin_until(lambda: False, END_SETTLE)
        self.status = 'completed'
        return EXIT_COMPLETED

    def run_goal(self, goal, number, count):
        """Send one goal and wait for its end; return its entry for results.json."""
        self.check_clock()
        start = self.truth_pose()
        straight = math.dist(start[:2], (goal.x, goal.y))
        timeout = goal_timeout(straight)
        print(f'Goal {goal.index} ({number} of {count}): x {goal.x:.3f}, y {goal.y:.3f}, '
              f'yaw {goal.yaw:.4f}; {straight:.2f} m straight from the robot, timeout '
              f'{timeout:.1f} s')
        entry = {'index': goal.index, 'x': goal.x, 'y': goal.y, 'yaw': goal.yaw,
                 'straight_distance_m': round(straight, 3), 'timeout_s': round(timeout, 1),
                 'start_pose': [round(value, 4) for value in start]}
        self.nav.feedback = None          # BasicNavigator keeps the last goal's feedback
        wall_start = time.monotonic()
        if not self.nav.goToPose(pose_message(goal)):
            print('  REJECTED by Nav2 (its log says why)')
            entry.update(result='REJECTED', status=None, error_code=0, error_name='NONE',
                         error_msg='', time_s=0.0, wall_s=0.0, recoveries=0, goal_id=None,
                         sim_start=self.sim_now(), sim_end=self.sim_now())
            return self._finish_entry(entry)
        self.goal_active, self.cancel_sent = True, False
        sim_start = self.sim_now()
        entry['goal_id'] = str(uuid.UUID(bytes=bytes(self.nav.goal_handle.goal_id.uuid)))
        recoveries = 0
        next_line = time.monotonic() + FEEDBACK_PERIOD
        result = None
        # isTaskComplete() spins for up to 0.1 s and stores the feedback that arrives.
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback is not None:
                recoveries = max(recoveries, feedback.number_of_recoveries)
            if self.ctrl_c.pressed:
                self.cancel(f'goal {goal.index}')
                result = 'INTERRUPTED'
                break
            self.check_clock()
            elapsed = self.sim_now() - sim_start
            if elapsed > timeout:
                print(f'  no result after {timeout:.1f} s of sim time: cancelling')
                self.cancel(f'goal {goal.index}')
                result = 'TIMEOUT'
                break
            if time.monotonic() >= next_line and feedback is not None:
                print(f'  {elapsed:5.0f} s: {feedback.distance_remaining:.1f} m to go, '
                      f'recoveries {feedback.number_of_recoveries}')
                next_line = time.monotonic() + FEEDBACK_PERIOD
        self.goal_active = False
        feedback = self.nav.getFeedback()
        if feedback is not None:
            recoveries = max(recoveries, feedback.number_of_recoveries)
        response = self.nav.result_future.result()
        status = response.status
        code = int(getattr(response.result, 'error_code', 0))
        name, source = describe_error(code)
        entry.update(result=result or STATUS_NAMES.get(status, f'STATUS_{status}'),
                     status=int(status), error_code=code, error_name=name,
                     error_source=source, error_msg=getattr(response.result, 'error_msg', ''),
                     time_s=round(self.sim_now() - sim_start, 3),
                     wall_s=round(time.monotonic() - wall_start, 3), recoveries=int(recoveries),
                     sim_start=sim_start, sim_end=self.sim_now())
        if result is not None:
            # A goal this tool cancelled: let the robot come to a stop before anything else
            # happens (the next goal, or the end of the recording), so the bag shows the stop
            # and the next goal starts from rest.
            self.spin_until(self.standing, STOP_WAIT, interruptible=False)
        if result == 'INTERRUPTED':
            self.results.append(self._finish_entry(entry))
            raise Interrupted
        return self._finish_entry(entry)

    def _finish_entry(self, entry):
        """Add where the robot ended (ground truth) to an entry, print its line, return it."""
        x, y, yaw = self.truth_pose()
        error = math.dist((x, y), (entry['x'], entry['y']))
        turn = abs(rh.wrap_angle(yaw - entry['yaw']))
        entry.update(end_pose=[round(x, 4), round(y, 4), round(yaw, 4)],
                     end_error_m=round(error, 4), end_yaw_error_rad=round(turn, 4))
        code = '' if not entry['error_code'] else \
            f', error {entry["error_code"]} {entry["error_name"]}'
        print(f'  {entry["result"]} after {entry["time_s"]:.1f} s of sim time '
              f'({entry["wall_s"]:.1f} s wall){code}, {entry["recoveries"]} recoveries; ground '
              f'truth {error:.3f} m and {turn:.3f} rad from the goal')
        return entry

    def cancel(self, label):
        """Cancel the active goal and wait (bounded) until it has ended."""
        answer = self.nav.goal_handle.cancel_goal_async()
        self.cancel_sent = True
        if not self.spin_until(answer.done, CANCEL_WAIT, interruptible=False):
            raise RunError(f'Nav2 did not answer the cancel request for {label} within '
                           f'{CANCEL_WAIT:g} s')
        if answer.result().return_code not in (CancelGoal.Response.ERROR_NONE,
                                                CancelGoal.Response.ERROR_GOAL_TERMINATED):
            print(f'warning: Nav2 refused to cancel {label} (return code '
                  f'{answer.result().return_code})', file=sys.stderr)
        if not self.spin_until(self.nav.result_future.done, CANCELED_WAIT, interruptible=False):
            raise RunError(f'{label} had not ended {CANCELED_WAIT:g} s after the cancel request')

    def standing(self):
        """Return True if ground truth says the robot stands."""
        twist = self.truth.twist.twist
        return (math.hypot(twist.linear.x, twist.linear.y) < STOPPED
                and abs(twist.angular.z) < STOPPED)

    def abandon_goal(self):
        """
        Send the cancel request of a goal that is still active, without waiting for an answer.

        For the exits that cannot wait: a second Ctrl-C, or an error during a goal (Nav2 would
        otherwise drive on after nav_benchmark has gone).
        """
        if self.goal_active and not self.cancel_sent and self.nav.goal_handle is not None:
            self.nav.goal_handle.cancel_goal_async()
            self.cancel_sent = True
            print('Sent the cancel request for the active goal, without waiting for the answer',
                  file=sys.stderr)

    # results.json and the summary.

    def document(self):
        """Return results.json's content."""
        counts = {}
        for entry in self.results:
            counts[entry['result']] = counts.get(entry['result'], 0) + 1
        return {
            'tool': 'nav_benchmark',
            'schema_version': SCHEMA_VERSION,
            'status': self.status,
            'started': self.started,
            'finished': now_text() if self.status != 'running' else None,
            'namespace': self.namespace,
            'world': self.goals.world,
            'goal_set': {'file': self.goals.path, 'sha256': goal_set.sha256_of(self.goals.path),
                         'seed': self.goals.seed, 'goals': len(self.goals)},
            'first': self.args.first,
            'requested': self.args.goals,
            'timeout_rule': {'base_s': TIMEOUT_BASE, 'per_metre_s': TIMEOUT_PER_METRE,
                             'clock': 'sim time'},
            'recording': {'topics': self.topics, 'with_scans': self.args.with_scans},
            'nav2_parameters': self.parameters,
            'collision_monitor_bands': self.bands,
            'counts': counts,
            'goals': self.results,
        }

    def write_results(self):
        """Write results.json into the bag folder (whole, through a temporary file)."""
        if not os.path.isdir(self.out):
            return
        path = os.path.join(self.out, 'results.json')
        with open(path + '.tmp', 'w', encoding='utf-8') as stream:
            json.dump(self.document(), stream, indent=2)
            stream.write('\n')
        os.replace(path + '.tmp', path)

    def summary(self):
        """Return the table printed at the end."""
        lines = ['', f' goal  result       error                     time [s]  straight [m]  '
                     'timeout [s]  recoveries  end error [m]']
        for entry in self.results:
            error = '-' if not entry['error_code'] else \
                f'{entry["error_code"]} {entry["error_name"]}'
            lines.append(f'{entry["index"]:5d}  {entry["result"]:<11s}  {error:<24s}  '
                         f'{entry["time_s"]:8.1f}  {entry["straight_distance_m"]:12.2f}  '
                         f'{entry["timeout_s"]:11.1f}  {entry["recoveries"]:10d}  '
                         f'{entry["end_error_m"]:13.3f}')
        succeeded = sum(entry['result'] == 'SUCCEEDED' for entry in self.results)
        lines.append(f'{succeeded} of {len(self.results)} goals succeeded')
        load_path = os.path.join(self.out, 'load_summary.json')
        try:
            with open(load_path, encoding='utf-8') as stream:
                load = json.load(stream)
            lines.append(f'measure_load: RTF {load.get("rtf_mean", float("nan")):.3f} mean, run '
                         f'{load.get("run_cores_mean", float("nan")):.2f} cores mean over '
                         f'{load.get("duration", 0):.0f} s')
            lines += [f'WARNING: {warning}' for warning in load.get('warnings', [])]
        except (OSError, ValueError):
            lines.append(f'measure_load: no summary in {load_path}')
        return '\n'.join(lines)


def _parser():
    parser = argparse.ArgumentParser(
        prog='nav_benchmark',
        description='Send goals of the frozen goal set to Nav2 one after the other, record the '
                    'run into a bag and measure its load (simulation with navigation:=true).',
        epilog='Exit codes: 0 the run completed, whatever the goals did; 1 it could not run; '
               '2 a usage error; 130 Ctrl-C (the active goal is cancelled first, and the bag is '
               'closed).')
    parser.add_argument('--goals', type=int, required=True, metavar='N',
                        help='how many goals to send')
    parser.add_argument('--from', dest='first', type=int, default=1, metavar='K',
                        help='the index of the first goal (default 1)')
    parser.add_argument('--out', metavar='DIR',
                        help='the bag folder to create (default: '
                             'bags/nav_benchmark_<date>-<time> under the current folder)')
    parser.add_argument('--goal-set', metavar='FILE',
                        help='the goal set (default: the installed warehouse_50.yaml)')
    parser.add_argument('--with-scans', action='store_true',
                        help='also record the lidar scan (/<namespace>/scan)')
    parser.add_argument('--namespace', default='robot1',
                        help='the robot (default %(default)s)')
    return parser


def main(argv=None):
    """Run the nav_benchmark command; return its exit code."""
    # One line at a time, also into a pipe or a file: the output is a progress report.
    sys.stdout.reconfigure(line_buffering=True)
    argv = sys.argv if argv is None else ['nav_benchmark', *argv]
    parser = _parser()
    args = parser.parse_args(remove_ros_args(argv)[1:])
    try:
        goals = goal_set.load(args.goal_set or goal_set.default_path())
    except (goal_set.GoalSetError, PackageNotFoundError) as error:
        print(f'nav_benchmark: error: {error}', file=sys.stderr)
        return EXIT_USAGE
    if args.goals < 1 or args.first < 1 or args.first + args.goals - 1 > len(goals):
        parser.error(f'--from {args.first} --goals {args.goals}: the goal set has goals 1 to '
                     f'{len(goals)} ({goals.path})')
    out = args.out or os.path.join(
        'bags', datetime.datetime.now().strftime('nav_benchmark_%Y_%m_%d-%H_%M_%S'))
    out = os.path.normpath(out)
    if os.path.exists(out):
        parser.error(f'{out} exists already: the recorder creates the bag folder, so choose a '
                     'new --out')
    try:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    except OSError as error:
        parser.error(f'cannot create the folder for {out}: {error.strerror}')
    print(f'nav_benchmark: goals {args.first} to {args.first + args.goals - 1} of {goals.path} '
          f'(seed {goals.seed}) into {out}')
    return run(argv, args, goals, out)


def run(argv, args, goals, out):
    """Start ROS, run the benchmark, stop everything again; return the exit code."""
    ctrl_c = CtrlC()
    ctrl_c.install()
    # No rclpy signal handlers: they would shut the context down at the first Ctrl-C, before
    # the goal could be cancelled. CtrlC handles SIGINT and SIGTERM instead. Children started
    # from here begin with the default handlers (a handled signal is reset on exec).
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    bench = None
    code = EXIT_FAILED
    try:
        navigator = BasicNavigator(node_name='nav_benchmark', namespace=args.namespace.strip('/'))
        # BasicNavigator logs every goal it sends at info level; the tool prints its own lines.
        navigator.get_logger().set_level(LoggingSeverity.WARN)
        bench = Benchmark(navigator, ctrl_c, goals, args, out)
        code = bench.run()
    except Interrupted:
        print('Ctrl-C: the run stops here')
        code = EXIT_CTRL_C
    except (NotReady, RunError) as error:
        print(f'nav_benchmark: {error}', file=sys.stderr)
        code = EXIT_FAILED
    except KeyboardInterrupt:
        code = EXIT_CTRL_C
    except Exception:
        # A second Ctrl-C that lands inside rclpy's C++ code can come out as another exception
        # (drive_route saw a RuntimeError); it still means "stop now".
        if ctrl_c.count < 2:
            raise
        code = EXIT_CTRL_C
    finally:
        ctrl_c.ignore()
        if bench is not None:
            bench.abandon_goal()
            if code == EXIT_CTRL_C:
                bench.status = 'interrupted'
            elif code != EXIT_COMPLETED:
                bench.status = 'failed'
            bench.stop_children()
            bench.write_results()
            if os.path.isdir(out):
                print(bench.summary())
                print(f'wrote {os.path.join(out, "results.json")}; judge the run with:\n'
                      f'  ros2 run warehouse_system_tests evaluate_navigation {out}')
            bench.nav.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
