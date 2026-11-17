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
Send the robot to named locations, or to poses on the map, with Nav2.

Usage::

    ros2 run warehouse_ops navigate --list
    ros2 run warehouse_ops navigate --to pick_2_03
    ros2 run warehouse_ops navigate --to pick_1_01 --to drop_pack_1 [--through | --waypoints]
    ros2 run warehouse_ops navigate --pose X Y YAW [--to LOCATION ...] [--namespace robot1]

An operator tool (plan P3-D21): it moves the robot only through Nav2's actions, and it takes the
named locations from the location registry (locations.py). ``--list`` prints them and needs no
ROS graph. Otherwise, with ``sim.launch.py world:=warehouse navigation:=true`` running, it does
this, in order:

1. It checks, for up to NAV2_WAIT seconds, that AMCL (/<namespace>/amcl) is active and has
   published amcl_pose, that the navigator (bt_navigator, and waypoint_follower for
   --waypoints) is active, and that the goal's action server answers. The error names the
   check that did not pass in time.
2. It calls BasicNavigator.waitUntilNav2Active(localizer='robot_localization'). With the
   default localizer, 'amcl', BasicNavigator publishes an initial pose of (0, 0, 0) in the map
   frame once a second until it hears amcl_pose (robot_navigator.py:495-503, 795-801), and
   AMCL would jump to the origin wherever the robot stands. 'robot_localization' skips that,
   and AMCL's checks with it, which is why step 1 checks AMCL here. navigate never publishes
   on initialpose.
3. It sends the goals, in the order they were given:
   - by default one NavigateToPose goal per pose, each once the one before succeeded; the
     first that does not succeed ends the run;
   - with --through one NavigateThroughPoses goal: the robot drives through the poses without
     stopping, and stops at the last;
   - with --waypoints one FollowWaypoints goal: the robot stops at each pose, and goes on after
     one it cannot reach (the waypoint follower's stop_on_failure is false).
4. It prints a feedback line once a second, then each goal's result. When the planner reports
   START_OCCUPIED, it says what that means and what to do.

Exit codes: 0 every goal succeeded; 1 a goal failed, was rejected or was cancelled by another
client, a waypoint was missed, or AMCL and Nav2 were not ready in time; 2 a usage error (an
unknown or docking-only location, a bad number, options that conflict) or an unreadable
registry; 130 Ctrl-C.

**Ctrl-C.** rclpy's own signal handlers are off: at the first Ctrl-C they would shut the ROS
context down, and with it the action client, before the tool could cancel its goal. CtrlC
counts the presses instead. At the first, the tool cancels the goal: it notices the press
within SPIN_STEP, sends the cancel request, waits until Nav2 accepts it and the goal ends
CANCELED, and exits with 130; before a goal is sent, it just exits with 130. A second Ctrl-C
ends any wait at once, also one inside BasicNavigator, and exits with 130; if the first had no
time to send the cancel request, it is sent first. SIGTERM counts as Ctrl-C.
"""

import argparse
from dataclasses import dataclass
import difflib
import functools
import math
import os
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowPath
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from warehouse_ops import locations

NAV2_WAIT = 60.0        # s: how long to wait for AMCL and Nav2 to be ready
REPORT_PERIOD = 15.0    # s: while waiting, how often to repeat what is still missing
CHECK_PERIOD = 0.5      # s: between two rounds of the readiness checks
# s: how long one check waits for a service or action server to be discovered, for an answer,
# or for the latched amcl_pose, before it calls that missing. Discovery alone took up to 3 s
# with Gazebo and Nav2 running.
ANSWER_WAIT = 5.0
FEEDBACK_PERIOD = 1.0   # s: one feedback line a second
SPIN_STEP = 0.1         # s: the longest the tool waits before it looks for Ctrl-C again
CANCEL_WAIT = 5.0       # s: for Nav2 to answer a cancel request
CANCELED_WAIT = 10.0    # s: for the goal to end once the cancel was sent

EXIT_SUCCEEDED = 0
EXIT_FAILED = 1
EXIT_USAGE = 2           # also what argparse exits with
EXIT_CTRL_C = 130        # 128 + SIGINT, as a shell reports a command ended by Ctrl-C

# AMCL publishes amcl_pose reliable and transient local, and only after an update (the robot
# moved, or an initial pose came): a subscription with the same QoS still gets the last
# estimate when it joins late.
AMCL_POSE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

# A goal's end state, named as BasicNavigator's TaskResult names it: FAILED is the action status
# ABORTED.
RESULT_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'FAILED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}

# Where Nav2's error codes come from. Each action numbers its codes in a hundred of its own.
# bt_navigator's result carries the code of the planner or the controller that failed (the
# error_code_names in nav2_params.yaml; if both did, the lower code, the controller's), and a
# missed waypoint the code of its NavigateToPose goal.
ERROR_SOURCES = (
    ('controller', FollowPath.Result),                 # 100-107
    ('planner', ComputePathToPose.Result),             # 200-208
    ('planner', ComputePathThroughPoses.Result),       # 300-309
    ('waypoint follower', FollowWaypoints.Result),     # 600-601
)

# The planner's "start occupied": the robot's centre stands on a lethal cell of the global
# costmap, so no path can leave it (plan §3.3). NavigateToPose reports the code of
# ComputePathToPose, NavigateThroughPoses that of ComputePathThroughPoses.
START_OCCUPIED = (ComputePathToPose.Result.START_OCCUPIED,
                  ComputePathThroughPoses.Result.START_OCCUPIED)
START_OCCUPIED_HINT = 'the robot stands in a keepout margin or an obstacle: drive it out by hand'

# What each kind of location is, for --list.
KIND_TITLES = {
    'spawn': 'where each robot starts and parks',
    'pick': "on a pick aisle's centreline at a bay, facing along the aisle",
    'drop': "at a packing station's counter",
    'dock_staging': 'in front of a charging dock, where docking starts',
    'dock_docked': "on a dock's charging contacts: docking only, refused by --to",
}


class Interrupted(Exception):
    """The first Ctrl-C came before a goal was sent."""


class NotReady(Exception):
    """AMCL or Nav2 was not ready in time; the message says what was missing."""


@dataclass(frozen=True)
class Target:
    """Where one goal goes: a named location, or a pose given with --pose."""

    label: str
    x: float
    y: float
    yaw: float

    def where(self):
        """Return the pose as text."""
        return f'x {self.x:.2f} m, y {self.y:.2f} m, yaw {self.yaw:.2f} rad'

    def pose(self):
        """Return the goal as a PoseStamped in the map frame."""
        pose = PoseStamped()
        pose.header.frame_id = locations.FRAME_ID
        # The stamp stays zero. The goal is in the map frame, the frame Nav2 plans in, so Nav2
        # does not transform it and reads no time from it.
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        return pose


class CtrlC:
    """
    Counts Ctrl-C (SIGINT) and SIGTERM, which navigate handles itself (module docstring).

    The first press only sets ``pressed``. The tool looks at it between its own steps, at least
    every SPIN_STEP, and cancels the goal there. Raising KeyboardInterrupt at once, at whatever
    line runs, could stop rclpy halfway through a call: a Ctrl-C that landed inside rclpy ended
    drive_route with a traceback (warehouse_system_tests README). The second press raises
    KeyboardInterrupt, which ends any wait at once, even one inside BasicNavigator. Later
    presses are only counted, and once the tool exits, ignore() makes it deaf to them.
    """

    def __init__(self):
        self.count = 0

    @property
    def pressed(self):
        """Return True once Ctrl-C was pressed."""
        return self.count > 0

    def install(self):
        """Handle SIGINT and SIGTERM from now on."""
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def ignore(self):
        """
        Ignore SIGINT and SIGTERM from now on: the tool is exiting with the code it chose.

        When Python exits, it puts the default handler back for every signal that has a Python
        handler, and a Ctrl-C would then kill the process: `ros2 run` reports `Interrupt` and
        exits with 254 instead of the tool's code. Python leaves an ignored signal alone.
        """
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    def _on_signal(self, signum, frame):
        self.count += 1
        if self.count == 2:
            raise KeyboardInterrupt


class Navigate:
    """The ROS side of navigate: the readiness checks, the goals, their feedback and results."""

    def __init__(self, navigator, ctrl_c, targets, order):
        """
        Work through ``navigator``, a BasicNavigator in the robot's namespace.

        ``order`` is 'each' (one NavigateToPose goal per target), 'through' or 'waypoints'.
        """
        self.nav = navigator
        self.ctrl_c = ctrl_c
        self.targets = targets
        self.order = order
        self.namespace = navigator.get_namespace()                   # '/robot1'
        self.prefix = self.namespace.rstrip('/') + '/'                # '/robot1/'
        self.action_name, self.action_client = {
            'each': ('navigate_to_pose', navigator.nav_to_pose_client),
            'through': ('navigate_through_poses', navigator.nav_through_poses_client),
            'waypoints': ('follow_waypoints', navigator.follow_waypoints_client),
        }[order]
        self.amcl_pose = None
        self.label = None            # the goal that runs, for messages
        self.sending = False         # a goal request is on its way to Nav2
        self.goal_active = False     # Nav2 accepted a goal that has not ended yet
        self.cancel_sent = False
        self._state_clients = {}
        navigator.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                      self._on_amcl_pose, AMCL_POSE_QOS)
        # BasicNavigator waits for some answers without a timeout, and with rclpy's signal
        # handlers off (main), nothing wakes such a wait at a Ctrl-C: Python runs a signal
        # handler only once the wait returns. This timer makes every wait return at least
        # every SPIN_STEP, on the steady clock.
        navigator.create_timer(SPIN_STEP, self._on_wake,
                               clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_amcl_pose(self, message):
        self.amcl_pose = message

    def _on_wake(self):
        pass

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

        Raises NotReady after NAV2_WAIT seconds, naming the first check that has not passed.
        """
        print(f'Checking AMCL and Nav2 in {self.namespace} (for up to {NAV2_WAIT:g} s)')
        checks = [functools.partial(self._active, 'amcl'), self._localized,
                  functools.partial(self._active, 'bt_navigator')]
        if self.order == 'waypoints':
            checks.append(functools.partial(self._active, 'waypoint_follower'))
        checks.append(self._action_server)
        started = time.monotonic()
        shown, shown_at = None, started
        while True:
            # The checks run in order, and the first that fails says what is missing.
            missing = next((text for text in (check() for check in checks) if text), None)
            if missing is None:
                break
            now = time.monotonic()
            if now - started >= NAV2_WAIT:
                raise NotReady(f'not ready after {NAV2_WAIT:g} s: {missing}')
            # Say what is missing when that changes, and again every REPORT_PERIOD.
            if missing != shown or now - shown_at >= REPORT_PERIOD:
                print(f'  waiting ({now - started:.0f} s): {missing}')
                shown, shown_at = missing, now
            self.spin_until(lambda: False, CHECK_PERIOD)
        pose = self.amcl_pose.pose.pose
        print(f'AMCL is active, its last estimate x {pose.position.x:.2f} m, '
              f'y {pose.position.y:.2f} m, yaw {yaw_of(pose.orientation):.2f} rad; '
              f'{self.prefix}{self.action_name} is ready')

    def _active(self, node):
        """Return None if the node's lifecycle state is active, else what is wrong."""
        client = self._state_clients.get(node)
        if client is None:
            client = self.nav.create_client(GetState, f'{node}/get_state')
            self._state_clients[node] = client
        name = f'{self.prefix}{node}'
        if not self.spin_until(client.service_is_ready, ANSWER_WAIT):
            domain = os.environ.get('ROS_DOMAIN_ID', '0')
            return (f'no answer from {name}/get_state: is the simulation running, with '
                    f"navigation:=true and in this shell's ROS domain ({domain})?")
        future = client.call_async(GetState.Request())
        if not self.spin_until(future.done, ANSWER_WAIT):
            client.remove_pending_request(future)
            return f'{name} did not answer get_state within {ANSWER_WAIT:g} s'
        state = future.result().current_state
        if state.id != State.PRIMARY_STATE_ACTIVE:
            return f'{name} is {state.label}, not active'
        return None

    def _localized(self):
        """Return None once AMCL has published a pose, else what is wrong."""
        if not self.spin_until(lambda: self.amcl_pose is not None, ANSWER_WAIT):
            # Right after the launch this passes within seconds: AMCL publishes its first pose
            # once it has the map, the launch's initial pose and a scan.
            return (f'{self.prefix}amcl is active but has not published {self.prefix}amcl_pose '
                    'yet. It does once it has an initial pose (the launch sets one; else use '
                    "RViz's 2D Pose Estimate: navigate never sets one) and a scan")
        return None

    def _action_server(self):
        """Return None once the goal's action server is there, else what is wrong."""
        if not self.spin_until(self.action_client.server_is_ready, ANSWER_WAIT):
            return f'no action server {self.prefix}{self.action_name}'
        return None

    # The goals.

    def run(self):
        """Wait for AMCL and Nav2, then run the goals; return the exit code."""
        self.wait_until_ready()
        # BasicNavigator's own check, now that ours passed: with this localizer it asks only
        # the navigator, then sleeps 2 s, and it never touches initialpose.
        self.nav.waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')
        if self.ctrl_c.pressed:
            raise Interrupted
        if self.order == 'through':
            return self.drive_through()
        if self.order == 'waypoints':
            return self.follow_waypoints()
        return self.drive_each()

    def drive_each(self):
        """Send one NavigateToPose goal per target, in order, until one does not succeed."""
        count = len(self.targets)
        for number, target in enumerate(self.targets, 1):
            print(f'Goal {number} of {count}: {target.label} ({target.where()}), NavigateToPose')
            code = self.run_goal(target.label,
                                 functools.partial(self.nav.goToPose, target.pose()),
                                 functools.partial(self._pose_feedback, target),
                                 functools.partial(self._report_pose, target))
            if code != EXIT_SUCCEEDED:
                left = [later.label for later in self.targets[number:]]
                if left:
                    print(f'Not sent: {", ".join(left)}')
                return code
        return EXIT_SUCCEEDED

    def drive_through(self):
        """Send one NavigateThroughPoses goal over every target."""
        print(f'One goal through {plural(len(self.targets), "pose")}, NavigateThroughPoses: '
              f'{", ".join(target.label for target in self.targets)}')
        return self.run_goal(f'through {plural(len(self.targets), "pose")}',
                             functools.partial(self.nav.goThroughPoses,
                                               [target.pose() for target in self.targets]),
                             self._through_feedback,
                             functools.partial(self._report_pose, self.targets[-1]))

    def follow_waypoints(self):
        """Send one FollowWaypoints goal over every target."""
        print(f'One goal with {plural(len(self.targets), "waypoint")}, FollowWaypoints: '
              f'{", ".join(target.label for target in self.targets)}')
        return self.run_goal('waypoints',
                             functools.partial(self.nav.followWaypoints,
                                               [target.pose() for target in self.targets]),
                             self._waypoint_feedback, self._report_waypoints)

    def run_goal(self, label, send, describe, report):
        """
        Run one goal, and return the exit code for it.

        send() sends the goal and returns whether Nav2 accepted it; describe(feedback, seconds)
        is the feedback line printed once a second; report(status, result, seconds) prints the
        result and returns the exit code. After a Ctrl-C the goal is cancelled (130).
        """
        self.label = label
        self.sending = True
        accepted = send()      # BasicNavigator waits for the action server and for the answer
        self.sending = False
        if not accepted:
            print(f"{label}: Nav2 rejected the goal (Nav2's log says why)")
            return EXIT_CTRL_C if self.ctrl_c.pressed else EXIT_FAILED
        self.goal_active = True
        started = time.monotonic()
        next_line = started + FEEDBACK_PERIOD
        # isTaskComplete() spins for up to 0.1 s and stores the feedback that arrives.
        while not self.nav.isTaskComplete():
            if self.ctrl_c.pressed:
                return self.cancel()
            now = time.monotonic()
            if now >= next_line:
                feedback = self.nav.getFeedback()
                if feedback is not None:
                    print(describe(feedback, now - started))
                next_line = now + FEEDBACK_PERIOD
        self.goal_active = False
        response = self.nav.result_future.result()
        code = report(response.status, response.result, time.monotonic() - started)
        # A Ctrl-C as the goal ended: report the result, but send no further goal.
        return EXIT_CTRL_C if self.ctrl_c.pressed else code

    def cancel(self):
        """After the first Ctrl-C: cancel the goal, see Nav2 confirm it, and return 130."""
        print(f'Ctrl-C: cancelling {self.label}')
        asked = time.monotonic()
        answer = self.nav.goal_handle.cancel_goal_async()
        self.cancel_sent = True
        if not self.spin_until(answer.done, CANCEL_WAIT, interruptible=False):
            print(f'Nav2 did not answer the cancel request within {CANCEL_WAIT:g} s. '
                  f'{self._cancel_all_hint()}', file=sys.stderr)
            return EXIT_CTRL_C
        code = answer.result().return_code
        if code == CancelGoal.Response.ERROR_NONE:
            print(f'Nav2 accepted the cancel after {time.monotonic() - asked:.2f} s')
        elif code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
            print('The goal had ended already')
        else:
            print(f'Nav2 refused the cancel (CancelGoal return code {code})', file=sys.stderr)
        ended = self.nav.result_future
        if not self.spin_until(ended.done, CANCELED_WAIT, interruptible=False):
            print(f'The goal had not ended {CANCELED_WAIT:g} s after the cancel request. '
                  f'{self._cancel_all_hint()}', file=sys.stderr)
            return EXIT_CTRL_C
        self.goal_active = False
        status = ended.result().status
        print(f'{self.label}: {RESULT_NAMES.get(status, f"status {status}")}, '
              f'{time.monotonic() - asked:.2f} s after the cancel request')
        return EXIT_CTRL_C

    def forced_exit(self):
        """After the second Ctrl-C: stop waiting at once, and return 130."""
        if self.goal_active and not self.cancel_sent:
            # Both presses came before the tool saw the first: cancel now, and leave without
            # waiting for the answer. The request goes out with this call.
            self.nav.goal_handle.cancel_goal_async()
            self.cancel_sent = True
            print(f'Second Ctrl-C: sent the cancel request for {self.label}, and did not wait '
                  'for the answer', file=sys.stderr)
        elif self.goal_active:
            print('Second Ctrl-C: the cancel request had gone out; stopped waiting for the goal '
                  'to end', file=sys.stderr)
        elif self.sending:
            print(f'Second Ctrl-C while {self.label} was being sent: if Nav2 accepted it, the '
                  f'robot drives on. {self._cancel_all_hint()}', file=sys.stderr)
        else:
            print('Second Ctrl-C: stopped', file=sys.stderr)
        return EXIT_CTRL_C

    def _cancel_all_hint(self):
        # A CancelGoal request with a zero goal ID and a zero stamp cancels every goal.
        service = f'{self.prefix}{self.action_name}/_action/cancel_goal'
        return (f'To cancel every goal of {self.prefix}{self.action_name}: '
                f'ros2 service call {service} action_msgs/srv/CancelGoal "{{}}"')

    # Feedback and results.

    def _pose_feedback(self, target, feedback, elapsed):
        return (f'  {target.label}: {feedback.distance_remaining:.1f} m to go, '
                f'about {seconds(feedback.estimated_time_remaining):.0f} s left '
                f'({seconds(feedback.navigation_time):.0f} s so far, '
                f'recoveries {feedback.number_of_recoveries})')

    def _through_feedback(self, feedback, elapsed):
        count = len(self.targets)
        left = feedback.number_of_poses_remaining
        upcoming = f', next {self.targets[count - left].label}' if 0 < left <= count else ''
        return (f'  {left} of {count} poses to go{upcoming}: '
                f'{feedback.distance_remaining:.1f} m to go, '
                f'about {seconds(feedback.estimated_time_remaining):.0f} s left '
                f'({seconds(feedback.navigation_time):.0f} s so far, '
                f'recoveries {feedback.number_of_recoveries})')

    def _waypoint_feedback(self, feedback, elapsed):
        index = feedback.current_waypoint
        count = len(self.targets)
        name = f', {self.targets[index].label}' if index < count else ''
        return f'  waypoint {index + 1} of {count}{name} ({elapsed:.0f} s so far)'

    def _report_pose(self, target, status, result, took):
        """Print the result of a NavigateToPose or NavigateThroughPoses goal."""
        if status != GoalStatus.STATUS_SUCCEEDED:
            return self._report_failure(self.label, status, result, took)
        line = f'{self.label}: SUCCEEDED after {took:.1f} s'
        feedback = self.nav.getFeedback()
        if feedback is not None:
            # Where Nav2 last saw the robot (AMCL's estimate): within the goal tolerance.
            pose = feedback.current_pose.pose
            distance = math.hypot(pose.position.x - target.x, pose.position.y - target.y)
            turn = abs(math.remainder(yaw_of(pose.orientation) - target.yaw, math.tau))
            line += (f', the robot {distance:.2f} m and {turn:.2f} rad from {target.label} '
                     "by Nav2's last feedback")
        print(line)
        return EXIT_SUCCEEDED

    def _report_waypoints(self, status, result, took):
        """Print the result of the FollowWaypoints goal, with every missed waypoint."""
        missed = list(result.missed_waypoints) if result is not None else []
        total = len(self.targets)
        for waypoint in missed:
            index = waypoint.index
            name = self.targets[index].label if index < total else f'index {index}'
            print(f'  waypoint {index + 1} of {total}, {name}: missed, '
                  f'{describe_error(waypoint.error_code)}')
            if waypoint.error_code in START_OCCUPIED:
                print(f'    {START_OCCUPIED_HINT}')
        if status != GoalStatus.STATUS_SUCCEEDED:
            return self._report_failure(self.label, status, result, took)
        if missed:
            print(f'{self.label}: SUCCEEDED after {took:.1f} s, but it missed {len(missed)} of '
                  f'{total} waypoints')
            return EXIT_FAILED
        print(f'{self.label}: SUCCEEDED after {took:.1f} s, all {total} reached')
        return EXIT_SUCCEEDED

    def _report_failure(self, label, status, result, took):
        """Print why a goal did not succeed; return EXIT_FAILED."""
        if status == GoalStatus.STATUS_CANCELED:
            print(f'{label}: CANCELED after {took:.1f} s by another client, not by this tool')
            return EXIT_FAILED
        code = getattr(result, 'error_code', 0)
        message = getattr(result, 'error_msg', '') or 'no error message'
        print(f'{label}: {RESULT_NAMES.get(status, f"status {status}")} after {took:.1f} s, '
              f'{describe_error(code)}, {message}')
        if code in START_OCCUPIED:
            print(f'  {START_OCCUPIED_HINT}')
        elif code == 0 and status == GoalStatus.STATUS_ABORTED:
            print('  Nav2 gave no error code: a goal from another client (RViz, another '
                  'navigate) may have replaced this one')
        return EXIT_FAILED


def describe_error(code):
    """Return 'error code 205 START_OCCUPIED (planner)' for one of Nav2's error codes."""
    if code == 0:
        return 'error code 0 (none)'
    for source, result_type in ERROR_SOURCES:
        # The generated message classes keep their constants as class attributes.
        for name, value in vars(result_type).items():
            if name.isupper() and isinstance(value, int) and value == code:
                return f'error code {code} {name} ({source})'
    return f'error code {code}'


def plural(number, noun):
    """Return '1 pose' or '3 poses'."""
    return f'{number} {noun}' if number == 1 else f'{number} {noun}s'


def seconds(duration):
    """Return a builtin_interfaces/Duration in seconds."""
    return duration.sec + duration.nanosec * 1e-9


def yaw_of(orientation):
    """Return the yaw of a quaternion, in radians."""
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def listing(registry):
    """Return what --list prints: every location of the registry, grouped by kind."""
    width = max(len(name) for name in registry.locations)
    lines = [f'{len(registry.locations)} named poses in {registry.path}',
             f'(world {registry.world}, frame {locations.FRAME_ID}; x and y in metres, yaw in '
             'radians)']
    for kind in locations.KINDS:
        group = registry.of_kind(kind)
        if not group:
            continue
        lines += ['', f'{kind}: {len(group)}, {KIND_TITLES[kind]}']
        for location in group:
            notes = [_detail_text(key, value) for key, value in location.details]
            if location.docking_only:
                notes.append('docking only')
            lines.append(f'  {location.name:<{width}}  x {location.x:7.3f}  y {location.y:7.3f}  '
                         f'yaw {location.yaw:6.3f}  {", ".join(notes)}'.rstrip())
    return '\n'.join(lines)


def _detail_text(key, value):
    if isinstance(value, list):
        value = ' '.join(str(item) for item in value)
    return f'{key} {value}'


def resolve(goals, registry):
    """
    Return the goals of --to and --pose as Targets, in the order given.

    Raises ValueError for a location that is not in the registry or is docking only.
    """
    targets = []
    for goal in goals:
        if isinstance(goal, str):
            targets.append(_named(goal, registry))
        else:
            x, y, yaw = goal
            targets.append(Target(f'pose ({x:g}, {y:g}, {yaw:g})', x, y, yaw))
    return targets


def _named(name, registry):
    location = registry.locations.get(name)
    if location is None:
        close = difflib.get_close_matches(name, registry.locations, n=3)
        hint = f'; did you mean {" or ".join(close)}?' if close else ''
        raise ValueError(f'unknown location {name!r} (navigate --list prints them){hint}')
    if location.docking_only:
        staging = registry.staging_pose_of(location)
        instead = (f' Send the robot to {staging}, in front of the dock, instead.'
                   if staging else '')
        raise ValueError(f'{name} is docking only: the robot gets there by docking (the docking '
                         f'server, Phase 5), not with Nav2.{instead}')
    return Target(name, location.x, location.y, location.yaw)


def _number(text):
    """Return a command-line number as a finite float (an argparse type)."""
    try:
        value = float(text)
    except ValueError:
        value = math.nan
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f'{text!r} is not a number')
    return value


def _parser():
    parser = argparse.ArgumentParser(
        prog='navigate',
        description='Send the robot to named locations of the location registry, or to poses '
                    'on the map, with Nav2. Needs the simulation with navigation:=true, except '
                    'for --list.',
        epilog='Several goals without --through or --waypoints become one NavigateToPose goal '
               'each, sent once the one before succeeded. Exit codes: 0 every goal succeeded; '
               '1 a goal failed, was rejected or was cancelled by another client, a waypoint '
               f'was missed, or AMCL and Nav2 were not ready within {NAV2_WAIT:g} s; 2 a usage '
               'error; 130 Ctrl-C (the goal is cancelled first).')
    parser.add_argument('--to', dest='goals', action='append', metavar='LOCATION',
                        help='a named location (see --list); repeat it for several goals, '
                             'driven in the order given')
    parser.add_argument('--pose', dest='goals', action='append', nargs=3, type=_number,
                        metavar=('X', 'Y', 'YAW'),
                        help='a pose on the map: X and Y in metres, YAW in radians, '
                             'counter-clockwise from +x (1.5708 faces +y); repeatable, and in '
                             'order with --to')
    order = parser.add_mutually_exclusive_group()
    order.add_argument('--through', action='store_true',
                       help='one NavigateThroughPoses goal: drive through every pose without '
                            'stopping, and stop at the last')
    order.add_argument('--waypoints', action='store_true',
                       help='one FollowWaypoints goal: stop at each pose, and go on after one '
                            'that cannot be reached')
    parser.add_argument('--list', action='store_true',
                        help='print the named locations and exit (needs no ROS graph)')
    parser.add_argument('--namespace', default='robot1',
                        help='the robot (default %(default)s)')
    return parser


def main(argv=None):
    """Run the navigate command; return its exit code."""
    # One line at a time, also into a pipe or a file: the feedback is a progress report.
    sys.stdout.reconfigure(line_buffering=True)
    argv = sys.argv if argv is None else ['navigate', *argv]
    parser = _parser()
    args = parser.parse_args(remove_ros_args(argv)[1:])    # a usage error exits with 2 here
    try:
        registry = locations.load()
    except locations.RegistryError as error:
        print(f'navigate: error: {error}', file=sys.stderr)
        return EXIT_USAGE
    if args.list:
        if args.goals or args.through or args.waypoints:
            parser.error('--list takes no goals and no --through or --waypoints')
        print(listing(registry))
        return EXIT_SUCCEEDED
    if not args.goals:
        parser.error('give a goal with --to LOCATION or --pose X Y YAW, or use --list')
    try:
        targets = resolve(args.goals, registry)
    except ValueError as error:
        parser.error(str(error))
    order = 'through' if args.through else 'waypoints' if args.waypoints else 'each'
    return run(argv, args.namespace.strip('/'), targets, order)


def run(argv, namespace, targets, order):
    """Start ROS, run the goals and shut ROS down again; return the exit code."""
    ctrl_c = CtrlC()
    ctrl_c.install()
    # No rclpy signal handlers: they would shut the context down at the first Ctrl-C, before
    # the goal could be cancelled. CtrlC handles SIGINT and SIGTERM instead.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    tool = None
    try:
        tool = Navigate(BasicNavigator(node_name='navigate', namespace=namespace), ctrl_c,
                        targets, order)
        return tool.run()
    except Interrupted:
        print('Ctrl-C: stopped before a goal was sent')
        return EXIT_CTRL_C
    except NotReady as error:
        print(f'navigate: {error}', file=sys.stderr)
        return EXIT_FAILED
    except KeyboardInterrupt:
        return tool.forced_exit() if tool else EXIT_CTRL_C
    except Exception:
        # A second Ctrl-C that lands inside rclpy's C++ code can come out of it as another
        # exception (drive_route saw a RuntimeError); it still means "stop now".
        if ctrl_c.count >= 2:
            return tool.forced_exit() if tool else EXIT_CTRL_C
        raise
    finally:
        ctrl_c.ignore()
        if tool is not None:
            tool.nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
