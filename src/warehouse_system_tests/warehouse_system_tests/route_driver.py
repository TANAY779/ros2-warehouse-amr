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
Drive the robot along a route of the location registry, steering on ground truth.

Usage::

    ros2 run warehouse_system_tests drive_route --route survey [--speed 0.4] [--turn-rate 0.5]
        [--settle SECONDS] [--initial-pose-offset DX DY DYAW_DEG] [--namespace robot1]
        [--dry-run]

A test tool, never part of the robot stack (plan P2-D11). It steers on
/<namespace>/ground_truth/odom, the true pose Gazebo reports, so a mapping run or an
evaluation run follows the same path every time, whatever SLAM or AMCL make of it.

The routes are the ``routes`` of the location registry (warehouse_layout/generated/
locations.yaml, generated from layout.yaml). The layout's route fit rule keeps every waypoint
and every segment clear of the racks, so the robot must stay on the segments: the driver
follows the line between two waypoints, not just the direction to the next one.
``--route none`` drives nothing: use it to set a wrong initial pose, then drive by teleop.

What it does, in order:

1. Waits for ground truth, and with it the simulation clock.
2. With ``--initial-pose-offset DX DY DYAW_DEG``: waits until AMCL reports the lifecycle state
   active, then publishes a pose on /<namespace>/initialpose: the true pose moved by DX and
   DY metres (map frame) and turned by DYAW_DEG degrees, with a covariance of 0.5^2 m^2 and
   0.2^2 rad^2. AMCL then starts from a wrong pose and has to find the right one (plan F7).
   The pose goes out once the publisher is DISCOVERY_SETTLE (3 s) old, and again until AMCL
   answers it on amcl_pose (see RouteDriver.set_initial_pose).
3. With ``--settle SECONDS``: stands still that long in sim time (the IMU noise check of F7
   needs a still robot).
4. Waits until diff_drive_controller receives what the driver publishes on cmd_vel (see
   RouteDriver.wait_for_controller): commands published before that are lost or arrive too
   late. The first command goes out at least DISCOVERY_SETTLE (3 s) after the driver starts.
5. Drives the route: at each waypoint it turns in place to face along the next segment, then
   drives along it and stops on the next waypoint.

**Commands.** geometry_msgs/TwistStamped on /<namespace>/cmd_vel, 20 times a second while
driving, stamped with the simulation clock. diff_drive_controller brakes when its newest
command is older than ``cmd_vel_timeout`` (controllers.yaml), and it measures that age from the
header stamp, in sim time. So a driver must keep publishing, even when the command does not
change, and must stamp in sim time: a wall-clock stamp would never time out in simulation, or
always (plan R1). Between two commands 0.05 s pass, well inside the timeout.

**Steering.** Along the segment from waypoint A to waypoint B, the robot aims at the point
LOOKAHEAD metres ahead of its own projection on the line AB. That point lies on the line, so a
robot that drifted sideways steers back onto it. The yaw rate is proportional to the heading
error. The speed is the cruise speed, lowered near B so that the robot could still stop on B
braking at half the rate the command ramp allows (v = sqrt(2 a d); the other half is a reserve
for the robot's lag), lowered further in the last centimetres, and scaled by the cosine of the
heading error. Every command changes by at most half the
controller's acceleration limits per second, so diff_drive_controller never has to clip one:
what the driver commands is what the robot does.

Exit codes: 0 the route was driven (or there was nothing to drive); 1 it could not be (no
ground truth, AMCL not active or not answering the initial pose, diff_drive_controller not
receiving the commands, the robot not at the start of the route, no progress for 10 s, or the
simulation clock stopped); 2 a bad argument; 130 stopped by Ctrl-C. The robot is commanded to
stop before every exit.
"""

import argparse
from dataclasses import dataclass
import math
import os
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from warehouse_system_tests import ros_helpers as rh
import yaml

COMMAND_RATE = 20.0              # Hz: commands while driving
COMMAND_PERIOD = 1.0 / COMMAND_RATE
CONTROLLER = 'diff_drive_controller'   # subscribes to cmd_vel (controllers.yaml)
DEFAULT_SPEED = 0.4              # m/s (the mapping run, plan F5)
DEFAULT_TURN_RATE = 0.5          # rad/s: slow turns keep scan matching easy for SLAM
ACCELERATION_SHARE = 0.5         # of the controller's acceleration limits (never clipped)

# The controller (see the module docstring).
LOOKAHEAD = 0.6                  # m ahead on the segment line
HEADING_GAIN = 1.5               # rad/s of yaw rate per rad of heading error, driving
TURN_GAIN = 2.0                  # rad/s per rad at the end of a turn in place (no overshoot)
APPROACH_GAIN = 2.0              # m/s per m left, in the last centimetres before a waypoint
# Stops are planned with this share of the deceleration the command ramps allow. The robot
# lags behind its commands (pose and control delays); with room to brake harder than planned,
# the commands can still catch up and the robot stops on the waypoint. On a modelled robot whose
# pose and commands come 0.1 s late, at 1.2 m/s: 0.6 cm off the route, and 28 cm without this
# margin.
BRAKING_MARGIN = 0.5
FACING = math.radians(2.0)       # a turn in place ends below this heading error
REALIGN = math.radians(30.0)     # driving with a larger error: stop and turn in place again
REACHED = 0.03                   # m left along the segment: the waypoint is reached
START_TOLERANCE = 0.5            # m: the robot must start this close to the first waypoint
MAX_STEP = 0.2                   # s: the longest time step a command ramp accepts

# Progress: no step of this size towards the waypoint (distance) or towards its direction
# (angle) for STALL_TIMEOUT seconds of sim time means the robot is stuck.
STALL_TIMEOUT = 10.0             # s
PROGRESS_DISTANCE = 0.05         # m
PROGRESS_ANGLE = 0.05            # rad

# Waiting, in wall-clock seconds: these guard against a simulation that is not running.
TRUTH_TIMEOUT = 30.0             # for the first ground truth
CLOCK_TIMEOUT = 10.0             # for the simulation clock to move on
AMCL_TIMEOUT = 60.0              # for AMCL to become active
CONTROLLER_TIMEOUT = 30.0        # for diff_drive_controller to receive the commands
# The first command and the initial pose go out only once their publisher is this old: the
# subscriber's side of the DDS match can come later than ours (see
# RouteDriver.wait_for_controller).
DISCOVERY_SETTLE = 3.0           # s
STALE_POSE = 0.5                 # s of sim time: older ground truth is not used to steer

# The initial pose for AMCL: how unsure it is (standard deviations), plan F7.
INITIAL_POSE_SIGMA_XY = 0.5      # m
INITIAL_POSE_SIGMA_YAW = 0.2     # rad
# AMCL answers an initial pose at its next scan (10 Hz); without an answer the pose goes out
# again (see RouteDriver.set_initial_pose).
INITIAL_POSE_WAIT = 5.0          # s, wall clock
INITIAL_POSE_TRIES = 3


class DriveError(RuntimeError):
    """The route cannot be driven; the message says why. The command exits with 1."""


# ----------------------------------------------------------------------------- pure logic


@dataclass(frozen=True)
class Pose2D:
    """A pose in the map frame: x and y in metres, yaw in radians from +x."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Limits:
    """How fast the driver goes, turns and changes speed (all positive numbers)."""

    speed: float                 # m/s, cruise speed along a segment
    turn_rate: float             # rad/s, largest yaw rate
    acceleration: float          # m/s^2, speeding up
    deceleration: float          # m/s^2, braking
    angular_acceleration: float  # rad/s^2, both ways


def limits_for(speed, turn_rate, controller):
    """
    Return the Limits for a cruise speed and turn rate, checked against the controller.

    ``controller`` is diff_drive_controller's parameters (controllers.yaml). Raises ValueError
    for a speed or turn rate outside the controller's limits.
    """
    linear = controller['linear']['x']
    angular = controller['angular']['z']
    if not 0.0 < speed <= linear['max_velocity']:
        raise ValueError(f'--speed {speed:g} m/s: must be above 0 and at most '
                         f"{linear['max_velocity']:g} m/s (diff_drive_controller's limit)")
    if not 0.0 < turn_rate <= angular['max_velocity']:
        raise ValueError(f'--turn-rate {turn_rate:g} rad/s: must be above 0 and at most '
                         f"{angular['max_velocity']:g} rad/s (diff_drive_controller's limit)")
    return Limits(
        speed=speed,
        turn_rate=turn_rate,
        acceleration=ACCELERATION_SHARE * linear['max_acceleration'],
        # The controller writes a deceleration limit as a negative number.
        deceleration=ACCELERATION_SHARE * abs(linear['max_deceleration']),
        angular_acceleration=ACCELERATION_SHARE * angular['max_acceleration'])


def registry_path():
    """Return the path of the installed location registry (warehouse_layout)."""
    share = get_package_share_directory('warehouse_layout')
    return os.path.join(share, 'generated', 'locations.yaml')


def load_routes(path=None):
    """Return {route name: ((x, y), ...)} from the location registry (default: installed)."""
    with open(path or registry_path()) as stream:
        registry = yaml.safe_load(stream)
    routes = registry.get('routes') or {}
    return {name: tuple((float(x), float(y)) for x, y in points)
            for name, points in routes.items()}


def route_length(waypoints):
    """Return the length of a route in metres."""
    return sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))


def check_route(waypoints):
    """Raise ValueError unless the route has two waypoints or more, none repeated in a row."""
    if len(waypoints) < 2:
        raise ValueError('a route needs at least two waypoints')
    for number, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
        if math.dist(a, b) < 1e-6:
            raise ValueError(f'waypoints {number} and {number + 1} are the same point {a}')


def segment_errors(pose, a, b):
    """
    Return where a pose stands against the segment from waypoint a to waypoint b.

    Returns (remaining, cross, heading_error):
    ``remaining``: metres still to go along the segment (negative once past b);
    ``cross``: metres to the left of the line (negative: to the right);
    ``heading_error``: the turn, in radians, that would make the robot face the point
    LOOKAHEAD metres ahead of its projection on the line (positive: turn left).
    """
    length = math.dist(a, b)
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    dx, dy = pose.x - a[0], pose.y - a[1]
    along = ux * dx + uy * dy
    cross = ux * dy - uy * dx
    heading = math.atan2(uy, ux) + math.atan2(-cross, LOOKAHEAD)
    return length - along, cross, rh.wrap_angle(heading - pose.yaw)


def ramp(current, target, accelerate, decelerate):
    """
    Return ``current`` moved towards ``target`` by at most one step.

    The step is ``accelerate`` while the magnitude grows (speeding up) and ``decelerate``
    while it shrinks (slowing down).
    """
    speeding_up = abs(target) > abs(current) and target * current >= 0.0
    step = accelerate if speeding_up else decelerate
    return current + max(-step, min(step, target - current))


def turn_rate_for(heading_error, limits):
    """
    Return the yaw rate for a turn in place with ``heading_error`` radians still to turn.

    The smallest of: the turn rate; the rate from which the robot can still stop in time
    (sqrt(2 alpha angle), with the braking margin); and a rate proportional to the angle
    left, so the turn ends smoothly instead of overshooting.
    """
    left = abs(heading_error)
    braking = BRAKING_MARGIN * limits.angular_acceleration
    rate = min(limits.turn_rate, math.sqrt(2.0 * braking * left), TURN_GAIN * left)
    return math.copysign(rate, heading_error)


class RouteFollower:
    """
    The driver's controller: give it the pose, get the command. No ROS in here.

    States: TURNING (in place, towards the next segment), DRIVING (along it) and DONE.
    ``target`` is the index of the waypoint being driven to. ``stalled`` turns True after
    STALL_TIMEOUT seconds without progress; ``finished`` once the robot was commanded to
    stand still at the last waypoint.
    """

    TURNING = 'turning'
    DRIVING = 'driving'
    DONE = 'done'

    def __init__(self, waypoints, limits):
        check_route(waypoints)
        self.waypoints = tuple(waypoints)
        self.limits = limits
        self.target = 1
        self.state = self.TURNING
        self.v = 0.0                 # the last command, m/s
        self.w = 0.0                 # rad/s
        self.stalled = False
        self._time = None
        self._best = math.inf        # the best progress measure of this phase so far
        self._last_progress = None

    @property
    def finished(self):
        """Return True once the robot stands at the end of the route."""
        return self.state == self.DONE and self.v == 0.0 and self.w == 0.0

    def update(self, pose, now):
        """
        Return the command (v, w) for the robot at ``pose`` at time ``now`` (seconds).

        ``pose`` None means no fresh pose: the command is a stop, and no progress is made.
        """
        dt = COMMAND_PERIOD if self._time is None else min(max(now - self._time, 0.0),
                                                           MAX_STEP)
        self._time = now
        if self._last_progress is None:
            self._last_progress = now
        v, w = 0.0, 0.0
        if pose is not None and self.state != self.DONE:
            v, w = self._steer(pose, now)
        if self.state != self.DONE and now - self._last_progress > STALL_TIMEOUT:
            self.stalled = True
        limits = self.limits
        self.v = ramp(self.v, v, limits.acceleration * dt, limits.deceleration * dt)
        self.w = ramp(self.w, w, limits.angular_acceleration * dt,
                      limits.angular_acceleration * dt)
        return self.v, self.w

    def _steer(self, pose, now):
        remaining, _, error = self._errors(pose)
        if self.state == self.DRIVING and remaining <= REACHED:
            if self.target == len(self.waypoints) - 1:
                self.state = self.DONE
                return 0.0, 0.0
            self.target += 1
            self._new_phase(self.TURNING, now)
            remaining, _, error = self._errors(pose)
        if self.state == self.DRIVING and abs(error) > REALIGN:
            self._new_phase(self.TURNING, now)
        if self.state == self.TURNING:
            if abs(error) >= FACING:
                self._progress(abs(error), PROGRESS_ANGLE, now)
                return 0.0, turn_rate_for(error, self.limits)
            self._new_phase(self.DRIVING, now)
        self._progress(remaining, PROGRESS_DISTANCE, now)
        left = max(remaining, 0.0)
        limits = self.limits
        braking = BRAKING_MARGIN * limits.deceleration
        v = min(limits.speed, math.sqrt(2.0 * braking * left), APPROACH_GAIN * left)
        v *= max(math.cos(error), 0.0)
        w = max(-limits.turn_rate, min(limits.turn_rate, HEADING_GAIN * error))
        return v, w

    def _errors(self, pose):
        return segment_errors(pose, self.waypoints[self.target - 1], self.waypoints[self.target])

    def _new_phase(self, state, now):
        self.state = state
        self._best = math.inf
        self._last_progress = now

    def _progress(self, measure, step, now):
        """Count progress when ``measure`` (distance or angle left) beats its best by ``step``."""
        if measure < self._best - step:
            self._best = measure
            self._last_progress = now


def advance(pose, v, w, dt):
    """Return the pose of an ideal robot after driving (v, w) for dt seconds (exact arc)."""
    if abs(w) < 1e-9:
        return Pose2D(pose.x + v * dt * math.cos(pose.yaw), pose.y + v * dt * math.sin(pose.yaw),
                      pose.yaw)
    yaw = pose.yaw + w * dt
    radius = v / w
    return Pose2D(pose.x + radius * (math.sin(yaw) - math.sin(pose.yaw)),
                  pose.y - radius * (math.cos(yaw) - math.cos(pose.yaw)),
                  rh.wrap_angle(yaw))


@dataclass
class Trace:
    """What happened in simulate(): poses and commands at each tick, and the outcome."""

    times: list
    poses: list
    commands: list
    finished: bool
    stalled: bool

    @property
    def duration(self):
        """Return the simulated time from the first command to the last, in seconds."""
        return self.times[-1] - self.times[0] if self.times else 0.0


def simulate(waypoints, limits, start=None, max_time=3600.0):
    """
    Drive a route with an ideal robot that does exactly what it is commanded; return a Trace.

    The robot starts at ``start`` (default: on the first waypoint, facing +x, as robot1
    spawns). --dry-run reports the time this takes.
    """
    follower = RouteFollower(waypoints, limits)
    pose = start or Pose2D(waypoints[0][0], waypoints[0][1], 0.0)
    trace = Trace([], [], [], False, False)
    tick = 0
    while not (follower.finished or follower.stalled) and tick * COMMAND_PERIOD <= max_time:
        now = tick * COMMAND_PERIOD
        v, w = follower.update(pose, now)
        trace.times.append(now)
        trace.poses.append(pose)
        trace.commands.append((v, w))
        pose = advance(pose, v, w, COMMAND_PERIOD)
        tick += 1
    trace.finished = follower.finished
    trace.stalled = follower.stalled
    return trace


def perturbed_pose(pose, dx, dy, dyaw_deg):
    """Return a pose moved by (dx, dy) metres in the map frame and turned by dyaw degrees."""
    return Pose2D(pose.x + dx, pose.y + dy, rh.wrap_angle(pose.yaw + math.radians(dyaw_deg)))


def initial_pose_covariance():
    """
    Return the 6 x 6 covariance (row-major, 36 values) of the initial pose for AMCL.

    Order x, y, z, roll, pitch, yaw. AMCL spreads its particles with the x, y and yaw terms.
    """
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = INITIAL_POSE_SIGMA_XY ** 2
    covariance[35] = INITIAL_POSE_SIGMA_YAW ** 2
    return covariance


def describe(name, waypoints, limits, plan):
    """
    Return the --dry-run text: the route, the expected time and what would be published.

    ``plan`` holds the run's other settings: namespace, registry, timeout, settle, offset.
    """
    ns = plan['namespace']
    lines = []
    if waypoints:
        lines.append(f'route {name}: {len(waypoints)} waypoints, {route_length(waypoints):.1f} m '
                     f"(location registry {plan['registry']})")
        lines.append('    #    x [m]    y [m]   leg [m]  turn [deg]')
        headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                    for a, b in zip(waypoints, waypoints[1:])]
        # The robot starts facing +x (yaw 0), as robot1 spawns.
        previous = 0.0
        for index, (x, y) in enumerate(waypoints):
            leg = f'{math.dist(waypoints[index - 1], (x, y)):9.2f}' if index else ' ' * 9
            turn = ''
            if index < len(headings):
                turn = f'{math.degrees(rh.wrap_angle(headings[index] - previous)):+11.0f}'
                previous = headings[index]
            lines.append(f'  {index:3d} {x:8.2f} {y:8.2f} {leg} {turn}'.rstrip())
        trace = simulate(waypoints, limits)
        lines.append(f'drive: {limits.speed:g} m/s, turns at {limits.turn_rate:g} rad/s, speeding '
                     f'up at {limits.acceleration:g} m/s^2, braking at {limits.deceleration:g} '
                     "m/s^2 (half of diff_drive_controller's limits)")
        settle = f", after standing still for {plan['settle']:g} s" if plan['settle'] else ''
        lines.append(f'expected: about {rh.duration_text(trace.duration)} of sim time{settle} '
                     '(the route driven by an ideal robot that does exactly what it is told)')
        if not trace.finished:
            lines.append('warning: the ideal robot did not reach the end of the route')
        lines.append(f'commands: geometry_msgs/TwistStamped on /{ns}/cmd_vel at '
                     f"{COMMAND_RATE:g} Hz, stamped in sim time (the controller's "
                     f"cmd_vel_timeout is {plan['timeout']:g} s)")
        lines.append(f'steering: nav_msgs/Odometry on /{ns}/ground_truth/odom')
    else:
        lines.append('route none: nothing to drive')
        if plan['settle']:
            lines.append(f"stand still for {plan['settle']:g} s of sim time")
    offset = plan['offset']
    if offset:
        lines.append(f'initial pose: once /{ns}/amcl is active, publish on /{ns}/initialpose '
                     f'the true pose moved by ({offset[0]:+g}, {offset[1]:+g}) m and turned by '
                     f'{offset[2]:+g} deg, covariance {INITIAL_POSE_SIGMA_XY:g}^2 m^2 and '
                     f'{INITIAL_POSE_SIGMA_YAW:g}^2 rad^2')
    else:
        lines.append('initial pose: not set (AMCL keeps its own)')
    lines.append('dry run: nothing was published')
    return '\n'.join(lines)


# ----------------------------------------------------------------------------- ROS


def pose_of(odometry):
    """Return the Pose2D of a nav_msgs/Odometry message."""
    pose = odometry.pose.pose
    return Pose2D(pose.position.x, pose.position.y, rh.yaw_of(pose.orientation))


def pose_text(pose):
    """Return a pose as text, for example '(1.50, 0.00, 90.0 deg)'."""
    return f'({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.yaw):.1f} deg)'


class RouteDriver(Node):
    """The ROS side of drive_route: ground truth in, stamped commands and the initial pose out."""

    def __init__(self, namespace, cmd_vel_timeout):
        # Sim time, always: commands must be stamped with the controller's clock (plan R1),
        # and ground truth exists only in simulation anyway.
        super().__init__('drive_route', namespace=namespace,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.truth = None
        # Only the newest pose matters: sensor-data QoS (best effort, a short queue), which
        # also matches the bridge's reliable publisher.
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth,
                                 qos_profile_sensor_data)
        self.cmd_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._cmd_vel_created = time.monotonic()
        self.cmd_vel_timeout = cmd_vel_timeout    # s, for the log
        self.follower = None
        self._driven = 0.0           # metres, for the summary
        self._previous = None

    def _on_truth(self, message):
        self.truth = message

    def now_s(self):
        """Return the node's time (sim time) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def truth_pose(self, now):
        """Return the newest ground-truth Pose2D, or None if there is none from STALE_POSE s."""
        if self.truth is None or now - rh.stamp_to_seconds(self.truth.header.stamp) > STALE_POSE:
            return None
        return pose_of(self.truth)

    def publish_command(self, v, w):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_footprint'
        command.twist.linear.x = float(v)
        command.twist.angular.z = float(w)
        self.cmd_vel.publish(command)

    def stop(self):
        """Command a stop, and give the middleware a moment to deliver it."""
        self.publish_command(0.0, 0.0)
        rh.spin_for(self, 0.1)
        self.publish_command(0.0, 0.0)
        rh.spin_for(self, 0.1)

    def spin_while(self, busy):
        """Process callbacks while busy() is true; fail if the simulation clock stops."""
        last_sim, last_wall = self.now_s(), time.monotonic()
        while busy():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.now_s()
            if now != last_sim:
                last_sim, last_wall = now, time.monotonic()
            elif time.monotonic() - last_wall > CLOCK_TIMEOUT:
                raise DriveError(f'the simulation clock has not moved for {CLOCK_TIMEOUT:g} s: '
                                 'is Gazebo paused or gone?')

    def run(self, name, waypoints, limits, settle, offset):
        """Wait for ground truth, set the initial pose, settle, then drive the route."""
        # Sim time is 0 until the first /clock message arrives.
        if not rh.spin_until(self, lambda: self.truth is not None and self.now_s() > 0.0,
                             TRUTH_TIMEOUT):
            if self.truth is None:
                raise DriveError(
                    f'no ground truth on {self.resolve_topic_name("ground_truth/odom")} after '
                    f'{TRUTH_TIMEOUT:g} s: is the simulation running (hardware:=gz) in this '
                    'namespace?')
            raise DriveError(f'no /clock after {TRUTH_TIMEOUT:g} s: the simulation clock is not '
                             'bridged to ROS')
        self.get_logger().info(f'ground truth: the robot is at {pose_text(pose_of(self.truth))}')
        if offset:
            self.set_initial_pose(offset)
        if settle > 0.0:
            self.get_logger().info(f'standing still for {settle:g} s of sim time')
            end = self.now_s() + settle
            self.spin_while(lambda: self.now_s() < end)
        if waypoints:
            self.drive(name, waypoints, limits)

    def wait_for_controller(self):
        """
        Wait until diff_drive_controller receives what this node publishes on cmd_vel.

        DDS delivers a message only between a publisher and a subscription that discovery has
        matched, a fraction of a second after the publisher is created, and it matches the two
        ends separately. Until the subscription's end knows the publisher, it drops what
        arrives; cmd_vel is reliable, so the publisher sends the dropped commands again later,
        but by then they are older than cmd_vel_timeout and the controller ignores them. In one
        mapping run the first four commands reached the controller and a bag recorder 0.3 to
        0.9 s late, all at the same moment, as such a resend does, and the robot started late,
        behind the driver's speed ramps.

        The publisher's end can be checked: get_subscription_count() counts the subscriptions
        it is matched with. One is not enough proof, because a bag recorder may subscribe to
        cmd_vel too and be matched first, so this waits until ROS lists the controller's
        subscription and the publisher is matched with every subscription listed. The
        subscription's end cannot be checked from here, so this then waits, still processing
        callbacks, until the publisher is DISCOVERY_SETTLE seconds old.
        """
        topic = self.cmd_vel.topic_name
        started = time.monotonic()

        def matched():
            listed = self.get_subscriptions_info_by_topic(topic)
            return (any(endpoint.node_name == CONTROLLER for endpoint in listed)
                    and self.cmd_vel.get_subscription_count() >= len(listed))

        if not rh.spin_until(self, matched, CONTROLLER_TIMEOUT):
            raise DriveError(
                f'{CONTROLLER} does not receive {topic} after {CONTROLLER_TIMEOUT:g} s '
                f'({self.cmd_vel.get_subscription_count()} of {self.count_subscribers(topic)} '
                'subscriptions matched): is the controller active in this namespace?')
        rh.spin_for(self, DISCOVERY_SETTLE - (time.monotonic() - self._cmd_vel_created))
        self.get_logger().info(
            f'{CONTROLLER} receives {topic} ({self.cmd_vel.get_subscription_count()} '
            f'subscriptions matched; waited {time.monotonic() - started:.2f} s)')

    def wait_for_amcl(self):
        """Wait until AMCL's lifecycle state is active (it ignores initial poses before)."""
        client = self.create_client(GetState, 'amcl/get_state')
        self.get_logger().info(f'waiting for {self.resolve_service_name(client.srv_name)} to '
                               'report active')
        deadline = time.monotonic() + AMCL_TIMEOUT
        try:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                    result = future.result()
                    if result and result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        return
                rh.spin_for(self, 0.5)
        finally:
            self.destroy_client(client)
        raise DriveError(f'AMCL is not active after {AMCL_TIMEOUT:g} s: start the simulation '
                         'with localization:=amcl')

    def set_initial_pose(self, offset):
        """
        Give AMCL the true pose moved by ``offset`` (dx, dy, dyaw_deg), once it is active.

        AMCL subscribes to initialpose best effort (the system default QoS), so DDS never sends
        a lost pose again: a pose that reaches AMCL before its end of the match knows this
        publisher is dropped for good (wait_for_controller explains the match). So the pose
        goes out only once the publisher is DISCOVERY_SETTLE seconds old, and AMCL's answer
        proves that it arrived: AMCL publishes amcl_pose after every initial pose, and a robot
        standing still gives it no other reason to. Without an answer the pose goes out again,
        up to INITIAL_POSE_TRIES times.
        """
        self.wait_for_amcl()
        publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        created = time.monotonic()
        # Listen for AMCL's answers from now on, so the subscription is matched in time too.
        # Best effort matches any publisher QoS.
        estimates = []
        subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', estimates.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        if not rh.spin_until(self, lambda: publisher.get_subscription_count() > 0, 10.0):
            raise DriveError(f'nothing subscribes to {publisher.topic_name}')
        rh.spin_for(self, DISCOVERY_SETTLE - (time.monotonic() - created))

        for attempt in range(1, INITIAL_POSE_TRIES + 1):
            truth = self.truth
            pose = perturbed_pose(pose_of(truth), *offset)
            message = PoseWithCovarianceStamped()
            # The stamp of the true pose it was made from: AMCL adds the odometry since then.
            message.header.stamp = truth.header.stamp
            message.header.frame_id = 'map'      # AMCL ignores an initial pose in another frame
            message.pose.pose.position.x = pose.x
            message.pose.pose.position.y = pose.y
            message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
            message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
            message.pose.covariance = initial_pose_covariance()
            estimates.clear()
            publisher.publish(message)
            self.get_logger().info(
                f'initial pose for AMCL on {publisher.topic_name} (try {attempt} of '
                f'{INITIAL_POSE_TRIES}): {pose_text(pose)}, the true pose '
                f'{pose_text(pose_of(truth))} moved by ({offset[0]:+g}, {offset[1]:+g}) m and '
                f'turned by {offset[2]:+g} deg')
            if rh.spin_until(self, lambda: bool(estimates), INITIAL_POSE_WAIT):
                break
        else:
            raise DriveError(f'AMCL did not answer the initial pose ({INITIAL_POSE_TRIES} tries, '
                             f'{INITIAL_POSE_WAIT:g} s each): is the lidar running?')
        estimate = estimates[0].pose.pose
        amcl = Pose2D(estimate.position.x, estimate.position.y, rh.yaw_of(estimate.orientation))
        true = pose_of(self.truth)
        self.get_logger().info(
            f'AMCL took it: it reports {pose_text(amcl)}, '
            f'{math.hypot(amcl.x - true.x, amcl.y - true.y):.2f} m and '
            f'{abs(math.degrees(rh.wrap_angle(amcl.yaw - true.yaw))):.1f} deg from the truth')
        self.destroy_subscription(subscription)
        self.destroy_publisher(publisher)

    def drive(self, name, waypoints, limits):
        """Drive the route; return when the robot stands at its end."""
        start = self.truth_pose(self.now_s())
        if start is None:
            raise DriveError('ground truth stopped arriving')
        away = math.dist((start.x, start.y), waypoints[0])
        if away > START_TOLERANCE:
            raise DriveError(
                f'the robot is {away:.2f} m from the start of route {name} '
                f'({waypoints[0][0]:g}, {waypoints[0][1]:g}); it must start within '
                f'{START_TOLERANCE:g} m of it: restart the simulation, or move the robot there')
        self.wait_for_controller()
        self.follower = RouteFollower(waypoints, limits)
        self._previous = start
        self.get_logger().info(
            f'driving route {name}: {len(waypoints)} waypoints, {route_length(waypoints):.1f} m '
            f'at {limits.speed:g} m/s; commands on {self.cmd_vel.topic_name} at '
            f'{COMMAND_RATE:g} Hz (cmd_vel_timeout {self.cmd_vel_timeout:g} s)')
        started_sim, started_wall = self.now_s(), time.monotonic()
        timer = self.create_timer(COMMAND_PERIOD, self._on_timer)
        try:
            self.spin_while(lambda: not (self.follower.finished or self.follower.stalled))
        finally:
            self.destroy_timer(timer)
        if self.follower.stalled:
            target = waypoints[self.follower.target]
            raise DriveError(
                f'no progress for {STALL_TIMEOUT:g} s while {self.follower.state} towards '
                f'waypoint {self.follower.target} ({target[0]:g}, {target[1]:g}); the robot is '
                f'at {pose_text(pose_of(self.truth))}: blocked?')
        self.stop()
        end = pose_of(self.truth)
        sim = rh.duration_text(self.now_s() - started_sim)
        wall = rh.duration_text(time.monotonic() - started_wall)
        self.get_logger().info(
            f'route {name} done: {self._driven:.1f} m in {sim} of sim time ({wall} of wall '
            f'time); the robot stopped {math.dist((end.x, end.y), waypoints[-1]):.2f} m from '
            'the last waypoint')

    def _on_timer(self):
        follower = self.follower
        now = self.now_s()
        pose = self.truth_pose(now)
        target, state = follower.target, follower.state
        v, w = follower.update(pose, now)
        self.publish_command(v, w)
        if pose is not None:
            self._driven += math.dist((pose.x, pose.y), (self._previous.x, self._previous.y))
            self._previous = pose
        just_done = follower.state == follower.DONE and state != follower.DONE
        if follower.target != target or just_done:
            x, y = follower.waypoints[target]
            self.get_logger().info(f'waypoint {target} of {len(follower.waypoints) - 1} '
                                   f'({x:g}, {y:g}) reached, {self._driven:.1f} m driven')


# ----------------------------------------------------------------------------- the command


def _parser():
    parser = argparse.ArgumentParser(
        prog='drive_route',
        description='Drive the robot along a route of the location registry, steering on '
                    'ground truth (simulation only; a test tool, never part of the robot '
                    'stack). Commands are TwistStamped at 20 Hz, stamped in sim time.')
    parser.add_argument('--route', required=True, metavar='NAME',
                        help="a route of the registry (survey, smoke), or 'none' to drive "
                             'nothing')
    parser.add_argument('--speed', type=float, default=DEFAULT_SPEED, metavar='M_PER_S',
                        help='cruise speed in m/s (default %(default)s)')
    parser.add_argument('--turn-rate', type=float, default=DEFAULT_TURN_RATE,
                        metavar='RAD_PER_S',
                        help='yaw rate of turns in place in rad/s (default %(default)s)')
    parser.add_argument('--settle', type=float, default=0.0, metavar='SECONDS',
                        help='stand still this many seconds of sim time before driving (for '
                             'the IMU noise check)')
    parser.add_argument('--initial-pose-offset', type=float, nargs=3,
                        metavar=('DX', 'DY', 'DYAW_DEG'),
                        help='once AMCL is active, publish the true pose moved by DX, DY metres '
                             '(map frame) and turned by DYAW_DEG degrees as its initial pose')
    parser.add_argument('--namespace', default='robot1',
                        help='the robot namespace (default %(default)s)')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the route, its length and the expected time, and exit '
                             'without touching the ROS graph')
    return parser


def _interrupt(signum, frame):
    """Turn SIGTERM into KeyboardInterrupt, so that it stops the robot like Ctrl-C does."""
    raise KeyboardInterrupt


def main(argv=None):
    """Run the drive_route command; return its exit code."""
    argv = sys.argv if argv is None else ['drive_route', *argv]
    args = _parser().parse_args(remove_ros_args(argv)[1:])
    namespace = args.namespace.strip('/')
    try:
        path = registry_path()
        routes = load_routes(path)
        controller = rh.diff_drive_parameters()
        limits = limits_for(args.speed, args.turn_rate, controller)
    except (OSError, KeyError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    if args.route != 'none' and args.route not in routes:
        names = ', '.join([*routes, 'none'])
        print(f'error: no route {args.route!r} in {path}; routes: {names}', file=sys.stderr)
        return 2
    if args.settle < 0.0:
        print('error: --settle must not be negative', file=sys.stderr)
        return 2
    waypoints = routes.get(args.route, ())
    if waypoints:
        try:
            check_route(waypoints)
        except ValueError as error:
            print(f'error: route {args.route} in {path}: {error}', file=sys.stderr)
            return 2

    if args.dry_run:
        print(describe(args.route, waypoints, limits, {
            'namespace': namespace, 'registry': path, 'settle': args.settle,
            'offset': args.initial_pose_offset, 'timeout': controller['cmd_vel_timeout']}))
        return 0

    # No rclpy signal handlers: Ctrl-C raises KeyboardInterrupt while the node can still
    # publish, so the driver can stop the robot before it exits.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)
    node = RouteDriver(f'/{namespace}', controller['cmd_vel_timeout'])
    try:
        node.run(args.route, waypoints, limits, args.settle, args.initial_pose_offset)
        return 0
    except DriveError as error:
        node.stop()
        node.get_logger().error(str(error))
        return 1
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().warning('interrupted: robot stopped')
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
