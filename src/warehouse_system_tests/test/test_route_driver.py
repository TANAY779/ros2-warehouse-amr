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
The route driver's logic without a ROS graph: geometry, the controller, the dry run.

simulate() drives a route with an ideal robot that does exactly what it is commanded. The
tests check what a real run relies on: every waypoint is reached, the robot stays on the
segments (the route fit rule leaves 0.16 m beyond its half width), it turns in place before it
drives, its commands stay within half the controller's limits, a robot that lags behind its
commands still stops on the waypoints, and a robot that cannot move is given up on after 10 s.
"""

import collections
import math

import pytest
from warehouse_system_tests import ros_helpers as rh
from warehouse_system_tests import route_driver as rd
from warehouse_system_tests.route_driver import Pose2D

ROUTE_MARGIN = 0.16              # m: what the layout's route fit rule leaves beyond the robot


@pytest.fixture(scope='module')
def controller():
    """Return diff_drive_controller's parameters from the installed controllers.yaml."""
    return rh.diff_drive_parameters()


@pytest.fixture(scope='module')
def routes():
    return rd.load_routes()


def limits(controller, speed=0.5, turn_rate=rd.DEFAULT_TURN_RATE):
    return rd.limits_for(speed, turn_rate, controller)


def distance_to_route(point, waypoints):
    """Return the distance from a point to the nearest segment of a route."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(waypoints, waypoints[1:]):
        length2 = (bx - ax) ** 2 + (by - ay) ** 2
        u = max(0.0, min(1.0, ((point[0] - ax) * (bx - ax) + (point[1] - ay) * (by - ay))
                         / length2))
        best = min(best, math.dist(point, (ax + u * (bx - ax), ay + u * (by - ay))))
    return best


# ----------------------------------------------------------------------------- pure geometry


def test_segment_errors_on_off_and_past_the_line():
    a, b = (0.0, 0.0), (4.0, 0.0)
    remaining, cross, error = rd.segment_errors(Pose2D(1.0, 0.0, 0.0), a, b)
    assert (remaining, cross, error) == pytest.approx((3.0, 0.0, 0.0))
    # 0.3 m left of the line, facing along it: steer right, towards the line.
    remaining, cross, error = rd.segment_errors(Pose2D(1.0, 0.3, 0.0), a, b)
    assert cross == pytest.approx(0.3)
    assert error == pytest.approx(-math.atan2(0.3, rd.LOOKAHEAD))
    # Facing back along the line: half a turn to go.
    _, _, error = rd.segment_errors(Pose2D(1.0, 0.0, math.pi), a, b)
    assert abs(error) == pytest.approx(math.pi)
    remaining, _, _ = rd.segment_errors(Pose2D(4.5, 0.0, 0.0), a, b)
    assert remaining == pytest.approx(-0.5)


def test_ramp_speeds_up_and_slows_down_at_their_own_rates():
    assert rd.ramp(0.0, 1.0, 0.1, 0.3) == pytest.approx(0.1)      # speeding up
    assert rd.ramp(1.0, 0.0, 0.1, 0.3) == pytest.approx(0.7)      # slowing down
    assert rd.ramp(-1.0, 0.0, 0.1, 0.3) == pytest.approx(-0.7)
    assert rd.ramp(0.05, 0.0, 0.1, 0.3) == 0.0                    # reaches the target exactly


def test_turn_in_place_rate(controller):
    lim = limits(controller)
    assert rd.turn_rate_for(2.0, lim) == pytest.approx(lim.turn_rate)
    assert rd.turn_rate_for(-2.0, lim) == pytest.approx(-lim.turn_rate)
    assert rd.turn_rate_for(0.01, lim) == pytest.approx(rd.TURN_GAIN * 0.01)


def test_limits_are_half_the_controllers(controller):
    lim = limits(controller, speed=0.4)
    linear, angular = controller['linear']['x'], controller['angular']['z']
    assert lim.acceleration == pytest.approx(0.5 * linear['max_acceleration'])
    assert lim.deceleration == pytest.approx(0.5 * abs(linear['max_deceleration']))
    assert lim.angular_acceleration == pytest.approx(0.5 * angular['max_acceleration'])
    with pytest.raises(ValueError, match='--speed'):
        limits(controller, speed=linear['max_velocity'] + 0.1)
    with pytest.raises(ValueError, match='--speed'):
        limits(controller, speed=0.0)
    with pytest.raises(ValueError, match='--turn-rate'):
        limits(controller, turn_rate=angular['max_velocity'] + 0.1)


def test_check_route_rejects_short_routes_and_repeated_points():
    with pytest.raises(ValueError, match='two waypoints'):
        rd.check_route(((0.0, 0.0),))
    with pytest.raises(ValueError, match='waypoints 1 and 2'):
        rd.check_route(((0.0, 0.0), (1.0, 0.0), (1.0, 0.0)))
    rd.check_route(((0.0, 0.0), (1.0, 0.0)))


def test_routes_start_at_the_spawn_of_robot1(routes):
    # The driver refuses to start farther than START_TOLERANCE from the first waypoint, and
    # the map frame's origin is robot1's spawn (plan P2-D2): mapping starts there.
    assert {'survey', 'smoke'} <= set(routes)
    for name, waypoints in routes.items():
        rd.check_route(waypoints)
        assert waypoints[0] == (0.0, 0.0), name


def test_perturbed_pose_and_covariance():
    pose = rd.perturbed_pose(Pose2D(1.0, 2.0, 3.1), 0.3, -0.3, 8.0)
    assert (pose.x, pose.y) == pytest.approx((1.3, 1.7))
    assert pose.yaw == pytest.approx(3.1 + math.radians(8.0) - 2.0 * math.pi)   # wrapped
    covariance = rd.initial_pose_covariance()
    assert len(covariance) == 36
    assert covariance[0] == covariance[7] == pytest.approx(0.5 ** 2)
    assert covariance[35] == pytest.approx(0.2 ** 2)
    assert sum(covariance) == pytest.approx(2 * 0.25 + 0.04)


# ----------------------------------------------------------------------------- the controller


@pytest.mark.parametrize('name, speed', [('survey', 0.4), ('survey', 1.2), ('smoke', 0.5)])
def test_ideal_robot_drives_the_route(routes, controller, name, speed):
    waypoints = routes[name]
    lim = limits(controller, speed=speed)
    trace = rd.simulate(waypoints, lim)
    assert trace.finished and not trace.stalled
    points = [(pose.x, pose.y) for pose in trace.poses]
    # Every waypoint reached, the last one stood on, never off the route.
    for waypoint in waypoints:
        assert min(math.dist(waypoint, point) for point in points) < 0.05, waypoint
    assert math.dist(points[-1], waypoints[-1]) < 0.05
    assert max(distance_to_route(point, waypoints) for point in points) < 0.05
    # Driving at the speed takes most of the time; turns and ramps take the rest.
    length = rd.route_length(waypoints)
    assert length / speed < trace.duration < length / speed + 150.0


def test_commands_stay_within_half_the_controllers_limits(routes, controller):
    lim = limits(controller, speed=0.5)
    trace = rd.simulate(routes['smoke'], lim)
    step = rd.COMMAND_PERIOD
    eps = 1e-9
    for (v0, w0), (v1, w1) in zip(trace.commands, trace.commands[1:]):
        assert 0.0 <= v1 <= lim.speed + eps
        assert abs(w1) <= lim.turn_rate + eps
        assert v1 - v0 <= lim.acceleration * step + eps
        assert v0 - v1 <= lim.deceleration * step + eps
        assert abs(w1 - w0) <= lim.angular_acceleration * step + eps
    linear = controller['linear']['x']
    assert lim.acceleration < linear['max_acceleration']
    assert lim.deceleration < abs(linear['max_deceleration'])


def test_turns_in_place_before_driving(controller):
    # Start facing east; the route goes north: a quarter turn on the spot, then drive.
    trace = rd.simulate(((0.0, 0.0), (0.0, 3.0)), limits(controller))
    first_move = next(i for i, (v, _) in enumerate(trace.commands) if v > 0.0)
    assert abs(trace.poses[first_move].yaw - math.pi / 2) < rd.FACING
    assert math.dist((trace.poses[first_move].x, trace.poses[first_move].y), (0.0, 0.0)) < 1e-9
    assert trace.finished


def lagging_drive(waypoints, lim, delay_ticks):
    """
    Drive with a robot that sees its pose and acts on commands ``delay_ticks`` ticks late.

    The robot itself keeps to the controller's hard acceleration limits (controllers.yaml).
    Returns (largest distance from the route, distance of the end from the last waypoint).
    """
    follower = rd.RouteFollower(waypoints, lim)
    pose = Pose2D(waypoints[0][0], waypoints[0][1], 0.0)
    seen = collections.deque([pose] * (delay_ticks + 1), maxlen=delay_ticks + 1)
    commands = collections.deque([(0.0, 0.0)] * (delay_ticks + 1), maxlen=delay_ticks + 1)
    v = w = 0.0
    worst, tick = 0.0, 0
    while not (follower.finished or follower.stalled) and tick < 20 * 3600:
        commands.append(follower.update(seen[0], tick * rd.COMMAND_PERIOD))
        v_cmd, w_cmd = commands[0]
        for _ in range(10):                     # 5 ms physics steps
            dt = rd.COMMAND_PERIOD / 10
            v += max(-1.5 * dt, min(0.8 * dt, v_cmd - v))
            w += max(-3.0 * dt, min(3.0 * dt, w_cmd - w))
            pose = rd.advance(pose, v, w, dt)
        seen.append(pose)
        worst = max(worst, distance_to_route((pose.x, pose.y), waypoints))
        tick += 1
    assert follower.finished
    return worst, math.dist((pose.x, pose.y), waypoints[-1])


def test_a_lagging_robot_still_stops_on_the_waypoints(routes, controller):
    # 0.1 s late both ways, at the robot's top speed: without the braking margin the robot
    # overshot each waypoint by about 0.2 m and left the route by more than the 0.16 m the
    # route fit rule leaves.
    worst, end = lagging_drive(routes['smoke'], limits(controller, speed=1.2), delay_ticks=2)
    assert worst < ROUTE_MARGIN / 2
    assert end < 0.10


def test_a_robot_that_cannot_move_is_reported_stuck(controller):
    follower = rd.RouteFollower(((0.0, 0.0), (0.0, 3.0)), limits(controller))
    stuck = Pose2D(0.0, 0.0, 0.0)               # it is told to turn, but never does
    t = 0.0
    while t <= rd.STALL_TIMEOUT - 0.5:
        follower.update(stuck, t)
        t += rd.COMMAND_PERIOD
    assert not follower.stalled
    while t <= rd.STALL_TIMEOUT + 0.5:
        follower.update(stuck, t)
        t += rd.COMMAND_PERIOD
    assert follower.stalled


def test_without_a_pose_the_command_ramps_to_a_stop(controller):
    follower = rd.RouteFollower(((0.0, 0.0), (5.0, 0.0)), limits(controller))
    t, pose = 0.0, Pose2D(0.0, 0.0, 0.0)
    for _ in range(60):                         # 3 s: up to speed
        v, _ = follower.update(pose, t)
        pose = rd.advance(pose, v, 0.0, rd.COMMAND_PERIOD)
        t += rd.COMMAND_PERIOD
    assert v > 0.3
    speeds = []
    for _ in range(40):
        speeds.append(follower.update(None, t)[0])
        t += rd.COMMAND_PERIOD
    assert all(b <= a for a, b in zip(speeds, speeds[1:]))
    assert speeds[-1] == 0.0


# ----------------------------------------------------------------------------- the command


def test_dry_run_prints_the_route_and_touches_no_graph(capsys):
    assert rd.main(['--route', 'smoke', '--speed', '0.5', '--settle', '30',
                    '--initial-pose-offset', '0.3', '-0.3', '8', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'route smoke: 7 waypoints' in out
    assert 'expected: about 1 min' in out
    assert 'after standing still for 30 s' in out
    assert '/robot1/cmd_vel at 20 Hz' in out
    assert 'moved by (+0.3, -0.3) m and turned by +8 deg' in out
    assert out.rstrip().endswith('dry run: nothing was published')


def test_dry_run_of_route_none(capsys):
    assert rd.main(['--route', 'none', '--dry-run', '--namespace', '/robot2']) == 0
    out = capsys.readouterr().out
    assert 'route none: nothing to drive' in out


@pytest.mark.parametrize('argv', [
    ['--route', 'nowhere', '--dry-run'],
    ['--route', 'smoke', '--speed', '5', '--dry-run'],
    ['--route', 'smoke', '--settle', '-1', '--dry-run'],
])
def test_bad_arguments_exit_with_2(capsys, argv):
    assert rd.main(argv) == 2
    assert 'error:' in capsys.readouterr().err
