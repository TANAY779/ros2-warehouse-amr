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
evaluate_localization on synthetic MCAP bags whose errors, drift and noise are known.

The designed run (sim time, seconds): the robot stands still from 100 to 130, drives a circle
of 10 m radius at 0.5 m/s until 300 (its heading wraps past +-180 degrees), stands still 5 s,
drives on until 490, then stands still until 502: 177.5 m in all. At 128 it gets a wrong
initial pose, (+0.3, -0.3) m and +8 degrees off. AMCL's error shrinks from 0.40 m when the
robot starts to 0.12 m at 133.0 s; from 133.5 s on it cycles through 0.02, 0.04, 0.06 and
0.08 m with a heading error of 0.5 degrees. So AMCL converges after 1.75 m of driving and
then tracks for 356 s. The EKF drifts north by 1 % of the distance driven, in an odom frame
turned and shifted against the map. The IMU has the noise of robot.yaml while standing still,
ten times more while driving, and some samples repeated.
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import Imu
from warehouse_description import robot_config
from warehouse_system_tests import bags
from warehouse_system_tests import localization_eval as le

DRIVING = ((130.0, 300.0), (305.0, 490.0))   # s: the robot drives in these intervals
START, END = 100.0, 502.0                    # s: the bag
SPEED = 0.5                                  # m/s
RADIUS = 10.0                                # m: the circle it drives
INITIAL_POSE_AT = 128.0                      # s
OFFSET = (0.3, -0.3, math.radians(8.0))
CONVERGED_AT = 133.5                         # s: the first pose of the converged run
CYCLE = (0.02, 0.04, 0.06, 0.08)             # m: AMCL's error after convergence
HEADING_ERROR = math.radians(0.5)
EKF_TURN, EKF_ORIGIN = 0.3, (1.0, 2.0)       # the odom frame in the map frame
EKF_DRIFT = 0.01                             # of the distance driven, northwards
GRAVITY = 9.80665
# The IMU is recorded around the still periods only, with some driving on either side (a
# smaller bag; the evaluator does not mind gaps).
IMU_WINDOWS = ((START, 140.0), (295.0, 310.0), (480.0, END + 1.0))
# IMU samples replaced by a copy of the one before: 14 inside the still periods the check
# uses, the others in the first second after a stop, in the 5 s stop, or while driving.
REPEATS_USED = [105.0 + 2.0 * k for k in range(11)] + [495.0, 497.0, 499.0]
REPEATS_IGNORED = [100.5, 135.0, 302.0, 485.0]


def driven(t):
    """Return the metres driven by time t."""
    return SPEED * sum(min(max(t, a), b) - a for a, b in DRIVING)


def moving(t):
    return any(a <= t < b for a, b in DRIVING)


def true_pose(t):
    """Return (x, y, yaw) on the circle: start at the origin facing +x, turning left."""
    angle = driven(t) / RADIUS
    return (RADIUS * math.sin(angle), RADIUS * (1.0 - math.cos(angle)),
            (angle + math.pi) % (2.0 * math.pi) - math.pi)


def times(rate):
    """Return the sample times from START to END at ``rate`` Hz (rounded: 130.0 is 130.0)."""
    return np.round(np.arange(START, END + 0.5 / rate, 1.0 / rate), 6)


def stamp(t):
    nanoseconds = int(round(t * 1e9))
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def pose_message(message_type, t, x, y, yaw, frame='map'):
    message = message_type()
    message.header.stamp = stamp(t)
    message.header.frame_id = frame
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def amcl_errors(excursion=None, last=None):
    """Return [(t, position error, heading error)] of the designed AMCL poses."""
    poses = [(110.0, 0.01, 0.0), (120.0, 0.01, 0.0)]          # before the start: not used
    poses.append((128.1, math.hypot(OFFSET[0], OFFSET[1]), OFFSET[2]))
    times = [t for a, b in DRIVING for t in np.arange(a, b, 0.5)]
    cycle = 0
    for t in times:
        if t < CONVERGED_AT:
            fraction = (t - DRIVING[0][0]) / (CONVERGED_AT - 0.5 - DRIVING[0][0])
            poses.append((t, 0.40 - 0.28 * fraction, math.radians(8.0 - 7.0 * fraction)))
        else:
            poses.append((t, CYCLE[cycle % 4], HEADING_ERROR))
            cycle += 1
    if excursion:
        poses = [(t, 0.15 if excursion[0] <= t <= excursion[1] else e, h) for t, e, h in poses]
    if last is not None:
        poses[-1] = (poses[-1][0], last, poses[-1][2])
    return poses


def make_bag(path, with_initial_pose=True, with_imu=True, excursion=None, last=None,
             roles=('truth', 'amcl', 'initial', 'ekf', 'imu')):
    """Write the designed run as an MCAP bag at ``path``; return the AMCL error list."""
    ns = '/robot1'
    topics = {'truth': f'{ns}/ground_truth/odom', 'amcl': f'{ns}/amcl_pose',
              'initial': f'{ns}/initialpose', 'ekf': f'{ns}/odom', 'imu': f'{ns}/imu'}
    messages = {role: [] for role in topics}
    for t in times(20.0):                                    # ground truth, 20 Hz
        x, y, yaw = true_pose(t)
        message = pose_message(Odometry, t, x, y, yaw)
        message.child_frame_id = 'base_footprint'
        if moving(t):
            message.twist.twist.linear.x = SPEED
            message.twist.twist.angular.z = SPEED / RADIUS
        messages['truth'].append((t, message))
    errors = amcl_errors(excursion, last)
    for t, error, heading_error in errors:
        x, y, yaw = true_pose(t)
        if t == 128.1:                                       # AMCL takes the initial pose
            x0, y0, yaw0 = true_pose(INITIAL_POSE_AT)
            x, y, yaw = x0 + OFFSET[0], y0 + OFFSET[1], yaw0 + OFFSET[2]
        else:                                                # to the left of the robot
            x, y, yaw = (x - error * math.sin(yaw), y + error * math.cos(yaw),
                         yaw + heading_error)
        messages['amcl'].append((t, pose_message(PoseWithCovarianceStamped, t, x, y, yaw)))
    if with_initial_pose:
        x, y, yaw = true_pose(INITIAL_POSE_AT)
        messages['initial'].append((INITIAL_POSE_AT, pose_message(
            PoseWithCovarianceStamped, INITIAL_POSE_AT, x + OFFSET[0], y + OFFSET[1],
            yaw + OFFSET[2])))
    cos, sin = math.cos(EKF_TURN), math.sin(EKF_TURN)
    for t in times(10.0):                                    # EKF odometry, 10 Hz
        x, y, yaw = true_pose(t)
        y += EKF_DRIFT * driven(t)                           # where the EKF thinks it is
        dx, dy = x - EKF_ORIGIN[0], y - EKF_ORIGIN[1]        # ... in its own odom frame
        messages['ekf'].append((t, pose_message(Odometry, t, cos * dx + sin * dy,
                                                -sin * dx + cos * dy, yaw - EKF_TURN,
                                                frame='odom')))
    if with_imu:
        messages['imu'] = imu_messages()
    chosen = sorted(((t, topics[role], message) for role in roles
                     for t, message in messages[role]), key=lambda m: m[0])
    bags.write(path, ((topic, message, int(round(t * 1e9))) for t, topic, message in chosen))
    return errors


def imu_messages():
    """Return [(t, Imu)] at 100 Hz: robot.yaml's noise while still, ten times more driving."""
    rng = np.random.default_rng(7)
    repeats = set(REPEATS_USED + REPEATS_IGNORED)
    messages, previous = [], None
    for t in times(100.0):
        if not any(a <= t < b for a, b in IMU_WINDOWS):
            continue
        scale = 10.0 if moving(t) else 1.0
        gyro = rng.normal(0.0, 0.002 * scale, 3)
        accel = rng.normal(0.0, 0.02 * scale, 3) + (0.0, 0.0, GRAVITY)
        values = previous if t in repeats else (gyro, accel)
        message = Imu()
        message.header.stamp = stamp(t)
        message.header.frame_id = 'imu_link'
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = \
            values[0].tolist()
        (message.linear_acceleration.x, message.linear_acceleration.y,
         message.linear_acceleration.z) = values[1].tolist()
        message.angular_velocity_covariance = [4.0e-6, 0, 0, 0, 4.0e-6, 0, 0, 0, 4.0e-6]
        message.linear_acceleration_covariance = [4.0e-4, 0, 0, 0, 4.0e-4, 0, 0, 0, 4.0e-4]
        messages.append((t, message))
        previous = values
    return messages


@pytest.fixture(scope='module')
def good_bag(tmp_path_factory):
    path = tmp_path_factory.mktemp('bags') / 'good'
    errors = make_bag(path)
    return path, errors


@pytest.fixture(scope='module')
def good(good_bag):
    return le.evaluate(good_bag[0])


@pytest.fixture(scope='module')
def robot():
    return robot_config.load()


def converged_errors(errors):
    return np.array([e for t, e, _ in errors if t >= CONVERGED_AT])


def test_convergence_from_the_initial_pose(good):
    loc = good.localization
    assert loc.start_from == 'initialpose'
    assert loc.start == pytest.approx(INITIAL_POSE_AT)
    assert loc.error[0] == pytest.approx(math.hypot(*OFFSET[:2]), abs=1e-6)
    assert loc.t[loc.converged] == pytest.approx(CONVERGED_AT)
    # Standing still until 130 s, then 0.5 m/s until 133.5 s.
    assert loc.convergence_distance == pytest.approx(1.75, abs=1e-3)


def test_statistics_after_convergence(good, good_bag):
    expected = converged_errors(good_bag[1])
    stats = good.localization.statistics()
    assert stats['poses'] == len(expected)
    assert stats['mean'] == pytest.approx(np.mean(expected), abs=1e-5)
    assert stats['rms'] == pytest.approx(np.sqrt(np.mean(expected ** 2)), abs=1e-5)
    assert stats['p95'] == pytest.approx(0.08, abs=1e-5)
    assert stats['max'] == pytest.approx(0.08, abs=1e-5)
    assert stats['max_heading_deg'] == pytest.approx(0.5, abs=1e-3)
    assert stats['duration'] == pytest.approx(489.5 - CONVERGED_AT)
    assert stats['distance'] == pytest.approx(driven(489.5) - driven(CONVERGED_AT), abs=1e-3)


def test_ekf_drift_in_a_turned_odom_frame(good):
    drift = good.drift
    assert drift.distance == pytest.approx(driven(END), abs=1e-3)
    assert drift.final == pytest.approx(EKF_DRIFT * driven(END), rel=1e-3)
    assert drift.largest == pytest.approx(drift.final, rel=1e-6)
    # Put on the truth at the start: no drift before the robot moves.
    assert drift.drift[0] == pytest.approx(0.0, abs=1e-6)


def test_imu_noise_from_still_periods_only(good):
    imu = good.imu
    # 100-130 s and 490-502 s, each without its first second; the 5 s stop is too short.
    assert [end for period in imu.periods for end in period] == pytest.approx(
        [101.0, 129.95, 491.0, 502.0])                # 129.95: the last still pose at 20 Hz
    assert imu.still_time == pytest.approx(39.95)
    assert imu.repeats == len(REPEATS_USED)
    # Driving samples (ten times the noise) would raise sigma far beyond these bounds.
    for axis, sigma in zip(le.IMU_AXES, imu.sigma):
        designed = 0.002 if axis.startswith('gyro') else 0.02
        assert sigma == pytest.approx(designed, rel=0.04), axis
    assert imu.covariance == pytest.approx([4.0e-6] * 3 + [4.0e-4] * 3)


def test_every_criterion_passes(good, robot):
    results = le.criteria(good, robot)
    assert [r.rule for r in results] == ['convergence', 'tracking', 'imu_gyro', 'imu_accel']
    assert all(r.passed for r in results), [str(r) for r in results]


def test_command_prints_the_numbers_and_draws_the_plot(good_bag, tmp_path, capsys):
    plot = tmp_path / 'loc.png'
    assert le.main([str(good_bag[0]), '--plot', str(plot)]) == 0
    out = capsys.readouterr().out
    assert 'converged at 133.50 s, after 1.75 m of driving' in out
    assert 'max 0.080 m' in out
    assert '14 repeated samples removed' in out
    assert '4 of 4 localization checks pass' in out
    assert plot.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'


def test_without_initial_pose_the_start_is_the_first_amcl_pose(tmp_path):
    path = tmp_path / 'no_initial'
    make_bag(path, with_initial_pose=False, with_imu=False)
    loc = le.evaluate(path).localization
    assert loc.start_from == 'amcl_pose'
    assert loc.start == pytest.approx(110.0)
    # The robot stood still until 130 s: the same distance as from the initial pose.
    assert loc.convergence_distance == pytest.approx(1.75, abs=1e-3)


def test_a_late_excursion_moves_convergence(tmp_path, robot, capsys):
    path = tmp_path / 'excursion'
    make_bag(path, with_imu=False, excursion=(400.0, 402.0))
    evaluation = le.evaluate(path)
    loc = evaluation.localization
    assert loc.t[loc.converged] == pytest.approx(402.5)
    assert loc.convergence_distance == pytest.approx(driven(402.5) - driven(INITIAL_POSE_AT),
                                                     abs=1e-3)
    failed = {r.rule for r in le.criteria(evaluation, robot) if not r.passed}
    assert failed == {'convergence', 'tracking', 'imu_gyro', 'imu_accel'}   # no IMU recorded
    assert le.main([str(path)]) == 1
    assert '[FAIL] AMCL converges within 2 m' in capsys.readouterr().out


def test_not_converged_at_the_end(tmp_path, robot):
    path = tmp_path / 'lost'
    make_bag(path, with_imu=False, last=0.12)
    evaluation = le.evaluate(path)
    assert evaluation.localization.converged is None
    results = {r.rule: r for r in le.criteria(evaluation, robot)}
    assert not results['convergence'].passed
    assert 'not converged' in results['convergence'].summary
    assert not results['tracking'].passed


def test_a_bag_without_amcl_pose_is_refused(tmp_path, capsys):
    path = tmp_path / 'mapping'
    make_bag(path, roles=('truth', 'ekf'))
    assert le.main([str(path)]) == 2
    assert 'has no /robot1/amcl_pose' in capsys.readouterr().err


def test_a_missing_bag_is_refused(tmp_path, capsys):
    assert le.main([str(tmp_path / 'nothing')]) == 2
    assert 'error:' in capsys.readouterr().err
