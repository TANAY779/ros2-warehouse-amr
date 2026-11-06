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

r"""
Evaluate localization from a bag: AMCL against ground truth, the EKF's drift, the IMU noise.

Usage::

    ros2 run warehouse_system_tests evaluate_localization BAG [--namespace robot1]
        [--plot OUT.png]

Record the bag while the robot drives with ``localization:=amcl`` (plan F7)::

    ros2 bag record -o bags/loc_run --use-sim-time --topics /robot1/amcl_pose \
        /robot1/initialpose /robot1/ground_truth/odom /robot1/odom /robot1/imu /clock

Every time below is a header stamp, in sim time. When a message was recorded plays no part:
the recorder receives messages late, by varying amounts.

1. **AMCL's error.** AMCL stamps each /<ns>/amcl_pose with the scan it used. At that stamp,
   ground truth (/<ns>/ground_truth/odom, 50 Hz) is interpolated between its two nearest
   samples; the error is the distance between the two positions, and the heading error the
   difference of the two yaws. (Not from TF: AMCL dates its map -> odom transform into the
   future by transform_tolerance, plan P2-D12.)
2. **The start.** The newest /<ns>/initialpose in the bag: the wrong pose that drive_route
   --initial-pose-offset (or RViz's 2D Pose Estimate) gave AMCL. Without one, the first
   amcl_pose. AMCL poses before the start are not evaluated.
3. **Convergence.** AMCL has converged at the first pose after which the error stays below
   0.10 m to the end of the bag. The convergence distance is the length of the ground-truth
   path from the start to that pose: how far the robot drove until AMCL knew where it was.
4. **After convergence.** Mean, RMS, 95th percentile (P95) and maximum of the position
   error, the largest heading error, and how long and how far the robot drove.
5. **The EKF's drift**, for comparison. /<ns>/odom is the EKF's estimate in the odom frame:
   dead reckoning from wheels and gyro, with nothing to correct it. It is put on the ground
   truth at the start (shifted and turned so the two poses agree there), then compared at
   every stamp. Its error grows with the distance driven; AMCL's should not.
6. **IMU noise.** The standard deviation (sigma) of /<ns>/imu on each axis: gyro x, y, z and
   accelerometer x, y, z (z holds gravity; like every axis, it is measured about its mean).
   Only periods in which ground truth stands still for 10 s or longer count, each without
   its first second (the body may still rock after a stop). A sample identical to the one
   before is not a new measurement but the same Gazebo reading again (ros2_control can read
   one sample twice); it is removed and counted. The covariances in the messages
   (controllers.yaml) are compared with the measured sigma^2. 3000 samples give sigma within
   about 1.3 % (1 / sqrt(2 n)).

Then one [PASS] or [FAIL] line per acceptance criterion of plan F7: AMCL converges within
2.0 m; after that its error stays under 0.10 m for at least 5 min; gyro and accelerometer
sigma, each axis, within +-20 % of robot.yaml, from at least 25 s standing still.

Exit codes: 0 every criterion passes; 1 one fails; 2 the bag cannot be read, lacks amcl_pose
or ground truth, or the plot cannot be written.
"""

import argparse
from dataclasses import dataclass
import math
import os
import sys

import matplotlib
from matplotlib.figure import Figure
import numpy as np
from warehouse_description import robot_config
from warehouse_layout.rules import RuleResult
from warehouse_system_tests import bags
from warehouse_system_tests import ros_helpers as rh

# Acceptance criteria (plan F7).
CONVERGED = 0.10                 # m: converged once the error stays below this
MAX_CONVERGENCE_DISTANCE = 2.0   # m of driving from the start
MIN_TRACKING_TIME = 300.0        # s after convergence (5 min)
SIGMA_TOLERANCE = 0.20           # measured IMU sigma within +-20 % of robot.yaml
MIN_STILL_TIME = 25.0            # s of standing still for the IMU check (PROJECT_PLAN)

# Standing still: ground truth slower than this, for at least STILL_PERIOD seconds.
STILL_SPEED = 0.005              # m/s
STILL_TURN = 0.005               # rad/s
STILL_PERIOD = 10.0              # s
STILL_TRIM = 1.0                 # s left out at the start of each still period

IMU_AXES = ('gyro x', 'gyro y', 'gyro z', 'accel x', 'accel y', 'accel z')

# Plot colours from the dataviz reference palette: AMCL wears categorical slot 1 (blue), the
# EKF slot 2 (orange); validated as a pair, light mode. Ground truth is the reference, in ink.
AMCL_COLOUR = '#2a78d6'
EKF_COLOUR = '#eb6834'
INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
DPI = 120


class BagError(ValueError):
    """The bag cannot be evaluated (the message says why)."""


@dataclass(frozen=True, eq=False)
class Track:
    """Planar poses over time, in stamp order: t [s], x and y [m], yaw [rad] (numpy arrays)."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    @staticmethod
    def from_rows(rows):
        """Return a Track of (t, x, y, yaw) rows, sorted by t; of equal stamps the last stays."""
        array = np.asarray(rows, dtype=float).reshape(-1, 4)
        array = array[np.argsort(array[:, 0], kind='stable')]
        keep = np.append(np.diff(array[:, 0]) > 0.0, True)
        return Track(*array[keep].T)

    def __len__(self):
        return len(self.t)

    def at(self, t):
        """Return (x, y, yaw) interpolated at the times t (clamped to the track's span)."""
        yaw = np.interp(t, self.t, np.unwrap(self.yaw))
        return np.interp(t, self.t, self.x), np.interp(t, self.t, self.y), wrap(yaw)

    def travelled(self):
        """Return the path length from the first pose to each pose, in metres."""
        return np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))))


def wrap(angle):
    """Wrap angles (a number or an array) to [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True, eq=False)
class Recording:
    """What a bag holds for one robot (the topics are those of ``topics_for``)."""

    topics: dict
    truth: Track
    still: np.ndarray            # per ground-truth pose: True while standing still
    amcl: Track
    ekf: Track | None            # None if the bag has no EKF odometry
    initial: Track | None        # the initial poses, or None
    imu_t: np.ndarray            # IMU stamps [s] (empty without IMU)
    imu: np.ndarray              # (n, 6): gyro x, y, z [rad/s], accel x, y, z [m/s^2]
    imu_covariance: np.ndarray   # (n, 6): the diagonals of the two covariance matrices


def topics_for(namespace):
    """Return {role: topic} of the robot in ``namespace``."""
    ns = '/' + namespace.strip('/')
    return {'amcl': f'{ns}/amcl_pose', 'truth': f'{ns}/ground_truth/odom', 'ekf': f'{ns}/odom',
            'initial': f'{ns}/initialpose', 'imu': f'{ns}/imu'}


def read_recording(path, namespace='robot1'):
    """Read the bag at ``path``; raise BagError if it lacks amcl_pose or ground truth."""
    topics = topics_for(namespace)
    available = bags.topic_types(path)
    missing = [topics[role] for role in ('amcl', 'truth') if topics[role] not in available]
    if missing:
        raise BagError(f'{path} has no {" and no ".join(missing)} (it has: '
                       f'{", ".join(sorted(available)) or "no topics"})')
    role_of = {topic: role for role, topic in topics.items()}
    poses = {'amcl': [], 'truth': [], 'ekf': [], 'initial': []}
    speeds = []
    imu = []
    for topic, message, _ in bags.read(path, set(topics.values())):
        role = role_of[topic]
        stamp = rh.stamp_to_seconds(message.header.stamp)
        if role == 'imu':
            gyro, accel = message.angular_velocity, message.linear_acceleration
            gyro_cov = message.angular_velocity_covariance
            accel_cov = message.linear_acceleration_covariance
            imu.append((stamp, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z,
                        gyro_cov[0], gyro_cov[4], gyro_cov[8],
                        accel_cov[0], accel_cov[4], accel_cov[8]))
            continue
        pose = message.pose.pose
        poses[role].append((stamp, pose.position.x, pose.position.y, rh.yaw_of(pose.orientation)))
        if role == 'truth':
            twist = message.twist.twist
            speeds.append((stamp, math.hypot(twist.linear.x, twist.linear.y),
                           abs(twist.angular.z)))
    if not poses['amcl'] or not poses['truth']:
        raise BagError(f'{path}: {topics["amcl"]} or {topics["truth"]} has no messages')
    truth = Track.from_rows(poses['truth'])
    # The still flag of each kept ground-truth pose (Track.from_rows drops repeated stamps).
    speeds = np.asarray(speeds, dtype=float)
    speeds = speeds[np.argsort(speeds[:, 0], kind='stable')]
    speeds = speeds[np.append(np.diff(speeds[:, 0]) > 0.0, True)]
    still = (speeds[:, 1] < STILL_SPEED) & (speeds[:, 2] < STILL_TURN)
    imu = np.asarray(imu, dtype=float).reshape(-1, 13)
    imu = imu[np.argsort(imu[:, 0], kind='stable')]
    return Recording(
        topics=topics, truth=truth, still=still, amcl=Track.from_rows(poses['amcl']),
        ekf=Track.from_rows(poses['ekf']) if poses['ekf'] else None,
        initial=Track.from_rows(poses['initial']) if poses['initial'] else None,
        imu_t=imu[:, 0], imu=imu[:, 1:7], imu_covariance=imu[:, 7:13])


# ----------------------------------------------------------------------------- AMCL


@dataclass(frozen=True, eq=False)
class Localization:
    """AMCL against ground truth: the arrays per AMCL pose, and what follows from them."""

    start: float                 # s: the evaluation starts here
    start_from: str              # 'initialpose' or 'amcl_pose'
    t: np.ndarray                # stamps of the AMCL poses evaluated [s]
    error: np.ndarray            # position error [m]
    heading_error: np.ndarray    # signed heading error [rad]
    travelled: np.ndarray        # ground-truth path length from the start [m]
    converged: int | None        # index of the first pose of the converged run

    @property
    def convergence_distance(self):
        """Return the metres driven from the start until convergence (None: not converged)."""
        return None if self.converged is None else float(self.travelled[self.converged])

    def after(self):
        """Return (error, heading_error, t, travelled) from convergence on (empty if never)."""
        index = len(self.t) if self.converged is None else self.converged
        return (self.error[index:], self.heading_error[index:], self.t[index:],
                self.travelled[index:])

    def statistics(self):
        """Return the numbers after convergence as a dict (NaN where there is nothing)."""
        error, heading, t, travelled = self.after()
        if len(error) == 0:
            nan = float('nan')
            return {'poses': 0, 'mean': nan, 'rms': nan, 'p95': nan, 'max': nan,
                    'max_heading_deg': nan, 'duration': 0.0, 'distance': 0.0}
        return {
            'poses': len(error),
            'mean': float(np.mean(error)),
            'rms': float(np.sqrt(np.mean(error ** 2))),
            'p95': float(np.percentile(error, 95)),
            'max': float(np.max(error)),
            'max_heading_deg': math.degrees(float(np.max(np.abs(heading)))),
            'duration': float(t[-1] - t[0]),
            'distance': float(travelled[-1] - travelled[0]),
        }


def path_length_at(truth, t):
    """Return the ground-truth path length from its first pose up to the times t."""
    return np.interp(t, truth.t, truth.travelled())


def localization(recording):
    """Return the Localization of AMCL in the recording."""
    truth, amcl = recording.truth, recording.amcl
    if recording.initial is not None:
        start, start_from = float(recording.initial.t[-1]), 'initialpose'
    else:
        start, start_from = float(amcl.t[0]), 'amcl_pose'
    # Only poses inside the ground truth's time span can be compared.
    keep = (amcl.t >= start) & (amcl.t >= truth.t[0]) & (amcl.t <= truth.t[-1])
    t = amcl.t[keep]
    x, y, yaw = truth.at(t)
    error = np.hypot(amcl.x[keep] - x, amcl.y[keep] - y)
    heading_error = wrap(amcl.yaw[keep] - yaw)
    travelled = path_length_at(truth, t) - path_length_at(truth, start)
    above = np.nonzero(error >= CONVERGED)[0]
    if len(t) == 0 or (len(above) and above[-1] == len(t) - 1):
        converged = None                       # still (or again) above the limit at the end
    else:
        converged = int(above[-1]) + 1 if len(above) else 0
    return Localization(start, start_from, t, error, heading_error, travelled, converged)


# ----------------------------------------------------------------------------- EKF


@dataclass(frozen=True, eq=False)
class Drift:
    """The EKF's odometry against ground truth, put on it at the first EKF pose evaluated."""

    t: np.ndarray                # stamps [s]
    x: np.ndarray                # the EKF's path, put on the ground truth at the start [m]
    y: np.ndarray
    drift: np.ndarray            # distance to the ground truth [m]
    travelled: np.ndarray        # ground-truth path length since the first EKF pose [m]

    @property
    def final(self):
        return float(self.drift[-1])

    @property
    def largest(self):
        return float(np.max(self.drift))

    @property
    def distance(self):
        return float(self.travelled[-1])


def ekf_drift(recording, start):
    """Return the Drift of the EKF from ``start`` on, or None without EKF odometry."""
    ekf, truth = recording.ekf, recording.truth
    if ekf is None:
        return None
    keep = (ekf.t >= start) & (ekf.t >= truth.t[0]) & (ekf.t <= truth.t[-1])
    if np.count_nonzero(keep) < 2:
        return None
    t, x, y, yaw = ekf.t[keep], ekf.x[keep], ekf.y[keep], ekf.yaw[keep]
    true_x, true_y, true_yaw = truth.at(t)
    # The odom frame starts wherever the EKF started: turn and shift the EKF's path so that
    # its first pose lies on the true pose at that moment.
    turn = true_yaw[0] - yaw[0]
    cos, sin = math.cos(turn), math.sin(turn)
    dx, dy = x - x[0], y - y[0]
    placed_x = true_x[0] + cos * dx - sin * dy
    placed_y = true_y[0] + sin * dx + cos * dy
    drift = np.hypot(placed_x - true_x, placed_y - true_y)
    travelled = path_length_at(truth, t) - path_length_at(truth, t[0])
    return Drift(t, placed_x, placed_y, drift, travelled)


# ----------------------------------------------------------------------------- IMU


@dataclass(frozen=True, eq=False)
class ImuNoise:
    """The IMU noise measured while standing still (see the module docstring)."""

    periods: tuple               # (start, end) of each still period used [s]
    samples: int                 # samples used
    repeats: int                 # repeated samples removed from those periods
    sigma: np.ndarray            # per IMU_AXES
    covariance: np.ndarray       # per IMU_AXES: the messages' covariance (median)

    @property
    def still_time(self):
        return float(sum(end - start for start, end in self.periods))


def still_periods(recording):
    """Return the (start, end) of every still period long enough, without its first second."""
    still = recording.still.astype(int)
    edges = np.diff(np.concatenate(([0], still, [0])))
    firsts = np.nonzero(edges == 1)[0]
    lasts = np.nonzero(edges == -1)[0] - 1
    t = recording.truth.t
    periods = [(t[first] + STILL_TRIM, t[last]) for first, last in zip(firsts, lasts)]
    return tuple((float(a), float(b)) for a, b in periods if b - a >= STILL_PERIOD)


def imu_noise(recording):
    """Return the ImuNoise of the recording, or None without IMU samples while still."""
    t, values = recording.imu_t, recording.imu
    if len(t) < 2:
        return None
    repeat = np.zeros(len(t), dtype=bool)
    repeat[1:] = np.all(values[1:] == values[:-1], axis=1)
    squares, dof, samples, repeats, used, periods = np.zeros(6), 0, 0, 0, [], []
    for start, end in still_periods(recording):
        inside = (t >= start) & (t <= end)
        chosen = inside & ~repeat
        if np.count_nonzero(chosen) < 2:
            continue
        sample = values[chosen]
        # About each period's own mean: a slope of the floor or a gyro bias is no noise.
        squares += np.sum((sample - sample.mean(axis=0)) ** 2, axis=0)
        dof += len(sample) - 1
        samples += len(sample)
        repeats += int(np.count_nonzero(inside & repeat))
        used.append(recording.imu_covariance[chosen])
        periods.append((start, end))
    if not dof:
        return None
    return ImuNoise(tuple(periods), samples, repeats, np.sqrt(squares / dof),
                    np.median(np.concatenate(used), axis=0))


# ----------------------------------------------------------------------------- criteria


def _criterion(rule, title, passed, summary, advice):
    """Return a RuleResult: the same [PASS] / [FAIL] lines as check_map and the fit rules."""
    return RuleResult(rule, title, () if passed else (advice,), summary)


def criteria(evaluation, robot):
    """Return one RuleResult per acceptance criterion of plan F7."""
    loc, imu = evaluation.localization, evaluation.imu
    stats = loc.statistics()
    distance = loc.convergence_distance
    title = f'AMCL converges within {MAX_CONVERGENCE_DISTANCE:g} m of driving'
    if distance is None:
        if len(loc.error):
            summary = f'not converged: the error is {loc.error[-1]:.2f} m at the end'
        else:
            summary = 'no AMCL pose after the start'
        convergence = _criterion(
            'convergence', title, False, summary,
            'AMCL did not find the robot, or lost it again: look at the plot, then at the '
            "map (check_map) and AMCL's max_beams and alpha1-4 (plan section 4.4)")
    else:
        since = loc.t[loc.converged] - loc.start
        convergence = _criterion(
            'convergence', title, distance <= MAX_CONVERGENCE_DISTANCE,
            f'{distance:.2f} m (error below {CONVERGED:.2f} m from {since:.1f} s after the '
            'start to the end)',
            f'AMCL needed {distance:.2f} m: check the initial pose offset and the map, then '
            "AMCL's max_beams and alpha1-4 (plan section 4.4)")

    title = (f'error under {CONVERGED:.2f} m for at least {MIN_TRACKING_TIME / 60:g} min after '
             'convergence')
    if distance is None:
        tracking = _criterion('tracking', title, False, 'never converged',
                              'AMCL never converged (see above)')
    else:
        tracked = stats['duration']
        tracking = _criterion(
            'tracking', title, tracked >= MIN_TRACKING_TIME,
            f'{rh.duration_text(tracked)} and {stats["distance"]:.0f} m, max '
            f'{stats["max"]:.3f} m',
            'drive longer after convergence: drive_route --route survey --speed 0.5 takes '
            'about 7 min')
    return (convergence, tracking,
            _sigma_criterion('imu_gyro', 'gyro', imu, 0, robot.imu.gyro_noise_stddev, 'rad/s'),
            _sigma_criterion('imu_accel', 'accelerometer', imu, 3,
                             robot.imu.accel_noise_stddev, 'm/s^2'))


def _sigma_criterion(rule, sensor, imu, first, expected, unit):
    title = (f'{sensor} sigma within +-{SIGMA_TOLERANCE:.0%} of robot.yaml ({expected:g} {unit}) '
             f'on each axis, from >= {MIN_STILL_TIME:g} s standing still')
    if imu is None:
        return _criterion(rule, title, False, 'no IMU samples while standing still',
                          'record /<ns>/imu, and stand still first: drive_route --settle 30')
    sigma = imu.sigma[first:first + 3]
    ratios = sigma / expected
    summary = ', '.join(f'{axis} {value:.3g} ({ratio:.2f}x)' for axis, value, ratio
                        in zip('xyz', sigma, ratios))
    summary += f' from {imu.still_time:.1f} s'
    if imu.still_time < MIN_STILL_TIME:
        return _criterion(rule, title, False, summary,
                          f'only {imu.still_time:.1f} s standing still: start the drive with '
                          'drive_route --settle 30')
    worst = IMU_AXES[first + int(np.argmax(np.abs(ratios - 1.0)))]
    return _criterion(rule, title, bool(np.all(np.abs(ratios - 1.0) <= SIGMA_TOLERANCE)),
                      summary, f'{worst} is off by more than {SIGMA_TOLERANCE:.0%}: compare '
                      "the IMU's noise in warehouse_description (robot.yaml, imu.xacro) with "
                      'the bag, and check that the robot really stood still')


@dataclass(frozen=True, eq=False)
class Evaluation:
    """Everything evaluate() found in a bag."""

    recording: Recording
    localization: Localization
    drift: Drift | None          # None without EKF odometry
    imu: ImuNoise | None         # None without IMU samples while standing still


def evaluate(path, namespace='robot1'):
    """Read the bag at ``path`` and evaluate it; return an Evaluation."""
    recording = read_recording(path, namespace)
    loc = localization(recording)
    return Evaluation(recording, loc, ekf_drift(recording, loc.start), imu_noise(recording))


def report(path, evaluation, results):
    """Return the text report: the numbers, then the [PASS] / [FAIL] lines."""
    rec, loc = evaluation.recording, evaluation.localization
    drift, imu = evaluation.drift, evaluation.imu
    topics = rec.topics
    initial = len(rec.initial) if rec.initial is not None else 0
    lines = [f'bag {path}: {len(rec.amcl)} AMCL poses, {len(rec.truth)} ground-truth poses, '
             f'{len(rec.ekf) if rec.ekf is not None else 0} EKF poses, {len(rec.imu_t)} IMU '
             f'samples, {initial} initial pose{"" if initial == 1 else "s"}']
    lines.append(f'AMCL ({topics["amcl"]}) against ground truth ({topics["truth"]}):')
    start = ('the newest initial pose' if loc.start_from == 'initialpose'
             else 'the first AMCL pose (no initial pose in the bag)')
    if len(loc.t):
        lines.append(f'  start: {start}, at {loc.start:.2f} s; first error {loc.error[0]:.3f} m, '
                     f'{math.degrees(abs(loc.heading_error[0])):.1f} deg')
    else:
        lines.append(f'  start: {start}, at {loc.start:.2f} s; no AMCL pose after it')
    if loc.converged is None:
        lines.append(f'  not converged: the error is at or above {CONVERGED:.2f} m at the end')
    else:
        stats = loc.statistics()
        lines.append(f'  converged at {loc.t[loc.converged]:.2f} s, after '
                     f'{loc.convergence_distance:.2f} m of driving (error below {CONVERGED:.2f} m '
                     'from then on)')
        lines.append(f'  after convergence: {stats["poses"]} poses, '
                     f'{rh.duration_text(stats["duration"])}, {stats["distance"]:.1f} m')
        lines.append(f'    position error: mean {stats["mean"]:.3f} m, RMS {stats["rms"]:.3f} m, '
                     f'P95 {stats["p95"]:.3f} m, max {stats["max"]:.3f} m')
        lines.append(f'    heading error: max {stats["max_heading_deg"]:.2f} deg')
    if drift is None:
        lines.append(f'EKF drift: no {topics["ekf"]} in the bag after the start')
    else:
        percent = 100.0 * drift.final / drift.distance if drift.distance > 0.0 else float('nan')
        lines.append(f'EKF drift ({topics["ekf"]}, put on the ground truth at the start): '
                     f'{drift.final:.3f} m after {drift.distance:.1f} m ({percent:.2f} % of the '
                     f'distance), max {drift.largest:.3f} m')
    if imu is None:
        lines.append(f'IMU noise: no {topics["imu"]} samples while standing still for '
                     f'{STILL_PERIOD:g} s or longer')
    else:
        periods = len(imu.periods)
        lines.append(f'IMU noise ({topics["imu"]}) from {periods} still '
                     f'period{"" if periods == 1 else "s"}, '
                     f'{imu.still_time:.1f} s, {imu.samples} samples ({imu.repeats} repeated '
                     'samples removed):')
        lines.append('  axis       sigma     sigma^2   covariance   covariance / sigma^2')
        for axis, sigma, covariance in zip(IMU_AXES, imu.sigma, imu.covariance):
            ratio = covariance / sigma ** 2 if sigma > 0.0 else float('nan')
            lines.append(f'  {axis:8s} {sigma:8.5f}   {sigma ** 2:9.2e}   {covariance:9.2e}   '
                         f'{ratio:6.2f}')
    lines += [str(r) for r in results]
    passed = sum(r.passed for r in results)
    lines.append(f'{passed} of {len(results)} localization checks pass')
    return '\n'.join(lines)


# ----------------------------------------------------------------------------- the plot


def draw(path, evaluation, title):
    """
    Draw the run and save it as a PNG.

    Left: the paths seen from above (ground truth, AMCL, the EKF put on the start). Right,
    over the time since the start: AMCL's position error with the 0.10 m limit, its heading
    error, and the EKF's drift, each on its own axis (two scales never share one).
    """
    rec, loc, drift = evaluation.recording, evaluation.localization, evaluation.drift
    with matplotlib.rc_context({'font.size': 8, 'axes.edgecolor': AXIS,
                                'axes.labelcolor': INK_SECONDARY, 'xtick.color': INK_MUTED,
                                'ytick.color': INK_MUTED, 'axes.titlecolor': INK}):
        figure = Figure(figsize=(13.0, 7.0), dpi=DPI)
        grid = figure.add_gridspec(3, 2, width_ratios=[1.25, 1.0], hspace=0.35, wspace=0.16,
                                   left=0.05, right=0.985, top=0.9, bottom=0.12)
        _draw_paths(figure.add_subplot(grid[:, 0]), rec, loc, drift)
        error_ax = figure.add_subplot(grid[0, 1])
        heading_ax = figure.add_subplot(grid[1, 1], sharex=error_ax)
        drift_ax = figure.add_subplot(grid[2, 1], sharex=error_ax)
        _draw_errors(error_ax, heading_ax, drift_ax, loc, drift)
        figure.suptitle(title, x=0.05, y=0.975, ha='left', fontsize=11, color=INK)
        figure.savefig(path, dpi=DPI, facecolor='white')


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)


def _draw_paths(ax, rec, loc, drift):
    _style(ax)
    truth = rec.truth
    shown = truth.t >= loc.start
    ax.plot(truth.x[shown], truth.y[shown], color=INK, linewidth=1.0, label='ground truth')
    keep = rec.amcl.t >= loc.start
    ax.plot(rec.amcl.x[keep], rec.amcl.y[keep], color=AMCL_COLOUR, linewidth=1.2,
            label='AMCL (amcl_pose)')
    if drift is not None:
        ax.plot(drift.x, drift.y, color=EKF_COLOUR, linewidth=1.2,
                label='EKF odometry, put on the start')
    if rec.initial is not None:
        ax.plot(rec.initial.x[-1], rec.initial.y[-1], 'o', markersize=8, markerfacecolor='white',
                markeredgecolor=AMCL_COLOUR, markeredgewidth=1.5)
        ax.annotate('initial pose', (rec.initial.x[-1], rec.initial.y[-1]),
                    xytext=(8, -12), textcoords='offset points', color=INK_SECONDARY)
    if loc.converged is not None and len(loc.t):
        x, y, _ = truth.at(loc.t[loc.converged])
        ax.plot(x, y, 'o', markersize=8, color=AMCL_COLOUR, markeredgecolor='white',
                markeredgewidth=1.5)
        ax.annotate(f'converged after {loc.convergence_distance:.2f} m', (x, y), xytext=(8, 6),
                    textcoords='offset points', color=INK_SECONDARY)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m] (map frame)')
    ax.set_ylabel('y [m]')
    ax.set_title('Paths seen from above', loc='left')
    # Under the axes, in one row: inside, it would cover part of the path.
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, frameon=False,
              labelcolor=INK)


def _draw_errors(error_ax, heading_ax, drift_ax, loc, drift):
    for ax in (error_ax, heading_ax, drift_ax):
        _style(ax)
    t = loc.t - loc.start
    error_ax.plot(t, loc.error, color=AMCL_COLOUR, linewidth=1.2)
    error_ax.axhline(CONVERGED, color=INK_SECONDARY, linewidth=0.9, linestyle=(0, (4, 3)))
    error_ax.annotate(f'{CONVERGED:.2f} m', (1.0, CONVERGED), xycoords=('axes fraction', 'data'),
                      xytext=(-4, 3), textcoords='offset points', ha='right',
                      color=INK_SECONDARY)
    if loc.converged is not None and len(t):
        for ax in (error_ax, heading_ax, drift_ax):
            ax.axvline(t[loc.converged], color=AXIS, linewidth=0.9)
        error_ax.annotate('converged', (t[loc.converged], 1.0),
                          xycoords=('data', 'axes fraction'), xytext=(4, -10),
                          textcoords='offset points', color=INK_SECONDARY)
    error_ax.set_ylim(bottom=0.0)
    error_ax.set_ylabel('error [m]')
    error_ax.set_title('AMCL position error', loc='left')
    heading_ax.plot(t, np.degrees(np.abs(loc.heading_error)), color=AMCL_COLOUR, linewidth=1.2)
    heading_ax.set_ylim(bottom=0.0)
    heading_ax.set_ylabel('error [deg]')
    heading_ax.set_title('AMCL heading error', loc='left')
    if drift is not None:
        drift_ax.plot(drift.t - loc.start, drift.drift, color=EKF_COLOUR, linewidth=1.2)
    drift_ax.set_ylim(bottom=0.0)
    drift_ax.set_ylabel('drift [m]')
    drift_ax.set_xlabel('time since the start [s] (sim time)')
    drift_ax.set_title('EKF odometry drift (for comparison)', loc='left')
    # One series per panel, named by its title: no legend box (the colours are those of
    # the paths on the left).
    for ax in (error_ax, heading_ax):
        ax.tick_params(labelbottom=False)


# ----------------------------------------------------------------------------- the command


def main(argv=None):
    """Run the evaluate_localization command; return its exit code."""
    parser = argparse.ArgumentParser(
        prog='evaluate_localization',
        description='Evaluate AMCL against ground truth from a recorded bag (convergence from '
                    'a wrong start, error statistics, the EKF drift for comparison) and the '
                    "IMU's noise while standing still, against the acceptance criteria of "
                    'plan F7.')
    parser.add_argument('bag',
                        help='the bag: a folder made by ros2 bag record, or one .mcap file')
    parser.add_argument('--namespace', default='robot1',
                        help='the robot namespace (default %(default)s)')
    parser.add_argument('--plot', metavar='OUT.png', help='draw the run into this PNG file')
    args = parser.parse_args(argv)

    try:
        robot = robot_config.load()
        evaluation = evaluate(args.bag, args.namespace)
    except (OSError, RuntimeError, ValueError) as error:   # BagError is a ValueError
        print(f'error: {error}', file=sys.stderr)
        return 2
    results = criteria(evaluation, robot)
    print(report(args.bag, evaluation, results))
    failed = [r for r in results if not r.passed]
    if args.plot:
        verdict = ('all criteria pass' if not failed else
                   f'{len(failed)} of {len(results)} criteria fail')
        name = os.path.basename(os.path.normpath(args.bag))
        title = f'evaluate_localization: {name}: {verdict}'
        try:
            draw(args.plot, evaluation, title)
        except OSError as error:
            print(f'error: cannot write the plot {args.plot}: {error.strerror}', file=sys.stderr)
            return 2
        print(f'wrote {args.plot}')
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
