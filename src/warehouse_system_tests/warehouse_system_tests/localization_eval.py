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
        [--plot OUT.png] [--odometry]

Record the bag while the robot drives with ``localization:=amcl`` (plan F7)::

    ros2 bag record -o bags/loc_run --use-sim-time --topics /robot1/amcl_pose \
        /robot1/initialpose /robot1/ground_truth/odom /robot1/odom /robot1/odom/wheel \
        /robot1/imu

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

7. **Odometry** (with ``--odometry``): wheel odometry (/<ns>/odom/wheel) and the EKF
   (/<ns>/odom) against ground truth on every motion of the robot. Ground truth's speed and
   turn rate split the drive into straights and turns in place (see motions()). Over a
   straight, each source's motion is taken in its own heading at the start (forward and
   left), and the error is the distance between the source's motion and the true one, as a
   share of the distance; over a turn in place, the error of the angle turned, as a share of
   the angle. These are the measures of Phase 1's odometry checks, run on the bag.

Then one [PASS] or [FAIL] line per acceptance criterion of plan F7: AMCL converges within
2.0 m; after that its error stays under 0.10 m for at least 5 min; gyro and accelerometer
sigma, each axis, within +-20 % of robot.yaml, from at least 25 s standing still. With
--odometry, four more with Phase 1's limits, for wheel odometry and for the EKF: under 1 % on
every straight of 10 m or more, under 2 % on every turn in place of 80 degrees or more. A bag
without amcl_pose (or without imu) is then evaluated for the rest, and says so.

Exit codes: 0 every criterion passes; 1 one fails; 2 the bag cannot be read, lacks ground
truth or (without --odometry) amcl_pose, or the plot cannot be written (it needs amcl_pose).
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

# Odometry against ground truth (--odometry): Phase 1's limits, plan S1.
STRAIGHT_LIMIT = 0.01            # error under 1 % of the distance on a straight
TURN_LIMIT = 0.02                # error under 2 % of the angle of a turn in place
MIN_STRAIGHT = 10.0              # m: straights at least this long are judged
# Turns in place of at least this are judged. drive_route ends a turn in place a few degrees
# before it faces the next segment and turns the rest while it drives, so its 90 degree
# corners turn about 86 degrees in place (85.8 in the Phase 2 survey bag); its 45 degree
# turns stay below.
MIN_TURN = math.radians(80.0)
# Telling the motions apart, from ground truth's speed and turn rate.
MOVING_SPEED = 0.02              # m/s: faster than this is driving
TURNING_RATE = 0.02              # rad/s: not driving, but turning faster: turning in place
MAX_STRAIGHT_TURN = math.radians(10.0)   # a drive that turns more than this is a curve
STRAIGHT, TURN, CURVE = 'straight', 'turn in place', 'curve'

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

    def covers(self, start, end):
        """Return True if the track has poses from ``start`` to ``end`` (seconds)."""
        return len(self.t) > 0 and self.t[0] <= start and self.t[-1] >= end

    def heading_change(self, start, end):
        """Return the heading change from ``start`` to ``end`` in radians, full turns counted."""
        heading = np.unwrap(self.yaw)
        return float(np.interp(end, self.t, heading) - np.interp(start, self.t, heading))


def wrap(angle):
    """Wrap angles (a number or an array) to [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True, eq=False)
class Recording:
    """What a bag holds for one robot (the topics are those of ``topics_for``)."""

    topics: dict
    available: frozenset         # every topic in the bag
    truth: Track
    speed: np.ndarray            # per ground-truth pose: its speed [m/s]
    turn_rate: np.ndarray        # per ground-truth pose: its turn rate, unsigned [rad/s]
    still: np.ndarray            # per ground-truth pose: True while standing still
    amcl: Track | None           # None if the bag has no amcl_pose (only with --odometry)
    ekf: Track | None            # None if the bag has no EKF odometry
    wheel: Track | None          # None if the bag has no wheel odometry
    initial: Track | None        # the initial poses, or None
    imu_t: np.ndarray            # IMU stamps [s] (empty without IMU)
    imu: np.ndarray              # (n, 6): gyro x, y, z [rad/s], accel x, y, z [m/s^2]
    imu_covariance: np.ndarray   # (n, 6): the diagonals of the two covariance matrices


def topics_for(namespace):
    """Return {role: topic} of the robot in ``namespace``."""
    ns = '/' + namespace.strip('/')
    return {'amcl': f'{ns}/amcl_pose', 'truth': f'{ns}/ground_truth/odom', 'ekf': f'{ns}/odom',
            'wheel': f'{ns}/odom/wheel', 'initial': f'{ns}/initialpose', 'imu': f'{ns}/imu'}


def read_recording(path, namespace='robot1', need_amcl=True):
    """
    Read the bag at ``path``.

    Raises BagError if it lacks ground truth, or amcl_pose while ``need_amcl``.
    """
    topics = topics_for(namespace)
    available = bags.topic_types(path)
    needed = ('amcl', 'truth') if need_amcl else ('truth',)
    missing = [topics[role] for role in needed if topics[role] not in available]
    if missing:
        raise BagError(f'{path} has no {" and no ".join(missing)} (it has: '
                       f'{", ".join(sorted(available)) or "no topics"})')
    role_of = {topic: role for role, topic in topics.items()}
    poses = {'amcl': [], 'truth': [], 'ekf': [], 'wheel': [], 'initial': []}
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
    if not poses['truth'] or (need_amcl and not poses['amcl']):
        raise BagError(f'{path}: {topics["amcl"]} or {topics["truth"]} has no messages')
    truth = Track.from_rows(poses['truth'])
    # The speed, turn rate and still flag of each kept ground-truth pose (Track.from_rows
    # drops repeated stamps, keeping the last; so does this).
    speeds = np.asarray(speeds, dtype=float)
    speeds = speeds[np.argsort(speeds[:, 0], kind='stable')]
    speeds = speeds[np.append(np.diff(speeds[:, 0]) > 0.0, True)]
    still = (speeds[:, 1] < STILL_SPEED) & (speeds[:, 2] < STILL_TURN)
    imu = np.asarray(imu, dtype=float).reshape(-1, 13)
    imu = imu[np.argsort(imu[:, 0], kind='stable')]
    tracks = {role: Track.from_rows(rows) if rows else None for role, rows in poses.items()}
    return Recording(
        topics=topics, available=frozenset(available), truth=truth, speed=speeds[:, 1],
        turn_rate=speeds[:, 2], still=still, amcl=tracks['amcl'], ekf=tracks['ekf'],
        wheel=tracks['wheel'], initial=tracks['initial'],
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


# ----------------------------------------------------------------------------- odometry


@dataclass(frozen=True)
class Motion:
    """One motion of the robot, found in the ground truth: a drive or a turn in place."""

    kind: str                    # STRAIGHT, TURN or CURVE (a drive that turns too much)
    start: float                 # s: the last ground-truth pose before the motion
    end: float                   # s: the first ground-truth pose after it
    distance: float              # m: the ground-truth path length
    angle: float                 # rad: the ground-truth heading change, signed

    @property
    def judged(self):
        """Return True if Phase 1's limit applies: a long straight or a large turn."""
        if self.kind == STRAIGHT:
            return self.distance >= MIN_STRAIGHT
        return self.kind == TURN and abs(self.angle) >= MIN_TURN


def motions(recording):
    """
    Return the robot's motions, in time order (a tuple of Motion).

    Each ground-truth pose is driving (faster than MOVING_SPEED), turning in place (slower,
    but turning faster than TURNING_RATE) or standing still. A motion is a run of driving or
    turning poses, from the last pose before it to the first pose after it: the robot stands
    still there, or switches between turning and driving. A drive is a straight unless its
    heading changes by more than MAX_STRAIGHT_TURN.
    """
    standing, driving, turning = 0, 1, 2
    moving = recording.speed >= MOVING_SPEED
    state = np.where(moving, driving,
                     np.where(recording.turn_rate >= TURNING_RATE, turning, standing))
    # Each run of one state, as the index of its first and of its last pose.
    changes = np.flatnonzero(np.diff(state)) + 1
    firsts = np.concatenate(([0], changes))
    lasts = np.concatenate((changes, [len(state)])) - 1
    truth = recording.truth
    heading = np.unwrap(truth.yaw)
    travelled = truth.travelled()
    found = []
    for first, last in zip(firsts, lasts):
        if state[first] == standing:
            continue
        before, after = max(first - 1, 0), min(last + 1, len(state) - 1)
        angle = float(heading[after] - heading[before])
        if state[first] == turning:
            kind = TURN
        else:
            kind = STRAIGHT if abs(angle) <= MAX_STRAIGHT_TURN else CURVE
        found.append(Motion(kind, float(truth.t[before]), float(truth.t[after]),
                            float(travelled[after] - travelled[before]), angle))
    return tuple(found)


def straight_error(track, truth, motion):
    """
    Return the error of ``track`` over a drive, as a share of the distance (Phase 1's measure).

    Each source's motion is taken in its own heading at the start, as (forward, left): that
    cancels where its frame lies in the world and the heading error it had before the drive.
    """
    def moved(source):
        x, y, yaw = source.at(np.array([motion.start, motion.end]))
        dx, dy = x[1] - x[0], y[1] - y[0]
        cos, sin = math.cos(yaw[0]), math.sin(yaw[0])
        return np.array([cos * dx + sin * dy, -sin * dx + cos * dy])

    true = moved(truth)
    return float(np.linalg.norm(moved(track) - true) / np.linalg.norm(true))


def turn_error(track, truth, motion):
    """Return the error of ``track`` over a turn, as a share of the angle turned."""
    true = truth.heading_change(motion.start, motion.end)
    return abs(track.heading_change(motion.start, motion.end) - true) / abs(true)


def motion_error(track, truth, motion):
    """Return the error of ``track`` over one motion, or None without poses for all of it."""
    if track is None or not track.covers(motion.start, motion.end):
        return None
    if motion.kind == TURN:
        return turn_error(track, truth, motion)
    return straight_error(track, truth, motion)    # a curve is measured like a straight


@dataclass(frozen=True, eq=False)
class OdometryCheck:
    """Wheel odometry and the EKF against ground truth, motion by motion (--odometry)."""

    motions: tuple               # Motion, in time order
    errors: dict                 # source -> per motion its error (a share), None without poses
    topics: dict                 # source -> its topic


def odometry_check(recording):
    """Return the OdometryCheck of the recording's wheel odometry and EKF."""
    found = motions(recording)
    sources = {'wheel': (recording.wheel, recording.topics['wheel']),
               'EKF': (recording.ekf, recording.topics['ekf'])}
    errors = {name: tuple(motion_error(track, recording.truth, motion) for motion in found)
              for name, (track, _) in sources.items()}
    return OdometryCheck(found, errors, {name: topic for name, (_, topic) in sources.items()})


# ----------------------------------------------------------------------------- criteria


def _criterion(rule, title, passed, summary, advice):
    """Return a RuleResult: the same [PASS] / [FAIL] lines as check_map and the fit rules."""
    return RuleResult(rule, title, () if passed else (advice,), summary)


def criteria(evaluation, robot):
    """
    Return one RuleResult per acceptance criterion: plan F7, and Phase 1's odometry limits.

    With --odometry (``evaluation.odometry`` set), a bag without amcl_pose or without imu
    leaves out those criteria; without it, the bag has amcl_pose, and no IMU fails its two.
    """
    rec, imu, odometry = evaluation.recording, evaluation.imu, evaluation.odometry
    results = []
    if evaluation.localization is not None:
        results += _amcl_criteria(evaluation.localization)
    if odometry is None or rec.topics['imu'] in rec.available:
        results += [
            _sigma_criterion('imu_gyro', 'gyro', imu, 0, robot.imu.gyro_noise_stddev, 'rad/s'),
            _sigma_criterion('imu_accel', 'accelerometer', imu, 3,
                             robot.imu.accel_noise_stddev, 'm/s^2')]
    if odometry is not None:
        results += _odometry_criteria(odometry)
    return tuple(results)


def _amcl_criteria(loc):
    """Return the two AMCL criteria of plan F7: convergence, then tracking."""
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
    return [convergence, tracking]


def _odometry_criteria(check):
    """Return Phase 1's two odometry criteria for wheel odometry, then the two for the EKF."""
    kinds = ((STRAIGHT, 'straight', STRAIGHT_LIMIT, f'straight of {MIN_STRAIGHT:g} m or more'),
             (TURN, 'turn', TURN_LIMIT,
              f'turn in place of {math.degrees(MIN_TURN):g} deg or more'))
    results = []
    for source, errors in check.errors.items():
        topic = check.topics[source]
        for kind, short, limit, what in kinds:
            title = f'{source} odometry ({topic}) under {100.0 * limit:g} % on every {what}'
            rule = f'odometry_{source.lower()}_{short}'
            judged = [(number, motion, error) for number, (motion, error)
                      in enumerate(zip(check.motions, errors), 1)
                      if motion.kind == kind and motion.judged]
            missing = [number for number, _, error in judged if error is None]
            if not judged:
                results.append(_criterion(
                    rule, title, False, f'no {what} in the bag',
                    'drive a route that has one: drive_route --route survey'))
            elif missing:
                results.append(_criterion(
                    rule, title, False, f'no {topic} poses for motion '
                    f'{", ".join(f"#{number}" for number in missing)}',
                    f'record {topic} from the start of the drive to its end'))
            else:
                number, motion, worst = max(judged, key=lambda item: item[2])
                size = (f'{motion.distance:.2f} m' if kind == STRAIGHT
                        else f'{math.degrees(motion.angle):+.1f} deg')
                plural = 's' if len(judged) != 1 else ''
                results.append(_criterion(
                    rule, title, all(error < limit for _, _, error in judged),
                    f'worst {100.0 * worst:.3f} % (#{number}, {size}) on {len(judged)} '
                    f'{short}{plural}',
                    f'#{number} is off by {100.0 * worst:.2f} %: compare wheel_radius and '
                    'wheel_separation in controllers.yaml with robot.yaml, then the physics '
                    'step and wheel slip (the wheel collisions, warehouse_description README)'))
    return results


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
    localization: Localization | None   # None without amcl_pose (only with --odometry)
    drift: Drift | None          # None without EKF odometry
    imu: ImuNoise | None         # None without IMU samples while standing still
    odometry: OdometryCheck | None      # None unless --odometry


def evaluate(path, namespace='robot1', odometry=False):
    """
    Read the bag at ``path`` and evaluate it; return an Evaluation.

    ``odometry``: also check the odometry, motion by motion; the bag then needs no amcl_pose.
    """
    recording = read_recording(path, namespace, need_amcl=not odometry)
    loc = localization(recording) if recording.amcl is not None else None
    # Without AMCL, the EKF's drift counts from the first ground-truth pose.
    start = loc.start if loc is not None else float(recording.truth.t[0])
    return Evaluation(recording, loc, ekf_drift(recording, start), imu_noise(recording),
                      odometry_check(recording) if odometry else None)


def report(path, evaluation, results):
    """Return the text report: the numbers, then the [PASS] / [FAIL] lines."""
    rec, loc = evaluation.recording, evaluation.localization
    drift, imu = evaluation.drift, evaluation.imu
    topics = rec.topics

    def count(track):
        return 0 if track is None else len(track)

    initial = count(rec.initial)
    lines = [f'bag {path}: {count(rec.amcl)} AMCL poses, {len(rec.truth)} ground-truth poses, '
             f'{count(rec.ekf)} EKF poses, {count(rec.wheel)} wheel odometry poses, '
             f'{len(rec.imu_t)} IMU samples, {initial} initial pose{"" if initial == 1 else "s"}']
    if loc is None:
        lines.append(f'AMCL: no {topics["amcl"]} in the bag, not evaluated')
    else:
        lines += _amcl_lines(loc, topics)
    if drift is None:
        lines.append(f'EKF drift: no {topics["ekf"]} in the bag after the start')
    else:
        percent = 100.0 * drift.final / drift.distance if drift.distance > 0.0 else float('nan')
        lines.append(f'EKF drift ({topics["ekf"]}, put on the ground truth at the start): '
                     f'{drift.final:.3f} m after {drift.distance:.1f} m ({percent:.2f} % of the '
                     f'distance), max {drift.largest:.3f} m')
    if evaluation.odometry is not None and topics['imu'] not in rec.available:
        lines.append(f'IMU noise: no {topics["imu"]} in the bag, not evaluated')
    elif imu is None:
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
    if evaluation.odometry is not None:
        lines += _odometry_lines(evaluation.odometry, topics['truth'])
    lines += [str(r) for r in results]
    passed = sum(r.passed for r in results)
    lines.append(f'{passed} of {len(results)} localization checks pass')
    return '\n'.join(lines)


def _amcl_lines(loc, topics):
    """Return the AMCL part of the report."""
    lines = [f'AMCL ({topics["amcl"]}) against ground truth ({topics["truth"]}):']
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
    return lines


def _odometry_lines(check, truth_topic):
    """Return the odometry part of the report: one line per motion of the robot."""
    lines = [f'Odometry against ground truth ({truth_topic}), per motion: the error of wheel '
             f'odometry ({check.topics["wheel"]}) and of the EKF ({check.topics["EKF"]}), as a '
             'share of the distance driven or the angle turned:',
             f'  {"#":>3s}  {"motion":13s} {"sim time [s]":>20s}  {"ground truth":>13s}  '
             f'{"wheel":>9s}  {"EKF":>9s}']
    for number, motion in enumerate(check.motions, 1):
        size = (f'{math.degrees(motion.angle):+9.1f} deg' if motion.kind == TURN
                else f'{motion.distance:9.2f} m  ')
        cells = ['        -' if error is None else f'{100.0 * error:7.3f} %'
                 for error in (check.errors['wheel'][number - 1],
                               check.errors['EKF'][number - 1])]
        if motion.judged:
            note = ''
        elif motion.kind == STRAIGHT:
            note = f'   (under {MIN_STRAIGHT:g} m: not judged)'
        elif motion.kind == TURN:
            note = f'   (under {math.degrees(MIN_TURN):g} deg: not judged)'
        else:
            note = (f'   (turns {math.degrees(motion.angle):+.1f} deg while driving: not '
                    'judged)')
        lines.append(f'  {number:3d}  {motion.kind:13s} {motion.start:9.2f} -{motion.end:9.2f}  '
                     f'{size}  {cells[0]}  {cells[1]}{note}')
    if not check.motions:
        lines.append('  the robot did not move')
    return lines


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
    parser.add_argument('--odometry', action='store_true',
                        help='also check wheel odometry and the EKF against ground truth on '
                             "each straight and turn in place, with Phase 1's limits (1 %% and "
                             '2 %%); the bag then needs no amcl_pose')
    args = parser.parse_args(argv)

    try:
        robot = robot_config.load()
        evaluation = evaluate(args.bag, args.namespace, odometry=args.odometry)
    except (OSError, RuntimeError, ValueError) as error:   # BagError is a ValueError
        print(f'error: {error}', file=sys.stderr)
        return 2
    if args.plot and evaluation.localization is None:
        print(f'error: --plot draws AMCL against ground truth, and {args.bag} has no '
              f'{evaluation.recording.topics["amcl"]}', file=sys.stderr)
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
