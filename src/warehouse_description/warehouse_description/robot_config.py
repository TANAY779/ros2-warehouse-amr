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
The robot's dimensions for Python code: robot.yaml plus the numbers derived from it.

config/robot.yaml is the single source of the robot's geometry (plan section 2.6, D1). The
xacro reads it for the URDF; Python code reads it through this module: the warehouse layout's
fit rules, Nav2's footprint and the safety fields. Derived numbers (radii, the scan plane
height, the area a stop field must cover) are computed here and never typed in by hand, so a
change to robot.yaml reaches every user. This module contains no number from robot.yaml.

Usage::

    from warehouse_description import robot_config

    robot = robot_config.load()     # the installed robot.yaml
    robot.r_circumscribed_padded    # 0.456 m: corner radius of the padded outline
    robot.scan_plane_height         # 0.34 m: the lidar's scan plane above the floor

    base = robot_config.load_base(robot)             # the base's limits (controllers.yaml)
    robot.field_need(base, (0.05, 0.30), (-1.0, 1.0))
    # Extents(front=0.542..., rear=-0.426..., ...): what a stop field for that band must cover
"""

from dataclasses import dataclass
from dataclasses import field
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml

# The sweep model of the safety fields (plan section 3.4): its time step, and the commands per
# axis of a band, both bounds included. They are those of the plan's sweep_check.py, so the
# results are the plan's. For the six bands of plan section 3.4, 13 x 13 commands give the
# same millimetres; a 1 ms step moves some of the extents by 1 mm.
SWEEP_TIME_STEP = 0.002   # s
BAND_SAMPLES = 7


def default_path():
    """Return the path of the installed robot.yaml (in the package's share directory)."""
    share = get_package_share_directory('warehouse_description')
    return os.path.join(share, 'config', 'robot.yaml')


def controllers_path():
    """Return the path of the installed controllers.yaml (in the package's share directory)."""
    share = get_package_share_directory('warehouse_description')
    return os.path.join(share, 'config', 'controllers.yaml')


@dataclass(frozen=True)
class Lidar:
    """The 2D lidar (robot.yaml ``lidar_front``). x and y are measured from base_link."""

    x: float
    y: float
    height: float          # above the floor: the scan plane
    yaw_deg: float
    fov_deg: float
    samples: int
    update_rate: float     # Hz
    range_min: float
    range_max: float
    noise_stddev: float
    housing_radius: float
    housing_height: float   # from the deck to the top of the puck


@dataclass(frozen=True)
class Imu:
    """The IMU (robot.yaml ``imu``): where it sits, its rate and its noise."""

    x: float
    y: float
    height: float
    update_rate: float          # Hz
    gyro_noise_stddev: float    # rad/s
    accel_noise_stddev: float   # m/s^2


@dataclass(frozen=True)
class Safety:
    """The inputs of the safety fields (robot.yaml ``safety``, plan section 3.4)."""

    reaction_time: float     # s, from an obstacle appearing to the base starting to brake
    stop_margin: float       # m, added on every side of the swept outline
    rotation_margin: float   # m, added to the corner radius for the circle turning in place


@dataclass(frozen=True)
class Base:
    """
    What the base does with a command, as the sweep model needs it (see load_base()).

    diff_drive_controller clamps every command to its limits. When it brakes, v and w each
    fall towards zero at their own deceleration.
    """

    min_velocity: float          # m/s, the fastest reverse (negative)
    max_velocity: float          # m/s, forward
    min_angular_velocity: float  # rad/s, clockwise (negative)
    max_angular_velocity: float  # rad/s, counter-clockwise
    braking: float               # m/s^2, linear deceleration (positive)
    angular_braking: float       # rad/s^2, angular deceleration (positive)

    def clamp(self, v, w):
        """Return the command (v, w) as the base carries it out: within its limits."""
        return (min(max(v, self.min_velocity), self.max_velocity),
                min(max(w, self.min_angular_velocity), self.max_angular_velocity))


@dataclass(frozen=True)
class Extents:
    """How far an area round base_footprint reaches, in metres (x forward, y left)."""

    front: float        # the largest x
    rear: float         # the smallest x (negative: behind base_footprint)
    half_width: float   # the largest |y|
    radius: float       # the largest distance from base_footprint

    def union(self, other):
        """Return the Extents of both areas together."""
        return Extents(max(self.front, other.front), min(self.rear, other.rear),
                       max(self.half_width, other.half_width), max(self.radius, other.radius))

    def grown(self, margin):
        """Return the Extents grown by ``margin`` on every side."""
        return Extents(self.front + margin, self.rear - margin, self.half_width + margin,
                       self.radius + margin)


@dataclass(frozen=True)
class RobotConfig:
    """
    robot.yaml as typed values, plus the quantities derived from them (metres).

    The outline is a rectangle centred on base_link: ``length`` along x (forward),
    ``width`` along y. ``padding`` is Nav2's footprint_padding, added on every side.
    ``raw`` holds the whole file as loaded, for values that have no field here yet.
    """

    revision: str
    length: float
    width: float
    padding: float
    clearance: float        # floor to the underside of the body
    deck_height: float      # floor to the top of the deck
    max_deceleration: float  # m/s^2, braking from forward (negative, as in robot.yaml)
    lidar: Lidar
    imu: Imu
    safety: Safety
    raw: dict = field(repr=False, compare=False)

    @property
    def half_length(self):
        """Return the distance from base_link to the front (or rear) of the outline."""
        return self.length / 2

    @property
    def half_width(self):
        """Return the distance from base_link to either side of the outline."""
        return self.width / 2

    @property
    def r_inscribed(self):
        """Return the radius of the largest circle round base_link inside the outline."""
        return min(self.half_length, self.half_width)

    @property
    def r_circumscribed(self):
        """Return the radius of the circle round base_link through the outline's corners."""
        return math.hypot(self.half_length, self.half_width)

    @property
    def padded_half_length(self):
        """Return half the length of the outline grown by the padding."""
        return self.half_length + self.padding

    @property
    def padded_half_width(self):
        """Return half the width of the outline grown by the padding."""
        return self.half_width + self.padding

    @property
    def r_inscribed_padded(self):
        """Return the inscribed radius of the padded outline."""
        return min(self.padded_half_length, self.padded_half_width)

    @property
    def r_circumscribed_padded(self):
        """
        Return the circumscribed radius of the padded outline.

        A robot turning in place sweeps this circle, so goal poses and turning space are
        sized from it (layout fit rules, plan section 2.6).
        """
        return math.hypot(self.padded_half_length, self.padded_half_width)

    @property
    def rotation_circle_radius(self):
        """
        Return the radius of the circle that a robot standing or turning in place keeps clear.

        The corner radius plus the safety section's rotation margin (plan section 2.6). The
        collision monitor's field for turning in place is drawn round it.
        """
        return self.r_circumscribed + self.safety.rotation_margin

    @property
    def scan_plane_height(self):
        """Return the height above the floor of the plane the lidar scans."""
        return self.lidar.height

    def footprint_polygon(self, padded=False):
        """
        Return the outline's corners in base_link, counter-clockwise from front-left.

        With ``padded=True`` the outline is grown by ``padding`` on every side.
        """
        x = self.padded_half_length if padded else self.half_length
        y = self.padded_half_width if padded else self.half_width
        return ((x, y), (-x, y), (-x, -y), (x, -y))

    def nav2_footprint(self):
        """
        Return the outline as the string Nav2's ``footprint`` parameter expects.

        For example ``[[0.32, 0.24], [-0.32, 0.24], [-0.32, -0.24], [0.32, -0.24]]``.
        It is the unpadded outline: Nav2 adds ``footprint_padding`` itself.
        """
        points = ', '.join(f'[{x:g}, {y:g}]' for x, y in self.footprint_polygon())
        return f'[{points}]'

    def swept_extents(self, base, v, w):
        """
        Return the Extents of the outline swept by one command, until the robot stands.

        The sweep model of plan section 3.4: an obstacle appears, the command (v [m/s],
        w [rad/s]) clamped to the base's limits holds for the safety section's reaction time,
        then the base brakes v and w to zero, each at its own deceleration. The outline is
        moved in steps of SWEEP_TIME_STEP, in the base_footprint frame of the moment the
        obstacle appeared. A rectangle reaches furthest in every direction (and from
        base_footprint) at a corner, so the four corners give the extents. No margin is added.
        """
        v, w = base.clamp(v, w)
        step = SWEEP_TIME_STEP
        corners = self.footprint_polygon()
        x = y = heading = t = 0.0
        front, rear, half_width, radius = -math.inf, math.inf, 0.0, 0.0
        while True:
            if t > self.safety.reaction_time:
                # Braking: v and w each fall towards zero at the base's deceleration.
                v = math.copysign(max(abs(v) - base.braking * step, 0.0), v)
                w = math.copysign(max(abs(w) - base.angular_braking * step, 0.0), w)
                if v == 0.0 and w == 0.0:
                    break
            x += v * math.cos(heading) * step
            y += v * math.sin(heading) * step
            heading += w * step
            t += step
            c, s = math.cos(heading), math.sin(heading)
            for px, py in corners:
                cx, cy = x + c * px - s * py, y + s * px + c * py
                front, rear = max(front, cx), min(rear, cx)
                half_width = max(half_width, abs(cy))
                radius = max(radius, math.hypot(cx, cy))
        return Extents(front, rear, half_width, radius)

    def field_need(self, base, linear, angular):
        """
        Return the Extents that a stop field for one band of commands must cover.

        ``linear`` and ``angular`` are the band's (min, max) of v [m/s] and w [rad/s], bounds
        included. The need is the outline swept by BAND_SAMPLES x BAND_SAMPLES commands spread
        evenly over the band (swept_extents()), grown by the safety section's stop margin.
        """
        def samples(low, high):
            return [low + (high - low) * i / (BAND_SAMPLES - 1) for i in range(BAND_SAMPLES)]

        swept = None
        for v in samples(*linear):
            for w in samples(*angular):
                extents = self.swept_extents(base, v, w)
                swept = extents if swept is None else swept.union(extents)
        return swept.grown(self.safety.stop_margin)


def _number(data, key, source, valid=None, rule=''):
    """
    Return the number at a dotted key of a nested dict as a float.

    Raise a ValueError naming the key if it is missing or not a number, or if ``valid`` (a
    function of the value) is given and returns False; ``rule`` says what valid means.
    """
    node = data
    for part in key.split('.'):
        if not isinstance(node, dict) or part not in node:
            raise ValueError(f'{source}: missing key {key}')
        node = node[part]
    if isinstance(node, bool) or not isinstance(node, (int, float)):
        raise ValueError(f'{source}: {key} must be a number, not {node!r}')
    if valid is not None and not valid(node):
        raise ValueError(f'{source}: {key} must be {rule}, not {node}')
    return float(node)


def _negative(value):
    return value < 0.0


def _positive(value):
    return value > 0.0


def load(path=None):
    """Read robot.yaml (the installed one by default) and return a RobotConfig."""
    path = path or default_path()
    with open(path) as stream:
        data = yaml.safe_load(stream)
    return from_dict(data, source=path)


def from_dict(data, source='robot.yaml'):
    """
    Return the RobotConfig for robot.yaml's content (a dict).

    load() uses it. A missing key raises ValueError naming it, for example
    ``robot.yaml: missing key footprint.width``, and so does a safety value or braking
    deceleration with the wrong sign.
    """
    if not isinstance(data, dict):
        raise ValueError(f'{source}: expected a mapping, not {data!r}')

    def number(key, valid=None, rule=''):
        return _number(data, key, source, valid, rule)

    return RobotConfig(
        revision=str(data.get('revision', '')),
        length=number('footprint.length'),
        width=number('footprint.width'),
        padding=number('footprint.padding'),
        clearance=number('body.clearance'),
        deck_height=number('body.deck_height'),
        max_deceleration=number('limits.linear.max_deceleration', _negative,
                                'below 0 (a deceleration is negative by convention)'),
        lidar=Lidar(
            x=number('lidar_front.x'),
            y=number('lidar_front.y'),
            height=number('lidar_front.height'),
            yaw_deg=number('lidar_front.yaw_deg'),
            fov_deg=number('lidar_front.fov_deg'),
            samples=int(number('lidar_front.samples')),
            update_rate=number('lidar_front.update_rate'),
            range_min=number('lidar_front.range_min'),
            range_max=number('lidar_front.range_max'),
            noise_stddev=number('lidar_front.noise_stddev'),
            housing_radius=number('lidar_front.housing_radius'),
            housing_height=number('lidar_front.housing_height'),
        ),
        imu=Imu(
            x=number('imu.x'),
            y=number('imu.y'),
            height=number('imu.height'),
            update_rate=number('imu.update_rate'),
            gyro_noise_stddev=number('imu.gyro_noise_stddev'),
            accel_noise_stddev=number('imu.accel_noise_stddev'),
        ),
        safety=Safety(
            reaction_time=number('safety.reaction_time', _positive, 'above 0'),
            stop_margin=number('safety.stop_margin', lambda value: value >= 0.0, 'at least 0'),
            rotation_margin=number('safety.rotation_margin', lambda value: value >= 0.0,
                                   'at least 0'),
        ),
        raw=data,
    )


def load_base(robot, path=None):
    """
    Return the Base of the sweep model: controllers.yaml's limits, robot.yaml's braking.

    diff_drive_controller enforces the limits of its section in controllers.yaml (the
    installed one by default, key ``/**/diff_drive_controller``): the range of v and w it
    carries out, and the angular deceleration it brakes with. The linear braking is
    robot.yaml's ``limits.linear.max_deceleration`` (``robot``, a RobotConfig), which that
    file names as the safety fields' stop deceleration; controllers.yaml repeats it.
    """
    path = path or controllers_path()
    with open(path) as stream:
        data = yaml.safe_load(stream)
    source = f'{path}, /**/diff_drive_controller'
    section = data.get('/**/diff_drive_controller') if isinstance(data, dict) else None
    parameters = section.get('ros__parameters') if isinstance(section, dict) else None

    def number(key, valid, rule):
        return _number(parameters, key, source, valid, rule)

    return Base(
        min_velocity=number('linear.x.min_velocity', lambda value: value <= 0.0, 'at most 0'),
        max_velocity=number('linear.x.max_velocity', _positive, 'above 0'),
        min_angular_velocity=number('angular.z.min_velocity', _negative, 'below 0'),
        max_angular_velocity=number('angular.z.max_velocity', _positive, 'above 0'),
        braking=-robot.max_deceleration,
        angular_braking=-number('angular.z.max_deceleration', _negative, 'below 0'),
    )
