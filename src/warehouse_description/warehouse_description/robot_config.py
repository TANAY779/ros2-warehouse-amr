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
fit rules now, Nav2 footprints and safety fields later. Derived numbers (radii, the scan
plane height) are computed here and never typed in by hand, so a change to robot.yaml
reaches every user. This module contains no number from robot.yaml.

Usage::

    from warehouse_description import robot_config

    robot = robot_config.load()     # the installed robot.yaml
    robot.r_circumscribed_padded    # 0.456 m: corner radius of the padded outline
    robot.scan_plane_height         # 0.34 m: the lidar's scan plane above the floor
"""

from dataclasses import dataclass
from dataclasses import field
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml


def default_path():
    """Return the path of the installed robot.yaml (in the package's share directory)."""
    share = get_package_share_directory('warehouse_description')
    return os.path.join(share, 'config', 'robot.yaml')


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
    lidar: Lidar
    imu: Imu
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
    ``robot.yaml: missing key footprint.width``.
    """
    if not isinstance(data, dict):
        raise ValueError(f'{source}: expected a mapping, not {data!r}')

    def number(key):
        node = data
        for part in key.split('.'):
            if not isinstance(node, dict) or part not in node:
                raise ValueError(f'{source}: missing key {key}')
            node = node[part]
        if isinstance(node, bool) or not isinstance(node, (int, float)):
            raise ValueError(f'{source}: {key} must be a number, not {node!r}')
        return node

    return RobotConfig(
        revision=str(data.get('revision', '')),
        length=float(number('footprint.length')),
        width=float(number('footprint.width')),
        padding=float(number('footprint.padding')),
        clearance=float(number('body.clearance')),
        deck_height=float(number('body.deck_height')),
        lidar=Lidar(
            x=float(number('lidar_front.x')),
            y=float(number('lidar_front.y')),
            height=float(number('lidar_front.height')),
            yaw_deg=float(number('lidar_front.yaw_deg')),
            fov_deg=float(number('lidar_front.fov_deg')),
            samples=int(number('lidar_front.samples')),
            update_rate=float(number('lidar_front.update_rate')),
            range_min=float(number('lidar_front.range_min')),
            range_max=float(number('lidar_front.range_max')),
            noise_stddev=float(number('lidar_front.noise_stddev')),
            housing_radius=float(number('lidar_front.housing_radius')),
            housing_height=float(number('lidar_front.housing_height')),
        ),
        imu=Imu(
            x=float(number('imu.x')),
            y=float(number('imu.y')),
            height=float(number('imu.height')),
            update_rate=float(number('imu.update_rate')),
            gyro_noise_stddev=float(number('imu.gyro_noise_stddev')),
            accel_noise_stddev=float(number('imu.accel_noise_stddev')),
        ),
        raw=data,
    )
