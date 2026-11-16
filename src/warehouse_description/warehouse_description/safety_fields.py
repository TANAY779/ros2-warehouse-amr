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
Print what each safety field must cover next to the field as configured (plan section 3.4).

    ros2 run warehouse_description safety_fields

The collision monitor stops the robot when the scan has points inside the field of the band
that the command falls in: warehouse_navigation's config/nav2_params.yaml, collision_monitor,
velocity_fields. For each band this prints what its polygon must cover, from robot.yaml and
controllers.yaml through robot_config's sweep model, next to the polygon and the spare on each
side. Then it prints which bands the commands of Nav2, of the base and of the teleop window
select, and the commands that select none.

It is a table without a verdict (plan Q4 (c)): the safety scenarios of stages S8a and S8b are
the verdict. It reads files only, starts no ROS node, and exits 0 once it has printed the
table, whatever the spares are; 1 if it cannot read its inputs.
"""

import argparse
from dataclasses import dataclass
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import yaml

from warehouse_description import robot_config


@dataclass(frozen=True)
class Band:
    """One band of velocity_fields: the commands it takes and its polygon in base_footprint."""

    name: str
    linear: tuple    # (min, max) of v [m/s], bounds included
    angular: tuple   # (min, max) of w [rad/s], bounds included
    points: tuple    # ((x, y), ...) [m]

    def contains(self, v, w):
        """Return True if the command (v, w) falls in this band."""
        return (self.linear[0] <= v <= self.linear[1]
                and self.angular[0] <= w <= self.angular[1])


def nav2_params_path():
    """
    Return the path of warehouse_navigation's installed nav2_params.yaml.

    warehouse_navigation depends on this package, so this package cannot declare a dependency
    on it (colcon refuses the cycle): the file is looked up only when the tool runs.
    """
    try:
        share = get_package_share_directory('warehouse_navigation')
    except PackageNotFoundError:
        raise ValueError('warehouse_navigation is not installed: build it (colcon build '
                         '--packages-up-to warehouse_navigation) or give --nav2-params') from None
    return os.path.join(share, 'config', 'nav2_params.yaml')


def _parameters(data, node, path):
    """Return a node's ros__parameters from nav2_params.yaml (plain node keys, Nav2's style)."""
    section = data.get(node) if isinstance(data, dict) else None
    parameters = section.get('ros__parameters') if isinstance(section, dict) else None
    if not isinstance(parameters, dict):
        raise ValueError(f'{path}: no {node}: ros__parameters section (plain node keys)')
    return parameters


def _number(value, what):
    """Return a YAML number as a float; raise ValueError naming it otherwise."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{what} must be a number, not {value!r}')
    return float(value)


def read_nav2(path):
    """
    Return the bands of velocity_fields, and the (v, w) ranges of the velocity smoother.

    The smoother's limits ([x, y, theta] lists) bound every command Nav2 sends.
    """
    with open(path) as stream:
        data = yaml.safe_load(stream)
    fields = _parameters(data, 'collision_monitor', path).get('velocity_fields')
    names = fields.get('velocity_polygons') if isinstance(fields, dict) else None
    if not names:
        raise ValueError(f'{path}: collision_monitor has no velocity_fields.velocity_polygons')
    bands = []
    for name in names:
        where = f'{path}: collision_monitor velocity_fields.{name}'
        entry = fields.get(name)
        if not isinstance(entry, dict):
            raise ValueError(f'{where} is missing')
        # Nav2 1.3.13 takes the points as one string, "[[x, y], ...]", which reads as YAML.
        text = entry.get('points')
        points = yaml.safe_load(text) if isinstance(text, str) else None
        if not (isinstance(points, list) and len(points) >= 3
                and all(isinstance(p, list) and len(p) == 2 for p in points)):
            raise ValueError(f'{where}.points must be a string of three or more [x, y] pairs')
        bands.append(Band(
            name=name,
            linear=tuple(_number(entry.get(key), f'{where}.{key}')
                         for key in ('linear_min', 'linear_max')),
            angular=tuple(_number(entry.get(key), f'{where}.{key}')
                          for key in ('theta_min', 'theta_max')),
            points=tuple((_number(x, f'{where}.points'), _number(y, f'{where}.points'))
                         for x, y in points)))
    smoother = _parameters(data, 'velocity_smoother', path)
    limits = {}
    for key in ('min_velocity', 'max_velocity'):
        values = smoother.get(key)
        if not (isinstance(values, list) and len(values) == 3):
            raise ValueError(f'{path}: velocity_smoother {key} must be a list [x, y, theta]')
        limits[key] = [_number(value, f'{path}: velocity_smoother {key}') for value in values]
    return (bands, (limits['min_velocity'][0], limits['max_velocity'][0]),
            (limits['min_velocity'][2], limits['max_velocity'][2]))


def _distance_from_origin(a, b):
    """Return the distance from base_footprint (0, 0) to the segment from a to b."""
    (x1, y1), (x2, y2) = a, b
    dx, dy = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy)))
    return math.hypot(x1 + t * dx, y1 + t * dy)


def _contains_origin(points):
    """Return True if the polygon contains base_footprint (0, 0): count edge crossings."""
    inside = False
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        if (y1 > 0.0) != (y2 > 0.0) and x1 + (0.0 - y1) * (x2 - x1) / (y2 - y1) > 0.0:
            inside = not inside
    return inside


def polygon_reach(points):
    """
    Return how far a polygon round base_footprint is sure to reach, in metres.

    An axis-aligned rectangle: ('box', front, rear, half_width), where half_width is the nearer
    of its two sides. Any other polygon: ('radius', r), the distance from base_footprint to its
    nearest side (the largest circle round base_footprint inside it), negative if
    base_footprint lies outside it.
    """
    xs = sorted({x for x, _ in points})
    ys = sorted({y for _, y in points})
    if len(points) == 4 and set(points) == {(x, y) for x in xs for y in ys}:
        return 'box', xs[1], xs[0], min(ys[1], -ys[0])
    nearest = min(_distance_from_origin(a, b) for a, b in zip(points, points[1:] + points[:1]))
    return 'radius', nearest if _contains_origin(points) else -nearest


def coverage(bands, linear, angular):
    """
    Return the bands that commands in a range select, and the parts of the range in none.

    The monitor takes the first band that contains the command, bounds included. The bands'
    bounds cut the range into cells: each bound, and each gap between two bounds, on each axis.
    One command per cell tests it, which is exact for rectangles. The parts in no band come
    back as ((v_low, v_high), (w_low, w_high)) rectangles of neighbouring cells; an edge of
    such a part may itself lie in a band.
    """
    def cells(low, high, bounds):
        """Return (a command, its cell's low end, high end) for each cell, in order."""
        values = sorted({low, high} | {b for b in bounds if low < b < high})
        samples = [(value, value, value) for value in values]
        samples += [((a + b) / 2, a, b) for a, b in zip(values, values[1:])]
        return sorted(samples)

    v_bounds = [bound for band in bands for bound in band.linear]
    w_bounds = [bound for band in bands for bound in band.angular]
    selected = set()
    rows = []   # per cell of v: its runs of neighbouring w cells that are in no band
    for v, v_low, v_high in cells(*linear, v_bounds):
        runs = []
        in_run = False
        for w, w_low, w_high in cells(*angular, w_bounds):
            first = next((band for band in bands if band.contains(v, w)), None)
            if first is not None:
                selected.add(first.name)
                in_run = False
            elif in_run:
                runs[-1] = (runs[-1][0], w_high)
            else:
                runs.append((w_low, w_high))
                in_run = True
        rows.append((v_low, v_high, tuple(runs)))
    # Neighbouring cells of v with the same runs of w form one rectangle.
    merged = []
    for v_low, v_high, runs in rows:
        if runs and merged and merged[-1][2] == runs and merged[-1][1] == v_low:
            merged[-1] = (merged[-1][0], v_high, runs)
        elif runs:
            merged.append((v_low, v_high, runs))
    uncovered = [((v_low, v_high), run) for v_low, v_high, runs in merged for run in runs]
    return [band.name for band in bands if band.name in selected], uncovered


def _span(low, high, decimals=2):
    """Return a range as text, '-0.30 … 1.20', or one value when low equals high."""
    if low == high:
        return f'{low:.{decimals}f}'
    return f'{low:.{decimals}f} … {high:.{decimals}f}'


def print_inputs(paths, robot, base, bands, nav2_linear, nav2_angular):
    """Print the files read and every input with the file and key it comes from."""
    safety = robot.safety
    print('Safety fields of the collision monitor (plan §3.4): what each band needs, next to '
          'its polygon.')
    print('A table without a verdict: the safety scenarios of stages S8a and S8b are the '
          'verdict.')
    print('\nFiles')
    for name, path in paths.items():
        print(f'  {name:<17} {path}')
    # Numbers as the files write them (str() of a float: 0.25, -0.3, 1.0).
    inputs = [
        ('outline', f'{robot.length} x {robot.width} m: x ±{robot.half_length}, '
                    f'y ±{robot.half_width}', 'robot.yaml', 'footprint.length, .width'),
        ('braking', f'{base.braking} m/s²', 'robot.yaml', 'limits.linear.max_deceleration'),
        ('reaction time', f'{safety.reaction_time} s', 'robot.yaml', 'safety.reaction_time'),
        ('stop margin', f'{safety.stop_margin} m', 'robot.yaml', 'safety.stop_margin'),
        ('rotation margin', f'{safety.rotation_margin} m', 'robot.yaml',
         'safety.rotation_margin'),
        ("base's limits",
         f'v {base.min_velocity} … {base.max_velocity} m/s, '
         f'ω {base.min_angular_velocity} … {base.max_angular_velocity} rad/s',
         'controllers.yaml', 'linear.x, angular.z: min/max_velocity'),
        ('angular braking', f'{base.angular_braking} rad/s²', 'controllers.yaml',
         'angular.z.max_deceleration'),
        ("Nav2's limits",
         f'v {nav2_linear[0]} … {nav2_linear[1]} m/s, '
         f'ω {nav2_angular[0]} … {nav2_angular[1]} rad/s',
         'nav2_params.yaml', 'velocity_smoother: min/max_velocity'),
        ('bands', ' '.join(band.name for band in bands), 'nav2_params.yaml',
         'collision_monitor: velocity_fields'),
    ]
    print('\nInputs (the base is diff_drive_controller, configured in controllers.yaml)')
    for name, value, source, key in inputs:
        print(f'  {name:<16} {value:<41} {source:<17} {key}')


def print_bands(robot, base, bands):
    """Print one row per band: its commands, its need, its polygon and the spare."""
    samples = robot_config.BAND_SAMPLES
    print(f'\nNeed: the outline swept from the moment an obstacle appears until the robot '
          f'stands, for {samples} x {samples}\ncommands spread over the band, each clamped to '
          'the base\'s limits: the command holds for the reaction\ntime, then the base brakes '
          'v and ω to zero. Plus the stop margin. Spare: how much further the polygon\n'
          'reaches on that side (negative: not far enough). Metres, in base_footprint.\n')
    group = 23   # the width of one group of three sides, as in `sides` below
    print(f'{"":<45}{"need":<{group}}{"polygon":<{group}}spare')
    sides = f'{"front":>6} {"rear":>7} {"|y|":>6}  '
    print(f' #  {"band":<10} {"v [m/s]":<14} {"ω [rad/s]":<14} {sides * 3}'.rstrip())
    notes = []
    for order, band in enumerate(bands, start=1):
        need = robot.field_need(base, band.linear, band.angular)
        reach = polygon_reach(band.points)
        if reach[0] == 'box':
            _, front, rear, half_width = reach
            cells = (f'{need.front:6.3f} {need.rear:7.3f} {need.half_width:6.3f}  '
                     f'{front:6.3f} {rear:7.3f} {half_width:6.3f}  '
                     f'{front - need.front:6.3f} {need.rear - rear:7.3f} '
                     f'{half_width - need.half_width:6.3f}')
        else:
            radius = reach[1]
            cells = ''.join(f'{" radius " + format(value, ".3f"):<{group}}' for value in (
                need.radius, radius, radius - need.radius)).rstrip()
            notes.append(
                f'{band.name}: drawn round the circle that a robot standing or turning in '
                f'place keeps clear (plan §2.6):\n  corner radius {robot.r_circumscribed:.3f} '
                f'+ rotation margin {robot.safety.rotation_margin:.3f} = '
                f'{robot.rotation_circle_radius:.3f} m.\n  Its polygon\'s sides are '
                f'{radius:.4f} m from base_footprint at the nearest.')
        print(f'{order:2d}  {band.name:<10} {_span(*band.linear):<14} '
              f'{_span(*band.angular):<14} {cells}')
    for note in notes:
        print(f'\n{note}')


def print_coverage(base, bands, nav2_linear, nav2_angular):
    """Print, for the commands of Nav2, the base and the teleop window, the bands they select."""
    print('\nThe band each command selects: the first that contains it, bounds included.')
    # The teleop window's default top speeds (max_linear_speed, max_angular_speed) are the
    # base's forward and turn limits, and it reverses as fast as it drives forward.
    base_angular = (base.min_angular_velocity, base.max_angular_velocity)
    for name, linear, angular in (
            ("Nav2's commands (the velocity smoother's limits)", nav2_linear, nav2_angular),
            ('the base (every command it carries out)',
             (base.min_velocity, base.max_velocity), base_angular),
            ("the teleop window (the base's top speeds, also backwards)",
             (-base.max_velocity, base.max_velocity), base_angular)):
        selected, uncovered = coverage(bands, linear, angular)
        print(f'  {name}: v {_span(*linear)} m/s, ω {_span(*angular)} rad/s')
        print(f'    selects {", ".join(selected) or "no band"}')
        if not uncovered:
            print('    in no band: none')
        for v_range, w_range in uncovered:
            print(f'    in no band: v {_span(*v_range)} m/s, ω {_span(*w_range)} rad/s')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Print what each safety field must cover (from robot.yaml and '
                    'controllers.yaml) next to the polygons in nav2_params.yaml. A table, no '
                    'verdict.')
    parser.add_argument('--robot', metavar='FILE',
                        help='robot.yaml (default: the installed one)')
    parser.add_argument('--controllers', metavar='FILE',
                        help='controllers.yaml (default: the installed one)')
    parser.add_argument('--nav2-params', metavar='FILE',
                        help="nav2_params.yaml (default: warehouse_navigation's installed one)")
    args = parser.parse_args(argv)
    try:
        paths = {
            'robot.yaml': args.robot or robot_config.default_path(),
            'controllers.yaml': args.controllers or robot_config.controllers_path(),
            'nav2_params.yaml': args.nav2_params or nav2_params_path(),
        }
        robot = robot_config.load(paths['robot.yaml'])
        base = robot_config.load_base(robot, paths['controllers.yaml'])
        bands, nav2_linear, nav2_angular = read_nav2(paths['nav2_params.yaml'])
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'safety_fields: {error}', file=sys.stderr)
        return 1
    print_inputs(paths, robot, base, bands, nav2_linear, nav2_angular)
    print_bands(robot, base, bands)
    print_coverage(base, bands, nav2_linear, nav2_angular)
    return 0
