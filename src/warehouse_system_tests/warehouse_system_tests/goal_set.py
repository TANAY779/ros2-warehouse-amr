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
Write the frozen goal set of the navigation benchmark, or read it.

Usage, from the workspace root::

    ros2 run warehouse_system_tests goal_set --count 50 --seed 1 [--src src] [--out FILE]

It reads the source tree, as ``warehouse_layout generate`` does: the layout
(src/warehouse_layout/config/layout.yaml), the robot (src/warehouse_description/config/
robot.yaml) and the committed SLAM map (src/warehouse_navigation/maps/<world>.yaml and its
image). It writes src/warehouse_system_tests/config/goal_sets/<world>_<count>.yaml, the file
nav_benchmark sends goals from, or FILE with --out (to try a seed without touching the
committed set). A file is rewritten only when its text changes: a second run with the same
inputs prints "unchanged".

**Why frozen** (plan P3-D22). Every stage that judges navigation drives the same goals, so its
numbers compare with every other stage's: S3b goals 1-10, S4 and S6b 1-20, S8b 21-40, and the
acceptance runs (3a-runs, S9) all 50. The goals come from the layout model and the SLAM map,
never from the keepout masks of Phase 3b, so the masks do not change the set.

**The rules**, for every goal:

1. Its goal circle, 0.556 m round the centre (the layout's goal clearance: the padded
   circumscribed radius 0.456 m plus 0.10 m, so the robot can turn in place there), holds only
   free cells, both in the SLAM map and in rasterize's ideal map. The distance is exact: from
   the centre to the nearest point of any occupied or unknown cell.
2. Its centre is at least 0.95 m from every keepout zone's true polygon: outside the planning
   core (0.65 m) and its cost ramp (0.30 m) of plan section 3.3.
3. Its centre lies in the free space of the robot's centre that is connected to robot1's
   spawn, in both maps: cells more than the padded inscribed radius (0.28 m) from every
   occupied or unknown cell, as Nav2's planner sees free space, without the planning cores. So
   the masks of Phase 3b cannot cut a goal off.
4. It is at least 3.0 m from the goal before it (goal 1: from the spawn).
5. Its yaw is uniform over a full turn.

**Deterministic.** Candidates come from ``random.Random(seed).random()``, the one stream Python
promises never to change for a seed. x and y are drawn uniformly over the hall's floor and
rounded to 1 mm, the yaw rounded to 0.0001 rad, before they are checked, so the file holds
exactly the values that were checked. The text is written with fixed decimals, never with
``repr()``, and holds no date; zones and maps are visited in the layout's order. The header
lists the inputs with their sha256, so a changed map or layout shows.

Exit codes: 0 written (or unchanged); 2 an input cannot be read, the rules leave no goal within
MAX_CANDIDATES draws, or the file cannot be written.
"""

import argparse
from dataclasses import dataclass
import hashlib
import math
import os
import random
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import occupancy
from warehouse_layout import rasterize
from warehouse_layout import rules
from warehouse_system_tests import zones
import yaml

SCHEMA_VERSION = 1
FRAME_ID = 'map'
SPAWN_ROBOT = 'robot1'           # goal 1's distance and the connected free space start here
KEEPOUT_DISTANCE = zones.PLANNING_CORE + zones.PLANNING_RAMP     # 0.95 m (rule 2)
MIN_STEP = 3.0                   # m from the goal before (rule 4)
XY_DECIMALS = 3                  # 1 mm
YAW_DECIMALS = 4                 # 0.0001 rad
MAX_CANDIDATES = 200000          # draws before the tool gives up

# The order of the rules as candidates are checked (cheapest first), and their names in the
# report. The order only decides which rule a rejected candidate is counted under.
CHECKS = (
    ('step', f'closer than {MIN_STEP:g} m to the goal before'),
    ('keepout', f'closer than {KEEPOUT_DISTANCE:.2f} m to a keepout zone'),
    ('slam_circle', 'goal circle not free in the SLAM map'),
    ('ideal_circle', "goal circle not free in rasterize's ideal map"),
    ('slam_connected', 'not connected to the spawn in the SLAM map'),
    ('ideal_connected', "not connected to the spawn in the ideal map"),
)


class GoalSetError(ValueError):
    """A goal set file cannot be read; the message names the file and the problem."""


@dataclass(frozen=True)
class Goal:
    """One goal of the set: its 1-based index and its pose in the map frame."""

    index: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GoalSet:
    """A goal set file as read by load()."""

    path: str
    world: str
    seed: int
    start: tuple                 # (x, y, yaw) of robot1's spawn
    goals: tuple                 # Goal, in index order

    def __len__(self):
        return len(self.goals)


# ----------------------------------------------------------------------------- reading


def default_path(world='warehouse', count=50):
    """Return the installed goal set file <world>_<count>.yaml (share/<package>/config/...)."""
    share = get_package_share_directory('warehouse_system_tests')
    return os.path.join(share, 'config', 'goal_sets', f'{world}_{count}.yaml')


def load(path):
    """Read a goal set file and return the GoalSet; raise GoalSetError if it is not one."""
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except OSError as error:
        raise GoalSetError(f'{path}: {error.strerror}') from None
    except yaml.YAMLError as error:
        raise GoalSetError(f'{path}: not valid YAML: {error}') from None
    if not isinstance(data, dict) or data.get('schema_version') != SCHEMA_VERSION:
        raise GoalSetError(f'{path}: not a goal set (schema_version {SCHEMA_VERSION} expected); '
                           'write one with ros2 run warehouse_system_tests goal_set')
    if data.get('frame_id') != FRAME_ID:
        raise GoalSetError(f'{path}: frame_id must be {FRAME_ID}, not {data.get("frame_id")!r}')
    try:
        start = data['start']
        goals = tuple(Goal(int(item['index']), float(item['x']), float(item['y']),
                           float(item['yaw'])) for item in data['goals'])
        result = GoalSet(path, str(data['world']), int(data['seed']),
                         (float(start['x']), float(start['y']), float(start['yaw'])), goals)
    except (KeyError, TypeError, ValueError) as error:
        raise GoalSetError(f'{path}: malformed ({error!r})') from None
    if [goal.index for goal in goals] != list(range(1, len(goals) + 1)):
        raise GoalSetError(f'{path}: the goals must be numbered 1, 2, 3, ... in order')
    return result


# ----------------------------------------------------------------------------- one map


class FreeSpace:
    """
    One map as the rules see it: exact clearances, and the free space connected to the spawn.

    ``clearance(x, y)`` is the distance from the point to the nearest point of a cell that is
    not free (occupied or unknown). ``connected(x, y)`` says whether the point's cell belongs
    to the robot-centre free space connected to the spawn (rule 3).
    """

    def __init__(self, name, occupancy_map, keepouts, spawn, centre_clearance):
        self.name = name
        self.map = occupancy_map
        if occupancy_map.origin[2] != 0.0:
            raise ValueError(f'{name}: the map is turned by {occupancy_map.origin[2]} rad; the '
                             'goal rules expect a map frame without a turn (plan P2-D2)')
        resolution = occupancy_map.resolution
        not_free = occupancy_map.cells != occupancy.FREE
        # Every cell that is not free, by its centre: the nearest one to a point is then found
        # with a k-d tree, and the exact distance to its square follows (see clearance()).
        self._tree = cKDTree(np.vstack((occupancy_map.centres(occupancy.OCCUPIED),
                                        occupancy_map.centres(occupancy.UNKNOWN))))
        self._half = resolution / 2.0
        self._half_diagonal = math.hypot(self._half, self._half)
        # The robot-centre free space: like plan section 3.3's keepout_check.py, by the
        # distance between cell centres (as Nav2's inflation measures it), without the cores.
        free_distance = ndimage.distance_transform_edt(~not_free) * resolution
        rows, columns = np.indices(occupancy_map.cells.shape)
        xs = occupancy_map.origin[0] + (columns + 0.5) * resolution
        ys = occupancy_map.origin[1] + (rows + 0.5) * resolution
        zone_distance = np.full(xs.shape, np.inf)
        for zone in keepouts:
            zone_distance = np.minimum(zone_distance,
                                       zones.polygon_distance(xs, ys, zone.polygon))
        centre_free = (free_distance > centre_clearance) & (zone_distance > zones.PLANNING_CORE)
        # 4-connected (cells sharing an edge): the stricter choice.
        labels, self.regions = ndimage.label(centre_free)
        spawn_label = self._label_at(labels, spawn.x, spawn.y)
        if spawn_label == 0:
            raise ValueError(f'{name}: the spawn ({spawn.x:g}, {spawn.y:g}) is not in the free '
                             'space of the robot centre')
        self._spawn_region = labels == spawn_label
        self.spawn_area = float(np.count_nonzero(self._spawn_region)) * resolution ** 2

    def _label_at(self, labels, x, y):
        rows, columns = self.map.cell_index([(x, y)])
        if not self.map.contains(rows, columns)[0]:
            return 0
        return int(labels[rows[0], columns[0]])

    def clearance(self, x, y):
        """Return the exact distance from (x, y) to the nearest cell that is not free."""
        nearest, _ = self._tree.query((x, y))
        # The nearest square is among the centres no farther than the nearest centre plus half
        # a cell's diagonal: a square is never closer than its centre minus that.
        near = self._tree.data[self._tree.query_ball_point(
            (x, y), nearest + self._half_diagonal + 1e-9)]
        dx = np.maximum(np.abs(near[:, 0] - x) - self._half, 0.0)
        dy = np.maximum(np.abs(near[:, 1] - y) - self._half, 0.0)
        return float(np.min(np.hypot(dx, dy)))

    def connected(self, x, y):
        """Return True if (x, y) lies in the robot-centre free space connected to the spawn."""
        rows, columns = self.map.cell_index([(x, y)])
        return bool(self.map.contains(rows, columns)[0]
                    and self._spawn_region[rows[0], columns[0]])


# ----------------------------------------------------------------------------- the rules


@dataclass(frozen=True)
class Measured:
    """What the rules measured for one goal, for the report and the file's comments."""

    step: float                  # m from the goal before (or the spawn)
    slam_clearance: float        # m to the nearest occupied or unknown cell, SLAM map
    ideal_clearance: float       # the same in rasterize's ideal map
    keepout: float               # m to the nearest keepout zone's polygon
    keepout_name: str


def nearest_keepout(layout, x, y):
    """Return (distance, name) of the keepout zone nearest to (x, y)."""
    distance, zone = rules.nearest_keepout(layout, x, y)
    return distance, zone.name if zone is not None else '-'


def generate(layout, slam, ideal, count, seed, goal_circle):
    """
    Draw ``count`` goals by the rules (module docstring); return (goals, measured, rejected).

    ``slam`` and ``ideal`` are FreeSpace; ``rejected`` counts the candidates each rule turned
    down. Raises ValueError if MAX_CANDIDATES draws do not give enough goals.
    """
    rng = random.Random(seed)
    hall = layout.hall.box
    spawn = layout.spawn(SPAWN_ROBOT)
    previous = (spawn.x, spawn.y)
    goals, measured = [], []
    rejected = dict.fromkeys([name for name, _ in CHECKS], 0)
    draws = 0
    while len(goals) < count:
        if draws >= MAX_CANDIDATES:
            raise ValueError(f'only {len(goals)} of {count} goals after {MAX_CANDIDATES} '
                             'candidates: the rules leave too little floor')
        draws += 1
        # Two draws per candidate, always: the stream does not depend on which rule failed.
        x = round(hall.x0 + (hall.x1 - hall.x0) * rng.random(), XY_DECIMALS)
        y = round(hall.y0 + (hall.y1 - hall.y0) * rng.random(), XY_DECIMALS)
        failed = _first_failed_rule(layout, slam, ideal, previous, x, y, goal_circle)
        if failed:
            rejected[failed] += 1
            continue
        yaw = round(-math.pi + 2.0 * math.pi * rng.random(), YAW_DECIMALS)
        keepout, keepout_name = nearest_keepout(layout, x, y)
        goals.append(Goal(len(goals) + 1, x, y, yaw))
        measured.append(Measured(math.dist(previous, (x, y)), slam.clearance(x, y),
                                 ideal.clearance(x, y), keepout, keepout_name))
        previous = (x, y)
    return tuple(goals), tuple(measured), rejected


def _first_failed_rule(layout, slam, ideal, previous, x, y, goal_circle):
    """Return the name of the first rule the point (x, y) breaks (CHECKS order), or None."""
    if math.dist(previous, (x, y)) < MIN_STEP:
        return 'step'
    if nearest_keepout(layout, x, y)[0] < KEEPOUT_DISTANCE:
        return 'keepout'
    if slam.clearance(x, y) <= goal_circle:
        return 'slam_circle'
    if ideal.clearance(x, y) <= goal_circle:
        return 'ideal_circle'
    if not slam.connected(x, y):
        return 'slam_connected'
    if not ideal.connected(x, y):
        return 'ideal_connected'
    return None


# ----------------------------------------------------------------------------- the file


def number(value, decimals):
    """Return a number with fixed decimals, never '-0.000' (the file must not depend on it)."""
    text = f'{value:.{decimals}f}'
    return text[1:] if text.startswith('-') and float(text) == 0.0 else text


def render(layout, goals, measured, seed, inputs, rule_numbers):
    """Return the goal set file's text (see the module docstring for the rules)."""
    spawn = layout.spawn(SPAWN_ROBOT)
    hall = layout.hall.box
    goal_circle, centre_clearance = rule_numbers
    count = len(goals)
    core, ramp = zones.PLANNING_CORE, zones.PLANNING_RAMP
    floor = (f'x {number(hall.x0, 1)} .. {number(hall.x1, 1)}, '
             f'y {number(hall.y0, 1)} .. {number(hall.y1, 1)}')
    header = [
        'The frozen goal set of the navigation benchmark (plan stage S3b, decision P3-D22):',
        f'{count} goal poses in the world "{layout.name}", {FRAME_ID} frame (x east, y north, '
        'yaw counter-',
        'clockwise from +x; metres, radians). nav_benchmark sends them in this order, and each',
        'stage drives its own part: S3b goals 1-10, S4 and S6b 1-20, S8b 21-40, and the',
        'acceptance runs (3a-runs, S9) all 50.',
        '',
        'Do not edit this file, and do not write it again to suit a run: runs compare only on',
        'the same goals. It was written, from the workspace root, by',
        f'  ros2 run warehouse_system_tests goal_set --count {count} --seed {seed}',
        'and the same inputs and seed always give the same bytes.',
        '',
        'Rules (every goal):',
        f'1. Its goal circle, {goal_circle:.3f} m round the centre (padded circumscribed '
        f'radius + {rules.GOAL_MARGIN:.2f} m),',
        "   holds only free cells, in the SLAM map and in rasterize's ideal map.",
        f'2. Its centre is at least {KEEPOUT_DISTANCE:.2f} m from every keepout zone of the '
        f'layout: the',
        f'   planning core ({core:.2f} m) and its cost ramp ({ramp:.2f} m) of plan section 3.3.',
        '3. Its centre lies in the free space of the robot centre that is connected to the',
        f'   spawn of {SPAWN_ROBOT} ({number(spawn.x, 1)}, {number(spawn.y, 1)}), in both maps: '
        f'cells more than {centre_clearance:.2f} m (the padded',
        '   inscribed radius) from every occupied or unknown cell, outside the '
        f'{core:.2f} m cores.',
        f'4. It is at least {MIN_STEP:.1f} m from the goal before it (goal 1: from the spawn).',
        '5. Its yaw is uniform over a full turn.',
        f'Candidates: x and y uniform over the hall floor ({floor}),',
        f"from Python's random.Random({seed}).random(), rounded to 1 mm (the yaw to 0.0001 rad)",
        'before the checks.',
        '',
        'Inputs (sha256 of the files under src/):',
    ]
    width = max(len(name) for name, _ in inputs)
    header += [f'  {name:<{width}}  {digest}' for name, digest in inputs]
    header += [
        'If one of them changes, a goal may break a rule: write a copy with --out and compare.',
        'Keep this file unless the owner decides to replace it.',
    ]
    lines = [f'# {line}' if line else '#' for line in header]
    lines += [
        '',
        f'schema_version: {SCHEMA_VERSION}',
        f'world: {layout.name}',
        f'frame_id: {FRAME_ID}',
        f'seed: {seed}',
        f'start: {{x: {number(spawn.x, XY_DECIMALS)}, y: {number(spawn.y, XY_DECIMALS)}, '
        f'yaw: {number(spawn.yaw, YAW_DECIMALS)}}}   # the spawn of {SPAWN_ROBOT}',
        '# Per goal: step = m from the goal before; clear = m from the centre to the nearest',
        '# occupied or unknown cell, SLAM map / ideal map; keepout = m to the nearest keepout '
        'zone.',
        'goals:',
    ]
    for goal, values in zip(goals, measured):
        lines.append(
            f'  - {{index: {goal.index}, x: {number(goal.x, XY_DECIMALS)}, '
            f'y: {number(goal.y, XY_DECIMALS)}, yaw: {number(goal.yaw, YAW_DECIMALS)}}}'
            f'   # step {number(values.step, 2)}, clear {number(values.slam_clearance, 3)} / '
            f'{number(values.ideal_clearance, 3)}, keepout {number(values.keepout, 2)} '
            f'({values.keepout_name})')
    return '\n'.join(lines) + '\n'


def report(goals, measured, rejected, slam, ideal, goal_circle):
    """Return the text printed after writing: one line per goal, then the tightest values."""
    draws = len(goals) + sum(rejected.values())
    lines = [f'{len(goals)} goals from {draws} candidates. Turned down:']
    lines += [f'  {count:6d}  {title}' for (name, title), count
              in zip(CHECKS, rejected.values())]
    for space in (slam, ideal):
        lines.append(f'{space.name}: the free space of the robot centre connected to the spawn '
                     f'covers {space.spawn_area:.1f} m^2 ({space.regions} separate regions in '
                     'all)')
    lines += ['', 'Clearance: m from the goal centre to the nearest occupied or unknown cell '
                  f'(the goal circle needs more than {goal_circle:.3f} m).',
              '    #        x        y      yaw   step   SLAM map  ideal map   keepout zone',
              ]
    for goal, values in zip(goals, measured):
        lines.append(f'  {goal.index:3d} {goal.x:8.3f} {goal.y:8.3f} {goal.yaw:8.4f} '
                     f'{values.step:6.2f} {values.slam_clearance:10.3f} '
                     f'{values.ideal_clearance:10.3f} {values.keepout:9.2f} '
                     f'{values.keepout_name}')
    lines.append(
        f'tightest: SLAM map {min(m.slam_clearance for m in measured):.3f} m, ideal map '
        f'{min(m.ideal_clearance for m in measured):.3f} m, keepout zone '
        f'{min(m.keepout for m in measured):.2f} m, step {min(m.step for m in measured):.2f} m')
    return '\n'.join(lines)


def sha256_of(path):
    """Return the sha256 of a file's bytes, as hex."""
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def _write(path, text):
    """Write the text unless the file holds it already; return 'wrote' or 'unchanged'."""
    data = text.encode('utf-8')     # bytes: no newline translation on any system
    try:
        with open(path, 'rb') as stream:
            if stream.read() == data:
                return 'unchanged'
    except FileNotFoundError:
        pass
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as stream:
        stream.write(data)
    return 'wrote'


def main(argv=None):
    """Run the goal_set command; return its exit code."""
    parser = argparse.ArgumentParser(
        prog='goal_set',
        description='Write the frozen goal set of the navigation benchmark from the layout and '
                    'the committed SLAM map, read from the source tree (run it from the '
                    'workspace root, or pass --src).')
    parser.add_argument('--count', type=int, default=50, help='goals to draw (default 50)')
    parser.add_argument('--seed', type=int, default=1, help='random seed (default 1)')
    parser.add_argument('--src', default='src',
                        help='the workspace source directory (default: ./src)')
    parser.add_argument('--out', metavar='FILE',
                        help='write here instead of src/warehouse_system_tests/config/goal_sets/'
                             '<world>_<count>.yaml')
    args = parser.parse_args(argv)
    if args.count < 1:
        parser.error('--count must be at least 1')

    files = {
        'layout': os.path.join('warehouse_layout', 'config', 'layout.yaml'),
        'robot': os.path.join('warehouse_description', 'config', 'robot.yaml'),
    }
    try:
        robot = robot_config.load(os.path.join(args.src, files['robot']))
        layout = model.load(os.path.join(args.src, files['layout']), robot=robot)
        files['map'] = os.path.join('warehouse_navigation', 'maps', f'{layout.name}.yaml')
        map_path = os.path.join(args.src, files['map'])
        slam_map = occupancy.load(map_path)
        with open(map_path, encoding='utf-8') as stream:
            image = yaml.safe_load(stream)['image']
        files['map image'] = os.path.join(os.path.dirname(files['map']), image)
        inputs = tuple((path, sha256_of(os.path.join(args.src, path)))
                       for path in files.values())
    except OSError as error:
        print(f'error: {error}\nRun this from the workspace root (the folder with src/ in it), '
              'or pass --src.', file=sys.stderr)
        return 2
    except ValueError as error:   # a LayoutError, a MapError or a bad robot.yaml
        print(f'error: {error}', file=sys.stderr)
        return 2

    goal_circle = rules.goal_clearance(robot)          # 0.556 m (rule 1)
    centre_clearance = robot.r_inscribed_padded        # 0.28 m (rule 3)
    spawn = layout.spawn(SPAWN_ROBOT)
    try:
        slam = FreeSpace('SLAM map', slam_map, layout.keepouts, spawn, centre_clearance)
        ideal = FreeSpace('ideal map', rasterize.rasterize(layout), layout.keepouts, spawn,
                          centre_clearance)
        goals, measured, rejected = generate(layout, slam, ideal, args.count, args.seed,
                                             goal_circle)
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return 2

    text = render(layout, goals, measured, args.seed, inputs, (goal_circle, centre_clearance))
    out = args.out or os.path.join(args.src, 'warehouse_system_tests', 'config', 'goal_sets',
                                   f'{layout.name}_{args.count}.yaml')
    try:
        done = _write(out, text)
    except OSError as error:
        print(f'error: cannot write {out}: {error.strerror}', file=sys.stderr)
        return 2
    print(report(goals, measured, rejected, slam, ideal, goal_circle))
    print(f'{done} {out} ({len(goals)} goals, seed {args.seed}, sha256 {sha256_of(out)})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
