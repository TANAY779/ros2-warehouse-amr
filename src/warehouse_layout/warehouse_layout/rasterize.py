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
What the lidar sees at its scan plane, from the layout: the faces it can hit, and a map.

The lidar scans one plane, ``scan_plane_height`` above the floor (0.34 m on the Rev B robot,
from robot.yaml). It sees a solid if the plane cuts it, that is if the solid reaches from below
the plane to above it: walls, pilasters, shelf plinths, guards, the counter, dock pedestals,
fence kick plates and fixtures. The dock contact bars (0.12 m) and base plates (0.01 m) lie
below the plane, and the lidar never sees them (plan section 3.4). test_world.py checks that
the generated world agrees: every solid has a visual that fills its footprint across the
plane, and no visual reaches out of a solid below 1.20 m.

Two views of the same thing:

- ``surfaces()``: the faces of those solids that border the free floor, as points every
  centimetre with the way each face looks, and as line segments. check_map fits a saved map
  to these points.
- ``rasterize()``: a map, as an ideal SLAM run would make it. The cells behind every face the
  lidar can see are occupied, the rest of the hall's floor is free, and what the lidar cannot
  see (the inside of a solid, behind the walls) is unknown. It is the fallback map of plan
  risk R6, and in Phase 3 the grid on which the costmap masks are drawn.

The command::

    ros2 run warehouse_layout rasterize --out /tmp/layout_map [--layout FILE] [--robot FILE]

writes /tmp/layout_map.yaml and /tmp/layout_map.pgm in map_server's format. The map frame is
the world frame (plan P2-D2), so the map needs no shift or turn: its origin's yaw is 0, and its
lower-left corner sits on a multiple of the resolution, so cell edges fall on the layout's
round numbers (the hall's inner faces at x -1.5 and 28.5, y -2.0 and 18.0).

Exit codes: 0 written; 2 the layout or robot.yaml cannot be read, or the map cannot be written.
"""

import argparse
from dataclasses import dataclass
import math
import sys

import numpy as np
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import occupancy

RESOLUTION = 0.05    # m per cell: Nav2's default, and the SLAM map's (plan P2-D15)
MARGIN = 0.5         # m of unknown round the outer walls, as a SLAM map has
FACE_STEP = 0.01     # m between the points sampled along a face
PROBE = 0.005        # m: a face is visible if the point this far in front of it is free floor


def seen_solids(layout):
    """Return the solids the scan plane cuts: they reach from below it to above it."""
    height = layout.robot.scan_plane_height
    return tuple(solid for solid in layout.solids
                 if solid.box.z0 <= height <= solid.solid_to)


def floor_boxes(layout):
    """Return the hall's floor as boxes: inside the walls, and in each recess."""
    return (layout.hall.box,) + tuple(recess.floor for recess in layout.hall.recesses)


def _box_array(boxes):
    """Return boxes as an N x 4 array of (x0, x1, y0, y1): the plan view only."""
    return np.array([(box.x0, box.x1, box.y0, box.y1) for box in boxes]).reshape(-1, 4)


def _inside(points, boxes):
    """Return which of the N x 2 points lie strictly inside any of the boxes (N x 4)."""
    x, y = points[:, 0:1], points[:, 1:2]      # columns, so that they broadcast against boxes
    return ((boxes[:, 0] < x) & (x < boxes[:, 1])
            & (boxes[:, 2] < y) & (y < boxes[:, 3])).any(axis=1)


# ----------------------------------------------------------------------------- faces


@dataclass(frozen=True, eq=False)
class Surfaces:
    """
    The faces the lidar can see, in the map frame.

    ``points`` (N x 2) lie on the faces, at most FACE_STEP apart; ``normals`` (N x 2) are unit
    vectors, the way each point's face looks (into the free floor). ``segments`` are the
    visible stretches of the faces as ((x0, y0), (x1, y1)), for drawing.
    """

    points: np.ndarray
    normals: np.ndarray
    segments: tuple


def surfaces(layout):
    """
    Return the faces of the seen solids that border the free floor, as Surfaces.

    Every face of every seen solid is sampled every centimetre. A sample is kept if the point
    PROBE in front of it is free floor: on the hall's floor and inside no seen solid. That
    drops the outer faces of the walls (outside the hall), a pilaster's back against its wall,
    and the faces where two solids touch, like a shelf and its end guard.
    """
    solids = _box_array(solid.box for solid in seen_solids(layout))
    floor = _box_array(floor_boxes(layout))
    points, normals, segments = [], [], []
    for x0, x1, y0, y1 in solids:
        # The four faces, each from corner to corner, with the way it looks.
        for start, end, normal in (((x0, y0), (x1, y0), (0.0, -1.0)),    # south face
                                   ((x1, y0), (x1, y1), (1.0, 0.0)),     # east face
                                   ((x1, y1), (x0, y1), (0.0, 1.0)),     # north face
                                   ((x0, y1), (x0, y0), (-1.0, 0.0))):   # west face
            count = max(2, math.ceil(math.dist(start, end) / FACE_STEP) + 1)
            t = np.linspace(0.0, 1.0, count)[:, np.newaxis]
            samples = (1.0 - t) * np.array(start) + t * np.array(end)
            probes = samples + PROBE * np.array(normal)
            visible = _inside(probes, floor) & ~_inside(probes, solids)
            points.append(samples[visible])
            normals.append(np.tile(normal, (int(visible.sum()), 1)))
            segments.extend(_runs(samples, visible))
    return Surfaces(np.vstack(points), np.vstack(normals), tuple(segments))


def _runs(samples, visible):
    """Return the (first, last) sample of every run of consecutive visible samples."""
    runs, first = [], None
    for i, seen in enumerate(visible):
        if seen and first is None:
            first = i
        if first is not None and (not seen or i == len(visible) - 1):
            last = i if seen else i - 1
            if last > first:    # a single point has no length to draw
                runs.append((tuple(map(float, samples[first])), tuple(map(float, samples[last]))))
            first = None
    return runs


# ----------------------------------------------------------------------------- the map


def rasterize(layout, resolution=RESOLUTION):
    """
    Return the map an ideal SLAM run would make of the layout, as an OccupancyMap.

    A cell is solid if a seen solid covers any part of it: a lidar beam that hits the face
    ends in that cell. A cell is floor if its centre lies on the hall's floor. Free cells are
    floor cells that are not solid. A solid cell next to a free one (sharing an edge) holds a
    face the lidar can see: it is occupied. Every other cell is unknown, as in a SLAM map: the
    inside of a solid and the space behind the walls cannot be seen. (One exception: where two
    thin solids meet off the cell grid, as the fence's 0.05 m pieces do, the corner cell can be
    partly covered on every side and stay unknown although a corner of it is open.)
    """
    boxes = [solid.box for solid in seen_solids(layout)]
    # The grid covers every seen solid plus MARGIN, its corners on multiples of the resolution.
    x0 = _snap(min(box.x0 for box in boxes) - MARGIN, resolution, math.floor)
    y0 = _snap(min(box.y0 for box in boxes) - MARGIN, resolution, math.floor)
    x1 = _snap(max(box.x1 for box in boxes) + MARGIN, resolution, math.ceil)
    y1 = _snap(max(box.y1 for box in boxes) + MARGIN, resolution, math.ceil)
    width, height = round((x1 - x0) / resolution), round((y1 - y0) / resolution)

    solid = np.zeros((height, width), dtype=bool)     # [row, column], row 0 in the south
    for box in boxes:
        first_column, end_column = _cell_span(box.x0, box.x1, x0, resolution)
        first_row, end_row = _cell_span(box.y0, box.y1, y0, resolution)
        solid[first_row:end_row, first_column:end_column] = True

    xs = x0 + (np.arange(width) + 0.5) * resolution      # cell centres
    ys = y0 + (np.arange(height) + 0.5) * resolution
    centres = np.column_stack((np.tile(xs, height), np.repeat(ys, width)))  # row by row
    floor = _inside(centres, _box_array(floor_boxes(layout))).reshape(height, width)
    free = floor & ~solid

    beside_free = np.zeros_like(free)
    beside_free[1:, :] |= free[:-1, :]     # the cell to the south is free
    beside_free[:-1, :] |= free[1:, :]     # the cell to the north
    beside_free[:, 1:] |= free[:, :-1]     # the cell to the west
    beside_free[:, :-1] |= free[:, 1:]     # the cell to the east

    cells = np.full((height, width), occupancy.UNKNOWN, dtype=np.int8)
    cells[free] = occupancy.FREE
    cells[solid & beside_free] = occupancy.OCCUPIED
    return occupancy.OccupancyMap(cells, resolution, (x0, y0, 0.0))


def _snap(value, resolution, rounding):
    """Return the value moved down (math.floor) or up (math.ceil) to a multiple of resolution."""
    # round() first: in floating point -2.2 / 0.05 is -44.00000000000001, which floor()
    # would turn into -45.
    return round(rounding(round(value / resolution, 6)) * resolution, 9)


def _cell_span(low, high, origin, resolution):
    """Return the first cell, and the one after the last, that the span low-high overlaps."""
    return (math.floor(round((low - origin) / resolution, 6)),
            math.ceil(round((high - origin) / resolution, 6)))


# ----------------------------------------------------------------------------- the command


def main(argv=None):
    """Run the rasterize command; return its exit code."""
    parser = argparse.ArgumentParser(
        description='Draw what the lidar sees at its scan plane into a map (map_server '
                    'format, map frame = world frame), from layout.yaml.')
    parser.add_argument('--out', required=True,
                        help='the map to write: NAME or NAME.yaml writes NAME.yaml and NAME.pgm')
    parser.add_argument('--layout', help='layout.yaml to draw (default: the installed one)')
    parser.add_argument('--robot', help='robot.yaml to use (default: the installed one)')
    parser.add_argument('--resolution', type=float, default=RESOLUTION,
                        help=f'metres per cell (default: {RESOLUTION:g})')
    args = parser.parse_args(argv)
    if args.resolution <= 0.0:
        parser.error('--resolution must be positive')

    try:
        robot = robot_config.load(args.robot)
        layout = model.load(args.layout, robot=robot)
    except (OSError, ValueError) as error:   # a missing file, or a LayoutError
        print(f'error: {error}', file=sys.stderr)
        return 2
    occupancy_map = rasterize(layout, args.resolution)
    yaml_path = args.out.removesuffix('.yaml') + '.yaml'    # like map_saver_cli's -f NAME
    height = robot.scan_plane_height
    comment = (
        f'What the lidar sees at its scan plane, {height:.2f} m above the floor, drawn from the',
        f'layout "{layout.name}" by `ros2 run warehouse_layout rasterize`. Occupied: the faces it',
        'can see. Free: the rest of the hall floor. Unknown: what it cannot see (inside the',
        'solids, behind the walls). The map frame is the world frame, so the yaw is 0.',
    )
    try:
        image = occupancy.save(occupancy_map, yaml_path, comment)
    except OSError as error:
        print(f'error: cannot write {yaml_path}: {error.strerror}', file=sys.stderr)
        return 2

    cells = occupancy_map.cells
    x0, y0, _ = occupancy_map.origin
    seen = {solid.kind for solid in seen_solids(layout)}
    hidden = sorted({solid.kind for solid in layout.solids} - seen)
    print(f'wrote {yaml_path} and {image}: {occupancy_map.width} x {occupancy_map.height} cells '
          f'of {args.resolution:g} m, origin ({x0:g}, {y0:g}, 0)')
    print(f'{np.sum(cells == occupancy.OCCUPIED)} occupied cells (the faces the lidar sees at '
          f'{height:.2f} m), {np.sum(cells == occupancy.FREE)} free, '
          f'{np.sum(cells == occupancy.UNKNOWN)} unknown')
    if hidden:
        print(f'below the scan plane, so not drawn: {", ".join(hidden)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
