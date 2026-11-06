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
Check a saved map against the layout: that it shows this warehouse, in the right place.

Usage::

    ros2 run warehouse_layout check_map MAP.yaml [--overlay OUT.png] [--layout F] [--robot F]

1. Read the map (map_server's YAML and image) and take the centre of every occupied cell.
2. Take the faces the lidar can see in the layout at its scan plane (rasterize.surfaces):
   points every centimetre, each with the way its face looks.
3. Fit the cells to the faces with ICP (below): the best fit is the move and turn that put
   the map on the layout.
4. After that fit, measure every cell's distance to the nearest face.
5. Print one [PASS] or [FAIL] line per acceptance criterion of plan F5, and draw the overlay.

Exit codes: 0 every criterion passes; 1 one fails, or the cells cannot be fitted at all; 2 the
map, the layout or robot.yaml cannot be read, or the overlay cannot be written.

**ICP (iterative closest point).** Fitting needs to know which face each cell belongs to, and
pairing cells with faces needs the fit: ICP alternates between the two. It pairs every cell
with the nearest face point (a KD-tree finds it among some 33,000 points without comparing
with each), solves for the small turn and shift that best move the cells onto their faces,
moves the map, and pairs again, until the map stops moving.

This is the point-to-plane form: a cell is pulled across its face, never along it. A cell on
a long wall has no true partner along the wall, so only the distance across the wall is an
error. For a cell p paired with the face point q whose face looks along n, the error is
e = n . (p - q). A small turn by a about the origin and a shift (tx, ty) move p by
a * (-py, px) + (tx, ty), which changes the error by a * n . (-py, px) + nx * tx + ny * ty:
linear in (a, tx, ty), so the best step over all pairs is a small least-squares problem.
On this warehouse it settles in about 5 iterations to within a millimetre; point-to-point ICP
(pull each cell towards the face point itself) took 10 to 30 and stopped 2 to 3 mm short.

Pairs farther apart than a cut-off take no part: those cells have no face near them (ghost
walls). The cut-off starts at 1.0 m, so the fit can catch a map that is off by more, and
shrinks to 0.25 m, the ghost-wall distance.

The fit turns the map about the map frame's origin, robot1's spawn, where the mapping run
starts. Its (x, y, yaw) is the transform from the map frame to the world frame: the identity
for a map made as plan P2-D2 describes.
"""

import argparse
from dataclasses import dataclass
import math
import os
import sys
import textwrap

import matplotlib
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.transforms import Affine2D
import numpy as np
from scipy.spatial import cKDTree
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import occupancy
from warehouse_layout import rasterize
from warehouse_layout.rules import RuleResult

# Acceptance criteria (plan F5). A cell is 0.05 m.
MAX_OFFSET = 0.05            # m: the map may sit at most one cell away from the world
MAX_ROTATION = 0.2           # deg: turning 0.2 deg moves a wall 30 m from the origin by 0.10 m
NEAR = 0.10                  # m: two cells; SLAM draws a wall one or two cells thick
MIN_NEAR_FRACTION = 0.95     # of the occupied cells, within NEAR of a face
GHOST = 0.25                 # m: a cell farther than this from every face is a ghost wall
MAX_GHOST_FRACTION = 0.01    # of the occupied cells
YAW_TOLERANCE = 1e-9         # rad: the map origin's yaw must be 0

# ICP
CUTOFFS = (1.0, 0.5, GHOST)  # m: pairs farther apart take no part; the last value then stays
MAX_ITERATIONS = 50
SETTLED = 1e-6               # m and rad: a step smaller than this ends the fit
MIN_PAIRS = 10               # fewer cells near a face cannot pin the map down

# Overlay colours, from the dataviz reference palette. The layout's faces wear the first
# categorical colour. Occupied cells are ink when they lie on a face; otherwise they wear the
# status colours "warning" and "critical", drawn larger and each with its own marker, so that
# colour is never the only cue. Free and unknown cells stay quiet, as in RViz.
FACE_COLOUR = '#2a78d6'
CELL_COLOUR = '#0b0b0b'
FAR_COLOUR = '#fab219'
GHOST_COLOUR = '#d03b3b'
FREE_COLOUR = '#fcfcfb'
UNKNOWN_COLOUR = '#e1e0d9'
INK = '#0b0b0b'
INK_LIGHT = '#52514e'
DPI = 120


class FitError(ValueError):
    """The map's occupied cells cannot be fitted to the layout (for example, too few near it)."""


@dataclass(frozen=True)
class Fit:
    """
    The best fit of a map to the layout: the transform from the map frame to the world frame.

    A map point p lies at R(yaw) p + (x, y) in the world; for a map that lines up with the
    world this is the identity. ``pairs`` cells (those within the last cut-off of a face)
    took part; ``settled`` is False if the fit was still moving after MAX_ITERATIONS.
    """

    x: float
    y: float
    yaw: float
    iterations: int
    pairs: int
    settled: bool

    @property
    def offset(self):
        """Return how far the fit moves the map frame's origin, in metres."""
        return math.hypot(self.x, self.y)

    def apply(self, points):
        """Return N x 2 map-frame points moved into the world frame."""
        return _turn(points, self.yaw) + (self.x, self.y)


@dataclass(frozen=True, eq=False)
class MapCheck:
    """
    The outcome of check().

    ``cells`` are the occupied cells' centres as the map file places them; ``distances`` are
    their distances to the nearest face after the fit; ``results`` holds one RuleResult per
    acceptance criterion.
    """

    cells: np.ndarray
    fit: Fit
    distances: np.ndarray
    results: tuple

    @property
    def passed(self):
        """Return True if every acceptance criterion passes."""
        return all(result.passed for result in self.results)


def _turn(points, yaw):
    """Return N x 2 points turned by ``yaw`` radians about (0, 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points, dtype=float) @ np.array([[c, s], [-s, c]])   # p @ R^T


# ----------------------------------------------------------------------------- the fit


def fit(cells, faces, tree=None):
    """
    Fit the cells (N x 2, map frame) to the faces (rasterize.Surfaces) with ICP; return a Fit.

    ``tree`` is a cKDTree of ``faces.points``, if the caller has one. Raises FitError if
    fewer than MIN_PAIRS cells lie near a face, or if the paired faces all run one way.
    """
    if len(cells) == 0:
        raise FitError('the map has no occupied cells')
    tree = tree if tree is not None else cKDTree(faces.points)
    yaw, shift = 0.0, np.zeros(2)
    settled = False
    for iteration in range(MAX_ITERATIONS):
        cutoff = CUTOFFS[min(iteration, len(CUTOFFS) - 1)]
        moved = _turn(cells, yaw) + shift
        # Pair every cell with its nearest face point; beyond the cut-off the tree reports an
        # infinite distance and no partner.
        distance, nearest = tree.query(moved, distance_upper_bound=cutoff)
        paired = np.isfinite(distance)
        pairs = int(paired.sum())
        if pairs < MIN_PAIRS:
            raise FitError(f'only {pairs} of {len(cells)} occupied cells lie within '
                           f'{cutoff:g} m of a face of the layout: is this a map of this '
                           'warehouse, in its map frame?')
        p = moved[paired]
        q = faces.points[nearest[paired]]
        n = faces.normals[nearest[paired]]
        error = np.einsum('ij,ij->i', n, p - q)        # n . (p - q), one per pair
        # How each pair's error changes with the turn (about the origin) and the shift.
        jacobian = np.column_stack((n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1], n[:, 0], n[:, 1]))
        step, _, rank, _ = np.linalg.lstsq(jacobian, -error, rcond=None)
        if rank < 3:
            raise FitError('every paired face runs the same way, so nothing fixes the map '
                           'along them')
        turn, dx, dy = step
        yaw += turn
        shift = _turn(shift, turn) + (dx, dy)   # the new turn applies to the old shift too
        if (iteration >= len(CUTOFFS) - 1
                and abs(turn) < SETTLED and math.hypot(dx, dy) < SETTLED):
            settled = True
            break
    return Fit(float(shift[0]), float(shift[1]), yaw, iteration + 1, pairs, settled)


def check(occupancy_map, faces):
    """Fit the map's occupied cells to the faces and judge the result; return a MapCheck."""
    tree = cKDTree(faces.points)
    cells = occupancy_map.centres(occupancy.OCCUPIED)
    best = fit(cells, faces, tree)
    distances, _ = tree.query(best.apply(cells))
    return MapCheck(cells, best, distances, _criteria(best, distances, occupancy_map.origin[2]))


def _criteria(best, distances, origin_yaw):
    """Return one RuleResult per acceptance criterion of plan F5."""
    near = float(np.mean(distances <= NEAR))
    ghosts = int(np.count_nonzero(distances > GHOST))
    ghost_fraction = ghosts / len(distances)
    p50, p95 = np.percentile(distances, (50, 95))
    turn = math.degrees(best.yaw)
    return (
        _criterion(
            'offset', f'best-fit offset <= {MAX_OFFSET:.2f} m', best.offset <= MAX_OFFSET,
            f'{best.offset:.3f} m (move the map x {_signed(best.x)}, y {_signed(best.y)} m)',
            'the map frame is not the world frame: make the map in a run that starts at '
            "robot1's spawn, with the EKF started there"),
        _criterion(
            'rotation', f'best-fit rotation <= {MAX_ROTATION:.1f} deg',
            abs(turn) <= MAX_ROTATION, f'{abs(turn):.3f} deg (turn the map {_signed(turn)} deg)',
            'the map is turned against the world: make the map in a run that starts at '
            "robot1's spawn facing +x, with the EKF started there"),
        _criterion(
            'near_faces', f'{MIN_NEAR_FRACTION:.0%} of occupied cells within {NEAR:.2f} m of a '
            'face', near >= MIN_NEAR_FRACTION,
            f'{near:.1%} (P50 {p50:.3f} m, P95 {p95:.3f} m, max {distances.max():.2f} m)',
            f'{1.0 - near:.1%} of the cells lie farther than {NEAR:.2f} m from every face: '
            'blurred or doubled walls (the amber squares and red crosses of the overlay)'),
        _criterion(
            'ghost_walls', f'at most {MAX_GHOST_FRACTION:.0%} of occupied cells farther than '
            f'{GHOST:.2f} m from every face (ghost walls)',
            ghost_fraction <= MAX_GHOST_FRACTION, f'{ghost_fraction:.2%} ({ghosts} cells)',
            'the map has walls the layout has not (the red crosses of the overlay): a false '
            'loop closure, or something in the world that is not in layout.yaml'),
        _criterion(
            'origin_yaw', 'map origin yaw 0', abs(origin_yaw) <= YAW_TOLERANCE,
            f'{origin_yaw:g} rad',
            "Nav2's AMCL and costmaps use only the origin's x and y, and the Phase 4 dashboard "
            'draws the map unturned: save the map from a map frame that is the world frame'),
    )


def _signed(value, decimals=3):
    """Return the value with its sign, and no minus sign on a value that rounds to 0."""
    return f'{round(value, decimals) + 0.0:+.{decimals}f}'    # + 0.0 turns -0.0 into 0.0


def _criterion(rule, title, passed, summary, advice):
    """Return a RuleResult: the same [PASS] / [FAIL] lines as the layout's fit rules."""
    return RuleResult(rule, title, () if passed else (advice,), summary)


# ----------------------------------------------------------------------------- the overlay


def draw_overlay(path, occupancy_map, faces, map_check, title):
    """
    Draw the map with the layout's faces on top and save it as a PNG.

    Cells are drawn where the map file places them. Their colour comes from their distance
    to the nearest face after the best fit, so the colours show the map's shape (blurred or
    ghost walls), and the gap between the cells and the faces shows its offset. With
    ``map_check`` None (no fit), every occupied cell is drawn in ink.
    """
    with matplotlib.rc_context({'font.size': 8}):
        figure = Figure(figsize=(16.0, 9.0), dpi=DPI)
        grid = figure.add_gridspec(1, 2, width_ratios=[3.3, 1.0], wspace=0.03, left=0.045,
                                   right=0.99, top=0.93, bottom=0.07)
        _draw_map(figure.add_subplot(grid[0, 0]), occupancy_map, faces, map_check)
        _draw_key(figure.add_subplot(grid[0, 1]), map_check)
        figure.suptitle(title, x=0.045, y=0.975, ha='left', fontsize=11, color=INK)
        figure.savefig(path, dpi=DPI, facecolor='white')


def _draw_map(ax, occupancy_map, faces, map_check):
    cells = occupancy_map.cells
    colours = np.empty(cells.shape + (3,))
    colours[...] = to_rgb(UNKNOWN_COLOUR)
    colours[cells == occupancy.FREE] = to_rgb(FREE_COLOUR)
    colours[cells == occupancy.OCCUPIED] = to_rgb(CELL_COLOUR)
    x0, y0, yaw = occupancy_map.origin
    size = occupancy_map.resolution
    extent = (x0, x0 + occupancy_map.width * size, y0, y0 + occupancy_map.height * size)
    image = ax.imshow(colours, origin='lower', extent=extent, interpolation='nearest', zorder=1)
    if yaw:   # turn the picture about its lower-left corner, as the YAML says
        image.set_transform(Affine2D().rotate_around(x0, y0, yaw) + ax.transData)

    ax.add_collection(LineCollection(faces.segments, colors=FACE_COLOUR, linewidths=0.9,
                                     zorder=3))
    if map_check is not None:
        far = (map_check.distances > NEAR) & (map_check.distances <= GHOST)
        ghost = map_check.distances > GHOST
        for chosen, marker, colour, area, z in ((far, 's', FAR_COLOUR, 10, 4),
                                                (ghost, 'X', GHOST_COLOUR, 26, 5)):
            ax.scatter(map_check.cells[chosen, 0], map_check.cells[chosen, 1], s=area,
                       marker=marker, facecolors=colour, edgecolors=INK, linewidths=0.4,
                       zorder=z)
    ax.plot([0.0], [0.0], marker='+', markersize=10, markeredgewidth=1.4, color=INK, zorder=6)
    ax.annotate('origin (robot1 spawn)', (0.0, 0.0), xytext=(6, 6), textcoords='offset points',
                fontsize=7.5, color=INK, zorder=6)

    # Show the whole map and the whole layout, even where one reaches past the other.
    ends = np.array(faces.segments).reshape(-1, 2)
    corners = _turn([(0.0, 0.0), (extent[1] - x0, 0.0), (0.0, extent[3] - y0),
                     (extent[1] - x0, extent[3] - y0)], yaw) + (x0, y0)
    everything = np.vstack((ends, corners))
    low, high = everything.min(axis=0) - 0.3, everything.max(axis=0) + 0.3
    ax.set_xlim(low[0], high[0])
    ax.set_ylim(low[1], high[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]  (east)', color=INK_LIGHT)
    ax.set_ylabel('y [m]  (north)', color=INK_LIGHT)
    ax.tick_params(colors=INK_LIGHT, length=2)
    for spine in ax.spines.values():
        spine.set_color('#c3c2b7')


def _draw_key(ax, map_check):
    ax.axis('off')
    if map_check is None:
        counts = None
    else:
        d = map_check.distances
        counts = (np.count_nonzero(d <= NEAR), np.count_nonzero((d > NEAR) & (d <= GHOST)),
                  np.count_nonzero(d > GHOST))

    def label(text, index):
        return text if counts is None else f'{text}: {counts[index]}'

    handles = [
        Line2D([], [], color=FACE_COLOUR, linewidth=1.4,
               label='layout face the lidar sees at its scan plane'),
        Patch(facecolor=CELL_COLOUR, label=label(f'map cell within {NEAR:.2f} m of a face', 0)),
        Line2D([], [], linestyle='none', marker='s', markersize=5, markerfacecolor=FAR_COLOUR,
               markeredgecolor=INK, markeredgewidth=0.5,
               label=label(f'map cell {NEAR:.2f}-{GHOST:.2f} m from a face', 1)),
        Line2D([], [], linestyle='none', marker='X', markersize=7,
               markerfacecolor=GHOST_COLOUR, markeredgecolor=INK, markeredgewidth=0.5,
               label=label(f'ghost: map cell over {GHOST:.2f} m from every face', 2)),
        Patch(facecolor=FREE_COLOUR, edgecolor='#c3c2b7', linewidth=0.6, label='free cell'),
        Patch(facecolor=UNKNOWN_COLOUR, label='unknown cell'),
    ]
    ax.legend(handles=handles, loc='upper left', frameon=False, fontsize=8, handlelength=2.0,
              borderaxespad=0.0, labelcolor=INK)
    note = ('Cells are drawn where the map file puts them. Their colour is their distance to the '
            'nearest face after the best fit: the colours show blurred and ghost walls, and a '
            'gap between the cells and the faces shows an offset or a turn.')
    ax.text(0.0, 0.80, textwrap.fill(note, 60), transform=ax.transAxes, va='top', fontsize=7.5,
            color=INK_LIGHT)
    if map_check is None:
        return
    lines = []
    for result in map_check.results:
        for line in str(result).split('\n'):
            lines.append(textwrap.fill(line, 56, subsequent_indent='       '))
    ax.text(0.0, 0.69, '\n'.join(lines), transform=ax.transAxes, va='top', fontsize=7.0,
            family='monospace', color=INK, linespacing=1.35)


# ----------------------------------------------------------------------------- the command


def main(argv=None):
    """Run the check_map command; return its exit code."""
    parser = argparse.ArgumentParser(
        description='Check a saved map against the layout: fit its occupied cells to the faces '
                    'the lidar sees (ICP), then report the offset, the rotation and the '
                    'distances against the acceptance criteria of plan F5.')
    parser.add_argument('map', help='the map YAML file (map_server format)')
    parser.add_argument('--overlay', help='PNG file to draw the map and the layout in')
    parser.add_argument('--layout', help='layout.yaml to check against (default: the installed '
                                         'one)')
    parser.add_argument('--robot', help='robot.yaml to use (default: the installed one)')
    args = parser.parse_args(argv)

    try:
        occupancy_map = occupancy.load(args.map)
        robot = robot_config.load(args.robot)
        layout = model.load(args.layout, robot=robot)
    except (OSError, ValueError) as error:   # a MapError, a LayoutError or a missing file
        print(f'error: {error}', file=sys.stderr)
        return 2
    faces = rasterize.surfaces(layout)
    x0, y0, yaw = occupancy_map.origin
    occupied = int(np.count_nonzero(occupancy_map.cells == occupancy.OCCUPIED))
    print(f'map {args.map}: {occupancy_map.width} x {occupancy_map.height} cells of '
          f'{occupancy_map.resolution:g} m, origin ({x0:g}, {y0:g}, {yaw:g}), '
          f'{occupied} occupied')
    print(f'layout "{layout.name}": {len(faces.points)} points on the faces the lidar sees at '
          f'{robot.scan_plane_height:.2f} m')

    title = f'check_map: {os.path.basename(args.map)} against the layout "{layout.name}"'
    try:
        map_check = check(occupancy_map, faces)
    except FitError as error:
        print(f'[FAIL] no fit: {error}')
        if args.overlay and not _write_overlay(args.overlay, occupancy_map, faces, None,
                                               f'{title}: no fit'):
            return 2
        return 1

    best = map_check.fit
    print(f'best fit after {best.iterations} ICP iterations'
          f'{"" if best.settled else " (still moving: the fit did not settle)"}: '
          f'{best.pairs} of {len(map_check.cells)} cells within {GHOST:.2f} m of a face took '
          'part')
    for result in map_check.results:
        print(result)
    failed = [result.title for result in map_check.results if not result.passed]
    if args.overlay:
        verdict = ('all criteria pass' if not failed
                   else '1 criterion fails' if len(failed) == 1
                   else f'{len(failed)} criteria fail')
        if not _write_overlay(args.overlay, occupancy_map, faces, map_check,
                              f'{title}: {verdict}'):
            return 2
    print(f'{len(map_check.results) - len(failed)} of {len(map_check.results)} map checks pass')
    return 0 if not failed else 1


def _write_overlay(path, occupancy_map, faces, map_check, title):
    """Draw the overlay; print what happened and return True if it was written."""
    try:
        draw_overlay(path, occupancy_map, faces, map_check, title)
    except OSError as error:
        print(f'error: cannot write the overlay {path}: {error.strerror}', file=sys.stderr)
        return False
    print(f'wrote {path}')
    return True


if __name__ == '__main__':
    sys.exit(main())
