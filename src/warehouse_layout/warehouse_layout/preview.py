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
Draw a to-scale top view of the warehouse layout and print the fit-rule results.

Usage::

    ros2 run warehouse_layout preview --out /tmp/layout.png [--layout FILE] [--robot FILE]

The picture shows the hall in the map frame (origin = robot1's spawn) with every object in
its style colour, the zones, the floor paint, the goal poses, the corridor widths and the
survey route, plus three side panels: how the robot fits a pick aisle, the heights against
the lidar's scan plane, and a legend with the counts. The command exits with 1 if a fit rule
fails; it writes the picture anyway, so you can see what is wrong.

It uses matplotlib's object-oriented API (a Figure, no pyplot), so it needs no display and
changes no global matplotlib state.
"""

import argparse
import math
import sys

import matplotlib
from matplotlib.colors import to_rgba
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Patch
from matplotlib.patches import Polygon
from matplotlib.patches import Rectangle
import matplotlib.patheffects
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import rules

# The drawing's own colours: text, grid and annotations. The warehouse's colours come from
# the layout's style block; the robot's from its body in warehouse_description.
INK = '#1b1b1b'
INK_LIGHT = '#52514e'
EDGE = '#2d2f33'
ROUTE = '#4b6a88'
ROBOT_BODY = '#f2c200'      # safety yellow
ROBOT_TRIM = '#333333'      # charcoal
AXIS_X = '#d62728'          # REP-103 colours: x red, y green
AXIS_Y = '#2ca02c'
SCAN_PLANE = '#d62728'
DPI = 110


def main(argv=None):
    """Run the preview command; return its exit code."""
    parser = argparse.ArgumentParser(
        description='Draw the warehouse layout to scale and check the fit rules.')
    parser.add_argument('--out', required=True, help='PNG file to write')
    parser.add_argument('--layout', help='layout.yaml to draw (default: the installed one)')
    parser.add_argument('--robot', help='robot.yaml to use (default: the installed one)')
    args = parser.parse_args(argv)

    try:
        robot = robot_config.load(args.robot)
        layout = model.load(args.layout, robot=robot)
    except (OSError, ValueError) as error:   # a missing file, or a LayoutError
        print(f'error: {error}', file=sys.stderr)
        return 2
    results = rules.check(layout)
    for result in results:
        print(result)
    draw(layout, results, args.out)
    failed = sum(not result.passed for result in results)
    print(f'wrote {args.out}; {len(results) - failed} of {len(results)} fit rules pass')
    return 1 if failed else 0


def draw(layout, results, path):
    """Draw the layout with its side panels and save it as a PNG."""
    with matplotlib.rc_context({'hatch.linewidth': 0.6, 'font.size': 7.5}):
        figure = Figure(figsize=(19.0, 11.6), dpi=DPI)
        grid = figure.add_gridspec(3, 2, width_ratios=[3.55, 1.0], height_ratios=[1.0, 1.0, 1.25],
                                   wspace=0.04, hspace=0.30, left=0.035, right=0.985, top=0.94,
                                   bottom=0.05)
        _draw_plan(figure.add_subplot(grid[:, 0]), layout)
        _draw_aisle_fit(figure.add_subplot(grid[0, 1]), layout)
        _draw_heights(figure.add_subplot(grid[1, 1]), layout)
        _draw_legend(figure.add_subplot(grid[2, 1]), layout, results)
        figure.savefig(path, dpi=DPI, facecolor='white')


# ----------------------------------------------------------------------------- helpers


def _box(ax, box, face, edge='none', width=0.0, z=2.0, alpha=1.0, hatch=None, style='-'):
    ax.add_patch(Rectangle((box.x0, box.y0), box.x1 - box.x0, box.y1 - box.y0, facecolor=face,
                           edgecolor=edge, linewidth=width, zorder=z, alpha=alpha, hatch=hatch,
                           linestyle=style))


def _label(ax, x, y, text, size=7.5, colour=INK, z=20, boxed=True, **kwargs):
    bbox = ({'boxstyle': 'round,pad=0.18', 'facecolor': 'white', 'edgecolor': 'none',
             'alpha': 0.85} if boxed else None)
    ax.text(x, y, text, fontsize=size, color=colour, zorder=z, ha=kwargs.pop('ha', 'center'),
            va=kwargs.pop('va', 'center'), bbox=bbox, **kwargs)


def _arrow(ax, start, end, colour, z=13, width=1.0, scale=8, style='-|>'):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=scale,
                                 color=colour, lw=width, zorder=z, shrinkA=0, shrinkB=0))


def _pose(ax, pose, colour, size=0.35, z=13):
    end = (pose.x + size * math.cos(pose.yaw), pose.y + size * math.sin(pose.yaw))
    _arrow(ax, (pose.x, pose.y), end, colour, z=z)
    ax.add_patch(Circle((pose.x, pose.y), 0.06, color=colour, zorder=z))


def _dimension(ax, start, end, text, offset=(0.0, 0.0), size=7.0, where=0.5):
    """Draw a double arrow from start to end, labelled at ``where`` (0 = start, 1 = end)."""
    _arrow(ax, start, end, INK_LIGHT, z=18, width=0.8, scale=7, style='<|-|>')
    x = start[0] + where * (end[0] - start[0]) + offset[0]
    y = start[1] + where * (end[1] - start[1]) + offset[1]
    _label(ax, x, y, text, size=size, z=19)


def _robot(ax, robot, x, y, yaw, padded=False, face=ROBOT_BODY, edge=ROBOT_TRIM, z=12,
           width=0.9):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(x + c * px - s * py, y + s * px + c * py)
               for px, py in robot.footprint_polygon(padded=padded)]
    ax.add_patch(Polygon(corners, closed=True, facecolor=face, edgecolor=edge, linewidth=width,
                         zorder=z, linestyle='--' if padded else '-'))
    if not padded:
        tip = robot.half_length * 1.05
        _arrow(ax, (x, y), (x + c * tip, y + s * tip), edge, z=z + 1, width=0.9, scale=7)


def _title(name):
    return name.replace('_', ' ')


def _across_wall(box, wall):
    """Return a box's depth out from its wall: y on the south and north walls, else x."""
    return box.y1 - box.y0 if wall in ('south', 'north') else box.x1 - box.x0


def _top_middle(polygon, inset=0.4):
    xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
    return (min(xs) + max(xs)) / 2, max(ys) - inset


# ----------------------------------------------------------------------------- the plan view


def _draw_plan(ax, layout):
    hall, style = layout.hall, layout.style
    ax.set_aspect('equal')
    ax.set_xlim(hall.x0 - 1.9, hall.x1 + 1.3)
    ax.set_ylim(hall.y0 - 1.75, hall.y1 + 1.0)
    ax.set_xticks(range(math.ceil(hall.x0), math.floor(hall.x1) + 1, 2))
    ax.set_yticks(range(math.ceil(hall.y0), math.floor(hall.y1) + 1, 2))
    ax.tick_params(labelsize=7.5, colors=INK_LIGHT, length=2)
    for spine in ax.spines.values():
        spine.set_color('#c9c8c3')
    ax.grid(True, color='#bdbcb6', lw=0.3, alpha=0.6, zorder=1)
    ax.set_xlabel('x [m]  (east)', fontsize=8, color=INK_LIGHT)
    ax.set_ylabel('y [m]  (north)', fontsize=8, color=INK_LIGHT)

    # The floor, lightened so that labels and paint stay readable.
    _box(ax, hall.box, style.floor, z=0.5, alpha=0.55)
    for recess in hall.recesses:
        _box(ax, recess.floor, style.floor, z=0.5, alpha=0.55)

    _draw_zones(ax, layout)
    _draw_paint(ax, layout)
    _draw_solids(ax, layout)
    _draw_rows(ax, layout)
    _draw_poses(ax, layout)
    _draw_corridors(ax, layout)
    _draw_labels(ax, layout)
    _draw_route(ax, layout)

    # Scale bar, under the south-west corner.
    x0, y = hall.x0 + 0.5, hall.y0 - 1.25
    ax.plot([x0, x0 + 5.0], [y, y], color=INK, lw=2.2, solid_capstyle='butt', zorder=20,
            clip_on=False)
    for x in (x0, x0 + 5.0):
        ax.plot([x, x], [y - 0.15, y + 0.15], color=INK, lw=1.0, zorder=20, clip_on=False)
    ax.text(x0 + 2.5, y - 0.3, '5 m', fontsize=7.5, color=INK, ha='center', va='top')

    blocks = sorted({row.block for row in layout.rows})
    ax.set_title(
        f'Warehouse layout "{layout.name}" (to scale): {hall.x1 - hall.x0:.0f} x '
        f'{hall.y1 - hall.y0:.0f} m hall, {len({r.row for r in layout.rows})} shelf rows, '
        f'{len({a.aisle for a in layout.aisles})} pick aisles x {len(blocks)} blocks, '
        f'{len(layout.docks)} docks, {len(layout.spawns)} spawns.  '
        'Map frame = Gazebo world frame; origin = robot1 spawn',
        fontsize=10.5, color=INK, loc='left')


def _draw_zones(ax, layout):
    style = layout.style
    for zone in layout.speed_zones:
        if zone.generated:   # the pick aisles: a light tint only
            ax.add_patch(Polygon(zone.polygon, closed=True, facecolor=style.speed_paint,
                                 alpha=0.13, edgecolor='none', zorder=1.2))
        else:
            ax.add_patch(Polygon(zone.polygon, closed=True, facecolor=style.speed_paint,
                                 alpha=0.16, edgecolor=style.speed_paint, lw=1.2, zorder=1.3,
                                 hatch='//'))
    for zone in layout.keepouts:
        ax.add_patch(Polygon(zone.polygon, closed=True, facecolor=style.keepout_paint,
                             alpha=0.10, edgecolor='none', zorder=1.4))
        ax.add_patch(Polygon(zone.polygon, closed=True, facecolor='none',
                             edgecolor=style.keepout_paint, lw=1.0, zorder=1.5,
                             hatch=None if zone.generated else 'xx'))


def _draw_paint(ax, layout):
    style = layout.style
    # White paint on a light floor gets a thin grey outline, as on the real floor's texture.
    cased = [matplotlib.patheffects.withStroke(linewidth=2.6, foreground='#a9a8a2')]
    for marking in layout.markings:
        box = marking.box
        if marking.kind in ('zebra_stripe', 'aisle_stripe'):
            _box(ax, box, style.lane_paint, edge='#b8b7b1', width=0.4, z=1.7)
        elif marking.kind == 'parking_bay':
            ax.add_patch(Rectangle((box.x0, box.y0), box.x1 - box.x0, box.y1 - box.y0,
                                   facecolor='none', edgecolor=style.lane_paint, lw=1.4,
                                   zorder=1.7, path_effects=cased))
        elif marking.kind == 'dock_bay':
            _box(ax, box, 'none', edge=style.dock_paint, width=1.0, z=1.7, style='--')


def _draw_solids(ax, layout):
    style = layout.style
    colours = {
        'wall': style.wall_band, 'pilaster': style.pilaster, 'shelf': style.rack_board,
        'guard': style.rack_guard, 'station': style.station, 'dock_pedestal': style.dock,
        'contact_bar': style.contact, 'dock_plate': style.dock_plate,
    }
    for solid in layout.solids:
        if solid.kind == 'fence':
            continue   # drawn below as a line with posts
        colour = colours.get(solid.kind) or style.fixture(solid.kind)
        if solid.kind == 'wall':
            _box(ax, solid.box, colour, z=5)
        elif solid.kind == 'dock_plate':
            _box(ax, solid.box, colour, z=3.5)
        else:
            _box(ax, solid.box, colour, edge=EDGE, width=0.5, z=4.5 if solid.kind == 'contact_bar'
                 else 4)
    for door in layout.hall.doors:
        (x0, y0), (x1, y1) = door.line
        ax.plot([x0, x1], [y0, y1], color=style.door, lw=4.0, solid_capstyle='butt', zorder=5.5)
    for fence in layout.fences:
        for (x0, y0), (x1, y1) in fence.pieces:
            ax.plot([x0, x1], [y0, y1], color=style.fence, lw=2.4, solid_capstyle='butt',
                    zorder=6)
            posts = max(1, round(math.hypot(x1 - x0, y1 - y0) / 1.5))
            for i in range(posts + 1):
                x = x0 + (x1 - x0) * i / posts
                y = y0 + (y1 - y0) * i / posts
                ax.add_patch(Rectangle((x - 0.06, y - 0.06), 0.12, 0.12, facecolor=style.fence,
                                       zorder=6.1))


def _draw_rows(ax, layout):
    style, shelving = layout.style, layout.shelving
    for row in layout.rows:
        box = row.box
        # Uprights, to scale, at both ends of every bay.
        for i in range(len(row.bays) + 1):
            y = box.y0 + i * shelving.bay_width
            ax.add_patch(Rectangle((box.x0, y), box.x1 - box.x0, shelving.upright,
                                   facecolor=style.rack_upright, edgecolor='none', zorder=4.2))
        _label(ax, (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, row.row, size=8, z=21,
               fontweight='bold')
    # Bay numbers (from the longest row of each block) beside the west-most row.
    for block in sorted({row.block for row in layout.rows}):
        segments = [r for r in layout.rows if r.block == block]
        west = min(segments, key=lambda r: r.box.x0)
        longest = max(segments, key=lambda r: len(r.bays))
        for bay in longest.bays:
            ax.text(west.box.x0 - 0.12, bay.centre_y, f'{bay.number:02d}', fontsize=6,
                    color=INK_LIGHT, ha='right', va='center', zorder=21)
    first = min(layout.rows, key=lambda r: (r.box.y0, r.box.x0))
    ax.text(first.box.x0 - 0.12, first.box.y0 - 0.45, 'bay', fontsize=6, color=INK_LIGHT,
            ha='right', va='center')


def _draw_poses(ax, layout):
    style, robot = layout.style, layout.robot
    for pick in layout.picks:
        ax.add_patch(Circle((pick.x, pick.y), 0.07, color=INK, zorder=9))
    for station in layout.stations:
        _pose(ax, station.drop, style.station)
        _label(ax, station.drop.x + 0.5, station.drop.y, 'drop', size=6.5, colour=INK_LIGHT,
               ha='left')
    for dock in layout.docks:
        _pose(ax, dock.docked, style.dock, size=0.28)
        _pose(ax, dock.staging, style.dock, size=0.28)
        _label(ax, dock.staging.x, dock.staging.y + 0.42, dock.name, size=6.5)
    for spawn in layout.spawns:
        pose = spawn.pose
        _robot(ax, robot, pose.x, pose.y, pose.yaw)
        text = spawn.robot + (' = origin' if spawn.robot == 'robot1' else '')
        _label(ax, pose.x - robot.half_length - 0.6, pose.y, text, size=6.5, ha='right')
    # The origin's axes (REP-103 colours).
    _arrow(ax, (0.0, 0.0), (1.1, 0.0), AXIS_X, z=14, width=1.4, scale=9)
    _arrow(ax, (0.0, 0.0), (0.0, 1.1), AXIS_Y, z=14, width=1.4, scale=9)


def _draw_corridors(ax, layout):
    # Aisle names just outside the aisle mouths (off the pick poses): at the north end in the
    # northern-most block, at the south end elsewhere.
    north_block = max(layout.aisles, key=lambda a: a.box.y1).block if layout.aisles else None
    for aisle in layout.aisles:
        y = aisle.box.y1 + 0.35 if aisle.block == north_block else aisle.box.y0 - 0.35
        _label(ax, aisle.centre_x, y, f'aisle {aisle.aisle}', size=7, z=22)
    # The width across the first and last aisle of each block.
    for block in sorted({a.block for a in layout.aisles}):
        aisles = sorted((a for a in layout.aisles if a.block == block), key=lambda a: a.box.x0)
        for aisle in {aisles[0].name: aisles[0], aisles[-1].name: aisles[-1]}.values():
            y = aisle.box.y0 + 0.75 * (aisle.box.y1 - aisle.box.y0)
            _dimension(ax, (aisle.box.x0, y), (aisle.box.x1, y), f'{aisle.width:.2f} m',
                       offset=(0.0, 0.25))
    for corridor in layout.corridors:
        width, low, high, _, _ = rules.corridor_extent(layout, corridor)
        # Labels on north-south lines sit 70 % of the way up, clear of the zone labels.
        _dimension(ax, low, high, f'{_title(corridor.name)}\n{width:.2f} m', size=6.5,
                   where=0.7 if corridor.axis == 'y' else 0.5)


def _draw_labels(ax, layout):
    for station in layout.stations:
        x, y, _ = station.counter.centre
        _label(ax, x, y, _title(station.name), size=7, colour='white', boxed=False,
               fontweight='bold', z=22)
    for solid in layout.solids:
        if solid.group != solid.name or solid.kind in ('wall', 'pilaster', 'station'):
            continue   # parts of rows, docks and fences, and the hall itself
        box = solid.box
        x, y, _ = box.centre
        if min(box.x1 - box.x0, box.y1 - box.y0) < 1.0:   # small: label it above
            y = box.y1 + 0.45
        _label(ax, x, y, _title(solid.name), size=6.5)
    for zone in layout.keepouts:
        if not zone.generated:
            x, y = _top_middle(zone.polygon)
            _label(ax, x, y, f'KEEPOUT: {_title(zone.name)}', size=7)
    for zone in layout.speed_zones:
        if not zone.generated:
            x, y = _top_middle(zone.polygon, inset=0.55)
            _label(ax, x, y, f'{_title(zone.name)}\n{zone.limit:g} m/s', size=7)
    hall = layout.hall
    for door in hall.doors:
        (x0, y0), (x1, y1) = door.line
        x, y = (x0 + x1) / 2, (y0 + y1) / 2
        outside = {'south': (0, -0.55), 'north': (0, 0.55), 'west': (-0.6, 0),
                   'east': (0.6, 0)}[door.wall]
        _label(ax, x + outside[0], y + outside[1], _title(door.name), size=6,
               colour=INK_LIGHT, boxed=False)
    for recess in hall.recesses:
        x, y, _ = recess.floor.centre
        vertical = recess.wall in ('west', 'east')
        inward = {'south': (0, 0.45), 'north': (0, -0.45), 'west': (0.45, 0),
                  'east': (-0.45, 0)}[recess.wall]
        _label(ax, x + inward[0], y + inward[1], _title(recess.name), size=6.5,
               rotation=90 if vertical else 0)
    for fence in layout.fences:
        for (x0, y0), (x1, y1) in fence.gates:
            _label(ax, (x0 + x1) / 2, (y0 + y1) / 2 - 0.45, 'gate (open)', size=6.5)


def _draw_route(ax, layout):
    if not layout.routes:
        return
    route = next((r for r in layout.routes if r.name == 'survey'), layout.routes[0])
    xs = [p[0] for p in route.waypoints]
    ys = [p[1] for p in route.waypoints]
    ax.plot(xs, ys, color=ROUTE, lw=0.9, ls=(0, (4, 3)), alpha=0.85, zorder=8)
    # Arrows on the first few long segments show the direction of travel.
    long_segments = [(a, b) for a, b in zip(route.waypoints, route.waypoints[1:])
                     if math.dist(a, b) > 3.0][:4]
    for a, b in long_segments:
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        _arrow(ax, (mx - 0.3 * dx / n, my - 0.3 * dy / n), (mx + 0.3 * dx / n, my + 0.3 * dy / n),
               ROUTE, z=8, width=0.9)
    a, b = long_segments[0] if long_segments else route.waypoints[:2]
    _label(ax, (a[0] + b[0]) / 2 + 2.0, (a[1] + b[1]) / 2 - 0.45, f'{route.name} route',
           size=6.5, colour=ROUTE)


# ----------------------------------------------------------------------------- side panels


def _draw_aisle_fit(ax, layout):
    """Draw a robot stopped on a pick aisle's centreline, to scale, with the clearances."""
    robot, style = layout.robot, layout.style
    width = layout.aisles[0].width if layout.aisles else rules.PICK_AISLE_WIDTH
    depth = layout.shelving.depth
    half = width / 2
    inflation = rules.INFLATION_RADIUS
    clearance = rules.goal_clearance(robot)
    ax.set_aspect('equal')
    ax.set_title('Pick aisle fit (plan view, to scale)', fontsize=9, color=INK, loc='left')
    for x in (-half - depth, half):
        ax.add_patch(Rectangle((x, -0.9), depth, 1.8, facecolor=style.rack_board,
                               edgecolor=EDGE, lw=0.6))
    for x in (-half, half - inflation):
        ax.add_patch(Rectangle((x, -0.9), inflation, 1.8, facecolor=style.keepout_paint,
                               alpha=0.10, edgecolor='none'))
    _robot(ax, robot, 0.0, 0.0, math.pi / 2, padded=True, face='none', width=0.8)
    _robot(ax, robot, 0.0, 0.0, math.pi / 2)
    ax.add_patch(Circle((0, 0), robot.r_circumscribed_padded, fill=False, ec=INK_LIGHT, lw=0.8,
                        ls='--'))
    ax.add_patch(Circle((0, 0), clearance, fill=False, ec=INK, lw=0.9, ls=':'))
    ax.plot([0, 0], [-0.9, 0.9], color=ROUTE, lw=0.8, ls=(0, (4, 3)))
    _dimension(ax, (-half, -1.05), (half, -1.05), f'{width:.2f} m between faces',
               offset=(0, -0.13))
    for x in (-half + inflation / 2, half - inflation / 2):
        ax.text(x, 0.98, f'inflation {inflation:.2f} m', fontsize=6.5, color='#a12d2d',
                ha='center')
    ax.text(0, -1.42,
            f'robot {robot.length:.2f} x {robot.width:.2f} m; padded circle r '
            f'{robot.r_circumscribed_padded:.3f} m (dashed),\n'
            f'goal clearance r {clearance:.3f} m (dotted); centreline {half:.2f} m from each\n'
            f'face, outside the inflation. A stopped robot leaves {half - robot.half_width:.2f} m '
            f'each side;\nturning in place, its padded circle leaves '
            f'{half - robot.r_circumscribed_padded:.2f} m',
            fontsize=6.6, color=INK_LIGHT, ha='center', va='top')
    ax.set_xlim(-half - depth - 0.15, half + depth + 0.15)
    ax.set_ylim(-1.95, 1.15)
    ax.axis('off')


def _draw_heights(ax, layout):
    """Elevations, to scale: a shelf, the robot, a dock and the fence against the scan plane."""
    robot, style, shelving = layout.robot, layout.style, layout.shelving
    scan, needed = robot.scan_plane_height, rules.visible_height(robot)
    tote = layout.inventory.tote
    ax.set_title(f'Heights vs the {scan:.2f} m scan plane (elevation)', fontsize=9, color=INK,
                 loc='left')

    # A shelf seen from the side: plinth, boards, top, upright; totes where they will stand.
    depth = shelving.depth
    ax.add_patch(Rectangle((0.0, 0.0), depth, shelving.plinth_height, facecolor=style.rack_plinth))
    for top in shelving.board_heights + (shelving.top_height,):
        ax.add_patch(Rectangle((0.0, top - shelving.board_thickness), depth,
                               shelving.board_thickness, facecolor=style.rack_board))
    ax.add_patch(Rectangle((depth, 0.0), 0.03, shelving.top_height, facecolor=style.rack_upright))
    shown = layout.style.show_totes
    for level in shelving.level_heights:
        ax.add_patch(Rectangle((depth - tote.setback - tote.length, level), tote.length,
                               tote.height, facecolor=style.tote if shown else 'none',
                               edgecolor=INK_LIGHT, lw=0.5, ls='-' if shown else '--'))
    ax.text(depth / 2, shelving.top_height + 0.08, 'shelf', fontsize=6.5, color=INK, ha='center')
    ax.text(-0.04, shelving.plinth_height / 2, f'plinth\n{shelving.plinth_height:.2f}',
            fontsize=6, color=INK_LIGHT, ha='right', va='center')
    ax.text(-0.04, 0.85, 'totes' + ('' if shown else '\n(Phase 7)'), fontsize=6,
            color=INK_LIGHT, ha='right', va='center')

    # The robot seen from the side: body, lidar puck on the deck.
    x = 0.95
    ax.add_patch(Rectangle((x, robot.clearance), robot.length,
                           robot.deck_height - robot.clearance, facecolor=ROBOT_BODY,
                           edgecolor=ROBOT_TRIM, lw=0.6))
    lidar_x = x + robot.half_length + robot.lidar.x
    ax.add_patch(Rectangle((lidar_x - robot.lidar.housing_radius, robot.deck_height),
                           2 * robot.lidar.housing_radius, robot.lidar.housing_height,
                           facecolor='#222222'))
    ax.text(x + robot.half_length, robot.deck_height + 0.12, 'robot', fontsize=6.5, color=INK,
            ha='center')

    # A dock from the side: pedestal and contact bar.
    if layout.docks:
        dock = layout.docks[0]
        pedestal_depth = _across_wall(dock.pedestal, dock.wall)
        x = 2.05
        ax.add_patch(Rectangle((x, 0.0), pedestal_depth, dock.pedestal.z1, facecolor=style.dock))
        setback = math.dist(dock.tag, dock.contact)
        bar_depth = _across_wall(dock.contact_bar, dock.wall)
        ax.add_patch(Rectangle((x - setback, 0.0), bar_depth, dock.contact_bar.z1,
                               facecolor=style.contact))
        ax.text(x + pedestal_depth / 2, dock.pedestal.z1 + 0.06, 'dock', fontsize=6.5,
                color=INK, ha='center')
        ax.text(x - setback + bar_depth / 2, dock.contact_bar.z1 + 0.05,
                f'bar\n{dock.contact_bar.z1:.2f}', fontsize=5.5, color=INK_LIGHT, ha='center',
                va='bottom')

    # A fence from the side: kick plate and mesh.
    if layout.fences:
        fence = layout.fences[0]
        x = 2.75
        ax.add_patch(Rectangle((x, 0.0), 0.05, fence.kick_plate_height, facecolor=style.fence))
        ax.add_patch(Rectangle((x + 0.01, fence.kick_plate_height), 0.03,
                               fence.height - fence.kick_plate_height, facecolor=style.fence,
                               alpha=0.35))
        ax.text(x + 0.02, fence.height + 0.06, 'fence', fontsize=6.5, color=INK, ha='center')

    ax.axhline(scan, color=SCAN_PLANE, lw=1.0, ls='--')
    ax.axhspan(scan, needed, color=SCAN_PLANE, alpha=0.08)
    ax.text(3.0, scan + 0.03, f'scan plane {scan:.2f} m', fontsize=6.5, color='#a12d2d',
            va='bottom', ha='left')
    ax.axhline(0, color=INK, lw=0.8)
    ax.set_xlim(-0.8, 3.95)
    ax.set_ylim(-0.05, max(shelving.top_height, 1.8) + 0.2)
    ax.set_aspect('equal')
    ax.tick_params(labelsize=6.5, colors=INK_LIGHT, length=2)
    ax.set_xticks([])
    ticks = sorted({0.0, needed, 1.0, shelving.top_height})
    ax.set_yticks(ticks)
    ax.set_yticklabels([f'{t:.2f}'.rstrip('0').rstrip('.') for t in ticks])
    for side in ('top', 'right', 'bottom'):
        ax.spines[side].set_visible(False)
    ax.text(-0.78, -0.03, f'what the lidar must see is solid from the floor to >= {needed:.2f} m',
            fontsize=6.3, color=INK_LIGHT, va='top')


def _draw_legend(ax, layout, results):
    style, inventory, shelving = layout.style, layout.inventory, layout.shelving
    ax.axis('off')
    kinds = sorted({s.kind for s in layout.solids} & set(dict(style.fixtures)))
    handles = [
        Patch(facecolor=style.rack_board, edgecolor=EDGE,
              label='shelf rows (top board), steel uprights and end guards'),
        Patch(facecolor=style.station, edgecolor=EDGE, label='packing station (drop pose arrow)'),
        Patch(facecolor=style.dock, edgecolor=EDGE,
              label='charging dock + contact bar (docked, staging arrows)'),
    ] + [
        Patch(facecolor=style.fixture(kind), edgecolor=EDGE, label=_title(kind))
        for kind in kinds
    ] + [
        Patch(facecolor=style.wall_band,
              label=f'walls and pilasters ({layout.hall.wall_height:g} m)'),
        Line2D([], [], color=style.fence, lw=2.4, label='fence with a solid kick plate'),
        Patch(facecolor=to_rgba(style.keepout_paint, 0.10), edgecolor=style.keepout_paint,
              hatch='xx', label='keepout zone (Phase 3 mask)'),
        Patch(facecolor=to_rgba(style.speed_paint, 0.16), edgecolor=style.speed_paint,
              hatch='//', label='speed zone (limit in the label)'),
        Patch(facecolor=to_rgba(style.speed_paint, 0.13), edgecolor='none',
              label='pick aisle speed zone'),
        Patch(facecolor=ROBOT_BODY, edgecolor=ROBOT_TRIM, label='robot at a spawn (to scale)'),
        Line2D([], [], marker='o', color=INK, lw=0, markersize=4,
               label='pick pose (aisle centreline, one per bay)'),
        Line2D([], [], color=ROUTE, lw=0.9, ls=(0, (4, 3)), label='survey route'),
    ]
    ax.legend(handles=handles, loc='upper left', fontsize=7, frameon=False, handlelength=2.2,
              borderaxespad=0.0, labelcolor=INK)
    explicit_keepouts = sum(not z.generated for z in layout.keepouts)
    explicit_speed = sum(not z.generated for z in layout.speed_zones)
    example = layout.slots[len(layout.slots) // 2].name if layout.slots else '-'
    passed = sum(result.passed for result in results)
    failed = [result.rule for result in results if not result.passed]
    ax.text(0.0, 0.005,
            f'{len({r.row for r in layout.rows})} rows, '
            f'{sum(len(r.bays) for r in layout.rows)} bays, {len(layout.slots)} slots '
            f'({len(shelving.level_heights)} levels x {shelving.positions_per_level} positions)\n'
            f'{inventory.tote_count} totes at fill {inventory.fill_ratio:g}'
            f'{" (shown)" if style.show_totes else " (data only: bare shelves until Phase 7)"}\n'
            f'slot ID: <row><bay>-<level>-<position>, for example {example}\n'
            f'{len(layout.picks)} pick poses; keepouts: {explicit_keepouts} + '
            f'{len(layout.keepouts) - explicit_keepouts} dock bars; speed zones: '
            f'{explicit_speed} + {len(layout.speed_zones) - explicit_speed} aisle segments\n'
            f'fit rules: {passed} of {len(results)} pass'
            + (f' (FAIL: {", ".join(failed)})' if failed else ''),
            fontsize=7, color=INK if not failed else '#a12d2d', transform=ax.transAxes,
            va='bottom')


if __name__ == '__main__':
    sys.exit(main())
