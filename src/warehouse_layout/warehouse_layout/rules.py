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
The layout fit rules (plan section 3.6): what a layout must satisfy for this robot.

Each rule is a function that takes a model.Layout and returns a RuleResult; ``check()`` runs
them all. The robot's numbers come from robot.yaml through warehouse_description.robot_config
and are never copied here. With the Rev B robot (0.64 x 0.48 m, padding 0.04 m, scan plane
0.34 m) the derived limits are:

- goal clearance: padded circumscribed radius + 0.10 m = 0.456 + 0.10 = 0.556 m
- visible height: scan plane + 0.06 m = 0.34 + 0.06 = 0.40 m
- route clearance: half the robot's width + 0.16 m = 0.24 + 0.16 = 0.40 m

The design constants below come from the project plan (section 2.6) and the Phase 2 plan;
each rule's docstring gives its derivation. test/test_rules.py runs every rule on layout.yaml
and on copies of it with one deliberate mistake each.
"""

from collections import defaultdict
from dataclasses import dataclass
import math

from warehouse_layout import geometry
from warehouse_layout.geometry import EPS
from warehouse_layout.model import SLOT_ID_PATTERN

PICK_AISLE_WIDTH = 1.60          # m between the two shelf faces (P2-D3)
INFLATION_RADIUS = 0.70          # m: Nav2's inflation radius (Phase 3)
GOAL_MARGIN = 0.10               # m of free space round a robot turning in place at a goal
VISIBLE_MARGIN = 0.06            # m above the scan plane that what the lidar sees must reach
OVERHANG_LIMIT = 1.20            # m: the Rev E mast top (1.10 m) plus 0.10 m
MIN_CORRIDOR_WIDTH = 2.40        # m clear for the main aisle, cross aisles and corridors
TOTE_TAG_IDS = range(20, 250)    # the ArUco IDs reserved for totes (D23): 230 of them
DOCK_TAG_SETBACK = 0.25          # m from the contact face back to the pedestal face (tag)
DOCK_STAGING_DISTANCE = 0.70     # m from the docked pose out to the staging pose
MAX_CONTACT_BAR_HEIGHT = 0.15    # m: a contact bar stays far below the scan plane
ROUTE_MARGIN = 0.16              # m beyond the robot's half width along route segments
LOW_KINDS = ('contact_bar', 'dock_plate')   # below the scan plane on purpose (plan 3.4)


@dataclass(frozen=True)
class RuleResult:
    """
    The outcome of one fit rule on one layout.

    ``violations`` has one line per offending object, starting with the object's name.
    ``summary`` says what was measured, for example the tightest value.
    """

    rule: str
    title: str
    violations: tuple
    summary: str

    @property
    def passed(self):
        """Return True if nothing breaks the rule."""
        return not self.violations

    def __str__(self):
        lines = [f'[{"PASS" if self.passed else "FAIL"}] {self.title}: {self.summary}']
        lines += [f'    - {violation}' for violation in self.violations]
        return '\n'.join(lines)


# ----------------------------------------------------------------------------- derived limits


def goal_clearance(robot):
    """Return the free radius every goal pose needs (padded circumscribed radius + margin)."""
    return robot.r_circumscribed_padded + GOAL_MARGIN


def visible_height(robot):
    """Return how high from the floor an object must be solid for the lidar to see it."""
    return robot.scan_plane_height + VISIBLE_MARGIN


def route_clearance(robot):
    """Return how far a route segment must stay from every object."""
    return robot.half_width + ROUTE_MARGIN


# ----------------------------------------------------------------------------- measurements


def nearest_solid(layout, x, y):
    """Return (distance, solid) for the solid nearest to (x, y) in plan view."""
    return min(((s.box.distance_to(x, y), s) for s in layout.solids), key=lambda d: d[0])


def nearest_keepout(layout, x, y):
    """Return (distance, zone) for the keepout zone nearest to (x, y); (inf, None) if none."""
    return min(((geometry.distance_to_polygon(x, y, z.polygon), z) for z in layout.keepouts),
               key=lambda d: d[0], default=(math.inf, None))


def corridor_extent(layout, corridor):
    """
    Measure a corridor's clear width.

    From ``corridor.at``, a ray runs both ways along ``corridor.axis`` to the first solid
    or keepout zone. Returns (width, end_low, end_high, obstacle_low, obstacle_high): the
    width, the two end points and the names of what each ray hit.
    """
    x, y = corridor.at
    obstacles = ([(s.name, s.box.corners()) for s in layout.solids]
                 + [(z.name, z.polygon) for z in layout.keepouts])
    ends = []
    for sign in (-1.0, 1.0):
        direction = (sign, 0.0) if corridor.axis == 'x' else (0.0, sign)
        distance, hit = math.inf, None
        for name, polygon in obstacles:
            d = geometry.ray_distance((x, y), direction, polygon)
            if d < distance:
                distance, hit = d, name
        if math.isinf(distance):
            distance, hit = 0.0, 'nothing (the ray leaves the hall)'
        ends.append(((x + direction[0] * distance, y + direction[1] * distance), distance, hit))
    (low, d_low, hit_low), (high, d_high, hit_high) = ends
    return d_low + d_high, low, high, hit_low, hit_high


# ----------------------------------------------------------------------------- the rules


def check_pick_aisle_width(layout):
    """
    Check that pick aisles are exactly 1.60 m between the shelf faces (P2-D3).

    Lower bounds, from the robot: a robot on the centreline turns in place inside its padded
    circle and needs the goal margin on both sides, 2 x (0.456 + 0.10) = 1.11 m; and the
    centreline (0.80 m from each face) stays outside Nav2's 0.70 m inflation. Upper bound, from
    Phase 7: a 0.12 m tote tag must be >= 40 px in the side cameras anywhere within 0.15 m of
    the centreline, which holds up to about 1.62 m (derivation in the package README).
    """
    robot = layout.robot
    violations = []
    for aisle in layout.aisles:
        if abs(aisle.width - PICK_AISLE_WIDTH) > EPS:
            west, east = aisle.rows
            violations.append(f'{aisle.name}: {aisle.width:.3f} m between the faces of rows '
                              f'{west} and {east}; must be {PICK_AISLE_WIDTH:.2f} m')
    turning = 2 * goal_clearance(robot)
    if PICK_AISLE_WIDTH < turning - EPS:
        violations.append(f'robot: turning in place needs {turning:.3f} m, more than the '
                          f'{PICK_AISLE_WIDTH:.2f} m aisle (rethink the rule, plan 2.6)')
    if PICK_AISLE_WIDTH / 2 <= INFLATION_RADIUS:
        violations.append(f'robot: the centreline is inside the {INFLATION_RADIUS:.2f} m '
                          'inflation (rethink the rule, plan 2.6)')
    widths = sorted({round(a.width, 3) for a in layout.aisles})
    return RuleResult(
        'pick_aisle_width', f'pick aisles exactly {PICK_AISLE_WIDTH:.2f} m between the faces',
        tuple(violations),
        f'{len(layout.aisles)} aisle segments, widths {widths} m; turning in place needs '
        f'{turning:.2f} m')


def check_corridor_width(layout):
    """
    Check that the main and cross aisles and other corridors are >= 2.40 m clear (plan 2.6).

    Each corridor in layout.yaml is measured along its axis to the first solid or keepout
    zone on either side: keepout zones are not floor a robot may use.
    """
    violations, widths = [], []
    for corridor in layout.corridors:
        width, _, _, hit_low, hit_high = corridor_extent(layout, corridor)
        widths.append(f'{corridor.name} {width:.2f}')
        if width < MIN_CORRIDOR_WIDTH - EPS:
            violations.append(f'{corridor.name}: {width:.2f} m clear between {hit_low} and '
                              f'{hit_high}; needs {MIN_CORRIDOR_WIDTH:.2f} m')
    return RuleResult(
        'corridor_width', f'corridors at least {MIN_CORRIDOR_WIDTH:.2f} m clear',
        tuple(violations), ', '.join(widths) + ' m' if widths else 'no corridors listed')


def check_goal_clearance(layout):
    """
    Check that every goal pose has a free circle and no keepout zone nearby (plan 2.6).

    A robot stopping at a goal (spawn, drop, dock staging, pick) may turn in place, sweeping
    its padded circle: the nearest object must be at least padded R_circ + 0.10 m away (0.556
    m), and no keepout zone may reach into the padded circle itself (0.456 m).
    """
    robot = layout.robot
    clearance = goal_clearance(robot)
    radius = robot.r_circumscribed_padded
    violations = []
    tightest = (math.inf, None, None)
    for pose in layout.goals():
        distance, solid = nearest_solid(layout, pose.x, pose.y)
        tightest = min(tightest, (distance, pose.name, solid.name), key=lambda t: t[0])
        if distance < clearance - EPS:
            violations.append(f'{pose.name}: {distance:.3f} m from {solid.name}; needs '
                              f'{clearance:.3f} m')
        distance, zone = nearest_keepout(layout, pose.x, pose.y)
        if distance < radius - EPS:
            where = 'inside' if distance == 0.0 else f'{distance:.3f} m from'
            violations.append(f'{pose.name}: {where} keepout {zone.name}; needs {radius:.3f} m')
    return RuleResult(
        'goal_clearance',
        f'goal poses: free circle >= {clearance:.3f} m, no keepout within {radius:.3f} m',
        tuple(violations),
        f'{len(layout.goals())} goals; tightest {tightest[1]}, {tightest[0]:.3f} m from '
        f'{tightest[2]}')


def check_visibility(layout):
    """
    Check that what the lidar must see is solid to >= 0.40 m, and that nothing overhangs.

    The lidar sees only its scan plane (0.34 m). Every object is solid from the floor (the
    model builds them so) up to at least the scan plane + 0.06 m: walls, shelf plinths, guards,
    the counter, dock pedestals, fence kick plates, fixtures. The only exceptions are the dock
    contact bars and base plates, and each must lie inside a keepout zone. Nothing may stick
    out past the face the lidar sees below 1.20 m (the Rev E mast top plus 0.10 m): the totes
    stay behind the shelf face.
    """
    robot = layout.robot
    needed = visible_height(robot)
    violations = []
    for solid in layout.solids:
        if solid.kind in LOW_KINDS:
            if not any(geometry.polygon_contains_box(zone.polygon, solid.box)
                       for zone in layout.keepouts):
                violations.append(f'{solid.name}: {solid.solid_to:.2f} m high, below the scan '
                                  'plane, and not inside a keepout zone')
        elif solid.solid_to < needed - EPS:
            violations.append(f'{solid.name}: solid only to {solid.solid_to:.2f} m; the lidar '
                              f'needs {needed:.2f} m')

    # Overhang: a tote's plan view must stay inside its shelf (behind the face).
    shelves = {(row.row, bay.number): row for row in layout.rows for bay in row.bays}
    overhangs = defaultdict(list)
    for slot in layout.slots:
        shelf = shelves[(slot.row, slot.bay)]
        if slot.tote.z0 < OVERHANG_LIMIT and not shelf.box.contains(slot.tote):
            sticking_out = max(shelf.box.x0 - slot.tote.x0, slot.tote.x1 - shelf.box.x1,
                               shelf.box.y0 - slot.tote.y0, slot.tote.y1 - shelf.box.y1)
            overhangs[shelf.name].append((sticking_out, slot.name))
    for shelf, slots in overhangs.items():
        worst = max(slots)
        violations.append(f'shelf_{shelf}: {len(slots)} totes stick out past the face, up to '
                          f'{worst[0]:.3f} m ({worst[1]}); nothing may overhang below '
                          f'{OVERHANG_LIMIT:.2f} m')
    low = sorted({s.kind for s in layout.solids if s.kind in LOW_KINDS})
    return RuleResult(
        'visibility',
        f'solid from the floor to >= {needed:.2f} m; no overhang below {OVERHANG_LIMIT:.2f} m',
        tuple(violations),
        f'{len(layout.solids)} solids; below the scan plane by design: {", ".join(low)}')


def check_docks(layout):
    """
    Check that the docks have the geometry docking needs (plan 2.6, Phase 5).

    The docked pose is half the robot's length (0.32 m) out from the contact face, because the
    robot docks forwards; the staging pose is 0.70 m further out; the contact bar is at most
    0.15 m high (far below the scan plane, inside a keepout box); the pedestal face, which
    carries the dock tag, stands 0.25 m behind the contact face, so the front camera keeps the
    whole tag in view from staging to docked.
    """
    half_length = layout.robot.half_length
    violations = []
    for dock in layout.docks:
        docked = (dock.docked.x, dock.docked.y)
        staging = (dock.staging.x, dock.staging.y)
        bar_height = dock.contact_bar.z1
        if bar_height > MAX_CONTACT_BAR_HEIGHT + EPS:
            violations.append(f'{dock.name}: contact bar {bar_height:.2f} m high; at most '
                              f'{MAX_CONTACT_BAR_HEIGHT:.2f} m')
        setback = math.dist(dock.tag, dock.contact)
        if abs(setback - DOCK_TAG_SETBACK) > EPS:
            violations.append(f'{dock.name}: pedestal face {setback:.2f} m behind the contact '
                              f'face; must be {DOCK_TAG_SETBACK:.2f} m')
        if abs(math.dist(docked, dock.contact) - half_length) > EPS:
            violations.append(f'{dock.name}: docked pose {math.dist(docked, dock.contact):.3f} m '
                              f'from the contact face; must be {half_length:.3f} m')
        staging_distance = math.dist(staging, docked)
        if abs(staging_distance - DOCK_STAGING_DISTANCE) > EPS:
            violations.append(f'{dock.name}: staging pose {staging_distance:.2f} m from the '
                              f'docked pose; must be {DOCK_STAGING_DISTANCE:.2f} m')
    return RuleResult(
        'docks',
        f'docks: docked = contact + {half_length:.2f} m, staging {DOCK_STAGING_DISTANCE:.2f} m '
        f'further, bar <= {MAX_CONTACT_BAR_HEIGHT:.2f} m, tag {DOCK_TAG_SETBACK:.2f} m back',
        tuple(violations), f'{len(layout.docks)} docks')


def check_spawns(layout):
    """
    Check that robot1 spawns at the origin facing +x, and spawns are 2 goal circles apart.

    The map frame is the world frame only if mapping starts at the world origin (plan 3.1),
    and the mapping run starts at robot1's spawn. Two robots parked side by side must each
    have their goal circle free: 2 x 0.556 = 1.11 m between spawns.
    """
    apart = 2 * goal_clearance(layout.robot)
    spawns = {spawn.robot: spawn.pose for spawn in layout.spawns}
    violations = []
    robot1 = spawns.get('robot1')
    if robot1 is None:
        violations.append('robot1: has no spawn; it must spawn at the origin')
    elif max(abs(robot1.x), abs(robot1.y), abs(robot1.yaw)) > EPS:
        violations.append(f'robot1: spawns at ({robot1.x:g}, {robot1.y:g}, yaw {robot1.yaw:g}); '
                          'must be (0, 0, yaw 0) so that map = world')
    closest = math.inf
    names = sorted(spawns)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            distance = math.hypot(spawns[a].x - spawns[b].x, spawns[a].y - spawns[b].y)
            closest = min(closest, distance)
            if distance < apart - EPS:
                violations.append(f'{a} and {b}: spawns {distance:.2f} m apart; need '
                                  f'{apart:.2f} m')
    return RuleResult(
        'spawns', f'robot1 at the origin; spawns >= {apart:.2f} m apart', tuple(violations),
        f'{len(spawns)} spawns, closest pair {closest:.2f} m apart')


def check_totes(layout):
    """
    Check that there are at most 230 totes, standing flush with the shelf face.

    Every tote carries its own ArUco tag, and D23 reserves IDs 20-249 for totes: 230 tags.
    Totes stand flush with the face (setback 0) because the Phase 7 tag rule has no margin:
    2 cm further back a tag at the far edge of the aisle drops from 40.4 to 39.6 px.
    """
    inventory = layout.inventory
    limit = len(TOTE_TAG_IDS)
    violations = []
    if inventory.tote_count > limit:
        violations.append(f'inventory: {inventory.tote_count} totes ({len(layout.slots)} slots '
                          f'x fill ratio {inventory.fill_ratio:g}); there are only {limit} tote '
                          f'tag IDs ({TOTE_TAG_IDS.start}-{TOTE_TAG_IDS.stop - 1})')
    if abs(inventory.tote.setback) > EPS:
        violations.append(f'inventory.tote: setback {inventory.tote.setback:g} m; totes stand '
                          'flush with the shelf face (0) so that Phase 7 tags stay >= 40 px')
    return RuleResult(
        'totes', f'at most {limit} totes, flush with the shelf face', tuple(violations),
        f'{len(layout.slots)} slots, {inventory.tote_count} totes at fill ratio '
        f'{inventory.fill_ratio:g}')


def check_containment(layout):
    """
    Check containment: all inside the hall, no overlaps, slot IDs unique and valid.

    Overlap means sharing an area in plan view; touching is fine, and so are parts of the same
    object (a row segment and its guards, the pieces of a fence). Slot IDs follow
    <row><bay:02d>-<level>-<position> with row letters A-Z except I and O.
    """
    hall = layout.hall
    violations = []
    inside = [s for s in layout.solids if s.kind != 'wall']
    for solid in inside:
        if not hall.box.contains(solid.box):
            violations.append(f'{solid.name}: not inside the hall')
    points = [(p.name, (p.x, p.y)) for p in layout.goals()]
    points += [(d.docked.name, (d.docked.x, d.docked.y)) for d in layout.docks]
    points += [(f'zone {z.name}', p) for z in layout.keepouts + layout.speed_zones
               for p in z.polygon]
    points += [(f'corridor {c.name}', c.at) for c in layout.corridors]
    points += [(f'route {r.name}', p) for r in layout.routes for p in r.waypoints]
    points += [(m.name, c) for m in layout.markings for c in m.box.corners()]
    outside = {}
    for name, (x, y) in points:
        if not hall.contains_point(x, y):
            outside.setdefault(name, (x, y))
    violations += [f'{name}: ({x:g}, {y:g}) is not inside the hall'
                   for name, (x, y) in outside.items()]

    for i, a in enumerate(inside):
        for b in inside[i + 1:]:
            if a.group != b.group and a.box.overlaps(b.box):
                violations.append(f'{a.name}: overlaps {b.name}')

    counts = defaultdict(int)
    invalid = {}
    for slot in layout.slots:
        counts[slot.name] += 1
        if not SLOT_ID_PATTERN.match(slot.name):
            invalid.setdefault(slot.row, slot.name)
    violations += [f'{name}: slot ID used {count} times'
                   for name, count in counts.items() if count > 1]
    violations += [f'row {row}: slot IDs such as {name} are not valid '
                   '(<row><bay:02d>-<level>-<position>, row letters A-Z except I and O)'
                   for row, name in invalid.items()]
    return RuleResult(
        'containment', 'inside the hall, no overlaps, slot IDs unique and valid',
        tuple(violations),
        f'{len(inside)} objects, {len(layout.slots)} slot IDs')


def check_routes(layout):
    """
    Check that routes keep their distance from objects and keepout zones.

    The route driver turns in place at each waypoint, so a waypoint needs a goal circle. Along
    a segment it drives straight: half the robot's width plus 0.16 m for steering error. And the
    robot's half width must stay out of every keepout zone.
    """
    robot = layout.robot
    waypoint_clearance = goal_clearance(robot)
    segment_clearance = route_clearance(robot)
    violations, summaries = [], []
    for route in layout.routes:
        tightest_point = (math.inf, None)
        for number, (x, y) in enumerate(route.waypoints):
            distance, solid = nearest_solid(layout, x, y)
            tightest_point = min(tightest_point, (distance, solid.name), key=lambda t: t[0])
            if distance < waypoint_clearance - EPS:
                violations.append(f'route {route.name}: waypoint {number} ({x:g}, {y:g}) is '
                                  f'{distance:.2f} m from {solid.name}; needs '
                                  f'{waypoint_clearance:.3f} m')
        tightest_segment = (math.inf, None)
        for number, (a, b) in enumerate(zip(route.waypoints, route.waypoints[1:])):
            distance, solid = min(
                ((geometry.segment_polygon_distance(a, b, s.box.corners()), s.name)
                 for s in layout.solids), key=lambda t: t[0])
            tightest_segment = min(tightest_segment, (distance, solid), key=lambda t: t[0])
            if distance < segment_clearance - EPS:
                violations.append(f'route {route.name}: segment {number} {list(a)} -> {list(b)} '
                                  f'passes {distance:.2f} m from {solid}; needs '
                                  f'{segment_clearance:.2f} m')
            for zone in layout.keepouts:
                distance = geometry.segment_polygon_distance(a, b, zone.polygon)
                if distance < robot.half_width - EPS:
                    where = 'through' if distance == 0.0 else f'{distance:.2f} m from'
                    violations.append(f'route {route.name}: segment {number} {list(a)} -> '
                                      f'{list(b)} runs {where} keepout {zone.name}')
        summaries.append(f'{route.name} {route.length:.0f} m (waypoints >= '
                         f'{tightest_point[0]:.2f} m, segments >= {tightest_segment[0]:.2f} m)')
    return RuleResult(
        'routes',
        f'routes: waypoints >= {waypoint_clearance:.3f} m, segments >= '
        f'{segment_clearance:.2f} m from objects, out of keepouts',
        tuple(violations), '; '.join(summaries) if summaries else 'no routes')


RULES = (
    check_pick_aisle_width,
    check_corridor_width,
    check_goal_clearance,
    check_visibility,
    check_docks,
    check_spawns,
    check_totes,
    check_containment,
    check_routes,
)


def check(layout):
    """Run every fit rule on the layout and return their results, in the order of RULES."""
    return tuple(rule(layout) for rule in RULES)
