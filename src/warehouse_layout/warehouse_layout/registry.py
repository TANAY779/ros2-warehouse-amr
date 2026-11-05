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
The location registry: every named place in the warehouse, as generated/locations.yaml.

It holds, in the map frame:

- ``poses`` by location ID: a spawn pose per robot (also where it parks), the drop pose of
  each packing station, the docked and staging pose of each dock, and a pick pose at each bay
  of each pick aisle;
- ``docks``: their poses, the middle of the contact face and of the pedestal face (where the
  dock tag goes in Phase 7), and the keepout box round the contact bar;
- ``aisles``: each pick aisle segment's floor polygon, its two rows and its pick poses;
- ``zones``: the keepout and speed zones as polygons;
- ``routes``: waypoints for the route driver (the mapping run and test drives).

Readers: sim.launch.py takes the spawn poses from it (Phase 2), the location lookups of Phase
4 and docking in Phase 5 use it, and so does the route driver. They read this plain YAML
file, so they need neither this package's code nor the robot's size. Every location has an ID
that is unique in the file.
"""

from warehouse_layout import formatting
from warehouse_layout.formatting import yaml_float
from warehouse_layout.formatting import yaml_flow
from warehouse_layout.formatting import yaml_names
from warehouse_layout.formatting import yaml_point
from warehouse_layout.formatting import yaml_points
from warehouse_layout.formatting import yaml_str
from warehouse_layout.geometry import EPS

SCHEMA_VERSION = 1


def render(layout):
    """Return locations.yaml's text."""
    lines = formatting.yaml_comment(formatting.notice() + [
        '',
        'The location registry: every named place in the warehouse. All coordinates are in the',
        "map frame, which is the Gazebo world frame; its origin is robot1's spawn pose. x points",
        'east, y north, yaw is counter-clockwise from +x (REP-103). Metres, radians, m/s.',
    ])
    lines += [
        '',
        f'schema_version: {SCHEMA_VERSION}',
        f'world: {yaml_str(layout.name)}',
        'frame_id: map',
    ]
    lines += _poses(layout) + _docks(layout) + _aisles(layout) + _zones(layout)
    lines += _routes(layout)
    return '\n'.join(lines) + '\n'


def summary(layout):
    """Return a one-line summary of the registry, for the generate command."""
    count = formatting.count
    return ', '.join([
        count(len(layout.spawns), 'spawn'), count(len(layout.stations), 'drop'),
        count(len(layout.docks), 'dock'), count(len(layout.picks), 'pick'),
        count(len(layout.aisles), 'aisle segment'),
        f'{len(layout.keepouts)} keepout and {count(len(layout.speed_zones), "speed zone")}',
        count(len(layout.routes), 'route')])


def pick_places(layout):
    """
    Return {pick pose name: (aisle segment, bay number, bay labels)} for every pick pose.

    The bay labels are the bays on either side of the pose, west row first: ['A02', 'B02'].
    Where one row is shorter (rows A, D and E in the north block), there is only one.
    """
    places = {}
    for aisle in layout.aisles:
        segments = {s.row: s for s in layout.rows if s.block == aisle.block}
        for pick in layout.picks:
            if aisle.box.distance_to(pick.x, pick.y) > EPS:
                continue
            bays = [bay for row in aisle.rows for bay in segments[row].bays
                    if bay.y0 - EPS <= pick.y <= bay.y1 + EPS]
            places[pick.name] = (aisle, bays[0].number, [bay.name for bay in bays])
    return places


def _pose(pose, fields):
    """Return one line of ``poses``: the ID, then the fields, x, y and yaw."""
    position = [('x', yaml_float(pose.x)), ('y', yaml_float(pose.y)),
                ('yaw', yaml_float(pose.yaw))]
    return f'  {yaml_str(pose.name)}: {yaml_flow(fields + position)}'


def _poses(layout):
    lines = [
        '',
        '# Poses by location ID. kind: spawn (where the robot starts and parks), drop (at a',
        "# packing station's counter), dock_docked and dock_staging (Phase 5), pick (on a pick",
        "# aisle's centreline at a bay, facing along the aisle; bays: the bays on either side).",
        'poses:',
    ]
    for spawn in layout.spawns:
        lines.append(_pose(spawn.pose, [('kind', 'spawn'), ('robot', yaml_str(spawn.robot))]))
    for station in layout.stations:
        lines.append(_pose(station.drop, [('kind', 'drop'), ('station', yaml_str(station.name))]))
    for dock in layout.docks:
        for kind, pose in (('dock_docked', dock.docked), ('dock_staging', dock.staging)):
            lines.append(_pose(pose, [('kind', kind), ('dock', yaml_str(dock.name))]))
    places = pick_places(layout)
    for pick in layout.picks:
        aisle, bay, labels = places[pick.name]
        lines.append(_pose(pick, [('kind', 'pick'), ('aisle', yaml_str(aisle.aisle)),
                                  ('bay', str(bay)), ('bays', yaml_names(labels))]))
    return lines


def _docks(layout):
    lines = [
        '',
        '# Charging docks (Phase 5): the docked and staging poses (IDs in poses), the middle of',
        '# the contact face and of the pedestal face (the dock tag, Phase 7), both on the floor,',
        '# and the keepout zone round the contact bar (in zones).',
    ]
    if not layout.docks:
        return lines + ['docks: {}']
    lines.append('docks:')
    for dock in layout.docks:
        lines += [
            f'  {yaml_str(dock.name)}:',
            f'    wall: {yaml_str(dock.wall)}',
            f'    docked: {yaml_str(dock.docked.name)}',
            f'    staging: {yaml_str(dock.staging.name)}',
            f'    contact: {yaml_point(dock.contact)}',
            f'    tag: {yaml_point(dock.tag)}',
            f'    keepout: {yaml_str(dock.keepout.name)}',
        ]
    return lines


def _aisles(layout):
    lines = [
        '',
        '# Pick aisle segments (Phases 8 and 11): the floor between the two shelf faces, from end',
        '# guard to end guard. rows: [west row, east row]; picks: the pick poses on the',
        '# centreline, south to north.',
    ]
    if not layout.aisles:
        return lines + ['aisles: {}']
    lines.append('aisles:')
    places = pick_places(layout)
    for aisle in layout.aisles:
        picks = [p.name for p in layout.picks if places[p.name][0].name == aisle.name]
        lines += [
            f'  {yaml_str(aisle.name)}:',
            f'    aisle: {yaml_str(aisle.aisle)}',
            f'    block: {yaml_str(aisle.block)}',
            f'    rows: {yaml_names(aisle.rows)}',
            f'    width: {yaml_float(aisle.width)}',
            f'    polygon: {yaml_points(aisle.box.corners())}',
            f'    picks: {yaml_names(picks)}',
        ]
    return lines


def _zones(layout):
    lines = [
        '',
        '# Keepout and speed zones (the Phase 3 costmap filters) as polygons; limit in m/s.',
        '# generated: made from other objects (a box round each dock contact bar, a speed zone',
        '# over each pick aisle segment) rather than listed in layout.yaml.',
        'zones:',
    ]
    for kind, zones in (('keepout', layout.keepouts), ('speed', layout.speed_zones)):
        if not zones:
            lines.append(f'  {kind}: {{}}')
            continue
        lines.append(f'  {kind}:')
        for zone in zones:
            fields = [] if zone.limit is None else [('limit', yaml_float(zone.limit))]
            fields += [('generated', 'true' if zone.generated else 'false'),
                       ('polygon', yaml_points(zone.polygon))]
            lines.append(f'    {yaml_str(zone.name)}: {yaml_flow(fields)}')
    return lines


def _routes(layout):
    lines = [
        '',
        '# Routes for the route driver: waypoints [x, y], driven in order.',
    ]
    if not layout.routes:
        return lines + ['routes: {}']
    lines.append('routes:')
    for route in layout.routes:
        lines.append(f'  {yaml_str(route.name)}:   # {route.length:.1f} m')
        lines += [f'    - {yaml_point(point)}' for point in route.waypoints]
    return lines
