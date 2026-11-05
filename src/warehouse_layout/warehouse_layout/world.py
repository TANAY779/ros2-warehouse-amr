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
The Gazebo world made from the layout: warehouse_gazebo/worlds/<name>.sdf.

``render(layout)`` returns the SDF text. ``models(layout)`` returns the same world as data,
one ``WorldModel`` per static model, which the tests read more easily than XML. The world has:

- the physics (1 ms steps in real time) and the five systems of empty.sdf. A world that lists
  any <plugin> loads only the systems it lists, so the robot needs every one of them
  (warehouse_gazebo README, "Systems in the world");
- one static model per object: the walls with their pilasters and doors, each shelf row
  segment, each packing station, each dock, each fixture, each fence, and the floor paint.
  Static models never move: the physics engine only tests them for contacts;
- one collision box per solid of the layout model (model.Solid), named after it, so what a
  robot can run into is exactly what the fit rules checked;
- visuals for the detail. Gazebo's gpu_lidar renders visuals, not collisions, so the visuals
  are what the lidar sees. Every solid has a visual that fills its footprint from the floor to
  above the scan plane (a shelf's plinth, a fence's kick plate), and below 1.20 m no visual
  reaches out of the solids (test_world.py checks both);
- floor paint: thin boxes without collision, each kind in its own layer so that paint on paint
  does not flicker;
- the expected inventory's totes, only when ``style.show_totes`` is true (bare shelves until
  Phase 7).

Every pose is in the map frame, which is the Gazebo world frame. The models stay at the origin,
so a number in the world file is the number in layout.yaml and in the preview, and a layout
edit changes only the lines of the parts it moves.

The same layout gives the same bytes: numbers have 4 decimals at most (0.1 mm), everything is
written in the layout's order, and nothing depends on the time (see formatting.py).
"""

from dataclasses import dataclass
from dataclasses import replace
import math
from xml.etree import ElementTree

from warehouse_layout import formatting
from warehouse_layout import geometry
from warehouse_layout import inventory
from warehouse_layout.geometry import Box
from warehouse_layout.geometry import EPS

# The solid kinds that model.py builds itself; any other kind is a fixture's (model.Solid).
BUILT_IN_KINDS = ('wall', 'pilaster', 'shelf', 'guard', 'station', 'dock_pedestal',
                  'contact_bar', 'dock_plate', 'fence')

# The look. The collisions do not depend on these numbers, and at the scan plane the lidar
# sees them only as the door panels, which stand out of their wall by less than its noise.
#
# Two faces in one plane flicker in the renderer (z-fighting), so a part that passes through
# another (a board through the uprights, a fence mesh through its posts) stays INSET inside it.
INSET = 0.002              # m
DOOR_HEIGHT = 2.10         # m
DOOR_PROUD = 0.005         # m: a door panel stands this far out of its wall face
FENCE_RAIL = 0.04          # m: the top rail of a fence
FENCE_POST = 0.06          # m: a post's width along the fence
FENCE_POST_SPACING = 1.5   # m: posts at most this far apart
FENCE_MESH_TRANSPARENCY = 0.65
ZONE_TINT = 0.30           # a zone's floor: this share of its paint colour over the floor colour
DASH = 0.15                # m: the dashes of a dock bay's outline ...
DASH_GAP = 0.10            # m: ... and the shortest gap between them
# Floor paint, bottom to top. Each kind is a box from the floor up to its own height, 1 mm
# above the kind below it, so where two kinds overlap their top faces never share a plane.
PAINT_TOP = {'speed_tint': 0.001, 'keepout_tint': 0.002, 'speed_border': 0.003,
             'keepout_border': 0.004, 'lane': 0.005, 'dock': 0.006}
SPECULAR = '0.1 0.1 0.1 1'

# The direction from a wall's inner face into the hall.
_INTO_HALL = {'south': (0.0, 1.0), 'north': (0.0, -1.0), 'west': (1.0, 0.0), 'east': (-1.0, 0.0)}


@dataclass(frozen=True)
class Part:
    """
    One box of a model, in the map frame: a visual (when it has a colour), a collision, or both.

    ``yaw`` turns the box about its centre (only paint along a slanted zone edge needs it).
    ``transparency`` is 0 for opaque, 1 for invisible.
    """

    name: str
    box: Box
    colour: str = None
    collision: bool = False
    transparency: float = 0.0
    yaw: float = 0.0
    cast_shadows: bool = True


@dataclass(frozen=True)
class WorldModel:
    """A static model of the world: its name, a one-line comment and its parts."""

    name: str
    comment: str
    parts: tuple


# ----------------------------------------------------------------------------- colours


def linear_rgb(colour):
    """
    Return an sRGB colour '#rrggbb' as Gazebo material values: linear light, 0 to 1.

    Gazebo's ogre2 renderer treats material colours as linear light and stores the picture in
    sRGB, like a photo. Measured in Phase 2: a surface that emits 0.2 shows as pixel value 123,
    which is 0.2 encoded in sRGB, not 51 (0.2 x 255). Written as they are, the layout's colours
    would therefore look washed out ('#3d4148', a dark grey, as a mid grey); converted to linear
    light they show as themselves, before lighting.
    """
    channels = (int(colour[i:i + 2], 16) / 255 for i in (1, 3, 5))
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in channels)


def mix(base, paint, share):
    """Return the sRGB colour of ``paint`` laid over ``base`` at this share (0 to 1)."""
    channels = []
    for i in (1, 3, 5):
        low, high = int(base[i:i + 2], 16), int(paint[i:i + 2], 16)
        channels.append(round(low + (high - low) * share))
    return '#' + ''.join(f'{c:02x}' for c in channels)


# ----------------------------------------------------------------------------- the models


def models(layout):
    """Return the world's static models as WorldModel objects, in the order of the file."""
    groups = {}
    for solid in layout.solids:
        groups.setdefault(solid.group, []).append(solid)
    totes = _totes_by_segment(layout)

    built = [_walls(layout)]
    built += [_shelf(layout, segment, groups[segment.name], totes.get(segment.name, ()))
              for segment in layout.rows]
    built += [_station(layout, station, groups[station.name]) for station in layout.stations]
    built += [_dock(layout, dock, groups[dock.name]) for dock in layout.docks]
    built += [_fixture(layout, solid) for solid in layout.solids
              if solid.kind not in BUILT_IN_KINDS]
    built += [_fence(layout, fence, groups[fence.name]) for fence in layout.fences]
    built.append(_floor_paint(layout))
    _check(layout, built)
    return tuple(built)


def _walls(layout):
    style, hall = layout.style, layout.hall
    parts = []
    for solid in layout.solids:
        if solid.kind == 'wall':
            parts.append(Part(solid.name, solid.box, collision=True))
            band = min(style.wall_band_height, solid.box.z1)
            parts.append(Part(f'{solid.name}_band', replace(solid.box, z1=band), style.wall_band))
            if band < solid.box.z1:
                parts.append(Part(f'{solid.name}_upper', replace(solid.box, z0=band), style.wall))
        elif solid.kind == 'pilaster':
            parts.append(Part(solid.name, solid.box, style.pilaster, collision=True))
    # Door panels are visual only: the doors stay shut. A recess is the doorway of a door in
    # its back wall (the loading door).
    for door in hall.doors:
        parts.append(_door_panel(door.name, door.wall, door.line, style.door))
    for recess in hall.recesses:
        parts.append(_door_panel(recess.name, recess.wall, _back_face(recess), style.door))
    return WorldModel('walls', 'The hall: walls (a slate band below '
                      f'{_number(style.wall_band_height)} m), pilasters and door panels.',
                      tuple(parts))


def _door_panel(name, wall, line, colour):
    """Return a door panel on a wall face; ``line`` is the door's plan-view segment on it."""
    (x0, y0), (x1, y1) = line
    dx, dy = _INTO_HALL[wall]
    box = Box.spanning((x0, x1 + dx * DOOR_PROUD), (y0, y1 + dy * DOOR_PROUD), (0.0, DOOR_HEIGHT))
    return Part(name, box, colour)


def _back_face(recess):
    """Return the plan-view segment of a recess's back wall face: the far edge of its floor."""
    f = recess.floor
    faces = {'south': ((f.x0, f.y0), (f.x1, f.y0)), 'north': ((f.x0, f.y1), (f.x1, f.y1)),
             'west': ((f.x0, f.y0), (f.x0, f.y1)), 'east': ((f.x1, f.y0), (f.x1, f.y1))}
    return faces[recess.wall]


def _totes_by_segment(layout):
    """Return {row segment name: [Slot, ...]}: the slots that show a tote, by shelf."""
    if not layout.style.show_totes:
        return {}
    slots = {slot.name: slot for slot in layout.slots}
    segment_of = {(segment.row, bay.number): segment.name
                  for segment in layout.rows for bay in segment.bays}
    shown = {}
    for tote in inventory.expected_inventory(layout):
        slot = slots[tote.slot]
        shown.setdefault(segment_of[(slot.row, slot.bay)], []).append(slot)
    return shown


def _shelf(layout, segment, solids, slots):
    """
    Return a shelf row segment: plinth, uprights, boards, end guards and any totes.

    The collision is the whole shelf, floor to top: one box, cheap for the physics engine (plan
    P2-D6). The lidar sees the plinth, which fills the shelf's footprint from the floor to the
    first storage level. The uprights stand on the plinth and end under the top board; the
    level boards pass through the uprights' posts, INSET inside the shelf's outline.
    """
    shelving, style = layout.shelving, layout.style
    box = segment.box
    parts = []
    for solid in solids:   # the shelf and its two end guards
        if solid.kind == 'shelf':
            parts.append(Part(solid.name, solid.box, collision=True))
        else:
            parts.append(Part(solid.name, solid.box, style.rack_guard, collision=True))
    parts.append(Part('plinth', replace(box, z1=shelving.plinth_height), style.rack_plinth))
    under_top = shelving.top_height - shelving.board_thickness
    # An upright south of every bay and one north of the last. Each upright is an open frame,
    # a square post at the face and one at the back, so that cameras can look into the bays
    # at an angle (Phase 7 reads the tags on the totes).
    starts = [bay.y0 - shelving.upright for bay in segment.bays] + [segment.bays[-1].y1]
    post = shelving.upright
    east, west = (box.x1 - post, box.x1), (box.x0, box.x0 + post)
    front, back = (east, west) if segment.face == '+x' else (west, east)
    for number, y in enumerate(starts, 1):
        for side, (x0, x1) in (('front', front), ('back', back)):
            upright = Box(x0, x1, y, y + post, shelving.plinth_height, under_top)
            parts.append(Part(f'upright_{number}_{side}', upright, style.rack_upright))
    inner = _inset(box)
    for level, height in enumerate(shelving.board_heights, 2):
        board = replace(inner, z0=height - shelving.board_thickness, z1=height)
        parts.append(Part(f'board_{level}', board, style.rack_board))
    parts.append(Part('top_board', replace(box, z0=under_top), style.rack_board))
    parts += [Part(f'tote_{slot.name}', slot.tote, style.tote) for slot in slots]
    first, last = segment.bays[0].number, segment.bays[-1].number
    comment = (f'Shelf row {segment.row}, {segment.block} block, facing {segment.face}: bays '
               f'{first:02d}-{last:02d}' + (f', {len(slots)} totes.' if slots else '.'))
    return WorldModel(f'shelf_{segment.name}', comment, tuple(parts))


def _station(layout, station, solids):
    parts = tuple(Part(s.name, s.box, layout.style.station, collision=True) for s in solids)
    return WorldModel(station.name, f'Packing station {station.name}: the counter, solid to the '
                      'floor.', parts)


def _dock(layout, dock, solids):
    style = layout.style
    colours = {'dock_pedestal': style.dock, 'contact_bar': style.contact,
               'dock_plate': style.dock_plate}
    parts = []
    for solid in solids:
        if solid.kind == 'contact_bar' and dock.plate.contains(solid.box):
            # The bar stands on the base plate, so its visual starts on the plate: the two
            # front faces would otherwise share a plane.
            parts.append(Part(solid.name, solid.box, collision=True))
            parts.append(Part(solid.name, replace(solid.box, z0=dock.plate.z1), style.contact))
        else:
            parts.append(Part(solid.name, solid.box, colours[solid.kind], collision=True))
    return WorldModel(dock.name, f'Charging dock {dock.name} on the {dock.wall} wall: pedestal '
                      '(its face carries the dock tag in Phase 7), contact bar, base plate.',
                      tuple(parts))


def _fixture(layout, solid):
    colour = layout.style.fixture(solid.kind)
    return WorldModel(solid.name, f'Fixture {solid.name} ({solid.kind.replace("_", " ")}), solid '
                      'from the floor.', (Part(solid.name, solid.box, colour, collision=True),))


def _fence(layout, fence, solids):
    """
    Return a fence: per straight piece a solid kick plate, a see-through mesh and posts.

    The collision is the whole piece, floor to top. The lidar sees the kick plate, which is
    solid to above the scan plane.
    """
    colour = layout.style.fence
    kick, rail = fence.kick_plate_height, fence.height - FENCE_RAIL
    parts = []
    for solid in solids:   # one per straight piece between the corners and the gates
        box = solid.box
        parts += [
            Part(solid.name, box, collision=True),
            Part(f'{solid.name}_kick_plate', replace(box, z1=kick), colour),
            Part(f'{solid.name}_mesh', replace(_inset(box), z0=kick, z1=rail), colour,
                 transparency=FENCE_MESH_TRANSPARENCY),
            Part(f'{solid.name}_rail', replace(box, z0=rail), colour),
        ]
        parts += _posts(solid.name, box, kick, rail, colour)
    return WorldModel(fence.name, f'Fence {fence.name}: a solid kick plate to '
                      f'{_number(kick)} m, mesh above it, open gates.', tuple(parts))


def _posts(name, box, z0, z1, colour):
    """Return fence posts along a piece, at most FENCE_POST_SPACING apart, one at each end."""
    along_x = box.x1 - box.x0 >= box.y1 - box.y0
    start, end = (box.x0, box.x1) if along_x else (box.y0, box.y1)
    room = end - start - FENCE_POST
    gaps = max(1, math.ceil(room / FENCE_POST_SPACING - EPS))
    posts = []
    for i in range(gaps + 1):
        a = start + room * i / gaps
        if along_x:
            post = Box(a, a + FENCE_POST, box.y0, box.y1, z0, z1)
        else:
            post = Box(box.x0, box.x1, a, a + FENCE_POST, z0, z1)
        posts.append(Part(f'{name}_post_{i + 1}', post, colour))
    return posts


def _floor_paint(layout):
    """
    Return the floor paint: zones, the zebra crossing, parking and dock bays, aisle stripes.

    Keepout zones are red-tinted with a red border and the speed zones from layout.yaml amber;
    the pick aisles' speed zones are marked by the stripes across both ends of each aisle.
    """
    style = layout.style
    parts = []
    for zone in layout.keepouts:
        parts += _zone_paint(zone, style.keepout_paint, style.floor, layout.line_width)
    for zone in layout.speed_zones:
        if not zone.generated:
            parts += _zone_paint(zone, style.speed_paint, style.floor, layout.line_width)
    for marking in layout.markings:
        if marking.kind in ('zebra_stripe', 'aisle_stripe'):
            parts.append(_paint(marking.name, marking.box, 'lane', style.lane_paint))
        elif marking.kind == 'parking_bay':
            parts += _outline(marking.name, marking.box, layout.line_width, 'lane',
                              style.lane_paint)
        elif marking.kind == 'dock_bay':
            parts += _outline(marking.name, marking.box, layout.line_width, 'dock',
                              style.dock_paint, dashed=True)
        else:
            raise ValueError(f'{marking.name}: world.py cannot paint a {marking.kind!r}')
    return WorldModel('floor_paint', 'Floor paint, visual only: keepout and speed zones, the '
                      'zebra crossing, parking and dock bays, aisle stripes.', tuple(parts))


def _paint(name, box, layer, colour):
    """Return a painted area of the floor: the box's plan view, as high as its paint layer."""
    return Part(name, Box(box.x0, box.x1, box.y0, box.y1, 0.0, PAINT_TOP[layer]), colour,
                cast_shadows=False)


def _zone_paint(zone, colour, floor, width):
    """Return a zone's tint (rectangles only) and a border line along each of its edges."""
    parts = []
    box = _rectangle(zone.polygon)
    if box is not None:
        # One box can only tint an axis-parallel rectangle; any other polygon gets its border.
        tint = mix(floor, colour, ZONE_TINT)
        parts.append(_paint(f'{zone.name}_tint', box, f'{zone.kind}_tint', tint))
    for number, (a, b) in enumerate(geometry.edges(zone.polygon), 1):
        parts.append(_line(f'{zone.name}_border_{number}', a, b, width, f'{zone.kind}_border',
                           colour))
    return parts


def _rectangle(polygon):
    """Return the polygon as a Box if it is a rectangle with axis-parallel edges, else None."""
    if len(polygon) != 4:
        return None
    for (x0, y0), (x1, y1) in geometry.edges(polygon):
        if abs(x1 - x0) > EPS and abs(y1 - y0) > EPS:
            return None   # a slanted edge
    box = Box(min(p[0] for p in polygon), max(p[0] for p in polygon),
              min(p[1] for p in polygon), max(p[1] for p in polygon))
    return box if box.x1 - box.x0 > EPS and box.y1 - box.y0 > EPS else None


def _line(name, a, b, width, layer, colour):
    """
    Return a painted line from a to b, ``width`` wide and centred on the segment.

    It reaches half its width past both ends, so two lines that meet at a corner close it.
    """
    (x0, y0), (x1, y1) = a, b
    half, top = width / 2, PAINT_TOP[layer]
    if abs(y1 - y0) <= EPS:   # along x
        box = Box(min(x0, x1) - half, max(x0, x1) + half, y0 - half, y0 + half, 0.0, top)
        return Part(name, box, colour, cast_shadows=False)
    if abs(x1 - x0) <= EPS:   # along y
        box = Box(x0 - half, x0 + half, min(y0, y1) - half, max(y0, y1) + half, 0.0, top)
        return Part(name, box, colour, cast_shadows=False)
    # Slanted: a box along x through the middle of the segment, turned to the segment.
    length = math.hypot(x1 - x0, y1 - y0) + width
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    box = Box(cx - length / 2, cx + length / 2, cy - half, cy + half, 0.0, top)
    return Part(name, box, colour, yaw=math.atan2(y1 - y0, x1 - x0), cast_shadows=False)


def _outline(name, box, width, layer, colour, dashed=False):
    """
    Return the painted outline of a box: a line ``width`` wide inside each edge.

    The south and north lines run the whole width and the west and east lines fit between
    them, so no two lines overlap. ``dashed`` breaks every line into dashes, with a dash at
    each end so that the corners stay painted.
    """
    top = PAINT_TOP[layer]
    sides = (
        ('south', 'x', (box.x0, box.x1), (box.y0, box.y0 + width)),
        ('north', 'x', (box.x0, box.x1), (box.y1 - width, box.y1)),
        ('west', 'y', (box.y0 + width, box.y1 - width), (box.x0, box.x0 + width)),
        ('east', 'y', (box.y0 + width, box.y1 - width), (box.x1 - width, box.x1)),
    )
    parts = []
    for side, axis, (start, end), (low, high) in sides:
        spans = _dashes(start, end) if dashed else [(start, end)]
        for number, (a, b) in enumerate(spans, 1):
            if axis == 'x':
                line = Box(a, b, low, high, 0.0, top)
            else:
                line = Box(low, high, a, b, 0.0, top)
            label = f'{name}_{side}' if len(spans) == 1 else f'{name}_{side}_{number}'
            parts.append(Part(label, line, colour, cast_shadows=False))
    return parts


def _dashes(start, end):
    """Return (start, end) spans of DASH long dashes, at least DASH_GAP apart, one at each end."""
    length = end - start
    count = int((length + DASH_GAP + EPS) // (DASH + DASH_GAP))
    if count < 2:
        return [(start, end)]   # too short to break up: one solid line
    gap = (length - count * DASH) / (count - 1)
    return [(start + i * (DASH + gap), start + i * (DASH + gap) + DASH) for i in range(count)]


def _inset(box):
    """Return the box made INSET smaller on each side in plan view."""
    return replace(box, x0=box.x0 + INSET, x1=box.x1 - INSET, y0=box.y0 + INSET,
                   y1=box.y1 - INSET)


def _check(layout, built):
    """Raise ValueError if names clash or a solid of the layout got no collision."""
    names = [m.name for m in built] + ['ground_plane']
    for name in names:
        if names.count(name) > 1:
            raise ValueError(f'world: two models named {name!r}; rename one in layout.yaml')
    for m in built:
        elements = ([f'{p.name}_visual' for p in m.parts if p.colour is not None]
                    + [f'{p.name}_collision' for p in m.parts if p.collision])
        for element in elements:
            if elements.count(element) > 1:
                raise ValueError(f'world: model {m.name} has two parts named {element!r}')
    collided = {p.name for m in built for p in m.parts if p.collision}
    missing = [s.name for s in layout.solids if s.name not in collided]
    if missing:
        raise ValueError(f'world: no collision for {", ".join(missing)} (world.py does not '
                         'build their kind)')


# ----------------------------------------------------------------------------- SDF text


_HEAD = """\
<?xml version="1.0"?>
<!--
{notice}
-->
<sdf version="1.9">
  <world name="{name}">

    <!-- As in empty.sdf: 1 ms steps in real time. The controller manager runs at 100 Hz, so
         each control cycle covers 10 physics steps. -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- A world that lists any <plugin> loads only the systems it lists, so every system the
         robot relies on is here (warehouse_gazebo README, "Systems in the world"). -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1.0</ambient>
      <background>0.75 0.80 0.85 1.0</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- The floor, in the floor colour. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          {floor}
        </visual>
      </link>
    </model>
"""

_TAIL = """
  </world>
</sdf>
"""


def render(layout):
    """Return the world's SDF text."""
    description = [
        '',
        f'The {layout.name} world: every pose is in the map frame (= this world frame, origin at',
        "robot1's spawn), and every model is static and stays at the origin. Each solid of the",
        'layout is one collision box named after it; visuals carry the detail, and the lidar',
        '(gpu_lidar) sees visuals only. Floor paint has no collision.',
    ]
    # notice(check=False): an XML comment may not contain two hyphens in a row.
    notice = '\n'.join(f'  {line}' if line else '' for line in
                       formatting.notice(check=False) + description)
    head = _HEAD.format(notice=notice, name=layout.name,
                        floor=_material(layout.style.floor, 10))
    return head + ''.join(_model_xml(m) for m in models(layout)) + _TAIL


def summary(text):
    """Return what a world file holds, for example '32 models, 461 visuals, 89 collisions'."""
    world = ElementTree.fromstring(text).find('world')
    count = formatting.count
    return (f'{count(len(world.findall("model")), "model")}, '
            f'{count(len(list(world.iter("visual"))), "visual")}, '
            f'{count(len(list(world.iter("collision"))), "collision")}')


def _model_xml(m):
    if '--' in m.comment:
        raise ValueError(f'{m.name}: an XML comment may not contain "--"')
    lines = ['', f'    <!-- {m.comment} -->', f'    <model name="{m.name}">',
             '      <static>true</static>', '      <link name="link">']
    for part in m.parts:
        pose = _pose(part)
        geometry_xml = f'<geometry><box><size>{_size(part.box)}</size></box></geometry>'
        if part.colour is not None:
            lines += [f'        <visual name="{part.name}_visual">',
                      f'          <pose>{pose}</pose>',
                      f'          {geometry_xml}',
                      f'          {_material(part.colour, 10)}']
            if part.transparency:
                transparency = _number(part.transparency)
                lines.append(f'          <transparency>{transparency}</transparency>')
            if not part.cast_shadows:
                lines.append('          <cast_shadows>false</cast_shadows>')
            lines.append('        </visual>')
        if part.collision:
            lines += [f'        <collision name="{part.name}_collision">',
                      f'          <pose>{pose}</pose>',
                      f'          {geometry_xml}',
                      '        </collision>']
    lines += ['      </link>', '    </model>']
    return '\n'.join(lines) + '\n'


def _material(colour, indent):
    """Return the material lines of a colour, the first without indentation."""
    rgba = ' '.join(_number(c) for c in linear_rgb(colour)) + ' 1'
    pad = ' ' * indent
    return (f'<material>\n{pad}  <ambient>{rgba}</ambient>\n{pad}  <diffuse>{rgba}</diffuse>\n'
            f'{pad}  <specular>{SPECULAR}</specular>\n{pad}</material>')


def _pose(part):
    x, y, z = part.box.centre
    return f'{_number(x)} {_number(y)} {_number(z)} 0 0 {_number(part.yaw)}'


def _size(box):
    return ' '.join(_number(v) for v in box.size)


def _number(value):
    """Return a number for SDF: fixed to 4 decimals (0.1 mm), no trailing zeros, never -0."""
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text == '-0' else text
