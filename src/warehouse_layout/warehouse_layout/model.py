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
Read layout.yaml, check it against the schema and derive the whole warehouse from it.

``load()`` returns a ``Layout``: the hall and every solid object in it as boxes, the shelf rows
with their bays and slots, the pick aisles with their pick poses, the packing stations, the
docks and their poses, the spawns, the keepout and speed zones, the corridors whose widths
the fit rules check, the routes, the floor markings, the inventory settings and the style.
The fit rules (rules.py), the preview (preview.py) and the generators read the warehouse from
a Layout, never from the YAML directly, so every derived number has exactly one definition.

A layout that does not match the schema raises ``LayoutError`` naming the bad key, for
example ``layout.yaml: hall.wall_height: missing``. Unknown keys are errors too, which
catches typos. The package README describes the schema.

An object's ``id`` in layout.yaml is its ``name`` here: flake8 does not allow an attribute
that shadows Python's builtin ``id``.
"""

from dataclasses import dataclass
from dataclasses import fields
import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from warehouse_description import robot_config
from warehouse_layout.geometry import Box
from warehouse_layout.geometry import EPS
import yaml

SCHEMA_VERSION = 1

# Slot IDs: <row><bay:02d>-<level>-<position>, for example C07-2-1 (row C, bay 07, level 2,
# position 1, the south one). Row letters skip I and O, as warehouses do: on a label they
# read like 1 and 0.
SLOT_ID_PATTERN = re.compile(r'^[A-HJ-NP-Z][0-9]{2}-[1-9]-[1-9]$')

WALLS = ('south', 'north', 'west', 'east')

# The yaw of a robot that faces a wall (REP-103: counter-clockwise from +x, which is east).
_YAW_TOWARDS = {'south': -math.pi / 2, 'north': math.pi / 2, 'west': math.pi, 'east': 0.0}

# IDs become names in SDF, ROS parameters and file names: letters, digits and underscores.
_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_]+$')
_COLOUR_PATTERN = re.compile(r'^#[0-9a-fA-F]{6}$')


class LayoutError(ValueError):
    """layout.yaml does not match the schema. The message names the bad key."""


# ----------------------------------------------------------------------------- the warehouse


@dataclass(frozen=True)
class Pose:
    """A named pose in the map frame: x and y in metres, yaw in radians from +x."""

    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Solid:
    """
    A physical object, or one part of one, that a robot can run into.

    ``box.z1`` is its top. ``solid_to`` is the height up to which it is solid from the floor,
    which is what the lidar sees. Walls and fixtures are solid to the top; a shelf only to its
    plinth (the levels above are open) and a fence only to its kick plate (mesh above).

    ``kind`` is one of 'wall', 'pilaster', 'shelf', 'guard', 'station', 'dock_pedestal',
    'contact_bar', 'dock_plate', 'fence', or a fixture's kind ('column', 'machine', ...).
    ``group`` names the layout object the part belongs to (a row segment, a dock, a fence):
    parts of one object may touch and overlap each other.
    """

    name: str
    kind: str
    group: str
    box: Box
    solid_to: float


@dataclass(frozen=True)
class Recess:
    """
    A doorway set back into a wall: the wall steps out round it.

    ``span`` runs along the wall; ``floor`` is the floor inside the recess, behind the
    hall's inner wall face.
    """

    name: str
    wall: str
    span: tuple
    depth: float
    floor: Box


@dataclass(frozen=True)
class Door:
    """A door panel on a wall's inner face (visual only). ``line`` is its plan-view segment."""

    name: str
    wall: str
    span: tuple
    line: tuple


@dataclass(frozen=True)
class Hall:
    """The hall: the inner faces of its walls (x0, x1, y0, y1) and what is built into them."""

    x0: float
    x1: float
    y0: float
    y1: float
    wall_thickness: float
    wall_height: float
    recesses: tuple
    doors: tuple

    @property
    def box(self):
        """Return the floor inside the walls as a box."""
        return Box(self.x0, self.x1, self.y0, self.y1)

    def contains_point(self, x, y):
        """Return True if (x, y) is on the floor inside the walls."""
        return self.x0 - EPS <= x <= self.x1 + EPS and self.y0 - EPS <= y <= self.y1 + EPS


@dataclass(frozen=True)
class Shelving:
    """The shelf dimensions that every row shares (layout.yaml ``shelving``)."""

    depth: float
    bay_width: float
    upright: float
    plinth_height: float
    board_heights: tuple
    board_thickness: float
    top_height: float
    positions_per_level: int
    guard_depth: float
    guard_height: float

    @property
    def level_heights(self):
        """Return each storage level's surface height: the plinth top, then each board top."""
        return (self.plinth_height,) + self.board_heights


@dataclass(frozen=True)
class Bay:
    """One bay of a row segment: the clear opening between two uprights (y0 to y1)."""

    row: str
    number: int
    y0: float
    y1: float

    @property
    def name(self):
        """Return the bay's label, for example 'C07'."""
        return f'{self.row}{self.number:02d}'

    @property
    def centre_y(self):
        """Return the y of the middle of the opening."""
        return (self.y0 + self.y1) / 2


@dataclass(frozen=True)
class Slot:
    """
    One place for a tote: a position on one level of one bay.

    ``name`` is the slot ID (see SLOT_ID_PATTERN). ``tote`` is the box a tote fills there:
    its bottom on the level's surface, its front ``setback`` behind the shelf face.
    """

    name: str
    row: str
    bay: int
    level: int
    position: int
    tote: Box


@dataclass(frozen=True)
class RowSegment:
    """
    The part of one shelf row inside one block, named '<row>_<block>' (for example 'A_south').

    ``face`` is '+x' or '-x', the direction the shelf face looks. ``box`` is the shelf from
    the floor to its top; the end guards are separate solids.
    """

    name: str
    row: str
    block: str
    face: str
    x_face: float
    box: Box
    bays: tuple


@dataclass(frozen=True)
class AisleSegment:
    """
    One pick aisle inside one block, named 'aisle_<id>_<block>' (for example 'aisle_1_south').

    ``box`` is the floor between the two shelf faces, from end guard to end guard. ``rows``
    is (west row, east row).
    """

    name: str
    aisle: str
    block: str
    rows: tuple
    box: Box

    @property
    def width(self):
        """Return the clear width between the two faces."""
        return self.box.x1 - self.box.x0

    @property
    def centre_x(self):
        """Return the x of the centreline, where the pick poses are."""
        return (self.box.x0 + self.box.x1) / 2


@dataclass(frozen=True)
class Station:
    """A packing station: a counter, solid to the floor, and the drop pose in front of it."""

    name: str
    counter: Box
    drop: Pose


@dataclass(frozen=True)
class Zone:
    """
    A keepout or speed zone (Phase 3 costmap filters), as a polygon.

    ``limit`` is a speed zone's limit in m/s. ``generated`` zones are made from other objects
    (dock contact bars, pick aisles) rather than listed in layout.yaml.
    """

    name: str
    kind: str
    polygon: tuple
    limit: float = None
    generated: bool = False


@dataclass(frozen=True)
class Dock:
    """
    A charging dock against a wall, and the poses a robot uses there (Phase 5).

    The robot docks forwards: in the ``docked`` pose its front touches the contact face, so
    base_link is half the robot's length out from it. Docking starts at ``staging``.
    ``contact`` is the middle of the contact face and ``tag`` the middle of the pedestal face
    (where the dock tag goes in Phase 7), both on the floor. ``keepout`` covers the contact
    bar and the base plate, which are below the lidar's scan plane.
    """

    name: str
    wall: str
    at: float
    pedestal: Box
    contact_bar: Box
    plate: Box
    contact: tuple
    tag: tuple
    docked: Pose
    staging: Pose
    keepout: Zone


@dataclass(frozen=True)
class Spawn:
    """Where a robot is spawned and where it parks. ``pose`` is named 'spawn_<robot>'."""

    robot: str
    pose: Pose


@dataclass(frozen=True)
class Fence:
    """
    A fence along a path of axis-parallel segments, with open gates.

    ``pieces`` are the built fence lines and ``gates`` the openings, each a segment
    ((x0, y0), (x1, y1)). Each piece is also a Solid named '<fence>_<n>'.
    """

    name: str
    path: tuple
    pieces: tuple
    gates: tuple
    thickness: float
    kick_plate_height: float
    height: float


@dataclass(frozen=True)
class Corridor:
    """A place where the fit rules measure a clear width: along ``axis`` through ``at``."""

    name: str
    at: tuple
    axis: str


@dataclass(frozen=True)
class Route:
    """A named list of (x, y) waypoints for the route driver (mapping and test runs)."""

    name: str
    waypoints: tuple

    @property
    def length(self):
        """Return the length of the route in metres."""
        return sum(math.dist(a, b) for a, b in zip(self.waypoints, self.waypoints[1:]))


@dataclass(frozen=True)
class Marking:
    """
    A painted rectangle on the floor (visual only).

    ``kind`` is 'zebra_stripe' or 'aisle_stripe' (filled), or 'parking_bay' or 'dock_bay'
    (painted outline, Layout.line_width wide).
    """

    name: str
    kind: str
    box: Box


@dataclass(frozen=True)
class Tote:
    """The tote size, and how far its front stands behind the shelf face (``setback``)."""

    length: float
    width: float
    height: float
    setback: float


@dataclass(frozen=True)
class Inventory:
    """What the expected inventory is generated from, and how many totes that makes."""

    fill_ratio: float
    skus: int
    tote: Tote
    tote_count: int


@dataclass(frozen=True)
class Style:
    """
    The look of the world and the preview: colours as sRGB '#rrggbb', and switches.

    ``show_totes`` puts the expected inventory's totes on the shelves (Phase 7; bare shelves
    until then). ``fixtures`` maps a fixture kind to its colour.
    """

    show_totes: bool
    wall_band_height: float
    floor: str
    wall: str
    wall_band: str
    pilaster: str
    door: str
    rack_plinth: str
    rack_upright: str
    rack_board: str
    rack_guard: str
    tote: str
    station: str
    dock: str
    contact: str
    dock_plate: str
    fence: str
    keepout_paint: str
    speed_paint: str
    lane_paint: str
    dock_paint: str
    fixtures: tuple

    def fixture(self, kind):
        """Return the colour of a fixture kind."""
        return dict(self.fixtures)[kind]


@dataclass(frozen=True)
class Layout:
    """The warehouse: layout.yaml checked and expanded (see the module docstring)."""

    name: str
    seed: int
    robot: robot_config.RobotConfig
    hall: Hall
    shelving: Shelving
    solids: tuple
    rows: tuple
    aisles: tuple
    slots: tuple
    picks: tuple
    stations: tuple
    docks: tuple
    spawns: tuple
    fences: tuple
    keepouts: tuple
    speed_zones: tuple
    corridors: tuple
    routes: tuple
    markings: tuple
    line_width: float
    inventory: Inventory
    style: Style

    def goals(self):
        """Return the poses robots are sent to: spawns, drops, dock staging poses, picks."""
        return (tuple(spawn.pose for spawn in self.spawns)
                + tuple(station.drop for station in self.stations)
                + tuple(dock.staging for dock in self.docks)
                + self.picks)

    def spawn(self, robot):
        """Return the spawn pose of the robot with this name, for example 'robot1'."""
        for spawn in self.spawns:
            if spawn.robot == robot:
                return spawn.pose
        raise KeyError(f'no spawn for {robot!r}')

    def route(self, name):
        """Return the route with this name, for example 'survey'."""
        for route in self.routes:
            if route.name == name:
                return route
        raise KeyError(f'no route {name!r}')


# ----------------------------------------------------------------------------- loading


def default_path():
    """Return the path of the installed layout.yaml (in the package's share directory)."""
    share = get_package_share_directory('warehouse_layout')
    return os.path.join(share, 'config', 'layout.yaml')


def load(path=None, robot=None):
    """
    Read and check layout.yaml and return the Layout.

    ``path`` defaults to the installed layout.yaml; ``robot`` (a RobotConfig) defaults to
    the installed robot.yaml.
    """
    path = path or default_path()
    with open(path) as stream:
        data = yaml.safe_load(stream)
    return from_dict(data, robot=robot, source=path)


def from_dict(data, robot=None, source='layout.yaml'):
    """Return the Layout for layout.yaml's content (a dict), for example a modified copy."""
    robot = robot or robot_config.load()
    try:
        return _build(data, robot)
    except LayoutError as error:
        raise LayoutError(f'{source}: {error}') from None


# ----------------------------------------------------------------------------- schema reader

_MISSING = object()


class _Section:
    """
    One mapping of layout.yaml, read together with its key path for error messages.

    Use it in a ``with`` block: leaving the block raises LayoutError for any key that was
    never read, so a typo such as ``wall_thikness`` cannot slip through as an unused key.
    """

    def __init__(self, data, path):
        if not isinstance(data, dict):
            raise LayoutError(f'{path or "the file"}: expected a mapping, got {data!r}')
        self.data = data
        self.path = path
        self._used = set()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is None:
            unknown = [key for key in self.data if key not in self._used]
            if unknown:
                raise LayoutError(f'{self.key_path(unknown[0])}: unknown key')
        return False

    def key_path(self, key):
        """Return the dotted path of a key, for example 'hall.wall_height'."""
        return f'{self.path}.{key}' if self.path else str(key)

    def get(self, key, default=_MISSING):
        """Return a key's raw value; a missing key is an error unless a default is given."""
        self._used.add(key)
        if key in self.data:
            return self.data[key]
        if default is _MISSING:
            raise LayoutError(f'{self.key_path(key)}: missing')
        return default

    def number(self, key, positive=False, minimum=None, maximum=None):
        """Return a numeric value as a float (a YAML int or float; true/false is not one)."""
        return _number(self.get(key), self.key_path(key), positive, minimum, maximum)

    def integer(self, key, minimum=None):
        """Return a whole number."""
        value = self.get(key)
        if isinstance(value, bool) or not isinstance(value, int):
            raise LayoutError(f'{self.key_path(key)}: expected a whole number, got {value!r}')
        if minimum is not None and value < minimum:
            raise LayoutError(f'{self.key_path(key)}: must be at least {minimum}')
        return value

    def flag(self, key):
        """Return true or false."""
        value = self.get(key)
        if not isinstance(value, bool):
            raise LayoutError(f'{self.key_path(key)}: expected true or false, got {value!r}')
        return value

    def name(self, key='id'):
        """Return an ID: letters, digits and underscores (a bare number is read as text)."""
        return _name(self.get(key), self.key_path(key))

    def choice(self, key, choices):
        """Return one of the allowed strings."""
        value = self.get(key)
        if value not in choices:
            raise LayoutError(
                f'{self.key_path(key)}: expected one of {", ".join(choices)}, got {value!r}')
        return value

    def colour(self, key):
        """Return an sRGB colour written as '#rrggbb'."""
        value = self.get(key)
        if not isinstance(value, str) or not _COLOUR_PATTERN.match(value):
            raise LayoutError(f"{self.key_path(key)}: expected a colour '#rrggbb', got {value!r}")
        return value

    def pair(self, key, positive=False):
        """Return [low, high] with low < high, as a tuple."""
        path = self.key_path(key)
        low, high = _numbers(self.get(key), path, count=2)
        if not low < high:
            raise LayoutError(f'{path}: expected [low, high] with low < high, got {[low, high]}')
        if positive and low <= 0:
            raise LayoutError(f'{path}: both values must be more than 0')
        return (low, high)

    def point(self, key):
        """Return [x, y] as a tuple."""
        return tuple(_numbers(self.get(key), self.key_path(key), count=2))

    def numbers(self, key, default=_MISSING):
        """Return a list of numbers as a tuple."""
        return tuple(_numbers(self.get(key, default), self.key_path(key)))

    def points(self, key, minimum):
        """Return a list of at least ``minimum`` [x, y] points as a tuple of tuples."""
        path = self.key_path(key)
        value = self.get(key)
        if not isinstance(value, list) or len(value) < minimum:
            raise LayoutError(f'{path}: expected a list of at least {minimum} [x, y] points')
        return tuple(tuple(_numbers(p, f'{path}[{i}]', count=2)) for i, p in enumerate(value))

    def section(self, key, default=_MISSING):
        """Return a nested mapping as a _Section (use it in a ``with`` block)."""
        return _Section(self.get(key, default), self.key_path(key))

    def sections(self, key, default=_MISSING):
        """Return a list of mappings as _Sections, each path naming the item's id."""
        path = self.key_path(key)
        value = self.get(key, default)
        if not isinstance(value, list):
            raise LayoutError(f'{path}: expected a list, got {value!r}')
        items = []
        for index, item in enumerate(value):
            label = item.get('id', index) if isinstance(item, dict) else index
            items.append(_Section(item, f'{path}[{label}]'))
        return items


def _number(value, path, positive=False, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LayoutError(f'{path}: expected a number, got {value!r}')
    if positive and not value > 0:
        raise LayoutError(f'{path}: must be more than 0, got {value!r}')
    if minimum is not None and value < minimum:
        raise LayoutError(f'{path}: must be at least {minimum:g}, got {value!r}')
    if maximum is not None and value > maximum:
        raise LayoutError(f'{path}: must be at most {maximum:g}, got {value!r}')
    return float(value)


def _numbers(value, path, count=None):
    if not isinstance(value, list) or (count is not None and len(value) != count):
        expected = f'a list of {count} numbers' if count else 'a list of numbers'
        raise LayoutError(f'{path}: expected {expected}, got {value!r}')
    return [_number(v, f'{path}[{i}]') for i, v in enumerate(value)]


def _name(value, path):
    if isinstance(value, int) and not isinstance(value, bool):
        value = str(value)
    if not isinstance(value, str) or not _NAME_PATTERN.match(value):
        raise LayoutError(f'{path}: expected an ID of letters, digits and _, got {value!r}')
    return value


def _check_unique(names, what):
    seen = set()
    for name in names:
        if name in seen:
            raise LayoutError(f'{what}: {name!r} is used twice')
        seen.add(name)


# ----------------------------------------------------------------------------- building


def _build(data, robot):
    with _Section(data, '') as root:
        version = root.integer('schema_version')
        if version != SCHEMA_VERSION:
            raise LayoutError(
                f'schema_version: {version} is not supported (this code reads {SCHEMA_VERSION})')
        name = root.name('name')
        seed = root.integer('seed')
        with root.section('hall') as section:
            hall, solids = _read_hall(section)
        with root.section('shelving') as section:
            shelving = _read_shelving(section)
        blocks = _read_blocks(root)
        row_specs = _read_rows(root, blocks)
        aisle_specs = _read_aisles(root, row_specs)
        with root.section('inventory') as section:
            fill_ratio, skus, tote = _read_inventory(section)
        rows, shelf_solids, slots = _build_rows(row_specs, blocks, shelving, tote)
        solids += shelf_solids
        aisles, picks = _build_aisles(aisle_specs, row_specs, rows, blocks, shelving)
        stations = tuple(_read_station(item) for item in root.sections('packing_stations'))
        solids += [Solid(s.name, 'station', s.name, s.counter, s.counter.z1) for s in stations]
        with root.section('docks') as section:
            docks, dock_solids = _read_docks(section, hall, robot)
        solids += dock_solids
        spawns = _read_spawns(root)
        fixtures = [_read_fixture(item) for item in root.sections('fixtures', [])]
        solids += fixtures
        fences = tuple(_read_fence(item) for item in root.sections('fences', []))
        solids += [solid for fence in fences for solid in _fence_solids(fence)]
        with root.section('zones') as section:
            keepouts, speed_zones = _read_zones(section, docks, aisles)
        corridors = tuple(_read_corridor(item) for item in root.sections('corridors', []))
        with root.section('markings') as section:
            line_width, markings = _read_markings(section, hall, robot, spawns, docks, aisles)
        with root.section('style') as section:
            style = _read_style(section)
        routes = _read_routes(root)

    # Rounded half up: 414 slots at 0.5 give 207 totes.
    inventory = Inventory(fill_ratio, skus, tote, math.floor(len(slots) * fill_ratio + 0.5))

    for fixture in fixtures:
        if fixture.kind not in dict(style.fixtures):
            raise LayoutError(
                f'style.fixtures: no colour for kind {fixture.kind!r} (used by {fixture.name})')
    _check_unique([s.name for s in solids], 'object names')
    _check_unique([z.name for z in keepouts + speed_zones], 'zone names')
    _check_unique([c.name for c in corridors], 'corridors')
    _check_unique([p.name for p in picks + tuple(s.pose for s in spawns)
                   + tuple(s.drop for s in stations)
                   + tuple(p for d in docks for p in (d.docked, d.staging))], 'pose names')

    return Layout(
        name=name, seed=seed, robot=robot, hall=hall, shelving=shelving,
        solids=tuple(solids), rows=rows, aisles=aisles, slots=slots, picks=picks,
        stations=stations, docks=docks, spawns=spawns, fences=fences, keepouts=keepouts,
        speed_zones=speed_zones, corridors=corridors, routes=routes, markings=markings,
        line_width=line_width, inventory=inventory, style=style)


# -- the hall, its walls and pilasters


def _wall_box(hall, wall, along, out, zs=(0.0, 0.0)):
    """
    Return a box given in a wall's own coordinates.

    ``along`` is a (start, end) pair along the wall: x on the south and north walls, y on
    the west and east walls. ``out`` is a pair of distances from the wall's inner face into
    the hall (negative values are inside the wall).
    """
    (a0, a1), (o0, o1) = along, out
    if wall == 'south':
        return Box.spanning((a0, a1), (hall.y0 + o0, hall.y0 + o1), zs)
    if wall == 'north':
        return Box.spanning((a0, a1), (hall.y1 - o0, hall.y1 - o1), zs)
    if wall == 'west':
        return Box.spanning((hall.x0 + o0, hall.x0 + o1), (a0, a1), zs)
    return Box.spanning((hall.x1 - o0, hall.x1 - o1), (a0, a1), zs)


def _wall_point(hall, wall, along, out):
    """Return the (x, y) point ``along`` the wall and ``out`` metres from its inner face."""
    box = _wall_box(hall, wall, (along, along), (out, out))
    return (box.x0, box.y0)


def _wall_extent(hall, wall):
    """Return the inner face's (start, end) along a wall."""
    return (hall.x0, hall.x1) if wall in ('south', 'north') else (hall.y0, hall.y1)


def _read_on_wall(item, hall_extents):
    """Read ``wall`` and ``span`` of a recess or door and check the span is on the wall."""
    wall = item.choice('wall', WALLS)
    span = item.pair('span')
    start, end = hall_extents[wall]
    if span[0] < start - EPS or span[1] > end + EPS:
        raise LayoutError(f'{item.key_path("span")}: {list(span)} is not on the {wall} wall '
                          f'({start:g} to {end:g})')
    return wall, span


def _read_hall(section):
    x0, x1 = section.pair('x')
    y0, y1 = section.pair('y')
    thickness = section.number('wall_thickness', positive=True)
    height = section.number('wall_height', positive=True)
    hall = Hall(x0, x1, y0, y1, thickness, height, (), ())
    extents = {wall: _wall_extent(hall, wall) for wall in WALLS}

    recesses = []
    for item in section.sections('recesses', []):
        with item:
            wall, span = _read_on_wall(item, extents)
            depth = item.number('depth', positive=True)
            floor = _wall_box(hall, wall, span, (-depth, 0.0))
            recesses.append(Recess(item.name(), wall, span, depth, floor))
    doors = []
    for item in section.sections('doors', []):
        with item:
            wall, span = _read_on_wall(item, extents)
            line = (_wall_point(hall, wall, span[0], 0.0), _wall_point(hall, wall, span[1], 0.0))
            doors.append(Door(item.name(), wall, span, line))
    _check_unique([r.name for r in recesses] + [d.name for d in doors], 'hall recesses and doors')
    hall = Hall(x0, x1, y0, y1, thickness, height, tuple(recesses), tuple(doors))

    solids = _build_walls(hall)
    with section.section('pilasters', {}) as pilasters:
        if pilasters.data:
            # [along the wall, out into the hall]
            along, into = _numbers(pilasters.get('size'), pilasters.key_path('size'), count=2)
            for wall in WALLS:
                for number, at in enumerate(pilasters.numbers(wall, []), 1):
                    box = _wall_box(hall, wall, (at - along / 2, at + along / 2), (0.0, into),
                                    (0.0, height))
                    solids.append(Solid(f'pilaster_{wall}_{number}', 'pilaster',
                                        f'pilaster_{wall}_{number}', box, height))
    return hall, solids


def _build_walls(hall):
    """Return the walls as solids, stepping out round each recess."""
    t, h = hall.wall_thickness, hall.wall_height
    solids = []
    for wall in WALLS:
        # The south and north walls run on past the corners, so the four walls close the hall.
        start, end = _wall_extent(hall, wall)
        if wall in ('south', 'north'):
            start, end = start - t, end + t
        pieces = []
        for recess in sorted((r for r in hall.recesses if r.wall == wall),
                             key=lambda r: r.span[0]):
            a, b = recess.span
            back = -(recess.depth + t)
            pieces.append((start, a))
            solids += [
                Solid(f'{recess.name}_back', 'wall', 'walls',
                      _wall_box(hall, wall, (a, b), (back, -recess.depth), (0.0, h)), h),
                Solid(f'{recess.name}_side_1', 'wall', 'walls',
                      _wall_box(hall, wall, (a - t, a), (back, -t), (0.0, h)), h),
                Solid(f'{recess.name}_side_2', 'wall', 'walls',
                      _wall_box(hall, wall, (b, b + t), (back, -t), (0.0, h)), h),
            ]
            start = b
        pieces.append((start, end))
        for number, piece in enumerate(pieces, 1):
            name = f'wall_{wall}' if len(pieces) == 1 else f'wall_{wall}_{number}'
            solids.append(Solid(name, 'wall', 'walls',
                                _wall_box(hall, wall, piece, (-t, 0.0), (0.0, h)), h))
    return solids


# -- shelving, rows, aisles and slots


def _read_shelving(section):
    with section.section('end_guard') as guard:
        guard_depth = guard.number('depth', positive=True)
        guard_height = guard.number('height', positive=True)
    shelving = Shelving(
        depth=section.number('depth', positive=True),
        bay_width=section.number('bay_width', positive=True),
        upright=section.number('upright', positive=True),
        plinth_height=section.number('plinth_height', positive=True),
        board_heights=section.numbers('board_heights'),
        board_thickness=section.number('board_thickness', positive=True),
        top_height=section.number('top_height', positive=True),
        positions_per_level=section.integer('positions_per_level', minimum=1),
        guard_depth=guard_depth,
        guard_height=guard_height)
    levels = shelving.level_heights + (shelving.top_height,)
    if any(low >= high for low, high in zip(levels, levels[1:])):
        raise LayoutError('shelving: plinth_height, board_heights and top_height must rise '
                          f'from the floor up, got {list(levels)}')
    if shelving.upright >= shelving.bay_width:
        raise LayoutError('shelving.upright: must be narrower than bay_width')
    return shelving


@dataclass(frozen=True)
class _Block:
    name: str
    y_start: float
    bays: int


@dataclass(frozen=True)
class _RowSpec:
    name: str
    x_face: float
    face: str
    bays: dict


@dataclass(frozen=True)
class _AisleSpec:
    name: str
    rows: tuple


def _read_blocks(root):
    blocks = []
    for item in root.sections('blocks'):
        with item:
            blocks.append(_Block(item.name(), item.number('y_start'),
                                 item.integer('bays', minimum=1)))
    if not blocks:
        raise LayoutError('blocks: needs at least one block')
    _check_unique([b.name for b in blocks], 'blocks')
    return blocks


def _read_rows(root, blocks):
    block_bays = {b.name: b.bays for b in blocks}
    rows = []
    for item in root.sections('rows'):
        with item:
            bays = {}
            with item.section('bays', {}) as overrides:
                for block in list(overrides.data):
                    if block not in block_bays:
                        raise LayoutError(f'{overrides.key_path(block)}: no block {block!r}')
                    count = overrides.integer(block, minimum=1)
                    if count > block_bays[block]:
                        raise LayoutError(f'{overrides.key_path(block)}: {count} bays, but the '
                                          f'{block} block has only {block_bays[block]}')
                    bays[block] = count
            rows.append(_RowSpec(item.name(), item.number('x_face'),
                                 item.choice('face', ('+x', '-x')), bays))
    _check_unique([r.name for r in rows], 'rows')
    return rows


def _read_aisles(root, row_specs):
    rows = {r.name: r for r in row_specs}
    aisles = []
    for item in root.sections('aisles'):
        with item:
            name = item.name()
            path = item.key_path('rows')
            value = item.get('rows')
            if not isinstance(value, list) or len(value) != 2:
                raise LayoutError(f'{path}: expected two row IDs, got {value!r}')
            pair = [_name(v, path) for v in value]
            for row in pair:
                if row not in rows:
                    raise LayoutError(f'{path}: no row {row!r}')
            west, east = sorted(pair, key=lambda r: rows[r].x_face)
            if rows[west].face != '+x' or rows[east].face != '-x' or west == east:
                raise LayoutError(f'{path}: rows {pair[0]} and {pair[1]} do not face each other '
                                  '(the west row must face +x and the east row -x)')
            aisles.append(_AisleSpec(name, (west, east)))
    _check_unique([a.name for a in aisles], 'aisles')
    in_aisles = [row for aisle in aisles for row in aisle.rows]
    _check_unique(in_aisles, 'rows in aisles')
    for row in rows:
        if row not in in_aisles:
            raise LayoutError(
                f'rows[{row}]: not in any aisle; every shelf face needs a pick aisle')
    return aisles


def _build_rows(row_specs, blocks, shelving, tote):
    segments, solids, slots = [], [], []
    first_bay = 1    # bay numbers continue from one block to the next
    for block in blocks:
        for spec in row_specs:
            count = spec.bays.get(block.name, block.bays)
            y0 = block.y_start
            y1 = y0 + count * shelving.bay_width + shelving.upright
            towards_face = 1.0 if spec.face == '+x' else -1.0
            x_back = spec.x_face - towards_face * shelving.depth
            box = Box.spanning((spec.x_face, x_back), (y0, y1), (0.0, shelving.top_height))
            bays = tuple(
                Bay(spec.name, first_bay + i, y0 + i * shelving.bay_width + shelving.upright,
                    y0 + (i + 1) * shelving.bay_width)
                for i in range(count))
            segment = RowSegment(f'{spec.name}_{block.name}', spec.name, block.name, spec.face,
                                 spec.x_face, box, bays)
            segments.append(segment)

            # The shelf is solid only up to its plinth; above it the levels are open.
            solids.append(Solid(f'shelf_{segment.name}', 'shelf', segment.name, box,
                                shelving.plinth_height))
            guard_zs = (0.0, shelving.guard_height)
            for end, (g0, g1) in (('s', (y0 - shelving.guard_depth, y0)),
                                  ('n', (y1, y1 + shelving.guard_depth))):
                solids.append(Solid(f'guard_{segment.name}_{end}', 'guard', segment.name,
                                    Box.spanning((box.x0, box.x1), (g0, g1), guard_zs),
                                    shelving.guard_height))
            slots += _build_slots(segment, shelving, tote)
        first_bay += block.bays
    return tuple(segments), solids, tuple(slots)


def _build_slots(segment, shelving, tote):
    towards_face = 1.0 if segment.face == '+x' else -1.0
    front = segment.x_face - towards_face * tote.setback
    back = front - towards_face * tote.length
    slots = []
    for bay in segment.bays:
        # The positions share the bay's clear opening equally; position 1 is the south one.
        pitch = (bay.y1 - bay.y0) / shelving.positions_per_level
        for level, z in enumerate(shelving.level_heights, 1):
            for position in range(1, shelving.positions_per_level + 1):
                y = bay.y0 + (position - 0.5) * pitch
                box = Box.spanning((front, back), (y - tote.width / 2, y + tote.width / 2),
                                   (z, z + tote.height))
                slots.append(Slot(f'{segment.row}{bay.number:02d}-{level}-{position}',
                                  segment.row, bay.number, level, position, box))
    return slots


def _build_aisles(aisle_specs, row_specs, segments, blocks, shelving):
    x_face = {r.name: r.x_face for r in row_specs}
    aisles, picks = [], []
    for spec in aisle_specs:
        west, east = spec.rows
        for block in blocks:
            pair = [s for s in segments if s.block == block.name and s.row in spec.rows]
            y0 = min(s.box.y0 for s in pair) - shelving.guard_depth
            y1 = max(s.box.y1 for s in pair) + shelving.guard_depth
            aisle = AisleSegment(f'aisle_{spec.name}_{block.name}', spec.name, block.name,
                                 (west, east), Box(x_face[west], x_face[east], y0, y1))
            aisles.append(aisle)
            # One pick pose per bay, on the centreline, facing along the aisle (+y): the
            # Phase 7 side cameras then look at both shelf faces.
            longest = max(pair, key=lambda s: len(s.bays))
            for bay in longest.bays:
                picks.append(Pose(f'pick_{spec.name}_{bay.number:02d}', aisle.centre_x,
                                  bay.centre_y, math.pi / 2))
    return tuple(aisles), tuple(picks)


# -- stations, docks, spawns, fixtures and fences


def _read_pose(section, name):
    with section:
        return Pose(name, section.number('x'), section.number('y'), section.number('yaw'))


def _read_station(item):
    with item:
        name = item.name()
        with item.section('counter') as counter:
            box = Box.spanning(counter.pair('x'), counter.pair('y'),
                               (0.0, counter.number('height', positive=True)))
        drop = _read_pose(item.section('drop'), f'drop_{name}')
    return Station(name, box, drop)


def _read_docks(section, hall, robot):
    with section.section('pedestal') as pedestal:
        pedestal_width = pedestal.number('width', positive=True)
        pedestal_depth = pedestal.number('depth', positive=True)
        pedestal_height = pedestal.number('height', positive=True)
    with section.section('contact_bar') as bar:
        bar_width = bar.number('width', positive=True)
        bar_depth = bar.number('depth', positive=True)
        bar_height = bar.number('height', positive=True)
    plate_height = section.number('plate_height', positive=True)
    tag_setback = section.number('tag_setback', positive=True)
    staging_distance = section.number('staging_distance', positive=True)
    with section.section('keepout_margin') as margin:
        side_margin = margin.number('side', minimum=0.0)
        front_margin = margin.number('front', minimum=0.0)

    docks, solids = [], []
    for item in section.sections('units', []):
        with item:
            name = item.name()
            wall = item.choice('wall', WALLS)
            at = item.number('at')
        # Distances out from the wall: pedestal face, then the contact face tag_setback further.
        contact = pedestal_depth + tag_setback
        pedestal = _wall_box(hall, wall, (at - pedestal_width / 2, at + pedestal_width / 2),
                             (0.0, pedestal_depth), (0.0, pedestal_height))
        contact_bar = _wall_box(hall, wall, (at - bar_width / 2, at + bar_width / 2),
                                (contact - bar_depth, contact), (0.0, bar_height))
        plate = _wall_box(hall, wall, (at - pedestal_width / 2, at + pedestal_width / 2),
                          (pedestal_depth, contact), (0.0, plate_height))
        half = max(pedestal_width, bar_width) / 2 + side_margin
        keepout_box = _wall_box(hall, wall, (at - half, at + half),
                                (pedestal_depth, contact + front_margin))
        # The robot docks forwards: its front touches the contact face, so base_link stands
        # half the robot's length further out.
        docked_out = contact + robot.half_length
        yaw = _YAW_TOWARDS[wall]
        docks.append(Dock(
            name=name, wall=wall, at=at, pedestal=pedestal, contact_bar=contact_bar,
            plate=plate,
            contact=_wall_point(hall, wall, at, contact),
            tag=_wall_point(hall, wall, at, pedestal_depth),
            docked=Pose(f'{name}_docked', *_wall_point(hall, wall, at, docked_out), yaw),
            staging=Pose(f'{name}_staging',
                         *_wall_point(hall, wall, at, docked_out + staging_distance), yaw),
            keepout=Zone(f'{name}_bar_keepout', 'keepout', keepout_box.corners(),
                         generated=True)))
        solids += [Solid(f'{name}_pedestal', 'dock_pedestal', name, pedestal, pedestal_height),
                   Solid(f'{name}_bar', 'contact_bar', name, contact_bar, bar_height),
                   Solid(f'{name}_plate', 'dock_plate', name, plate, plate_height)]
    _check_unique([d.name for d in docks], 'docks.units')
    return tuple(docks), solids


def _read_spawns(root):
    spawns = []
    with root.section('spawns') as section:
        for robot in list(section.data):
            name = _name(robot, section.key_path(robot))
            spawns.append(Spawn(name, _read_pose(section.section(robot), f'spawn_{name}')))
    return tuple(spawns)


def _read_fixture(item):
    with item:
        name = item.name()
        kind = item.name('kind')
        height = item.number('height', positive=True)
        box = Box.spanning(item.pair('x'), item.pair('y'), (0.0, height))
    return Solid(name, kind, name, box, height)


def _read_fence(item):
    with item:
        name = item.name()
        path = item.points('path', minimum=2)
        thickness = item.number('thickness', positive=True)
        kick_plate_height = item.number('kick_plate_height', positive=True)
        height = item.number('height', positive=True)
        gates = {}
        for gate in item.sections('gates', []):
            with gate:
                segment = gate.integer('segment', minimum=0)
                if segment >= len(path) - 1:
                    raise LayoutError(f'{gate.key_path("segment")}: the path has only '
                                      f'{len(path) - 1} segments (numbered from 0)')
                gates.setdefault(segment, []).append(gate.pair('span'))

    pieces, openings = [], []
    for index, (a, b) in enumerate(zip(path, path[1:])):
        if a[0] != b[0] and a[1] != b[1]:
            raise LayoutError(f'fences[{name}].path: segment {index} is not parallel to x or y')
        horizontal = a[1] == b[1]
        axis = 0 if horizontal else 1
        low, high = sorted((a[axis], b[axis]))
        spans = [(low, high)]
        for g0, g1 in sorted(gates.get(index, [])):
            if not low < g0 < g1 < high:
                raise LayoutError(f'fences[{name}].gates: gate {[g0, g1]} is not inside '
                                  f'segment {index} ({low:g} to {high:g})')
            start, end = spans.pop()
            spans += [(start, g0), (g1, end)]
            openings.append((g0, g1, a, horizontal))
        for s0, s1 in spans:
            pieces.append(((s0, a[1]), (s1, a[1])) if horizontal else ((a[0], s0), (a[0], s1)))
    gate_lines = tuple(((g0, a[1]), (g1, a[1])) if horizontal else ((a[0], g0), (a[0], g1))
                       for g0, g1, a, horizontal in openings)
    return Fence(name, path, tuple(pieces), gate_lines, thickness, kick_plate_height, height)


def _fence_solids(fence):
    """Return a fence's pieces as solids: as thick as the fence, solid to the kick plate."""
    solids = []
    half = fence.thickness / 2
    for number, ((x0, y0), (x1, y1)) in enumerate(fence.pieces, 1):
        if y0 == y1:
            box = Box.spanning((x0, x1), (y0 - half, y0 + half), (0.0, fence.height))
        else:
            box = Box.spanning((x0 - half, x0 + half), (y0, y1), (0.0, fence.height))
        solids.append(Solid(f'{fence.name}_{number}', 'fence', fence.name, box,
                            fence.kick_plate_height))
    return solids


# -- zones, corridors, markings, routes, inventory and style


def _read_zones(section, docks, aisles):
    keepouts = []
    for item in section.sections('keepout', []):
        with item:
            keepouts.append(Zone(item.name(), 'keepout', item.points('polygon', minimum=3)))
    keepouts += [dock.keepout for dock in docks]

    speed_zones = []
    for item in section.sections('speed', []):
        with item:
            speed_zones.append(Zone(item.name(), 'speed', item.points('polygon', minimum=3),
                                    limit=item.number('limit', positive=True)))
    aisle_limit = section.number('pick_aisle_speed', positive=True)
    speed_zones += [Zone(aisle.name, 'speed', aisle.box.corners(), limit=aisle_limit,
                         generated=True) for aisle in aisles]
    return tuple(keepouts), tuple(speed_zones)


def _read_corridor(item):
    with item:
        return Corridor(item.name(), item.point('at'), item.choice('axis', ('x', 'y')))


def _read_markings(section, hall, robot, spawns, docks, aisles):
    line_width = section.number('line_width', positive=True)
    margin = section.number('bay_margin', minimum=0.0)
    markings = []
    for item in section.sections('zebras', []):
        with item:
            name = item.name()
            box = Box.spanning(item.pair('x'), item.pair('y'))
            stripe = item.number('stripe', positive=True)
            walk = item.choice('walk', ('x', 'y'))
        markings += _zebra_stripes(name, box, stripe, walk)

    for spawn in spawns:
        # The robot's outline turned to the spawn yaw, boxed, plus the margin.
        pose = spawn.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        corners = [(pose.x + c * x - s * y, pose.y + s * x + c * y)
                   for x, y in robot.footprint_polygon()]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        markings.append(Marking(f'{spawn.robot}_parking_bay', 'parking_bay', Box(
            min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)))

    for dock in docks:
        # From the contact face to the back of a robot standing at the staging pose.
        contact_out = _out_from_wall(hall, dock.wall, dock.contact)
        staging_out = _out_from_wall(hall, dock.wall, (dock.staging.x, dock.staging.y))
        half = robot.half_width + margin
        box = _wall_box(hall, dock.wall, (dock.at - half, dock.at + half),
                        (contact_out, staging_out + robot.half_length + margin))
        markings.append(Marking(f'{dock.name}_bay', 'dock_bay', box))

    for aisle in aisles:
        box = aisle.box
        markings.append(Marking(f'{aisle.name}_stripe_s', 'aisle_stripe',
                                Box(box.x0, box.x1, box.y0 - line_width, box.y0)))
        markings.append(Marking(f'{aisle.name}_stripe_n', 'aisle_stripe',
                                Box(box.x0, box.x1, box.y1, box.y1 + line_width)))
    return line_width, tuple(markings)


def _out_from_wall(hall, wall, point):
    """Return how far a point stands out from a wall's inner face, into the hall."""
    x, y = point
    return {'south': y - hall.y0, 'north': hall.y1 - y,
            'west': x - hall.x0, 'east': hall.x1 - x}[wall]


def _zebra_stripes(name, box, stripe, walk):
    """
    Return the painted bands of a zebra crossing.

    ``stripe`` is the repeat: a band and a gap, each half of it. People walk along ``walk``
    and step over the bands, so the bands run across it; they are centred in the box.
    """
    low, high = (box.y0, box.y1) if walk == 'y' else (box.x0, box.x1)
    count = int((high - low + EPS) // stripe)
    start = low + (high - low - (count * stripe - stripe / 2)) / 2
    bands = []
    for i in range(count):
        b0 = start + i * stripe
        b1 = b0 + stripe / 2
        band = Box(box.x0, box.x1, b0, b1) if walk == 'y' else Box(b0, b1, box.y0, box.y1)
        bands.append(Marking(f'{name}_{i + 1}', 'zebra_stripe', band))
    return bands


def _read_routes(root):
    routes = []
    with root.section('routes', {}) as section:
        for name in list(section.data):
            routes.append(Route(_name(name, section.key_path(name)),
                                section.points(name, minimum=2)))
    return tuple(routes)


def _read_inventory(section):
    fill_ratio = section.number('fill_ratio', minimum=0.0, maximum=1.0)
    skus = section.integer('skus', minimum=1)
    with section.section('tote') as tote:
        size = Tote(tote.number('length', positive=True), tote.number('width', positive=True),
                    tote.number('height', positive=True), tote.number('setback'))
    return fill_ratio, skus, size


def _read_style(section):
    colours = {}
    # Every str field of Style is a colour with a key of the same name in layout.yaml.
    for item in fields(Style):
        if item.type is str:
            colours[item.name] = section.colour(item.name)
    fixtures = []
    with section.section('fixtures') as kinds:
        for kind in list(kinds.data):
            fixtures.append((_name(kind, kinds.key_path(kind)), kinds.colour(kind)))
    return Style(show_totes=section.flag('show_totes'),
                 wall_band_height=section.number('wall_band_height', positive=True),
                 fixtures=tuple(fixtures), **colours)
