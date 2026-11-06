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
The generated Gazebo world, read back from its SDF text as Gazebo would read it.

A robot hits collisions and the lidar (gpu_lidar) sees visuals, so both are compared with the
layout's solids, which the fit rules checked: one collision box per solid; a visual that fills
each solid's footprint from the floor to above the scan plane; and below 1.20 m no visual that
reaches out of the solids. warehouse_gazebo's test_gazebo_config.py runs `gz sdf -k` on the
installed world and checks its systems.
"""

import dataclasses
from xml.etree import ElementTree

import pytest
from warehouse_description import robot_config
from warehouse_layout import model
from warehouse_layout import rules
from warehouse_layout import world
from warehouse_layout.geometry import Box

# Numbers in the SDF have 4 decimals, so a box read back is within 0.05 mm of the layout's.
TOLERANCE = 1e-4
# Floor paint and door panels are at most this thin: millimetres, below the lidar's noise.
THIN = max(max(world.PAINT_TOP.values()), world.DOOR_PROUD)


@pytest.fixture(scope='module')
def layout():
    return model.load(robot=robot_config.load())


@pytest.fixture(scope='module')
def with_totes(layout):
    """Return the layout with style.show_totes switched on (Phase 7)."""
    return dataclasses.replace(layout, style=dataclasses.replace(layout.style, show_totes=True))


def parts(text):
    """
    Return [(model, kind, name, Box)] for every box visual and collision of a world's SDF.

    The generator keeps every model at the origin and turns only slanted paint lines, so a
    part's box is its pose plus and minus half its size.
    """
    found = []
    for m in ElementTree.fromstring(text).find('world').findall('model'):
        assert m.find('pose') is None, f'{m.get("name")}: a model must stay at the origin'
        for element in m.find('link'):
            if element.tag not in ('visual', 'collision'):
                continue
            size = element.findtext('geometry/box/size')
            if size is None:
                continue   # the ground plane
            x, y, z, roll, pitch, yaw = (float(v) for v in element.findtext('pose').split())
            dx, dy, dz = (float(v) for v in size.split())
            assert roll == pitch == 0.0, element.get('name')
            assert yaw == 0.0, f'{element.get("name")}: turned parts are not expected here'
            box = Box(x - dx / 2, x + dx / 2, y - dy / 2, y + dy / 2, z - dz / 2, z + dz / 2)
            found.append((m.get('name'), element.tag, element.get('name'), box))
    return found


def close(a, b):
    """Return True if two boxes are equal within TOLERANCE."""
    pairs = zip(dataclasses.astuple(a), dataclasses.astuple(b))
    return all(abs(u - v) <= TOLERANCE for u, v in pairs)


def inside(inner, outer):
    """Return True if the plan view of ``inner`` lies inside ``outer`` (edges may touch)."""
    return (outer.x0 - TOLERANCE <= inner.x0 and inner.x1 <= outer.x1 + TOLERANCE
            and outer.y0 - TOLERANCE <= inner.y0 and inner.y1 <= outer.y1 + TOLERANCE)


def is_paint_or_door(layout, model_name, name):
    """Return True for floor paint and door panels: millimetres thin, below the lidar noise."""
    doors = {f'{d.name}_visual' for d in layout.hall.doors + layout.hall.recesses}
    return model_name == 'floor_paint' or (model_name == 'walls' and name in doors)


def test_documented_numbers(layout):
    # The package README quotes these: update them together.
    assert world.summary(world.render(layout)) == '32 models, 461 visuals, 89 collisions'


def test_world_is_named_after_the_layout(layout):
    root = ElementTree.fromstring(world.render(layout))
    assert root.get('version') == '1.9'
    assert root.find('world').get('name') == layout.name == 'warehouse'


def test_every_solid_is_one_collision_box(layout):
    collisions = [(name, box) for m, kind, name, box in parts(world.render(layout))
                  if kind == 'collision']
    assert len(collisions) == len(layout.solids)
    boxes = dict(collisions)
    for solid in layout.solids:
        assert close(boxes[f'{solid.name}_collision'], solid.box), solid.name


def test_the_lidar_sees_every_solid_across_its_whole_footprint(layout):
    # A solid is seen at the scan plane if one visual covers its footprint from the floor to
    # the visible height (0.40 m). The dock contact bars and plates are low on purpose and lie
    # in keepout zones (the visibility fit rule).
    needed = rules.visible_height(layout.robot)
    visuals = [box for m, kind, name, box in parts(world.render(layout)) if kind == 'visual']
    for solid in layout.solids:
        if solid.kind in rules.LOW_KINDS:
            continue
        assert any(inside(solid.box, box) and box.z0 <= TOLERANCE and box.z1 >= needed - TOLERANCE
                   for box in visuals), f'{solid.name}: no visual covers it up to {needed} m'


@pytest.mark.parametrize('totes', [False, True], ids=['bare shelves', 'with totes'])
def test_nothing_reaches_out_of_the_solids_below_the_overhang_limit(layout, with_totes, totes):
    # What the lidar does not see must not stick out where the robot or its load could meet
    # it: below 1.20 m every visual stays inside a solid. Floor paint and door panels are a few
    # millimetres thin, below the lidar's 0.01 m noise.
    current = with_totes if totes else layout
    solids = [solid.box for solid in current.solids]
    for model_name, kind, name, box in parts(world.render(current)):
        if kind != 'visual' or box.z0 >= rules.OVERHANG_LIMIT:
            continue
        if is_paint_or_door(current, model_name, name):
            thinnest = min(box.x1 - box.x0, box.y1 - box.y0, box.z1 - box.z0)
            assert thinnest <= THIN + TOLERANCE, name
            continue
        assert any(inside(box, solid) for solid in solids), f'{model_name}/{name} sticks out'


def test_bare_shelves_until_phase_7(layout):
    assert layout.style.show_totes is False
    names = [name for m, kind, name, box in parts(world.render(layout))]
    assert not [name for name in names if name.startswith('tote_')]


def test_show_totes_puts_the_inventory_on_the_shelves(layout, with_totes):
    totes = [(m, name, box) for m, kind, name, box in parts(world.render(with_totes))
             if name.startswith('tote_')]
    assert len(totes) == layout.inventory.tote_count == 207
    slots = {slot.name: slot for slot in layout.slots}
    for model_name, name, box in totes:
        slot = slots[name[len('tote_'):-len('_visual')]]
        assert close(box, slot.tote), name
        assert model_name.startswith(f'shelf_{slot.row}_'), name


def test_floor_paint_is_thin_and_has_no_collision(layout):
    paint = [(kind, box) for m, kind, name, box in parts(world.render(layout))
             if m == 'floor_paint']
    assert paint
    assert all(kind == 'visual' for kind, box in paint)
    assert all(box.z0 == 0.0 and box.z1 <= max(world.PAINT_TOP.values()) for _, box in paint)


def test_a_slanted_zone_edge_is_painted_as_a_turned_line(layout):
    # Only axis-parallel rectangles get a tint; any polygon gets a border along each edge.
    zone = model.Zone('triangle', 'keepout', ((20.0, 2.0), (22.0, 2.0), (20.0, 3.0)))
    painted = world._zone_paint(zone, '#d03b3b', '#d9d8d3', 0.05)
    assert [part.name for part in painted] == [f'triangle_border_{n}' for n in (1, 2, 3)]
    assert [round(part.yaw, 4) for part in painted] == [0.0, 2.6779, 0.0]


def test_colours_are_written_as_linear_light():
    assert world.linear_rgb('#000000') == (0.0, 0.0, 0.0)
    assert world.linear_rgb('#ffffff') == (1.0, 1.0, 1.0)
    # Measured in Gazebo: a surface emitting 0.5 shows as pixel value 187, so '#bbbbbb'
    # (187) must be written as about 0.5.
    assert world.linear_rgb('#bbbbbb') == pytest.approx((0.497,) * 3, abs=0.001)


def test_names_must_not_clash(layout):
    data = dataclasses.replace(layout, fences=layout.fences + layout.fences)
    with pytest.raises(ValueError, match="two models named 'machinery_fence'"):
        world.models(data)


def test_every_solid_must_be_built(layout):
    unknown = dataclasses.replace(layout.solids[0], name='mystery', kind='wall', group='walls')
    data = dataclasses.replace(layout, solids=layout.solids + (unknown,))
    world.models(data)   # a wall: built
    with pytest.raises(ValueError, match='no collision for mystery'):
        world.models(dataclasses.replace(
            layout, solids=layout.solids + (dataclasses.replace(unknown, kind='guard'),)))
