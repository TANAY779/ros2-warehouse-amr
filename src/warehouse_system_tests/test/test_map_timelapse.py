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
map_timelapse on a small synthetic mapping bag.

Four /map messages, 2 s apart, grow from 3 x 2 m to 6 x 3 m; from the third one on they hold
a wall along y = 1.5 m. Meanwhile the robot drives east from (0, 0) to (3, 0), then north to
(3, 1). The GIF must show the wall only once it is mapped, the whole path (both legs), and
fit the size limit.
"""

import math

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image
import pytest
from warehouse_system_tests import bags
from warehouse_system_tests import map_timelapse as mt

RESOLUTION = 0.05
ORIGIN = (-1.0, -1.0)
MAPS = ((10.0, 3.0, 2.0), (12.0, 4.0, 2.5), (14.0, 5.0, 3.0), (16.0, 6.0, 3.0))  # t, w, h
WALL_Y = 1.5                     # m: occupied row in the maps at least 2.5 m high


def stamp(t):
    nanoseconds = int(round(t * 1e9))
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def grid(t, width_m, height_m):
    message = OccupancyGrid()
    message.header.stamp = stamp(t)
    message.header.frame_id = 'map'
    message.info.resolution = RESOLUTION
    message.info.width = int(round(width_m / RESOLUTION))
    message.info.height = int(round(height_m / RESOLUTION))
    message.info.origin.position.x, message.info.origin.position.y = ORIGIN
    message.info.origin.orientation.w = 1.0
    cells = np.zeros((message.info.height, message.info.width), dtype=np.int8)   # free
    cells[:, -10:] = -1                                     # unknown along the east edge
    wall_row = int((WALL_Y - ORIGIN[1]) / RESOLUTION)
    if wall_row < message.info.height:
        cells[wall_row, :] = 100
    message.data = cells.ravel().tolist()
    return message


def truth(t):
    """Return the ground truth at t: east at 0.75 m/s from 10 to 14 s, then north."""
    message = Odometry()
    message.header.stamp = stamp(t)
    message.header.frame_id = 'map'
    if t <= 14.0:
        message.pose.pose.position.x = 0.75 * (t - 10.0)
        message.pose.pose.orientation.w = 1.0
    else:
        message.pose.pose.position.x = 3.0
        message.pose.pose.position.y = 0.5 * (t - 14.0)
        message.pose.pose.orientation.z = message.pose.pose.orientation.w = math.sqrt(0.5)
    return message


@pytest.fixture(scope='module')
def bag(tmp_path_factory):
    path = tmp_path_factory.mktemp('bags') / 'mapping'
    messages = [(t, '/map', grid(t, w, h)) for t, w, h in MAPS]
    messages += [(t, '/robot1/ground_truth/odom', truth(t)) for t in np.arange(10.0, 16.01, 0.02)]
    messages.sort(key=lambda m: m[0])
    bags.write(path, ((topic, message, int(round(t * 1e9))) for t, topic, message in messages))
    return path


def colour_at(image, canvas_size, extent, x, y):
    """Return the RGB colour of the pixel at map-frame point (x, y)."""
    width, height = canvas_size
    x_min, _, x_max, y_max = extent
    scale = width / (x_max - x_min)
    return image.getpixel((int((x - x_min) * scale), int((y_max - y) * scale) + mt.BAND))


def rgb(colour):
    return tuple(int(colour[i:i + 2], 16) for i in (1, 3, 5))


def test_gif_shows_the_map_growing_and_the_path(bag, tmp_path, capsys):
    out = tmp_path / 'mapping.gif'
    assert mt.main([str(bag), str(out), '--frames', '8', '--width', '240']) == 0
    assert 'wrote' in capsys.readouterr().out
    assert out.stat().st_size < 3e6
    extent = (-1.5, -1.5, 5.5, 2.5)          # the largest map and the path, plus 0.5 m
    with Image.open(out) as gif:
        assert gif.n_frames == 8
        expected_height = mt.BAND + round((extent[3] - extent[1]) * 240 / (extent[2] - extent[0]))
        assert gif.size == (240, expected_height)
        first = gif.convert('RGB')
        gif.seek(gif.n_frames - 1)
        last = gif.convert('RGB')
    wall = (2.0, WALL_Y + RESOLUTION / 2)
    assert colour_at(last, (240, 0), extent, *wall) == rgb(mt.PALETTE[mt.OCCUPIED])
    assert colour_at(first, (240, 0), extent, *wall) != rgb(mt.PALETTE[mt.OCCUPIED])
    assert colour_at(last, (240, 0), extent, 1.0, 0.8) == rgb(mt.PALETTE[mt.FREE])
    # Both legs of the path, not a shortcut from its start to the robot.
    assert colour_at(last, (240, 0), extent, 1.5, 0.0) == rgb(mt.PALETTE[mt.PATH])
    assert colour_at(last, (240, 0), extent, 3.0, 0.5) == rgb(mt.PALETTE[mt.PATH])
    assert colour_at(last, (240, 0), extent, 1.5, 0.5) == rgb(mt.PALETTE[mt.FREE])
    assert colour_at(last, (240, 0), extent, 3.0, 1.0) == rgb(mt.PALETTE[mt.ROBOT])
    assert colour_at(last, (240, 0), extent, 4.75, 1.0) == rgb(mt.PALETTE[mt.UNKNOWN])  # a cell
    assert colour_at(last, (240, 0), extent, 5.3, 1.0) == rgb(mt.PALETTE[mt.UNKNOWN])   # no map


def test_the_size_limit_drops_frames_then_pixels(bag):
    moments, maps, path = mt.read_mapping(bag, '/map', '/robot1/ground_truth/odom', 16)
    full, frames, width, _ = mt.make_gif(moments, maps, path, 400, 100.0, 3e6)
    assert (frames, width) == (16, 400)
    # Just too large for 16 frames: every second frame goes, the width stays.
    data, frames, width, _ = mt.make_gif(moments, maps, path, 400, 100.0, len(full) - 1)
    assert (frames, width) == (8, 400) and len(data) < len(full)
    # Smaller still: MIN_FRAMES frames, narrower.
    data, frames, width, _ = mt.make_gif(moments, maps, path, 400, 100.0, len(data) - 1)
    assert frames == mt.MIN_FRAMES and width < 400
    with pytest.raises(mt.TooLarge):
        mt.make_gif(moments, maps, path, 400, 100.0, 100)


def test_too_large_exits_with_1_and_a_bag_without_map_with_2(bag, tmp_path, capsys):
    assert mt.main([str(bag), str(tmp_path / 'x.gif'), '--max-mb', '0.0001']) == 1
    assert 'does not fit' in capsys.readouterr().err
    assert mt.main([str(bag), str(tmp_path / 'y.gif'), '--map-topic', '/nothing']) == 2
    assert 'has no /nothing' in capsys.readouterr().err


def test_a_turned_map_is_drawn_where_it_lies():
    # One row of 4 cells, turned a quarter turn: it runs north from its origin (1, 1).
    turned = mt.GridMap(0.0, 1.0, (1.0, 1.0, math.pi / 2),
                        np.array([[0, 100, -1, 0]], dtype=np.int8))
    x = np.array([0.5, 0.5, 0.5, 0.5, 1.5])
    y = np.array([1.5, 2.5, 3.5, 4.5, 1.5])
    assert turned.colours(x, y).tolist() == [mt.FREE, mt.OCCUPIED, mt.UNKNOWN, mt.FREE,
                                             mt.UNKNOWN]
    assert turned.corners() == pytest.approx(np.array([(1, 1), (1, 5), (0, 5), (0, 1)]))
