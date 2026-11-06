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
Occupancy grid maps in the format that nav2_map_server reads and map_saver_cli writes.

A map is two files: a YAML file that says where the map lies, and an image with one pixel
per cell, top row first::

    image: warehouse.pgm         # next to the YAML file
    mode: trinary                # every cell is free, occupied or unknown
    resolution: 0.05             # metres per cell
    origin: [-2.2, -2.7, 0.0]    # x, y, yaw of the image's lower-left corner in the map frame
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

In memory a map is an OccupancyMap: a numpy array of cells holding the values of
nav_msgs/OccupancyGrid (0 free, 100 occupied, -1 unknown), plus the resolution and the
origin. Row 0 of the array is the map's south edge, as in OccupancyGrid. An image's first row
is its top (north) edge, so the rows are flipped on the way in and out, as map_server does.

How a pixel becomes a cell (nav2_map_server's map_io.cpp, Jazzy): its grey value 0-255 gives a
shade from 0 (black) to 1 (white); the occupancy is 1 - shade, or the shade itself with
``negate: 1``; above ``occupied_thresh`` the cell is occupied, below ``free_thresh`` free, and
unknown in between. In trinary mode map_saver_cli writes occupied cells black (0), free cells
254 and unknown ones 205, with the thresholds 0.65 and 0.196: 205 is an occupancy of 0.19608,
just above 0.196, so it reads back as unknown. (With a free threshold of 0.25 it would read
back as free.) save() writes exactly that.

Pure Python (numpy, Pillow, PyYAML): reading or writing a map needs no ROS graph.
"""

from dataclasses import dataclass
import math
import os

import numpy as np
from PIL import Image
from PIL import UnidentifiedImageError
from warehouse_layout.formatting import yaml_float
from warehouse_layout.formatting import yaml_str
import yaml

# nav_msgs/OccupancyGrid cell values.
FREE = 0
OCCUPIED = 100
UNKNOWN = -1

# What map_saver_cli writes in trinary mode, and so what save() writes.
PIXELS = {FREE: 254, OCCUPIED: 0, UNKNOWN: 205}
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196

# The YAML keys nav2_map_server requires ('mode' is optional and defaults to trinary).
REQUIRED_KEYS = ('image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')


class MapError(ValueError):
    """A map cannot be read. The message names the file and what is wrong with it."""


@dataclass(frozen=True, eq=False)
class OccupancyMap:
    """
    An occupancy grid in the map frame.

    ``cells[row, column]`` holds FREE, OCCUPIED or UNKNOWN; row 0 is the south edge and
    column 0 the west edge. ``origin`` is (x, y, yaw): the pose of the south-west corner of
    cell [0, 0] in the map frame, as in a map YAML file. Cell [r, c] covers the square from
    (c, r) x resolution to (c + 1, r + 1) x resolution in the map's own axes, which the
    origin's yaw turns about that corner.
    """

    cells: np.ndarray
    resolution: float
    origin: tuple

    @property
    def width(self):
        """Return the number of columns (cells from west to east)."""
        return self.cells.shape[1]

    @property
    def height(self):
        """Return the number of rows (cells from south to north)."""
        return self.cells.shape[0]

    def centres(self, value=OCCUPIED):
        """Return the centres of the cells holding ``value``: map-frame (x, y), N x 2."""
        rows, columns = np.nonzero(self.cells == value)
        local = np.column_stack(((columns + 0.5) * self.resolution,
                                 (rows + 0.5) * self.resolution))
        return _rotate(local, self.origin[2]) + self.origin[:2]

    def cell_index(self, points):
        """
        Return the (rows, columns) of the cells holding the map-frame points, as int arrays.

        Points outside the map get indices outside the array; see ``contains``.
        """
        local = _rotate(np.asarray(points, dtype=float) - self.origin[:2], -self.origin[2])
        columns = np.floor(local[:, 0] / self.resolution).astype(int)
        rows = np.floor(local[:, 1] / self.resolution).astype(int)
        return rows, columns

    def contains(self, rows, columns):
        """Return which (row, column) indices lie inside the map, as a boolean array."""
        return (rows >= 0) & (rows < self.height) & (columns >= 0) & (columns < self.width)

    def value_at(self, x, y):
        """Return the value of the cell under the map-frame point (x, y)."""
        rows, columns = self.cell_index([(x, y)])
        if not self.contains(rows, columns)[0]:
            raise IndexError(f'({x}, {y}) is outside the map')
        return int(self.cells[rows[0], columns[0]])


def _rotate(points, yaw):
    """Return N x 2 points turned by ``yaw`` radians about (0, 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]])    # row vectors: p @ R^T


# ----------------------------------------------------------------------------- writing


def image_path(yaml_path):
    """Return the image file that save() writes next to a map YAML file (same name, .pgm)."""
    return os.path.splitext(yaml_path)[0] + '.pgm'


def save(occupancy_map, yaml_path, comment=()):
    """
    Write the map as ``yaml_path`` and its image next to it, as map_saver_cli would.

    ``comment`` lines go at the top of the YAML file (map_server ignores comments). The image
    is a binary PGM: a three-line text header (``P5``, width and height, the largest value
    255), then one byte per cell, the top (north) row first. Same map, same bytes.
    """
    image_file = image_path(yaml_path)
    pixels = np.full(occupancy_map.cells.shape, PIXELS[UNKNOWN], dtype=np.uint8)
    pixels[occupancy_map.cells == FREE] = PIXELS[FREE]
    pixels[occupancy_map.cells == OCCUPIED] = PIXELS[OCCUPIED]
    header = f'P5\n{occupancy_map.width} {occupancy_map.height}\n255\n'.encode('ascii')
    with open(image_file, 'wb') as stream:
        stream.write(header + np.flipud(pixels).tobytes())   # north row first

    x, y, yaw = occupancy_map.origin
    lines = [f'# {line}' if line else '#' for line in comment] + [
        f'image: {yaml_str(os.path.basename(image_file))}',
        'mode: trinary',
        f'resolution: {yaml_float(occupancy_map.resolution)}',
        f'origin: [{yaml_float(x)}, {yaml_float(y)}, {yaml_float(yaw)}]',
        'negate: 0',
        f'occupied_thresh: {yaml_float(OCCUPIED_THRESH)}',
        f'free_thresh: {yaml_float(FREE_THRESH)}',
    ]
    with open(yaml_path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write('\n'.join(lines) + '\n')
    return image_file


# ----------------------------------------------------------------------------- reading


def load(yaml_path):
    """
    Read a map YAML file and its image, as nav2_map_server does, and return the OccupancyMap.

    Raises MapError, naming the file and the problem, if the YAML is malformed, a required
    key is missing or wrong, or the image cannot be read.
    """
    try:
        with open(yaml_path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except OSError as error:
        raise MapError(f'{yaml_path}: {error.strerror}') from None
    except yaml.YAMLError as error:
        raise MapError(f'{yaml_path}: not valid YAML: {error}') from None
    if not isinstance(data, dict):
        raise MapError(f'{yaml_path}: expected the keys of a map file '
                       f'({", ".join(REQUIRED_KEYS)}), found {data!r}')
    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise MapError(f'{yaml_path}: missing key {", ".join(missing)} '
                       f'(a map file needs {", ".join(REQUIRED_KEYS)})')

    def number(key, low, high):
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not (
                low <= value <= high):
            raise MapError(f'{yaml_path}: {key} must be a number from {low:g} to {high:g}, '
                           f'not {value!r}')
        return float(value)

    resolution = number('resolution', 1e-6, 1e6)
    origin = data['origin']
    if (not isinstance(origin, list) or len(origin) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in origin)):
        raise MapError(f'{yaml_path}: origin must be [x, y, yaw] (three numbers), '
                       f'not {origin!r}')
    occupied_thresh = number('occupied_thresh', 0.0, 1.0)
    free_thresh = number('free_thresh', 0.0, 1.0)
    if free_thresh >= occupied_thresh:
        raise MapError(f'{yaml_path}: free_thresh ({free_thresh:g}) must be below '
                       f'occupied_thresh ({occupied_thresh:g})')
    negate = data['negate']
    if negate not in (0, 1):   # map_server reads it as an int, or else as a bool
        raise MapError(f'{yaml_path}: negate must be 0 or 1, not {negate!r}')
    mode = data.get('mode', 'trinary')
    if mode not in ('trinary', 'scale'):
        raise MapError(f'{yaml_path}: mode {mode!r} is not supported here; save the map in '
                       "trinary mode (map_saver_cli's default)")
    name = data['image']
    if not isinstance(name, str) or not name:
        raise MapError(f'{yaml_path}: image must name the image file, not {name!r}')

    # Like map_server: a relative image path is relative to the YAML file's folder.
    path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), name)
    shade = _read_shades(path, yaml_path)
    occupancy = shade if negate else 1.0 - shade
    cells = np.full(shade.shape, UNKNOWN, dtype=np.int8)
    cells[occupancy > occupied_thresh] = OCCUPIED
    cells[occupancy < free_thresh] = FREE
    # In scale mode map_server keeps the in-between values as partly occupied cells; they stay
    # unknown here, because the map tools only use the free and the occupied cells.
    return OccupancyMap(np.flipud(cells).copy(), resolution,
                        tuple(float(v) for v in origin))


def _read_shades(path, yaml_path):
    """Return the image's pixels as shades from 0.0 (black) to 1.0 (white), top row first."""
    try:
        with Image.open(path) as image:
            image.load()
    except FileNotFoundError:
        raise MapError(f'{yaml_path}: its image {path} does not exist') from None
    except (OSError, UnidentifiedImageError) as error:
        raise MapError(f'{yaml_path}: cannot read its image {path}: {error}') from None
    if image.mode == 'L':        # 8-bit grey: PGM, or a grey PNG
        values = np.asarray(image, dtype=float)
    elif image.mode == 'RGB':    # map_server averages the colour channels
        values = np.asarray(image, dtype=float).mean(axis=2)
    else:
        raise MapError(f'{yaml_path}: its image {path} is {image.mode}; use an 8-bit grey or '
                       'RGB image, such as the PGM that map_saver_cli writes')
    return values / 255.0
