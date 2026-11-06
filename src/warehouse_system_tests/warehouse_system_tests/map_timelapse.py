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

r"""
Make a time-lapse GIF of a map being built, from a recorded mapping bag.

Usage::

    ros2 run warehouse_system_tests map_timelapse BAG OUT.gif [--namespace robot1]
        [--map-topic /map] [--frames 60] [--width 640] [--frame-ms 100] [--max-mb 3.0]

Record the map and the ground truth while slam_toolbox maps (plan F5 and F8)::

    ros2 bag record -o bags/mapping --use-sim-time --topics /map \
        /robot1/ground_truth/odom /clock

Each frame shows the newest /map at its moment, coloured as RViz and check_map colour maps
(unknown light grey, free off-white, occupied black), the ground-truth path so far in blue and
the robot in orange. The frames are evenly spaced in sim time (header stamps) from the first
map to the last; the last frame stays on screen for 2 s.

The GIF is written with Pillow: no screen recorder, no ffmpeg, no display. All frames share one
six-colour palette, and Pillow stores of each frame only the rectangle that changed since the
frame before, which keeps the file small. If it would still be larger than --max-mb (3 MB, the
repository's limit for files in docs/media), the command keeps every second frame (each shown
twice as long, so the GIF plays as long as before), then makes the frames narrower, until the
file fits.

Exit codes: 0 written; 1 it cannot be made small enough; 2 the bag cannot be read or has no
map, or the GIF cannot be written.
"""

import argparse
from dataclasses import dataclass
import io
import math
import sys

import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from warehouse_system_tests import bags
from warehouse_system_tests import ros_helpers as rh

# The palette: one index per colour, shared by all frames. Map colours as in check_map's
# overlay; the path and the robot wear categorical slots 1 and 2 of the dataviz palette.
UNKNOWN, FREE, OCCUPIED, PATH, ROBOT, INK_SECONDARY = range(6)
PALETTE = ('#e1e0d9', '#fcfcfb', '#0b0b0b', '#2a78d6', '#eb6834', '#52514e')

# nav_msgs/OccupancyGrid values (0-100, -1 unknown) read with map_saver_cli's default
# thresholds (0.65 and 0.25). slam_toolbox publishes only 0, 100 and -1.
OCCUPIED_FROM = 65
FREE_UP_TO = 25

MARGIN = 0.5                     # m around the largest map and the path
BAND = 20                        # px: the label strip above the map
HOLD_MS = 2000                   # ms the last frame stays on screen
MIN_FRAMES = 8                   # fitting the size limit drops frames down to this many
MIN_WIDTH = 160                  # px, and shrinks the width down to this


class BagError(ValueError):
    """The bag cannot be made into a time-lapse (the message says why)."""


class TooLarge(ValueError):
    """The GIF does not fit the size limit even with the fewest frames and pixels."""


@dataclass(frozen=True, eq=False)
class GridMap:
    """One nav_msgs/OccupancyGrid: cells (row 0 is the south edge), resolution, origin."""

    stamp: float                 # s, the header stamp
    resolution: float            # m per cell
    origin: tuple                # x, y, yaw of the lower-left corner of cell (0, 0)
    cells: np.ndarray            # int8, (height, width)

    @staticmethod
    def from_message(message):
        info = message.info
        cells = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
        origin = info.origin
        return GridMap(rh.stamp_to_seconds(message.header.stamp), info.resolution,
                       (origin.position.x, origin.position.y, rh.yaw_of(origin.orientation)),
                       cells)

    def corners(self):
        """Return the map's four corners in the map frame, as an array (4, 2)."""
        x0, y0, yaw = self.origin
        height, width = self.cells.shape
        local = np.array([(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)])
        local *= self.resolution
        cos, sin = math.cos(yaw), math.sin(yaw)
        return np.column_stack((x0 + cos * local[:, 0] - sin * local[:, 1],
                                y0 + sin * local[:, 0] + cos * local[:, 1]))

    def colours(self, x, y):
        """Return the palette index (UNKNOWN, FREE or OCCUPIED) of the cells under points."""
        x0, y0, yaw = self.origin
        cos, sin = math.cos(yaw), math.sin(yaw)
        dx, dy = x - x0, y - y0
        columns = np.floor((cos * dx + sin * dy) / self.resolution).astype(int)
        rows = np.floor((-sin * dx + cos * dy) / self.resolution).astype(int)
        height, width = self.cells.shape
        inside = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
        values = self.cells[rows[inside], columns[inside]]
        colours = np.full(x.shape, UNKNOWN, dtype=np.uint8)
        colours[inside] = np.where(values >= OCCUPIED_FROM, OCCUPIED,
                                   np.where((values >= 0) & (values <= FREE_UP_TO), FREE, UNKNOWN))
        return colours


@dataclass(frozen=True, eq=False)
class Path:
    """The ground-truth path: stamps [s] and positions [m], in stamp order."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    def travelled(self, t):
        """Return the path length from the first pose up to the time t."""
        steps = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))))
        return float(np.interp(t, self.t, steps)) if len(self.t) else 0.0


def read_mapping(bag, map_topic, truth_topic, frames):
    """
    Read the bag: the maps for ``frames`` evenly spaced moments, and the ground-truth path.

    Returns (moments, maps, path): the frame times, the newest GridMap at each (repeated
    objects where several frames show the same map) and the Path. Two passes over the bag:
    the first reads every map's stamp and the path, the second keeps only the maps shown, so
    a long run never has all its maps in memory.
    """
    available = bags.topic_types(bag)
    if map_topic not in available:
        raise BagError(f'{bag} has no {map_topic} (it has: '
                       f'{", ".join(sorted(available)) or "no topics"})')
    stamps, poses = [], []
    for topic, message, _ in bags.read(bag, {map_topic, truth_topic}):
        stamp = rh.stamp_to_seconds(message.header.stamp)
        if topic == map_topic:
            stamps.append(stamp)
        else:
            pose = message.pose.pose
            poses.append((stamp, pose.position.x, pose.position.y, rh.yaw_of(pose.orientation)))
    if not stamps:
        raise BagError(f'{bag}: {map_topic} has no messages')
    stamps = np.asarray(stamps)
    order = np.argsort(stamps, kind='stable')          # bag order -> stamp order
    first, last = stamps[order[0]], stamps[order[-1]]
    count = 1 if last <= first else max(1, frames)
    moments = np.linspace(first, last, count) if count > 1 else np.array([last])
    # For each moment, the newest map stamped at or before it (its index in bag order).
    newest = order[np.searchsorted(stamps[order], moments, side='right') - 1]
    wanted = set(newest.tolist())
    loaded = {}
    for index, (_, message, _) in enumerate(bags.read(bag, {map_topic})):
        if index in wanted:
            loaded[index] = GridMap.from_message(message)
    poses = np.asarray(sorted(poses), dtype=float).reshape(-1, 4)
    return moments, [loaded[index] for index in newest], Path(*poses.T)


class Canvas:
    """The picture's pixel grid over the map frame: a label strip on top, then the map."""

    def __init__(self, extent, width):
        x_min, y_min, x_max, y_max = extent
        self.scale = width / (x_max - x_min)            # px per m
        self.width = width
        self.map_height = max(1, int(round((y_max - y_min) * self.scale)))
        self.height = self.map_height + BAND
        self.x_min, self.y_max = x_min, y_max
        # The map-frame point at the centre of every pixel of the map area.
        x = x_min + (np.arange(self.width) + 0.5) / self.scale
        y = y_max - (np.arange(self.map_height) + 0.5) / self.scale
        self.x, self.y = np.meshgrid(x, y)

    def pixel(self, x, y):
        """Return the pixel position (column, row) of map-frame points."""
        column = (np.asarray(x) - self.x_min) * self.scale
        row = (self.y_max - np.asarray(y)) * self.scale + BAND
        return column, row


def extent_of(maps, path):
    """Return (x_min, y_min, x_max, y_max) covering every map and the path, plus a margin."""
    points = [grid.corners() for grid in {id(grid): grid for grid in maps}.values()]
    if len(path.t):
        points.append(np.column_stack((path.x, path.y)))
    points = np.concatenate(points)
    low, high = points.min(axis=0) - MARGIN, points.max(axis=0) + MARGIN
    return low[0], low[1], high[0], high[1]


def _font(size):
    try:
        return ImageFont.load_default(size=size)        # FreeType, Pillow 10.1 and newer
    except TypeError:
        return ImageFont.load_default()


def render(canvas, grid, path, moment, start, font):
    """Return one frame (a palette image): the map, the path up to ``moment``, the robot."""
    image = Image.new('P', (canvas.width, canvas.height), FREE)
    image.putpalette([value for colour in PALETTE
                      for value in (int(colour[1:3], 16), int(colour[3:5], 16),
                                    int(colour[5:7], 16))])
    image.paste(Image.fromarray(grid.colours(canvas.x, canvas.y), mode='P'), (0, BAND))
    draw = ImageDraw.Draw(image)
    shown = path.t <= moment
    if np.count_nonzero(shown) >= 2:
        columns, rows = canvas.pixel(path.x[shown], path.y[shown])
        # About one point per pixel of path is enough for a 2 px line. (Measured along the
        # path: two poses 20 ms apart are much less than a pixel apart.)
        along = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(columns), np.diff(rows)))))
        keep = np.append(True, np.diff(np.floor(along)) > 0.0)
        keep[-1] = True
        draw.line(list(zip(columns[keep].tolist(), rows[keep].tolist())), fill=PATH, width=2,
                  joint='curve')
    if len(path.t):
        x = float(np.interp(moment, path.t, path.x))
        y = float(np.interp(moment, path.t, path.y))
        yaw = float(np.interp(moment, path.t, np.unwrap(path.yaw)))
        column, row = canvas.pixel(x, y)
        draw.ellipse((column - 6, row - 6, column + 6, row + 6), fill=FREE)   # a 2 px ring
        draw.ellipse((column - 4, row - 4, column + 4, row + 4), fill=ROBOT)
        draw.line((column, row, column + 10 * math.cos(yaw), row - 10 * math.sin(yaw)),
                  fill=ROBOT, width=2)
    label = (f'{rh.duration_text(moment - start)} of mapping, '
             f'{path.travelled(moment):.0f} m driven')
    draw.text((6, 4), label, fill=OCCUPIED, font=font)
    _legend(draw, canvas.width, 6 + draw.textlength(label, font=font) + 16, font)
    return image


def _legend(draw, width, left_limit, font):
    """Draw the key at the right of the label strip, as many entries as there is room for."""
    entries = (('path', PATH), ('robot', ROBOT), ('occupied', OCCUPIED), ('free', FREE),
               ('unknown', UNKNOWN))
    right = width - 6
    for text, colour in entries:
        size = draw.textlength(text, font=font) + 16
        if right - size < left_limit:
            break
        x = right - size
        if colour == PATH:
            draw.line((x, 10, x + 10, 10), fill=PATH, width=2)
        elif colour == ROBOT:
            draw.ellipse((x + 1, 6, x + 9, 14), fill=ROBOT)
        else:
            draw.rectangle((x + 1, 6, x + 9, 14), fill=colour, outline=INK_SECONDARY)
        draw.text((x + 13, 4), text, fill=INK_SECONDARY, font=font)
        right = x - 10


def encode(frames, frame_ms):
    """Return the GIF file's bytes: every frame for ``frame_ms``, the last for HOLD_MS."""
    buffer = io.BytesIO()
    durations = [int(round(frame_ms))] * (len(frames) - 1) + [HOLD_MS]
    frames[0].save(buffer, format='GIF', save_all=True, append_images=frames[1:],
                   duration=durations, loop=0)
    return buffer.getvalue()


def make_gif(moments, maps, path, width, frame_ms, max_bytes):
    """
    Return (gif bytes, frames, width, height): the time-lapse, fitted under ``max_bytes``.

    Raises TooLarge if even MIN_FRAMES frames MIN_WIDTH pixels wide do not fit.
    """
    extent = extent_of(maps, path)
    chosen = list(range(len(moments)))
    while True:
        canvas = Canvas(extent, width)
        font = _font(12 if width >= 400 else 10)
        frames = [render(canvas, maps[i], path, moments[i], moments[0], font) for i in chosen]
        data = encode(frames, frame_ms)
        if len(data) <= max_bytes:
            return data, len(frames), canvas.width, canvas.height
        if (len(chosen) + 1) // 2 >= MIN_FRAMES:
            # Every second frame, counted back from the last one so that it stays.
            kept = chosen[::-1][::2][::-1]
            frame_ms *= len(chosen) / len(kept)          # the GIF keeps playing as long
            chosen = kept
        elif width > MIN_WIDTH:
            width = max(MIN_WIDTH, int(width * 0.8))
        else:
            raise TooLarge(f'{len(data) / 1e6:.2f} MB with {len(chosen)} frames {width} px wide')


def main(argv=None):
    """Run the map_timelapse command; return its exit code."""
    parser = argparse.ArgumentParser(
        prog='map_timelapse',
        description='Make a time-lapse GIF of a map being built from a recorded bag: the map '
                    'snapshots, the ground-truth path and the robot.')
    parser.add_argument('bag',
                        help='the bag: a folder made by ros2 bag record, or one .mcap file')
    parser.add_argument('out', help='the GIF file to write')
    parser.add_argument('--namespace', default='robot1',
                        help='the robot whose ground truth to draw (default %(default)s)')
    parser.add_argument('--map-topic', default='/map', help='default %(default)s')
    parser.add_argument('--frames', type=int, default=60,
                        help='number of frames, at most (default %(default)s)')
    parser.add_argument('--width', type=int, default=640,
                        help='width of the GIF in pixels, at most (default %(default)s)')
    parser.add_argument('--frame-ms', type=float, default=100.0,
                        help='how long each frame is shown, in ms (default %(default)s)')
    parser.add_argument('--max-mb', type=float, default=3.0,
                        help='largest file size in MB (default %(default)s)')
    args = parser.parse_args(argv)
    if args.frames < 1 or args.width < MIN_WIDTH or args.frame_ms <= 0.0 or args.max_mb <= 0.0:
        parser.error(f'--frames must be at least 1, --width at least {MIN_WIDTH}, --frame-ms and '
                     '--max-mb above 0')

    truth_topic = f'/{args.namespace.strip("/")}/ground_truth/odom'
    try:
        moments, maps, path = read_mapping(args.bag, args.map_topic, truth_topic, args.frames)
    except (OSError, RuntimeError, ValueError) as error:   # BagError is a ValueError
        print(f'error: {error}', file=sys.stderr)
        return 2
    print(f'{args.bag}: {len(moments)} frames from {args.map_topic}, '
          f'{rh.duration_text(moments[-1] - moments[0])} of mapping; {len(path.t)} ground-truth '
          f'poses on {truth_topic}' + ('' if len(path.t) else ' (no path drawn)'))
    try:
        data, frames, width, height = make_gif(moments, maps, path, args.width, args.frame_ms,
                                               args.max_mb * 1e6)
    except TooLarge as error:
        print(f'error: the GIF does not fit in {args.max_mb:g} MB: {error}', file=sys.stderr)
        return 1
    try:
        with open(args.out, 'wb') as stream:
            stream.write(data)
    except OSError as error:
        print(f'error: cannot write {args.out}: {error.strerror}', file=sys.stderr)
        return 2
    print(f'wrote {args.out}: {frames} frames of {width} x {height} px, '
          f'{len(data) / 1e6:.2f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
