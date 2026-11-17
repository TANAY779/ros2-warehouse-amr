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
Distances between many points (or robot outlines) and the layout's zone polygons, with numpy.

warehouse_layout.geometry answers the same questions for one point at a time. The test tools
ask them for every cell of a map (goal_set) or every ground-truth pose of a drive
(evaluate_navigation), tens of thousands at once, so these versions take arrays.

A polygon is a sequence of (x, y) corners, as the zones of layout.yaml are written. Inside
means strictly inside by the even-odd rule, as geometry.point_in_polygon decides it.
"""

import numpy as np

# The margins round each keepout zone that Phase 3b's masks will use (plan section 3.3). The
# planning core: padded circumscribed radius 0.456 + controller margin 0.10 + one cell
# diagonal 0.071 = 0.627 m, rounded up to 0.65 m. Beyond it a cost ramp of 0.30 m.
PLANNING_CORE = 0.65            # m
PLANNING_RAMP = 0.30            # m


def _edges(polygon):
    """Return the polygon's edges as ((x0, y0), (x1, y1)) pairs, the closing edge included."""
    corners = [tuple(map(float, corner)) for corner in polygon]
    return list(zip(corners, corners[1:] + corners[:1]))


def inside_polygon(xs, ys, polygon):
    """Return which points (arrays xs, ys) lie strictly inside the polygon (even-odd rule)."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    inside = np.zeros(xs.shape, dtype=bool)
    for (x0, y0), (x1, y1) in _edges(polygon):
        crosses = (y0 > ys) != (y1 > ys)
        if not crosses.any():
            continue
        # Where the edge crosses the horizontal line through the point (only used where it
        # does cross, so y1 != y0 there).
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = x0 + (ys - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (xs < x_cross)
    return inside


def _segment_distance(xs, ys, a, b):
    """Return the distance from each point to the segment a-b."""
    (ax, ay), (bx, by) = a, b
    vx, vy = bx - ax, by - ay
    length_squared = vx * vx + vy * vy
    if length_squared == 0.0:
        return np.hypot(xs - ax, ys - ay)
    t = np.clip(((xs - ax) * vx + (ys - ay) * vy) / length_squared, 0.0, 1.0)
    return np.hypot(xs - (ax + t * vx), ys - (ay + t * vy))


def polygon_distance(xs, ys, polygon):
    """Return the distance from each point to the polygon's area: 0 inside or on an edge."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    distance = np.full(xs.shape, np.inf)
    for a, b in _edges(polygon):
        distance = np.minimum(distance, _segment_distance(xs, ys, a, b))
    return np.where(inside_polygon(xs, ys, polygon), 0.0, distance)


def outline_corners(xs, ys, yaws, half_length, half_width):
    """
    Return the corners of the robot's outline at each pose, as an (N, 4, 2) array.

    The outline is the rectangle +-half_length along the robot's x axis and +-half_width
    along its y axis, centred on base_footprint (robot.yaml's footprint, unpadded).
    """
    xs, ys, yaws = (np.asarray(v, dtype=float).reshape(-1) for v in (xs, ys, yaws))
    local = np.array([[half_length, half_width], [-half_length, half_width],
                      [-half_length, -half_width], [half_length, -half_width]])
    cos, sin = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
    corner_x = xs[:, None] + cos * local[:, 0] - sin * local[:, 1]
    corner_y = ys[:, None] + sin * local[:, 0] + cos * local[:, 1]
    return np.stack((corner_x, corner_y), axis=2)


def _cross(ox, oy, ax, ay, bx, by):
    """Return (a - o) x (b - o) for arrays or numbers: > 0 if o, a, b turn left."""
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)


def outline_clearance(xs, ys, yaws, half_length, half_width, polygon):
    """
    Return, per pose, the distance between the robot's outline and the polygon's area.

    0 where they overlap: a corner of one inside the other, or two edges crossing (a robot
    lying across a narrow zone, like a dock bar's box, has neither corner inside). Otherwise the
    smallest distance between them, which lies between a corner of one and an edge of the
    other.
    """
    corners = outline_corners(xs, ys, yaws, half_length, half_width)
    count = corners.shape[0]
    if count == 0:
        return np.zeros(0)
    cx, cy = corners[:, :, 0], corners[:, :, 1]                  # (N, 4)
    zone = [tuple(map(float, corner)) for corner in polygon]
    # Corners of the outline against the zone's edges.
    clearance = polygon_distance(cx.reshape(-1), cy.reshape(-1), zone).reshape(count, 4).min(1)
    # Corners of the zone against the outline's edges, and inside the outline.
    xs, ys, yaws = (np.asarray(v, dtype=float).reshape(-1) for v in (xs, ys, yaws))
    cos, sin = np.cos(yaws), np.sin(yaws)
    zone_inside = np.zeros(count, dtype=bool)
    for zx, zy in zone:
        # The zone's corner in each robot's own frame.
        dx, dy = zx - xs, zy - ys
        forward, left = cos * dx + sin * dy, -sin * dx + cos * dy
        zone_inside |= (np.abs(forward) < half_length) & (np.abs(left) < half_width)
        for k in range(4):
            a, b = corners[:, k, :], corners[:, (k + 1) % 4, :]
            vx, vy = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]
            t = np.clip(((zx - a[:, 0]) * vx + (zy - a[:, 1]) * vy) / (vx * vx + vy * vy),
                        0.0, 1.0)
            clearance = np.minimum(
                clearance, np.hypot(zx - (a[:, 0] + t * vx), zy - (a[:, 1] + t * vy)))
    # Edges crossing each other, with no corner of either inside the other.
    crossing = np.zeros(count, dtype=bool)
    for (ax, ay), (bx, by) in _edges(zone):
        for k in range(4):
            px, py = cx[:, k], cy[:, k]
            qx, qy = cx[:, (k + 1) % 4], cy[:, (k + 1) % 4]
            d1, d2 = _cross(ax, ay, bx, by, px, py), _cross(ax, ay, bx, by, qx, qy)
            d3, d4 = _cross(px, py, qx, qy, ax, ay), _cross(px, py, qx, qy, bx, by)
            crossing |= (d1 * d2 < 0.0) & (d3 * d4 < 0.0)
    return np.where(zone_inside | crossing, 0.0, clearance)
