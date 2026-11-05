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
Plan-view geometry for the layout: boxes, polygons, distances and rays.

Coordinates are metres in the map frame (x east, y north). Boxes are axis-aligned, which is
all the layout needs: walls, shelf rows and fixtures are parallel to the axes. A polygon is a
sequence of (x, y) corners in order, not closed (the last corner joins the first), and simple
(its edges do not cross).
"""

from dataclasses import dataclass
import math

# Two lengths closer than this are equal. Layout values have millimetre precision, so this
# only absorbs floating-point error (0.1 + 0.2 != 0.3).
EPS = 1e-6


@dataclass(frozen=True)
class Box:
    """An axis-aligned box: x0 <= x <= x1, y0 <= y <= y1 and z0 <= z <= z1."""

    x0: float
    x1: float
    y0: float
    y1: float
    z0: float = 0.0
    z1: float = 0.0

    @classmethod
    def spanning(cls, xs, ys, zs=(0.0, 0.0)):
        """Return the box between two x, two y and two z values, given in any order."""
        return cls(min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    @property
    def centre(self):
        """Return the middle of the box as (x, y, z)."""
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2, (self.z0 + self.z1) / 2)

    @property
    def size(self):
        """Return the box's extent along each axis as (dx, dy, dz)."""
        return (self.x1 - self.x0, self.y1 - self.y0, self.z1 - self.z0)

    def corners(self):
        """Return the plan-view corners, counter-clockwise from the south-west one."""
        return ((self.x0, self.y0), (self.x1, self.y0), (self.x1, self.y1), (self.x0, self.y1))

    def distance_to(self, x, y):
        """Return the plan-view distance from the point (x, y) to the box (0 inside it)."""
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        return math.hypot(dx, dy)

    def overlaps(self, other):
        """Return True if the two plan views share an area. Touching edges do not count."""
        return (self.x0 < other.x1 - EPS and other.x0 < self.x1 - EPS
                and self.y0 < other.y1 - EPS and other.y0 < self.y1 - EPS)

    def contains(self, other):
        """Return True if the other box's plan view lies inside this one (edges may touch)."""
        return (self.x0 <= other.x0 + EPS and other.x1 <= self.x1 + EPS
                and self.y0 <= other.y0 + EPS and other.y1 <= self.y1 + EPS)


def _cross(o, a, b):
    """Return the z component of (a - o) x (b - o): > 0 if o, a, b turn counter-clockwise."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def edges(polygon):
    """Return the polygon's edges as (start, end) pairs, including the closing edge."""
    return list(zip(polygon, list(polygon[1:]) + [polygon[0]]))


def point_in_polygon(x, y, polygon):
    """Return True if (x, y) is strictly inside the polygon (even-odd ray casting)."""
    inside = False
    for (x0, y0), (x1, y1) in edges(polygon):
        if (y0 > y) != (y1 > y):
            x_cross = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < x_cross:
                inside = not inside
    return inside


def distance_to_segment(point, a, b):
    """Return the distance from a point to the segment a-b."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    length_squared = vx * vx + vy * vy
    if length_squared == 0.0:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) / length_squared
    t = max(0.0, min(1.0, t))
    return math.hypot(point[0] - (a[0] + t * vx), point[1] - (a[1] + t * vy))


def distance_to_polygon(x, y, polygon):
    """Return the distance from (x, y) to the polygon's area (0 inside or on an edge)."""
    if point_in_polygon(x, y, polygon):
        return 0.0
    return min(distance_to_segment((x, y), a, b) for a, b in edges(polygon))


def polygon_contains_box(polygon, box):
    """Return True if every corner of the box is inside the polygon or on its edge."""
    return all(distance_to_polygon(x, y, polygon) <= EPS for x, y in box.corners())


def segments_intersect(p, q, a, b):
    """Return True if the segments p-q and a-b cross or touch."""
    d1, d2 = _cross(a, b, p), _cross(a, b, q)
    d3, d4 = _cross(p, q, a), _cross(p, q, b)
    if ((d1 > 0) != (d2 > 0) and d1 != 0 and d2 != 0
            and (d3 > 0) != (d4 > 0) and d3 != 0 and d4 != 0):
        return True
    # Touching or collinear: an end point lies on the other segment.
    return min(distance_to_segment(p, a, b), distance_to_segment(q, a, b),
               distance_to_segment(a, p, q), distance_to_segment(b, p, q)) <= EPS


def segment_distance(p, q, a, b):
    """Return the shortest distance between the segments p-q and a-b."""
    if segments_intersect(p, q, a, b):
        return 0.0
    return min(distance_to_segment(p, a, b), distance_to_segment(q, a, b),
               distance_to_segment(a, p, q), distance_to_segment(b, p, q))


def segment_polygon_distance(p, q, polygon):
    """Return the distance from the segment p-q to the polygon's area (0 if they meet)."""
    if point_in_polygon(p[0], p[1], polygon) or point_in_polygon(q[0], q[1], polygon):
        return 0.0
    return min(segment_distance(p, q, a, b) for a, b in edges(polygon))


def ray_distance(origin, direction, polygon):
    """
    Return how far a ray travels before it reaches the polygon's area.

    ``direction`` is a unit vector. The result is 0 if the ray starts inside the polygon
    or on its edge, and infinity if it misses.
    """
    if distance_to_polygon(origin[0], origin[1], polygon) <= EPS:
        return 0.0
    dx, dy = direction
    best = math.inf
    for a, b in edges(polygon):
        # Solve origin + t * direction = a + s * (b - a) for t >= 0 and 0 <= s <= 1.
        ex, ey = b[0] - a[0], b[1] - a[1]
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-12:
            continue  # parallel: the edges next to this one are hit first
        ax, ay = a[0] - origin[0], a[1] - origin[1]
        t = (ax * ey - ay * ex) / denominator
        s = (ax * dy - ay * dx) / denominator
        if t >= -EPS and -EPS <= s <= 1 + EPS:
            best = min(best, max(t, 0.0))
    return best
