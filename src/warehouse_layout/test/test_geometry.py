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

"""The plan-view geometry the fit rules measure with, on cases easy to check by hand."""

import math

import pytest
from warehouse_layout.geometry import Box
from warehouse_layout.geometry import distance_to_polygon
from warehouse_layout.geometry import point_in_polygon
from warehouse_layout.geometry import polygon_contains_box
from warehouse_layout.geometry import ray_distance
from warehouse_layout.geometry import segment_distance
from warehouse_layout.geometry import segment_polygon_distance

SQUARE = ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))


def test_box_distance_overlap_and_contains():
    box = Box.spanning((2.0, 0.0), (0.0, 1.0))          # corners in any order
    assert (box.x0, box.x1, box.y0, box.y1) == (0.0, 2.0, 0.0, 1.0)
    assert box.distance_to(1.0, 0.5) == 0.0             # inside
    assert box.distance_to(3.0, 0.5) == pytest.approx(1.0)
    assert box.distance_to(5.0, 5.0) == pytest.approx(5.0)   # to the corner (2, 1)
    assert box.overlaps(Box(1.0, 3.0, 0.5, 2.0))
    assert not box.overlaps(Box(2.0, 3.0, 0.0, 1.0))    # touching is not overlapping
    assert box.contains(Box(0.0, 2.0, 0.0, 0.5))        # edges may touch
    assert not box.contains(Box(0.0, 2.1, 0.0, 0.5))


def test_point_in_and_distance_to_polygon():
    assert point_in_polygon(1.0, 1.0, SQUARE)
    assert not point_in_polygon(3.0, 1.0, SQUARE)
    assert distance_to_polygon(1.0, 1.0, SQUARE) == 0.0
    assert distance_to_polygon(3.0, 1.0, SQUARE) == pytest.approx(1.0)
    assert distance_to_polygon(5.0, 6.0, SQUARE) == pytest.approx(5.0)


def test_polygon_contains_box():
    assert polygon_contains_box(SQUARE, Box(0.0, 2.0, 0.5, 1.0))   # edges on the boundary
    assert not polygon_contains_box(SQUARE, Box(1.5, 2.5, 0.5, 1.0))


def test_segment_distances():
    assert segment_distance((0, 0), (2, 2), (0, 2), (2, 0)) == 0.0    # crossing
    assert segment_distance((0, 0), (2, 0), (0, 1), (2, 1)) == pytest.approx(1.0)  # parallel
    assert segment_distance((0, 0), (1, 0), (2, 0), (3, 0)) == pytest.approx(1.0)  # in line
    assert segment_polygon_distance((-1, 1), (3, 1), SQUARE) == 0.0   # through it
    assert segment_polygon_distance((3, 0), (3, 2), SQUARE) == pytest.approx(1.0)


def test_ray_distance():
    assert ray_distance((-1.0, 1.0), (1.0, 0.0), SQUARE) == pytest.approx(1.0)
    assert ray_distance((1.0, 3.0), (0.0, -1.0), SQUARE) == pytest.approx(1.0)
    assert ray_distance((1.0, 1.0), (1.0, 0.0), SQUARE) == 0.0         # starts inside
    assert math.isinf(ray_distance((-1.0, 1.0), (-1.0, 0.0), SQUARE))  # points away
    assert ray_distance((-1.0, 2.0), (1.0, 0.0), SQUARE) == pytest.approx(1.0)  # grazes an edge
