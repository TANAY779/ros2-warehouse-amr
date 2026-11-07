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
Small helpers for the test tools: controllers.yaml, waiting for ROS, stamps and angles.

They use plain rclpy with spin_once loops. Timeouts are in wall-clock seconds; anything
measured on the robot uses message stamps, so it stays correct in simulation time when Gazebo
runs slower than real time.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml


def description_config(name):
    """
    Return a config file of warehouse_description (robot.yaml, controllers.yaml) as a dict.

    The tools read the robot's numbers (limits, rates) from the files that configure it, so a
    change there reaches them with no number copied into the tools (plan section 6).
    """
    path = os.path.join(get_package_share_directory('warehouse_description'), 'config', name)
    with open(path) as stream:
        return yaml.safe_load(stream)


def diff_drive_parameters():
    """Return the diff_drive_controller parameters from controllers.yaml."""
    controllers = description_config('controllers.yaml')
    return controllers['/**/diff_drive_controller']['ros__parameters']


def spin_for(node, seconds):
    """Process callbacks for ``seconds`` of wall-clock time."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def spin_until(node, predicate, timeout_s):
    """Process callbacks until ``predicate()`` is true; return False after ``timeout_s``."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def duration_text(seconds):
    """Return a duration as text, for example '8 min 05 s' or '45 s'."""
    seconds = int(round(seconds))
    if seconds < 60:
        return f'{seconds} s'
    return f'{seconds // 60} min {seconds % 60:02d} s'


def yaw_of(quaternion):
    """Return the yaw angle of a geometry_msgs Quaternion."""
    q = quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
