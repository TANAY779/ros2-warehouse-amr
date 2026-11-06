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
Serve a saved map on /map: map_server and its lifecycle manager (plan F6, the system scope).

Start it once for all robots, at the top level (in /), as sim.launch.py does with
localization:=amcl. map_server loads the map named by the map argument (a map_server YAML file
next to its image, such as maps/warehouse.yaml) and publishes it on /map, reliable and transient
local: AMCL, RViz and map_saver_cli receive it whenever they start (config/map_server.yaml).
lifecycle_manager_map configures and activates map_server and watches it through a bond.

  ros2 launch warehouse_navigation map_server.launch.py map:=/tmp/layout_map.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def check_map_file(context):
    """Stop the launch before anything starts if the map file does not exist."""
    path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(path):
        # map_server would fail to configure, and its manager would give up, with errors that
        # bury the cause.
        raise FileNotFoundError(
            f'map:={path} does not exist. Save the warehouse map first '
            '(warehouse_navigation/maps/README.md), or draw one from the layout: '
            '`ros2 run warehouse_layout rasterize --out /tmp/layout_map`, then '
            'map:=/tmp/layout_map.yaml')
    return []


def generate_launch_description():
    navigation_share = FindPackageShare('warehouse_navigation')
    config = PathJoinSubstitution([navigation_share, 'config', 'map_server.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution(
                [navigation_share, 'maps', 'warehouse.yaml']),
            description='Map to serve on /map: a map_server YAML file. Default: the '
                        'warehouse map saved from slam_toolbox'),
        OpaqueFunction(function=check_map_file),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             # value_type=str: a path stays a string even if it looks like a number.
             parameters=[config, {'yaml_filename': ParameterValue(
                 LaunchConfiguration('map'), value_type=str)}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[config], output='screen'),
    ])
