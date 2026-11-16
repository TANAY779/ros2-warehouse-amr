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
Navigate one robot with Nav2, localized by AMCL (plan D20 and P3-D1, the robot scope).

Include this file inside the robot's namespace together with the /tf remaps and use_sim_time,
as warehouse_bringup/launch/robot.launch.py does with navigation:=true, and give it the same
namespace. It starts one process, nav2_container (a component_container_isolated), and loads
into it:

- amcl and lifecycle_manager_localization, through localization.launch.py (container:=),
  starting at x, y and yaw, the robot's spawn pose;
- the ten servers that nav2_bringup's navigation_launch.py always starts (controller,
  planner, smoother, route, behavior, velocity smoother, collision monitor, BT navigator,
  waypoint follower, docking) and lifecycle_manager_navigation, which configures and activates
  them and watches them through bonds.

The isolated container gives every component its own executor thread, so a slow planner
cannot hold up another server's bond heartbeat (ROS 2 skill, lifecycle-components.md §6). One
process for the thirteen nodes (AMCL, the ten servers, two lifecycle managers) also means one DDS
participant instead of thirteen.

Something must serve /map: map_server.launch.py, once for all robots (sim.launch.py starts it).

The parameter file (params_file, default config/nav2_params.yaml) is processed before any node
starts, in two steps:

- file A: <robot_namespace> replaced by the namespace (/robot1), then footprint (robot.yaml's
  outline) and the two behaviour-tree paths written in by their full parameter paths.
  navigation_launch.py gets file A, wraps it under the namespace itself and hands each server
  its section.
- file B: file A wrapped under the namespace (robot1: controller_server: ...). The container
  gets file B on its own command line, with the /tf remaps. The costmaps and the TF listeners
  are nodes that the servers create inside the process, and such nodes see only the process's
  own arguments: without file B both costmaps would run on their defaults.

navigation_launch.py takes use_sim_time, use_composition and namespace from its own arguments,
whose defaults (false, False, none) are wrong here, so they are passed explicitly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from nav2_common.launch import RewrittenYaml
from warehouse_description import robot_config

CONTAINER = 'nav2_container'
# bt_navigator's tree parameters and our trees in behavior_trees/.
TREES = {
    'default_nav_to_pose_bt_xml': 'navigate_to_pose.xml',
    'default_nav_through_poses_bt_xml': 'navigate_through_poses.xml',
}
INITIAL_POSE = ('x', 'y', 'yaw')


def processed_parameters(context, params_file, namespace):
    """
    Return the paths of file A and file B (see the module docstring), both temporary files.

    namespace: the robot's namespace without slashes ('' for none).
    """
    if not os.path.isfile(params_file):
        raise FileNotFoundError(
            f'params_file:={params_file} does not exist. Give a Nav2 parameter file, such as '
            'a copy of warehouse_navigation/config/nav2_params.yaml, or leave params_file out')

    # Each key is a parameter's full path in the file, <node>.ros__parameters.<name>: it
    # rewrites exactly that key, and adds it if the file lacks it. A bare name would rewrite
    # every key of that name, in every section.
    share = get_package_share_directory('warehouse_navigation')
    rewrites = {f'{costmap}.{costmap}.ros__parameters.footprint':
                robot_config.load().nav2_footprint()
                for costmap in ('global_costmap', 'local_costmap')}
    rewrites.update({f'bt_navigator.ros__parameters.{name}':
                     os.path.join(share, 'behavior_trees', tree)
                     for name, tree in TREES.items()})

    # '' without a namespace, so <robot_namespace>/scan becomes /scan, not //scan.
    prefix = f'/{namespace}' if namespace else ''
    file_a = RewrittenYaml(
        source_file=ReplaceString(source_file=params_file,
                                  replacements={'<robot_namespace>': prefix}),
        param_rewrites=rewrites).perform(context)
    # root_key '' leaves the file as it is: plain keys match nodes in /.
    file_b = RewrittenYaml(source_file=file_a, param_rewrites={},
                           root_key=namespace).perform(context)
    return file_a, file_b


def launch_setup(context):
    """Return the container and the two includes that load AMCL and Nav2 into it."""
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    file_a, file_b = processed_parameters(
        context, LaunchConfiguration('params_file').perform(context), namespace)
    # The container's full name, which the loading actions need: /robot1/nav2_container.
    container = f'/{namespace}/{CONTAINER}' if namespace else f'/{CONTAINER}'

    return [
        Node(package='rclcpp_components', executable='component_container_isolated',
             name=CONTAINER, parameters=[file_b],
             # Also set by the including file's SetRemap; written here as well, because this
             # process must have them on its own command line (module docstring).
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('warehouse_navigation'), 'launch',
                 'localization.launch.py'])),
            launch_arguments={'container': container,
                              **{name: LaunchConfiguration(name) for name in INITIAL_POSE}
                              }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'params_file': file_a,
                'use_composition': 'True',
                'container_name': CONTAINER,
            }.items()),
    ]


def generate_launch_description():
    navigation_share = FindPackageShare('warehouse_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description="The robot's namespace, the one the including file pushes"),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([navigation_share, 'config', 'nav2_params.yaml']),
            description="Nav2 parameter file in Nav2's style (plain node keys, "
                        '<robot_namespace>). Default: config/nav2_params.yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Use Gazebo's /clock (Nav2 runs only in Gazebo here)"),
        DeclareLaunchArgument('x', default_value='0.0',
                              description="AMCL's initial x in the map frame [m]"),
        DeclareLaunchArgument('y', default_value='0.0',
                              description="AMCL's initial y in the map frame [m]"),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description="AMCL's initial heading in the map frame [rad]"),
        OpaqueFunction(function=launch_setup),
    ])
