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
Localize one robot on a saved map with AMCL (plan F6, the robot scope).

Include this file inside the robot's namespace together with the /tf remaps, as
warehouse_bringup/launch/robot.launch.py does with localization:=amcl. It starts amcl and
lifecycle_manager_localization in that namespace. AMCL reads scan, the robot's tf and the
shared /map, and publishes map -> odom on the robot's tf, amcl_pose and particle_cloud
(config/amcl.yaml). The manager configures and activates amcl and watches it through a bond.

Something must serve /map: map_server.launch.py, once for all robots (sim.launch.py starts it
with localization:=amcl). Until a map arrives, AMCL waits.

x, y and yaw are AMCL's initial pose in the map frame, which is the Gazebo world frame (plan
P2-D2). robot.launch.py passes the robot's spawn pose, so AMCL starts where the robot stands.
To move the estimate later, use RViz's "2D Pose Estimate" tool, which publishes on initialpose.

container chooses how the two nodes run:
- empty (the default): each as a node of its own (two processes), as in Phase 2's
  localization:=amcl.
- the full name of a running component container, for example /robot1/nav2_container: both
  are loaded into it as components. navigation.launch.py does this with navigation:=true, so
  AMCL shares one process with the Nav2 servers (plan P3-D27; the pattern of nav2_bringup's
  localization_launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

INITIAL_POSE = {
    'x': "AMCL's initial x in the map frame [m]: the robot's spawn x",
    'y': "AMCL's initial y in the map frame [m]: the robot's spawn y",
    'yaw': "AMCL's initial heading in the map frame [rad]: the robot's spawn yaw",
}


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('warehouse_navigation'), 'config', 'amcl.yaml'])
    # AMCL declares initial_pose.x, .y and .yaw as doubles and refuses an integer, which an
    # argument such as x:=1 would otherwise become; value_type=float converts it.
    initial_pose = {
        f'initial_pose.{name}': ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in INITIAL_POSE}
    container = LaunchConfiguration('container')
    plain_nodes = IfCondition(EqualsSubstitution(container, ''))

    arguments = [DeclareLaunchArgument(name, default_value='0.0', description=description)
                 for name, description in INITIAL_POSE.items()]
    arguments.append(DeclareLaunchArgument(
        'container', default_value='',
        description='Empty: start amcl and its lifecycle manager as nodes of their own. The '
                    'full name of a running component container (/robot1/nav2_container): '
                    'load both into it'))

    return LaunchDescription(arguments + [
        # The parameters given later win: the launch's initial pose over the file.
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[config, initial_pose], output='screen', condition=plain_nodes),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', parameters=[config], output='screen',
             condition=plain_nodes),
        # The same two nodes as components, with the same parameters. They get the namespace,
        # the /tf remaps and use_sim_time from the including file, as the plain nodes do.
        LoadComposableNodes(
            target_container=container,
            condition=UnlessCondition(EqualsSubstitution(container, '')),
            composable_node_descriptions=[
                ComposableNode(package='nav2_amcl', plugin='nav2_amcl::AmclNode', name='amcl',
                               parameters=[config, initial_pose]),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_localization', parameters=[config]),
            ]),
    ])
