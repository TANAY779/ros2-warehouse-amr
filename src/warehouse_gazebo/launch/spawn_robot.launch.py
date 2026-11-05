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
Spawn one robot into a running Gazebo world and bridge its gz topics (robot scope).

Include this file inside the robot's namespace (PushROSNamespace): every name here is
relative, so the robot is read from <namespace>/robot_description and its topics end up
under <namespace>. warehouse_bringup/launch/robot.launch.py does this.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # The URDF published (transient local) by robot_state_publisher.
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [FindPackageShare('warehouse_gazebo'), 'config', 'robot_bridge.yaml']),
            # Resolve the relative gz topics of the config file in this node's namespace.
            'expand_gz_topic_names': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='warehouse_amr',
                              description='Gazebo model name; unique per robot'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x in the world [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y in the world [m]'),
        DeclareLaunchArgument('z', default_value='0.0',
                              description='Spawn height of base_footprint [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Spawn heading in the world [rad]'),
        spawn,
        bridge,
    ])
