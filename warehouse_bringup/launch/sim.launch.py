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
Simulate the warehouse AMR: Gazebo (or mock hardware), one robot and RViz.

  ros2 launch warehouse_bringup sim.launch.py world:=empty namespace:=robot1
  ros2 launch warehouse_bringup sim.launch.py headless:=true     # no windows
  ros2 launch warehouse_bringup sim.launch.py hardware:=mock     # no simulator (CI)

Drive with teleop, stamped and in sim time (see the package README):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import NotSubstitution
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def rviz_setup(context):
    """Return RViz for one robot, in the robot's absolute namespace."""
    # Absolute (/robot1), like robot.launch.py: if this file is included inside another
    # namespace, RViz must stay with the robot at /robot1, not move to /<outer>/robot1.
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    return [
        PushROSNamespace('/' + namespace),
        # The same remaps as the robot's nodes. The config uses relative topic names (scan,
        # odom), which resolve in the namespace.
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('warehouse_bringup'), 'rviz', 'robot.rviz'])],
             output='screen'),
    ]


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    namespace = LaunchConfiguration('namespace')
    headless = LaunchConfiguration('headless')
    bringup_share = FindPackageShare('warehouse_bringup')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='empty',
            description='Gazebo world in warehouse_gazebo/worlds, without .sdf'),
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description='Robot namespace; topics, nodes, TF and gz topics live under it'),
        DeclareLaunchArgument(
            'hardware', default_value='gz',
            description='gz: Gazebo through gz_ros2_control; mock: mock_components, no '
                        'simulator (real is reserved and refused)'),
        # Declared after hardware, because its default depends on it.
        DeclareLaunchArgument(
            'use_sim_time', default_value=EqualsSubstitution(hardware, 'gz'),
            description="Use Gazebo's /clock. Default: true for hardware:=gz, false for mock"),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: no Gazebo window and no RViz (sensors still render)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz (never when headless)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading [rad]'),
    ]

    # World scope: the simulator and its /clock (hardware:=gz only).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('warehouse_gazebo'), 'launch', 'gazebo.launch.py'])),
        condition=IfCondition(EqualsSubstitution(hardware, 'gz')),
        launch_arguments={'world': LaunchConfiguration('world'), 'headless': headless}.items(),
    )

    # Robot scope: everything of one robot, inside its namespace.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'robot.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'hardware': hardware,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # RViz runs per robot. The namespace and remaps that rviz_setup pushes apply only inside
    # this group. rviz_setup is an OpaqueFunction because making the namespace absolute needs
    # its value as a string.
    rviz = GroupAction(
        condition=IfCondition(AndSubstitution(LaunchConfiguration('rviz'),
                                              NotSubstitution(headless))),
        actions=[OpaqueFunction(function=rviz_setup)],
    )

    return LaunchDescription(arguments + [
        # Every node launched below uses one clock: Gazebo's /clock or the wall clock.
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        gazebo,
        robot,
        rviz,
    ])
