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
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse   # the generated warehouse
  ros2 launch warehouse_bringup sim.launch.py headless:=true     # no windows
  ros2 launch warehouse_bringup sim.launch.py gz_gui:=false      # RViz, no Gazebo window
  ros2 launch warehouse_bringup sim.launch.py physics_step:=0.002   # 2 ms physics steps
  ros2 launch warehouse_bringup sim.launch.py hardware:=mock     # no simulator (CI)
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam   # make a map
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl   # use it

localization:=slam runs slam_toolbox in the robot's namespace; localization:=amcl runs AMCL
there and the map server, which serves the map:= file on /map for all robots, in the system
scope (plan F5, F6). The two never run together: each publishes map -> odom.

gz_gui and physics_step go to warehouse_gazebo's gazebo.launch.py. gz_gui:=false runs the
Gazebo server without its window and keeps RViz (the window costs more real time than RViz,
docs/research/PC_ASSESSMENT.md section 5.7). physics_step:=0.001, 0.002 or 0.005 runs a copy of
the world with that step; any other value stops the launch before Gazebo starts.

The robot spawns at x, y and yaw where they are given. The others come from the robot's
spawn pose in the location registry (warehouse_layout/generated/locations.yaml) when the
registry was generated for this world, and are 0 in any other world (spawn_pose below).

Drive with teleop, stamped and in sim time (see the package README):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
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
import yaml

SPAWN_ARGUMENTS = ('x', 'y', 'yaw')


def spawn_pose(world, robot, given, registry):
    """
    Return the robot's spawn pose {'x': .., 'y': .., 'yaw': ..} and where it came from.

    world: the world name; robot: the namespace without slashes ('' for none).
    given: the x, y and yaw launch arguments as strings, '' for one that was not given.
    registry: the location registry (locations.yaml) as a dict.

    A value that was given wins. The others come from the robot's spawn pose in the registry
    if the registry was generated for this world, and are 0 otherwise. The registry is in the
    map frame, which is the Gazebo world frame (plan P2-D2), so its poses go to Gazebo as
    they are.
    """
    pose = {name: 0.0 for name in SPAWN_ARGUMENTS}
    source = f'the origin, since the registry is for the {registry["world"]} world'
    if registry['world'] == world and not all(given.values()):
        spawns = {entry['robot']: (location, entry)
                  for location, entry in registry['poses'].items() if entry['kind'] == 'spawn'}
        if robot not in spawns:
            # Not the origin instead: another robot may stand there.
            raise ValueError(
                f'The location registry has no spawn pose for the namespace /{robot} in the '
                f'{world} world, only for {", ".join(sorted(spawns))}. Use one of those, or '
                'give x, y and yaw.')
        location, entry = spawns[robot]
        pose = {name: float(entry[name]) for name in SPAWN_ARGUMENTS}
        source = f'its registry pose {location}'

    named = [name for name in SPAWN_ARGUMENTS if given[name]]
    for name in named:
        pose[name] = _number(name, given[name])
    if len(named) == len(SPAWN_ARGUMENTS):
        source = 'the launch arguments'
    elif named:
        source = f'{", ".join(named)} from the launch arguments, the rest: {source}'
    return pose, source


def _number(name, text):
    """Return a launch argument as a finite float, or raise a ValueError naming it."""
    try:
        value = float(text)
    except ValueError:
        value = math.nan
    if not math.isfinite(value):
        raise ValueError(f'{name}:={text} is not a number')
    return value


def resolve_spawn_pose(context):
    """Fill in the x, y and yaw arguments that were not given (see spawn_pose)."""
    # Mock hardware spawns no model: x, y and yaw stay unused, and no registry is needed.
    if LaunchConfiguration('hardware').perform(context) != 'gz':
        return []
    robot = LaunchConfiguration('namespace').perform(context).strip('/')
    given = {name: LaunchConfiguration(name).perform(context).strip()
             for name in SPAWN_ARGUMENTS}
    # The registry is a plain YAML file installed by warehouse_layout (an exec_depend).
    registry_file = os.path.join(get_package_share_directory('warehouse_layout'), 'generated',
                                 'locations.yaml')
    with open(registry_file) as stream:
        registry = yaml.safe_load(stream)
    pose, source = spawn_pose(LaunchConfiguration('world').perform(context), robot, given,
                              registry)
    return [SetLaunchConfiguration(name, str(value)) for name, value in pose.items()] + [
        LogInfo(msg=f'/{robot} spawns at x {pose["x"]:g} m, y {pose["y"]:g} m, '
                    f'yaw {pose["yaw"]:g} rad ({source})')]


def check_map(context):
    """Stop the launch before Gazebo starts if localization:=amcl has no map file to serve."""
    # Mock hardware: robot.launch.py refuses any localization, before it starts anything.
    if (LaunchConfiguration('localization').perform(context) != 'amcl'
            or LaunchConfiguration('hardware').perform(context) != 'gz'):
        return []
    path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(path):
        # map_server.launch.py checks this too, but only once Gazebo is running, and an
        # error raised while gz sim is starting leaves the Gazebo server running.
        raise FileNotFoundError(
            f'map:={path} does not exist. Save the warehouse map first '
            '(warehouse_navigation/maps/README.md), or draw one from the layout: '
            '`ros2 run warehouse_layout rasterize --out /tmp/layout_map`, then '
            'map:=/tmp/layout_map.yaml')
    return []


def rviz_setup(context):
    """Return RViz for one robot, in the robot's absolute namespace."""
    # Absolute (/robot1), like robot.launch.py: if this file is included inside another
    # namespace, RViz must stay with the robot at /robot1, not move to /<outer>/robot1.
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    # With localization, the view of the map (fixed frame map); without, of the robot (odom).
    localization = LaunchConfiguration('localization').perform(context)
    config = 'robot.rviz' if localization == 'none' else 'map.rviz'
    return [
        PushROSNamespace('/' + namespace),
        # The same remaps as the robot's nodes. The config uses relative topic names (scan,
        # odom), which resolve in the namespace.
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('warehouse_bringup'), 'rviz', config])],
             output='screen'),
    ]


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    namespace = LaunchConfiguration('namespace')
    headless = LaunchConfiguration('headless')
    localization = LaunchConfiguration('localization')
    bringup_share = FindPackageShare('warehouse_bringup')
    navigation_share = FindPackageShare('warehouse_navigation')
    spawn_default = ('Default: from the spawn pose of this namespace in the location registry '
                     'when it is for this world, else 0')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='empty',
            description='Gazebo world in warehouse_gazebo/worlds, without .sdf: empty or '
                        'warehouse'),
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
        DeclareLaunchArgument(
            'gz_gui', default_value='true',
            description='false: the Gazebo server without its window; RViz stays on '
                        '(hardware:=gz only)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz (never when headless)'),
        DeclareLaunchArgument(
            'physics_step', default_value='',
            description='Physics step in seconds for this run: 0.001, 0.002 or 0.005; Gazebo '
                        'loads a copy of the world with it. Empty: the step in the world file '
                        '(hardware:=gz only)'),
        # Empty means "not given": resolve_spawn_pose fills them in.
        DeclareLaunchArgument('x', default_value='', description=f'Spawn x [m]. {spawn_default}'),
        DeclareLaunchArgument('y', default_value='', description=f'Spawn y [m]. {spawn_default}'),
        DeclareLaunchArgument('yaw', default_value='',
                              description=f'Spawn heading [rad]. {spawn_default}'),
        # choices: launch refuses any other value when it declares the argument, before
        # anything starts.
        DeclareLaunchArgument(
            'localization', default_value='none', choices=['none', 'slam', 'amcl'],
            description='none: odometry only; slam: map with slam_toolbox; amcl: localize on '
                        'the map:= file with AMCL. Needs hardware:=gz (a lidar)'),
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution(
                [navigation_share, 'maps', 'warehouse.yaml']),
            description='Map for localization:=amcl, a map_server YAML file. Default: the '
                        'warehouse map saved from slam_toolbox'),
    ]

    # World scope: the simulator and its /clock (hardware:=gz only). gazebo.launch.py checks
    # physics_step before it starts Gazebo.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('warehouse_gazebo'), 'launch', 'gazebo.launch.py'])),
        condition=IfCondition(EqualsSubstitution(hardware, 'gz')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': headless,
            'gz_gui': LaunchConfiguration('gz_gui'),
            'physics_step': LaunchConfiguration('physics_step'),
        }.items(),
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
            'localization': localization,
        }.items(),
    )

    # System scope: the map server, one for all robots, in / (localization:=amcl only). It
    # comes after the robot, so that robot.launch.py refuses mock hardware before it starts.
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'map_server.launch.py'])),
        condition=IfCondition(EqualsSubstitution(localization, 'amcl')),
        launch_arguments={'map': LaunchConfiguration('map')}.items(),
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
        # The spawn pose first. A substitution can compute a simple default (use_sim_time's
        # above), but this one needs Python: read a file of another package and look up the
        # namespace in it. An OpaqueFunction is a Python function that launch calls once every
        # argument has its value. It runs before Gazebo starts, so a bad x, y, yaw or namespace
        # stops the launch before anything runs (an error raised while gz sim is starting
        # leaves the Gazebo server running).
        OpaqueFunction(function=resolve_spawn_pose),
        # The same for the map file of localization:=amcl.
        OpaqueFunction(function=check_map),
        # Every node launched below uses one clock: Gazebo's /clock or the wall clock.
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        gazebo,
        robot,
        map_server,
        rviz,
    ])
