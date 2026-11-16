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
Bring up one robot inside its namespace (the robot scope of plan section 2.3).

Starts robot_state_publisher, the ros2_control controller manager, the three controllers and
the EKF. With hardware:=gz the controller manager runs inside Gazebo, so this file spawns the
robot into the running world and starts its bridge; with hardware:=mock it runs
ros2_control_node instead. Gazebo itself (world scope) is started by sim.launch.py.

localization:=slam adds slam_toolbox, localization:=amcl adds AMCL starting at the spawn pose
(warehouse_navigation's slam.launch.py and localization.launch.py). AMCL needs a map on /map
from the system scope: sim.launch.py starts the map server with it, and on its own this file
needs `ros2 launch warehouse_navigation map_server.launch.py` next to it.

navigation:=true adds Nav2 with AMCL, both in one container (warehouse_navigation's
navigation.launch.py, parameters from params_file). Nav2 always localizes with AMCL, so it
implies localization:=amcl and needs the map on /map too. It refuses localization:=slam: both
would publish map -> odom.

Every node runs inside PushROSNamespace(/<namespace>) with /tf and /tf_static remapped to the
relative tf and tf_static, so each robot keeps its own TF tree in /<namespace>/tf with
unprefixed frames (plan D7). Phase 9 includes this file once per robot.

The file also works on its own, next to a running Gazebo (gazebo.launch.py): it sets
use_sim_time for its nodes itself (true for hardware:=gz).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
import xacro


def spawner(controller, remaps=()):
    """Return a spawner that loads and activates one controller in this robot's manager."""
    # A relative manager name resolves in the spawner's namespace: /robot1/controller_manager.
    arguments = [controller, '--controller-manager', 'controller_manager']
    if remaps:
        # Controllers use private topics (~/cmd_vel = /robot1/diff_drive_controller/cmd_vel).
        # Remap them onto the robot's topics here; <ros><remapping> in the gz plugin tag is
        # deprecated in ros2_control 4.48 (plan section 2.3).
        arguments += ['--controller-ros-args', ' '.join(f'-r {remap}' for remap in remaps)]
    return Node(package='controller_manager', executable='spawner', arguments=arguments,
                output='screen')


def include(package, launch_file, arguments=None):
    """Return an action that includes a launch file of another package."""
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items())


def check_modes(hardware, localization, navigation):
    """Raise a ValueError naming the arguments if this combination cannot run."""
    # Mock hardware has no lidar: SLAM, AMCL and Nav2 would wait for scans forever.
    if hardware == 'mock' and navigation:
        raise ValueError(
            'navigation:=true needs the lidar (AMCL, the costmaps and the collision monitor '
            'read the scan), and hardware:=mock has none: mock hardware simulates the wheels '
            'and the IMU only. Use hardware:=gz, or leave navigation at false.')
    if hardware == 'mock' and localization != 'none':
        raise ValueError(
            f'localization:={localization} needs the lidar, and hardware:=mock has none: mock '
            'hardware simulates the wheels and the IMU only. Use hardware:=gz, or leave '
            'localization at none.')
    if navigation and localization == 'slam':
        raise ValueError(
            'navigation:=true localizes with AMCL on the saved map, and localization:=slam '
            'would run slam_toolbox next to it: both publish map -> odom. Leave localization '
            'at none or amcl with navigation:=true.')


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    hardware = LaunchConfiguration('hardware').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    navigation = evaluate_condition_expression(context, [LaunchConfiguration('navigation')])

    # Refuse before anything starts.
    check_modes(hardware, localization, navigation)

    # Build the URDF here, so a mistake fails the launch at once. The xacro refuses
    # hardware:=real (reserved for a future SystemInterface) with a clear message.
    description_share = get_package_share_directory('warehouse_description')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'robot.urdf.xacro'),
        mappings={'namespace': namespace, 'hardware': hardware}).toxml()

    actions = [
        # Every node of this robot uses one clock, also when this file runs on its own.
        # gz_ros2_control sets use_sim_time for the controllers inside Gazebo itself.
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # Absolute (/robot1, or / without a namespace): the xacro gives the gz topics and the
        # controller manager inside Gazebo absolute names, so a namespace around this file
        # must not move the ROS side to /fleet/robot1 while Gazebo stays at /robot1.
        PushROSNamespace('/' + namespace),
    ]

    if hardware == 'mock':
        # No simulator hosts the controller manager, so run it as a node. It reads the URDF
        # from the robot_description topic, like the one inside Gazebo.
        # It starts before the /tf remaps below, so they do not apply to it: it uses no TF,
        # and it would pass every remap on to all its controllers, which ros2_control 4.48
        # deprecates (one warning per controller). This also matches Gazebo, where the
        # controller manager never sees launch remaps.
        actions.append(Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[os.path.join(description_share, 'config', 'controllers.yaml')],
            output='screen'))

    actions += [
        # tf2 always uses the absolute /tf and /tf_static; these remaps send every node
        # started after them in this group to /<namespace>/tf instead (plan section 2.3).
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        # Publishes the URDF on robot_description and the transforms of the robot's links.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
    ]

    if hardware == 'gz':
        # Spawning the model starts the gz_ros2_control plugin, and with it the controller
        # manager, inside Gazebo.
        actions.append(include('warehouse_gazebo', 'spawn_robot.launch.py', {
            'name': namespace or 'warehouse_amr',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }))

    # The spawners wait for the controller manager, so they can start right away.
    actions += [
        spawner('joint_state_broadcaster'),
        spawner('diff_drive_controller', ['~/cmd_vel:=cmd_vel', '~/odom:=odom/wheel']),
        spawner('imu_sensor_broadcaster', ['~/imu:=imu']),
        include('warehouse_navigation', 'ekf.launch.py'),
    ]

    # Localization and navigation, like the EKF, run after the /tf remaps: they read this
    # robot's scan and tf, and publish map -> odom on this robot's tf (plan F5, F6). The
    # spawn pose is a pose in the map frame, because the map frame is the Gazebo world frame
    # (plan P2-D2): AMCL starts where the robot stands. One of the three at most, so only one
    # node publishes map -> odom.
    spawn_pose = {name: LaunchConfiguration(name) for name in ('x', 'y', 'yaw')}
    if navigation:
        # AMCL and the Nav2 servers in one container (plan D20). navigation.launch.py needs the
        # namespace this group pushed, to name the container and the parameter sections.
        actions.append(include('warehouse_navigation', 'navigation.launch.py', {
            'namespace': namespace,
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            **spawn_pose,
        }))
    elif localization == 'slam':
        actions.append(include('warehouse_navigation', 'slam.launch.py'))
    elif localization == 'amcl':
        actions.append(include('warehouse_navigation', 'localization.launch.py', spawn_pose))
    return [GroupAction(actions)]


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description='Robot namespace; topics, nodes, TF and gz topics live under it'),
        DeclareLaunchArgument(
            'hardware', default_value='gz',
            description='gz: Gazebo through gz_ros2_control (Gazebo must be running); '
                        'mock: mock_components, no simulator'),
        # Declared after hardware, because its default depends on it.
        DeclareLaunchArgument(
            'use_sim_time', default_value=EqualsSubstitution(hardware, 'gz'),
            description="Use Gazebo's /clock. Default: true for hardware:=gz, false for mock"),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m] (gz only)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m] (gz only)'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Spawn heading [rad] (gz only)'),
        # choices: launch refuses any other value when it declares the argument.
        DeclareLaunchArgument(
            'localization', default_value='none', choices=['none', 'slam', 'amcl'],
            description='none: odometry only; slam: map with slam_toolbox; amcl: localize with '
                        'AMCL on the map on /map (start map_server.launch.py too). Needs '
                        'hardware:=gz (a lidar)'),
        DeclareLaunchArgument(
            'navigation', default_value='false',
            description='true: Nav2 with AMCL, both in one container (implies '
                        'localization:=amcl, refuses slam; start map_server.launch.py too). '
                        'Needs hardware:=gz (a lidar)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('warehouse_navigation'), 'config', 'nav2_params.yaml']),
            description='Nav2 parameter file for navigation:=true. Default: '
                        'warehouse_navigation/config/nav2_params.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
