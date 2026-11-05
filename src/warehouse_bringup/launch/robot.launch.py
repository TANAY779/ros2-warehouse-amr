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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
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


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    hardware = LaunchConfiguration('hardware').perform(context)

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
        OpaqueFunction(function=launch_setup),
    ])
