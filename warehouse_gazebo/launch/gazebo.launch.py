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
Start Gazebo Sim with a warehouse world and bridge its clock to ROS (world scope).

Robots are added separately with spawn_robot.launch.py, one per namespace.

  ros2 launch warehouse_gazebo gazebo.launch.py world:=empty headless:=true

Gazebo is started directly, not through ros_gz_sim's gz_sim.launch.py: that file runs
`gz sim` through a shell (shell=True), so a SIGINT from launch stops only the shell and
leaves the Gazebo server running in the background. Started directly, Gazebo receives the
signal itself and shuts down. The plugin and resource paths Gazebo needs come from the ROS
environment (gz_ros2_control's setup hook adds its plugin directory).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# -r: start the simulation at once. -v 2: print errors and warnings.
# --force-version 8: the Gazebo Sim 8 (Harmonic) that ROS 2 Jazzy's vendor packages provide.
GZ_COMMON_ARGS = ['-r', '-v', '2', '--force-version', '8']


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    # A list of substitutions is joined into one argument: <share>/worlds/<world>.sdf
    world_file = [PathJoinSubstitution(
        [FindPackageShare('warehouse_gazebo'), 'worlds', LaunchConfiguration('world')]), '.sdf']

    gazebo_headless = ExecuteProcess(
        # -s: the server only, no window. --headless-rendering: the sensors render
        # through EGL, without a display.
        cmd=['gz', 'sim', '-s', '--headless-rendering', *GZ_COMMON_ARGS, world_file],
        name='gazebo',
        output='screen',
        condition=IfCondition(headless),
        # A crash of the simulator ends the whole launch.
        on_exit=Shutdown(reason='Gazebo exited'),
    )
    gazebo_with_gui = ExecuteProcess(
        cmd=['gz', 'sim', *GZ_COMMON_ARGS, world_file],
        name='gazebo',
        output='screen',
        condition=UnlessCondition(headless),
        # Closing the Gazebo window ends the whole launch.
        on_exit=Shutdown(reason='Gazebo exited'),
    )

    # Sim time for every node: gz /clock -> ROS /clock, once per world (not per robot).
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty',
            description='World file name in warehouse_gazebo/worlds, without .sdf'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: run only the Gazebo server, without a window'),
        gazebo_headless,
        gazebo_with_gui,
        clock_bridge,
    ])
