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
  ros2 launch warehouse_gazebo gazebo.launch.py world:=warehouse physics_step:=0.002

headless:=true or gz_gui:=false runs the Gazebo server without its window; the sensors still
render, through EGL. Here the two mean the same. In sim.launch.py they differ: headless:=true
also turns RViz off.

physics_step:=0.001, 0.002 or 0.005 runs the world with that physics step. The world file is
copied into this launch's log directory with the new <max_step_size>, and Gazebo loads the
copy; the world files themselves keep their step (1 ms). Any other value stops the launch
before Gazebo starts.

Gazebo is started directly, not through ros_gz_sim's gz_sim.launch.py: that file runs
`gz sim` through a shell (shell=True), so a SIGINT from launch stops only the shell and
leaves the Gazebo server running in the background. Started directly, Gazebo receives the
signal itself and shuts down. The plugin and resource paths Gazebo needs come from the ROS
environment (gz_ros2_control's setup hook adds its plugin directory).
"""

import math
import os
from xml.etree import ElementTree

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.conditions import evaluate_condition_expression
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# -r: start the simulation at once. -v 2: print errors and warnings.
# --force-version 8: the Gazebo Sim 8 (Harmonic) that ROS 2 Jazzy's vendor packages provide.
GZ_COMMON_ARGS = ['-r', '-v', '2', '--force-version', '8']
# -s: the server only, no window. --headless-rendering: the sensors render through EGL,
# without a display.
GZ_SERVER_ARGS = ['-s', '--headless-rendering']

# The steps physics_step accepts, in seconds. The controller manager (update_rate 100 in
# controllers.yaml) and the IMU (100 Hz in robot.yaml) work every 10 ms, so the step must
# divide 10 ms: then every control cycle and every IMU sample falls on a physics step (ROS 2
# skill, simulation.md section 9, "Physics step size vs sensor update rate"). 4 ms does not
# (2.5 steps). The lidar's 100 ms is a multiple of all three. Phase 3 compares these three
# steps and keeps the largest that passes its checks (plan Q5).
PHYSICS_STEPS = (0.001, 0.002, 0.005)
CONTROL_PERIOD = 0.010           # s


def physics_step(text):
    """
    Return the physics_step argument in seconds, or None if it is empty.

    Raises ValueError, with the reason and the accepted values, for anything that is not one
    of PHYSICS_STEPS. 0.002, 0.0020 and 2e-3 are the same step.
    """
    text = text.strip()
    if not text:
        return None
    *others, last = [f'{step:g}' for step in PHYSICS_STEPS]
    accepted = f'{", ".join(others)} or {last}'
    try:
        value = float(text)
    except ValueError:
        value = math.nan
    for step in PHYSICS_STEPS:
        if math.isclose(value, step, rel_tol=1e-9):
            return step
    reason = ''
    if math.isfinite(value) and value > 0.0:
        steps = CONTROL_PERIOD / value
        if not math.isclose(steps, round(steps), rel_tol=1e-9):
            reason = (f' ({CONTROL_PERIOD * 1000:g} ms would be {steps:.3g} steps of '
                      f'{value * 1000:g} ms)')
    raise ValueError(
        f'physics_step:={text} is refused. Use {accepted} (seconds): these divide the '
        f'{CONTROL_PERIOD * 1000:g} ms period of the controller manager and the IMU, so every '
        f'control cycle and every IMU sample falls on a physics step{reason}. Leave '
        'physics_step empty to keep the step in the world file.')


def world_copy(world_file, step, directory):
    """
    Write a copy of the SDF world ``world_file`` with the physics step ``step`` [s].

    The copy goes into ``directory``, named after the world and the step
    (warehouse_2ms.sdf); return its path. Only <max_step_size> and the physics profile's
    name change. Raises ValueError unless the world has exactly one physics step.
    """
    # Keep the comments, so the copy reads like the original. (The comment before <sdf> is
    # dropped: ElementTree keeps only what is inside the root element.)
    parser = ElementTree.XMLParser(target=ElementTree.TreeBuilder(insert_comments=True))
    tree = ElementTree.parse(world_file, parser)
    root = tree.getroot()
    found = [(physics, physics.find('max_step_size'))
             for physics in root.iterfind('world/physics')]
    found = [(physics, element) for physics, element in found if element is not None]
    if len(found) != 1:
        raise ValueError(f'{world_file} has {len(found)} <physics> elements with a '
                         '<max_step_size>; physics_step needs exactly one')
    physics, element = found[0]
    milliseconds = f'{step * 1000:g}ms'
    element.text = f'{step:g}'
    physics.set('name', milliseconds)
    note = ElementTree.Comment(
        f' A copy of {world_file} with max_step_size {step:g} s, written for one run by '
        f'warehouse_gazebo gazebo.launch.py physics_step:={step:g} ')
    note.tail = root.text        # the line break and indent that came before <world>
    root.insert(0, note)
    name = os.path.splitext(os.path.basename(world_file))[0]
    path = os.path.join(directory, f'{name}_{milliseconds}.sdf')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return path


def gazebo_setup(context):
    """
    Return the gz sim process for the world, physics_step, headless and gz_gui arguments.

    An OpaqueFunction, because choosing the world file needs Python (check the step, write
    the copy). It runs before Gazebo starts, so a bad argument stops the launch with nothing
    running: an error raised while gz sim is starting would leave the Gazebo server running.
    """
    world = LaunchConfiguration('world').perform(context)
    worlds = os.path.join(FindPackageShare('warehouse_gazebo').perform(context), 'worlds')
    world_file = os.path.join(worlds, f'{world}.sdf')
    if not os.path.isfile(world_file):
        names = sorted(os.path.splitext(name)[0] for name in os.listdir(worlds)
                       if name.endswith('.sdf'))
        raise FileNotFoundError(
            f'world:={world}: warehouse_gazebo has no worlds/{world}.sdf (worlds: '
            f'{", ".join(names)})')

    actions = []
    step = physics_step(LaunchConfiguration('physics_step').perform(context))
    if step is not None:
        # The log directory of this launch ("All log files can be found below ..."): one per
        # run, kept with that run's logs.
        world_file = world_copy(world_file, step, launch_config.log_dir)
        actions.append(LogInfo(
            msg=f'Gazebo steps the physics every {step * 1000:g} ms (physics_step): it loads '
                f'{world_file}'))

    window = (evaluate_condition_expression(context, [LaunchConfiguration('gz_gui')])
              and not evaluate_condition_expression(context, [LaunchConfiguration('headless')]))
    server_only = [] if window else GZ_SERVER_ARGS
    actions.append(ExecuteProcess(
        cmd=['gz', 'sim', *server_only, *GZ_COMMON_ARGS, world_file],
        name='gazebo',
        output='screen',
        # A crash of the simulator, or closing its window, ends the whole launch.
        on_exit=Shutdown(reason='Gazebo exited'),
    ))
    return actions


def generate_launch_description():
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
        DeclareLaunchArgument(
            'gz_gui', default_value='true',
            description='false: run only the Gazebo server, without a window (the same as '
                        'headless:=true here; sim.launch.py keeps RViz on with it)'),
        DeclareLaunchArgument(
            'physics_step', default_value='',
            description='Physics step for this run in seconds: 0.001, 0.002 or 0.005 (Gazebo '
                        "loads a copy of the world with it, in this launch's log directory). "
                        'Empty: the step in the world file'),
        OpaqueFunction(function=gazebo_setup),
        clock_bridge,
    ])
