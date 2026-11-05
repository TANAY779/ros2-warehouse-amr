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
Start the robot_localization EKF of one robot (plan D3).

Include this file inside the robot's namespace together with the /tf remaps, as
warehouse_bringup/launch/robot.launch.py does: the EKF reads odom/wheel and imu and
publishes odom and odom -> base_footprint, all within that namespace.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('warehouse_navigation'), 'config', 'ekf.yaml'])],
        # Nav2 and every other consumer read the fused odometry from odom (plan section 2.3).
        remappings=[('odometry/filtered', 'odom')],
        output='screen',
    )
    return LaunchDescription([ekf])
