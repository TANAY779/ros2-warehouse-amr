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
Map with slam_toolbox (online, asynchronous) for one robot (plan F5).

Include this file inside the robot's namespace together with the /tf remaps, as
warehouse_bringup/launch/robot.launch.py does with localization:=slam. slam_toolbox then reads
scan and the robot's tf in that namespace, publishes map -> odom on the robot's tf, and the map
on the shared /map (config/slam_toolbox.yaml).

slam_toolbox is a lifecycle node in Jazzy: it starts unconfigured and does nothing until it is
configured and then activated. This file asks for both with launch events, the way
slam_toolbox's own online_async_launch.py does: it emits a configure request at start-up, and
an event handler emits the activate request once the node reports that it has reached
`inactive`. No lifecycle manager and no bond (plan P2-D8): a long loop-closure optimization
could delay the heartbeat, and a manager would then reset the mapping.
"""

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        # LifecycleNode requires a namespace. An empty one is relative: the node lands in the
        # namespace of the group that includes this file (/robot1).
        namespace='',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('warehouse_navigation'), 'config', 'slam_toolbox.yaml'])],
        output='screen',
    )

    def request(transition_id):
        """Return an action that asks slam_toolbox for one lifecycle transition."""
        # matches_action picks this node by the launch action that started it, so the request
        # finds it whatever namespace it ended up in.
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox), transition_id=transition_id))

    # Registered before the configure request is emitted, so the handler cannot miss the
    # node's arrival in `inactive`.
    activate_when_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox is configured; activating it'),
            request(Transition.TRANSITION_ACTIVATE),
        ],
    ))

    # (launch_ros 0.26 can do the same with LifecycleNode(autostart=True). The events are
    # spelled out here to show the mechanism.)
    return LaunchDescription([
        slam_toolbox,
        activate_when_configured,
        request(Transition.TRANSITION_CONFIGURE),
    ])
