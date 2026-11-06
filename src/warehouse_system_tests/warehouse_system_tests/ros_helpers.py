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
Small rclpy helpers for launch tests: waiting, collecting messages, graph and TF checks.

They use plain rclpy with spin_once loops, so a test reads top to bottom. Timeouts are in
wall-clock seconds; anything measured on the robot (rates, durations) uses message stamps,
so it stays correct in simulation time when Gazebo runs slower than real time.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
import yaml

# The QoS tf2 uses: /tf is volatile, /tf_static keeps its messages for late subscribers.
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
TF_STATIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The controllers every robot runs (warehouse_description/config/controllers.yaml).
CONTROLLERS = ('joint_state_broadcaster', 'diff_drive_controller', 'imu_sensor_broadcaster')

# The Phase 1 TF tree of plan section 2.4 as child -> parent.
EXPECTED_TF_PARENTS = {
    'base_footprint': 'odom',
    'base_link': 'base_footprint',
    'lidar_front_link': 'base_link',
    'imu_link': 'base_link',
    'left_wheel_link': 'base_link',
    'right_wheel_link': 'base_link',
    'front_caster_link': 'base_link',
    'rear_caster_link': 'base_link',
    # Visual-only parts: each part with a colour of its own is a link
    # (warehouse_description/urdf/materials.xacro).
    'shell_link': 'base_link',
    'front_bumper_link': 'base_link',
    'rear_bumper_link': 'base_link',
    'tote_tray_link': 'base_link',
    'emergency_stop_link': 'base_link',
    'status_light_link': 'base_link',
    'lidar_front_mount_link': 'base_link',
    'left_wheel_hub_link': 'left_wheel_link',
    'right_wheel_hub_link': 'right_wheel_link',
}


def description_config(name):
    """
    Return a config file of warehouse_description (robot.yaml, controllers.yaml) as a dict.

    The tests read the robot's numbers (outline, rates, limits) from the files that configure
    it, so a change there cannot leave a test checking a stale value (plan section 6).
    """
    path = os.path.join(get_package_share_directory('warehouse_description'), 'config', name)
    with open(path) as stream:
        return yaml.safe_load(stream)


def diff_drive_parameters():
    """Return the diff_drive_controller parameters from controllers.yaml."""
    controllers = description_config('controllers.yaml')
    return controllers['/**/diff_drive_controller']['ros__parameters']


def controller_manager_parameters():
    """Return the controller_manager parameters from controllers.yaml."""
    controllers = description_config('controllers.yaml')
    return controllers['/**/controller_manager']['ros__parameters']


# The tests command the robot at 20 Hz, like Nav2's controller server and velocity smoother.
# cmd_vel_timeout must be well above this period, or the robot brakes between two commands.
COMMAND_PERIOD = 0.05  # s


def cmd_vel_timeout_windows(last_command, speed):
    """
    Return when odometry must show cmd_vel_timeout at work, once commands stop.

    The robot drove at ``speed`` [m/s, signed] until the command stamped ``last_command`` [s];
    nothing was published after it. Returns ``((moving_from, moving_to), stopped_by)``:

    - From ``moving_from`` to ``moving_to`` odom/wheel still reports about ``speed``: the
      controller keeps executing the last command until it is cmd_vel_timeout old. The window
      starts when the next command would have been due, so a robot that stops as soon as one
      command is missing fails, and it ends when the timeout expires.
    - From ``stopped_by`` on the robot stands still: braking starts within one controller
      update after the timeout and keeps to the deceleration limit; odom/wheel reports the
      mean of the last velocity_rolling_window_size updates, so it reaches zero one window
      later; 0.2 s more absorb timing jitter.

    Every time is derived from controllers.yaml, so the checks hold for any cmd_vel_timeout.
    """
    drive = diff_drive_parameters()
    update_period = 1.0 / controller_manager_parameters()['update_rate']  # s
    timeout = drive['cmd_vel_timeout']  # s
    limits = drive['linear']['x']
    # Braking from forward keeps to max_deceleration (<= 0), from reverse to
    # max_deceleration_reverse (>= 0).
    if speed >= 0.0:
        deceleration = -limits['max_deceleration']
    else:
        deceleration = limits['max_deceleration_reverse']
    # 10 updates is diff_drive_controller's default rolling window.
    mean_window = drive.get('velocity_rolling_window_size', 10) * update_period  # s
    moving = (last_command + COMMAND_PERIOD, last_command + timeout)
    stopped_by = (last_command + timeout + update_period + abs(speed) / deceleration
                  + mean_window + 0.2)
    return moving, stopped_by


def spin_for(node, seconds):
    """Process callbacks for ``seconds`` of wall-clock time."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def spin_until(node, predicate, timeout_s):
    """Process callbacks until ``predicate()`` is true; return False after ``timeout_s``."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


class Collector:
    """
    Subscribe to a topic and keep every message received, until close().

    Close a collector when its test ends (unittest's addCleanup does it): an open one keeps
    storing every message and costs a callback per message for the rest of the test run.
    """

    def __init__(self, node, msg_type, topic, qos=10):
        self.node = node
        self.messages = []
        self.subscription = node.create_subscription(msg_type, topic, self.messages.append, qos)

    @property
    def last(self):
        """Return the newest message, or None before the first one."""
        return self.messages[-1] if self.messages else None

    def clear(self):
        self.messages.clear()

    def close(self):
        """Unsubscribe; the messages received so far stay available."""
        self.node.destroy_subscription(self.subscription)


def controller_states(node, controller_manager, timeout_s=5.0):
    """Return {controller name: state} from a controller manager's list_controllers."""
    client = node.create_client(ListControllers, f'{controller_manager}/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return {}
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
        if future.result() is None:
            return {}
        return {controller.name: controller.state for controller in future.result().controller}
    finally:
        node.destroy_client(client)


def wait_for_active_controllers(node, controller_manager, names, timeout_s):
    """Poll until every controller in ``names`` is active; return the last states seen."""
    deadline = time.monotonic() + timeout_s
    states = {}
    while time.monotonic() < deadline:
        states = controller_states(node, controller_manager)
        if all(states.get(name) == 'active' for name in names):
            break
        spin_for(node, 0.5)
    return states


def endpoint_names(endpoints):
    """Return the set of node names in a list of TopicEndpointInfo."""
    return {endpoint.node_name for endpoint in endpoints}


def tf_parents(*message_lists):
    """Return {child frame: parent frame} from lists of TFMessage."""
    parents = {}
    for messages in message_lists:
        for message in messages:
            for transform in message.transforms:
                parents[transform.child_frame_id] = transform.header.frame_id
    return parents


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def duration_text(seconds):
    """Return a duration as text, for example '8 min 05 s' or '45 s'."""
    seconds = int(round(seconds))
    if seconds < 60:
        return f'{seconds} s'
    return f'{seconds // 60} min {seconds % 60:02d} s'


def rate_from_stamps(messages):
    """Return the message rate in Hz measured with header stamps (sim time in Gazebo)."""
    if len(messages) < 2:
        return 0.0
    span = stamp_to_seconds(messages[-1].header.stamp) - stamp_to_seconds(
        messages[0].header.stamp)
    return (len(messages) - 1) / span if span > 0.0 else 0.0


def braking_deceleration(odometry_messages, speed, after):
    """
    Return the deceleration [m/s^2] of a stop from ``speed``, measured in Odometry twists.

    Only messages stamped after ``after`` count. diff_drive_controller reports a rolling mean
    of the wheel speed, which rounds off the start and the end of the braking ramp; between
    70 % and 30 % of ``speed`` the ramp is straight, so its slope there is the deceleration.
    """
    ramp = [m for m in odometry_messages
            if stamp_to_seconds(m.header.stamp) > after
            and 0.3 <= m.twist.twist.linear.x / speed <= 0.7]
    if len(ramp) < 2:
        return 0.0
    first, last = ramp[0], ramp[-1]
    duration = stamp_to_seconds(last.header.stamp) - stamp_to_seconds(first.header.stamp)
    change = abs(first.twist.twist.linear.x - last.twist.twist.linear.x)
    return change / duration if duration > 0.0 else 0.0


def yaw_of(quaternion):
    """Return the yaw angle of a geometry_msgs Quaternion."""
    q = quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def total_yaw_change(odometry_messages):
    """Return the unwrapped yaw change over a sequence of Odometry messages."""
    yaws = [yaw_of(m.pose.pose.orientation) for m in odometry_messages]
    return sum(wrap_angle(b - a) for a, b in zip(yaws, yaws[1:]))


def planar_displacement(start, end):
    """
    Return (forward, left) motion between two Odometry poses, in the start heading.

    Expressing the motion in the robot's own start heading makes odometry frames and the
    ground-truth world frame comparable, even if they started rotated against each other.
    """
    dx = end.pose.pose.position.x - start.pose.pose.position.x
    dy = end.pose.pose.position.y - start.pose.pose.position.y
    yaw = yaw_of(start.pose.pose.orientation)
    return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)
