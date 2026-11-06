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
The teleop window: hold keys to drive the robot; release them and it stops.

    ros2 run warehouse_bringup teleop_window                               # drives robot1
    ros2 run warehouse_bringup teleop_window --ros-args -r __ns:=/robot2   # another robot

teleop_twist_keyboard reads characters from a terminal, and a terminal never learns that a key
was released: it sends one command per key event and leaves the stopping to cmd_vel_timeout. A
window gets a press and a release event for every key, so this one drives exactly while a key is
held (warehouse_bringup README, "Driving with teleop").

Four parts:

- HoldToDrive (hold_to_drive.py): which command to send and when. No Qt, no ROS.
- connection.py: which clock to stamp the commands with, and the coloured line at the top of the
  window. No Qt, no ROS.
- TeleopWindow: Qt key, focus and close events in, commands out, and a timer for the rate.
- TeleopNode: the ROS side. Parameters, the cmd_vel publisher, which stamps each command with
  the node's clock, and a look at the ROS graph five times a second: does the robot listen, and
  does /clock have a publisher.

The window lives in the main thread, as Qt requires. ROS callbacks run in a second thread: the
graph check, the /clock subscription that keeps sim time, and the parameter services.
"""

import signal
import threading
import time

from geometry_msgs.msg import TwistStamped
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.exceptions import ParameterException
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from warehouse_bringup.connection import choose_clock
from warehouse_bringup.connection import connection_status
from warehouse_bringup.connection import DISCOVERY_TIME
from warehouse_bringup.connection import OK
from warehouse_bringup.connection import PROBLEM
from warehouse_bringup.connection import SIM_TIME
from warehouse_bringup.connection import WAIT
from warehouse_bringup.hold_to_drive import HoldToDrive
from warehouse_bringup.hold_to_drive import is_bound

# sim.launch.py's default robot, so `ros2 run warehouse_bringup teleop_window` needs no
# arguments. rcl applies a __ns remap over the namespace a node is created with, so
# --ros-args -r __ns:=/robot2 still picks another robot (and __ns:=/ no namespace).
DEFAULT_NAMESPACE = 'robot1'

# A key pressed with one of these held is a shortcut (Ctrl+C, Alt+Tab), not driving. Releases
# always count, so a key released while Ctrl is down still stops the robot.
SHORTCUT_MODIFIERS = Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier

# The project rule (controllers.yaml): while the robot should move, publish at 10 Hz or more,
# or diff_drive_controller brakes between two commands (cmd_vel_timeout 0.15 s).
MIN_PUBLISH_RATE = 10.0  # Hz

STATUS_PERIOD_MS = 200
GRAPH_CHECK_PERIOD = 0.2  # s

# The line at the top of the window: bold, on a background in the colour of its level.
STATUS_STYLE = 'font-weight: bold; padding: 6px; border-radius: 4px;'
LEVEL_COLOURS = {
    OK: 'background-color: #2e7d32; color: white;',  # green
    WAIT: 'background-color: #f9a825; color: black;',  # amber
    PROBLEM: 'background-color: #c62828; color: white;',  # red
}

FOCUS_HINT = 'Click this window to drive: keys go to the window that has the focus.'

HELP = (
    '<p><b>Hold</b> keys to drive; <b>release</b> them and the robot stops.</p>'
    '<table cellspacing="6">'
    '<tr><td><b>&uarr;</b> or <b>W</b></td><td>forward</td>'
    '<td><b>&darr;</b> or <b>S</b></td><td>backward</td></tr>'
    '<tr><td><b>&larr;</b> or <b>A</b></td><td>turn left</td>'
    '<td><b>&rarr;</b> or <b>D</b></td><td>turn right</td></tr>'
    '<tr><td><b>Space</b></td><td>stop</td><td></td><td></td></tr>'
    '<tr><td><b>Q</b> / <b>Z</b></td><td>drive faster / slower</td>'
    '<td><b>E</b> / <b>C</b></td><td>turn faster / slower</td></tr>'
    '</table>'
    '<p>Keys combine: &uarr; and &larr; together drive a curve.<br>'
    'Clicking another window stops the robot too.</p>'
)


def key_name(event):
    """Return Qt's portable name of the event's key, such as 'Up', 'W' or 'Space'."""
    return QKeySequence(event.key()).toString()


class TeleopWindow(QWidget):
    """
    Qt key, focus and close events in, commands out.

    ``teleop`` is a HoldToDrive. ``publish(command)`` sends one command and returns True, or
    False when it held the command back. ``period`` is the publish period [s], ``topic`` is
    shown in the title, and ``status()`` (optional) returns the connection.Status for the
    coloured line at the top. No ROS in here.
    """

    def __init__(self, teleop, publish, period, topic, status=None):
        super().__init__()
        self._teleop = teleop
        self._publish = publish
        self._status = status
        self._sent = True  # whether publish() sent the last command
        self._level = None  # the level the top line is coloured for
        self.setWindowTitle(f'Teleop: {topic}')
        self.setMinimumWidth(420)
        # Take the keyboard focus when clicked, so the key events come to this widget.
        self.setFocusPolicy(Qt.StrongFocus)

        # Repeats the command while a key is held. A precise timer keeps the period steady (a
        # coarse one may fire 5 % late).
        self._publish_timer = QTimer(self)
        self._publish_timer.setTimerType(Qt.PreciseTimer)
        self._publish_timer.setInterval(round(period * 1000.0))
        self._publish_timer.timeout.connect(self._on_publish_period)

        # Refreshes the status lines. It also lets Ctrl-C through: Python runs a signal handler
        # only when the main thread runs Python code, and Qt's event loop is C++.
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(STATUS_PERIOD_MS)
        self._status_timer.timeout.connect(self._show_status)
        self._status_timer.start()

        # The line at the top says whether the keys can drive, and if not, why.
        self._status_label = QLabel()
        self._status_label.setObjectName('status')
        self._status_label.setTextFormat(Qt.PlainText)
        self._status_label.setWordWrap(True)
        self._status_label.setVisible(status is not None)
        self._speeds_label = QLabel()
        self._command_label = QLabel()
        self._focus_label = QLabel()
        self._focus_label.setStyleSheet('font-weight: bold;')
        layout = QVBoxLayout(self)
        for label in (self._status_label, QLabel(HELP), self._speeds_label, self._command_label,
                      self._focus_label):
            layout.addWidget(label)
        self._show_command()
        self._show_status()
        # The focus goes to this widget whenever the window is active.
        self.setFocus()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            # The desktop repeats a held key (GNOME: after 500 ms, then every 30 ms). The key is
            # still held, so nothing changes; the publish timer keeps the commands coming.
            return
        key = key_name(event)
        if event.modifiers() & SHORTCUT_MODIFIERS or not is_bound(key):
            super().keyPressEvent(event)
            return
        self._send(self._teleop.press(key))

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return  # on X11 each repeat arrives as a release and a press, both marked
        self._send(self._teleop.release(key_name(event)))

    def focusOutEvent(self, event):
        # Another window has the keyboard now, so this one will not see the key releases.
        self._send(self._teleop.keyboard_lost())
        super().focusOutEvent(event)

    def closeEvent(self, event):
        # Closed by its title bar, or by close_on_signals() on Ctrl-C.
        self._send(self._teleop.keyboard_lost())
        self._status_timer.stop()
        super().closeEvent(event)

    def _send(self, command):
        """Publish ``command`` now, if there is one, and keep the publish timer in step."""
        if command is not None:
            self._sent = self._publish(command)
            # Restart the period, so the next repeat comes one full period after this command.
            self._publish_timer.start()
        if not self._teleop.publishing:
            self._publish_timer.stop()
        self._show_command()

    def _on_publish_period(self):
        command = self._teleop.tick()
        if command is not None:
            self._sent = self._publish(command)
        if not self._teleop.publishing:
            self._publish_timer.stop()
        self._show_command()

    def _show_command(self):
        teleop = self._teleop
        self._speeds_label.setText(
            f'Driving speed {teleop.linear_speed:.2f} m/s (Q/Z), '
            f'turning speed {teleop.angular_speed:.2f} rad/s (E/C)')
        # The command follows the keys even when nothing is sent, so this line shows whether
        # the keys reach the window at all.
        if not teleop.publishing:
            state = 'idle: publishing nothing'
        elif not self._sent:
            state = 'not sent: see the line at the top'
        elif teleop.driving:
            state = 'driving'
        else:
            state = 'stopping: sending zero commands'
        command = teleop.command
        self._command_label.setText(
            f'Command: linear.x {command.linear:+.2f} m/s, angular.z {command.angular:+.2f} '
            f'rad/s ({state})')

    def _show_status(self):
        if self._status is not None:
            status = self._status()
            self._status_label.setText(status.text)
            if status.level != self._level:  # a new style sheet costs a new layout: only then
                self._level = status.level
                self._status_label.setStyleSheet(STATUS_STYLE + LEVEL_COLOURS[status.level])
        self._focus_label.setText('' if self.hasFocus() else FOCUS_HINT)


class TeleopNode(Node):
    """The ROS side: parameters, the cmd_vel publisher, and the choice of clock."""

    def __init__(self):
        super().__init__('teleop_window', namespace=DEFAULT_NAMESPACE)
        self.teleop = HoldToDrive(
            linear_speed=self._parameter(
                'linear_speed', 0.3, 'Driving speed at start [m/s]; Q and Z change it by 0.1'),
            angular_speed=self._parameter(
                'angular_speed', 0.6, 'Turning speed at start [rad/s]; E and C change it by 0.1'),
            max_linear_speed=self._parameter(
                'max_linear_speed', 1.2, 'Highest driving speed that Q sets [m/s]'),
            max_angular_speed=self._parameter(
                'max_angular_speed', 2.0, 'Highest turning speed that E sets [rad/s]'))
        rate = self._parameter(
            'publish_rate', 20.0, 'Commands per second while a key is held [Hz]')
        if rate < MIN_PUBLISH_RATE:
            raise ValueError(
                f'publish_rate must be at least {MIN_PUBLISH_RATE} Hz, not {rate}: '
                'diff_drive_controller brakes once the newest command is cmd_vel_timeout old')
        self.period = 1.0 / rate  # s
        self._frame_id = self._parameter(
            'frame_id', 'base_footprint', 'header.frame_id of the commands')
        # A relative name, so it resolves in the node's namespace: /robot1/cmd_vel by default.
        # Reliable with depth 10, as teleop_twist_keyboard does.
        self._publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # rclpy declares use_sim_time itself, false unless given. True means sim time from the
        # start; otherwise _check_graph() chooses (the rule is in connection.py).
        self._chosen_clock = SIM_TIME if self.get_parameter('use_sim_time').value else None
        self._started = time.monotonic()
        self._listening_since = None  # when something started to listen on cmd_vel
        self._logged_level = None
        if self._chosen_clock == SIM_TIME:
            clock_rule = 'sim time (use_sim_time is true)'
        else:
            clock_rule = ('sim time if /clock has a publisher; wall time if the robot listens '
                          f'and nothing publishes /clock for {DISCOVERY_TIME:g} s')
        self.get_logger().info(f'Publishing on {self.topic}. Clock: {clock_rule}.')
        self._check_graph()  # the first status, before the timer's first check
        # On a steady clock: the node's own clock may become sim time, which stands still
        # whenever the simulation does.
        self.create_timer(GRAPH_CHECK_PERIOD, self._check_graph,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _parameter(self, name, default, description):
        # Read once at the start; the window would never see a later change, so refuse it.
        descriptor = ParameterDescriptor(description=description, read_only=True)
        return self.declare_parameter(name, default, descriptor).value

    @property
    def topic(self):
        """Return the full name of the command topic, such as /robot1/cmd_vel."""
        return self._publisher.topic_name

    @property
    def chosen_clock(self):
        """Return the clock the commands are stamped with: SIM_TIME, WALL_TIME or None (yet)."""
        return self._chosen_clock

    def status(self):
        """Return the connection.Status for the line at the top of the window."""
        return self._status

    def publish(self, command):
        """Publish one command, stamped now with this node's clock; return False if held back."""
        # Only while the line at the top is green. Before that, nothing listens or the clock
        # is not chosen yet, and a stamp from the wrong clock either never gets old or is
        # ignored (connection.py).
        if not self._status.can_send:
            return False
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = command.linear
        message.twist.angular.z = command.angular
        self._publisher.publish(message)
        return True

    def _check_graph(self):
        """Look at the ROS graph: choose the clock, and update the line at the top."""
        listeners = self._publisher.get_subscription_count()
        clock_publishers = self.count_publishers('/clock')
        now = time.monotonic()
        if listeners == 0:
            self._listening_since = None
        elif self._listening_since is None:
            self._listening_since = now
        listening_for = None if self._listening_since is None else now - self._listening_since
        clock = choose_clock(self._chosen_clock, clock_publishers > 0, listening_for)
        if clock != self._chosen_clock:
            self._use_clock(clock)
        searching = now - self._started < DISCOVERY_TIME
        self._status = connection_status(
            topic=self.topic, clock=self._chosen_clock, listeners=listeners,
            clock_publishers=clock_publishers,
            sim_time_started=self.get_clock().now().nanoseconds > 0,
            domain_id=self.context.get_domain_id(),
            others=self._other_robots() if listeners == 0 and not searching else (),
            searching=searching)
        # Log each change between green and red once, for whoever reads the terminal.
        if self._status.level != WAIT and self._status.level != self._logged_level:
            self._logged_level = self._status.level
            if self._status.level == PROBLEM:
                self.get_logger().warning(self._status.text)
            else:
                self.get_logger().info(self._status.text)

    def _use_clock(self, clock):
        if clock == SIM_TIME:
            # rclpy's time source then subscribes to /clock, and the node's clock follows it.
            result = self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            if not result[0].successful:
                self.get_logger().error(f'Could not switch to sim time: {result[0].reason}')
                return
            reason = '/clock has a publisher: a simulation is running'
        else:
            reason = (f'the robot listens and nothing has published /clock for '
                      f'{DISCOVERY_TIME:g} s: mock hardware')
        self._chosen_clock = clock
        self.get_logger().info(f'Clock: {clock} ({reason}).')

    def _other_robots(self):
        """Return the other .../cmd_vel topics that something listens on, for the hint."""
        return tuple(sorted(
            name for name, _ in self.get_topic_names_and_types()
            if name.endswith('/cmd_vel') and name != self.topic
            and self.count_subscribers(name) > 0))


def close_on_signals(window):
    """Close the window on Ctrl-C (SIGINT) or SIGTERM, so it stops the robot first."""
    for signal_number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_number, lambda *_: window.close())


def main(args=None):
    # rclpy's own Ctrl-C handler would shut ROS down at once, before the window could publish
    # its last zero command; close_on_signals() handles Ctrl-C instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = TeleopNode()
    except (ValueError, ParameterException) as error:  # out of range, or -p speed:=1 (an int)
        get_logger('teleop_window').fatal(str(error))
        rclpy.try_shutdown()
        return 1
    app = QApplication(remove_ros_args(args))
    window = TeleopWindow(node.teleop, node.publish, node.period, node.topic, node.status)
    close_on_signals(window)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    window.show()
    try:
        return app.exec_()
    finally:
        executor.shutdown()
        spinner.join()
        node.destroy_node()
        rclpy.try_shutdown()
