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

    ros2 run warehouse_bringup teleop_window --ros-args -r __ns:=/robot1 -p use_sim_time:=true

teleop_twist_keyboard reads characters from a terminal, and a terminal never learns that a key
was released: it sends one command per key event and leaves the stopping to cmd_vel_timeout. A
window gets a press and a release event for every key, so this one drives exactly while a key is
held (warehouse_bringup README, "Driving with teleop").

Three layers:

- HoldToDrive (hold_to_drive.py): which command to send and when. No Qt, no ROS.
- TeleopWindow: Qt key, focus and close events in, commands out, and a timer for the rate.
- TeleopNode: the ROS side. Parameters, and the cmd_vel publisher, which stamps each command
  with the node's clock.

The window lives in the main thread, as Qt requires. ROS callbacks run in a second thread: the
/clock subscription that keeps sim time, and the parameter services.
"""

import signal
import threading

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
from rclpy.exceptions import ParameterException
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from warehouse_bringup.hold_to_drive import HoldToDrive
from warehouse_bringup.hold_to_drive import is_bound

# A key pressed with one of these held is a shortcut (Ctrl+C, Alt+Tab), not driving. Releases
# always count, so a key released while Ctrl is down still stops the robot.
SHORTCUT_MODIFIERS = Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier

# The project rule (controllers.yaml): while the robot should move, publish at 10 Hz or more,
# or diff_drive_controller brakes between two commands (cmd_vel_timeout 0.15 s).
MIN_PUBLISH_RATE = 10.0  # Hz

STATUS_PERIOD_MS = 200

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

    ``teleop`` is a HoldToDrive, ``publish(command)`` sends one command, ``period`` is the
    publish period [s], ``topic`` is shown in the title, and ``status()`` (optional) returns a
    line about the connection for the bottom of the window. No ROS in here.
    """

    def __init__(self, teleop, publish, period, topic, status=None):
        super().__init__()
        self._teleop = teleop
        self._publish = publish
        self._status = status
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

        # Refreshes the status line. It also lets Ctrl-C through: Python runs a signal handler
        # only when the main thread runs Python code, and Qt's event loop is C++.
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(STATUS_PERIOD_MS)
        self._status_timer.timeout.connect(self._show_status)
        self._status_timer.start()

        self._speeds_label = QLabel()
        self._command_label = QLabel()
        self._status_label = QLabel()
        self._status_label.setWordWrap(True)
        layout = QVBoxLayout(self)
        for label in (QLabel(HELP), self._speeds_label, self._command_label,
                      self._status_label):
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
            self._publish(command)
            # Restart the period, so the next repeat comes one full period after this command.
            self._publish_timer.start()
        if not self._teleop.publishing:
            self._publish_timer.stop()
        self._show_command()

    def _on_publish_period(self):
        command = self._teleop.tick()
        if command is not None:
            self._publish(command)
        if not self._teleop.publishing:
            self._publish_timer.stop()
        self._show_command()

    def _show_command(self):
        teleop = self._teleop
        self._speeds_label.setText(
            f'Driving speed {teleop.linear_speed:.2f} m/s (Q/Z), '
            f'turning speed {teleop.angular_speed:.2f} rad/s (E/C)')
        if teleop.driving:
            state = 'driving'
        elif teleop.publishing:
            state = 'stopping: sending zero commands'
        else:
            state = 'idle: publishing nothing'
        command = teleop.command
        self._command_label.setText(
            f'Command: linear.x {command.linear:+.2f} m/s, angular.z {command.angular:+.2f} '
            f'rad/s ({state})')

    def _show_status(self):
        lines = [self._status()] if self._status is not None else []
        if not self.hasFocus():
            lines.append('Click this window to drive: keys go to the window that has the focus.')
        self._status_label.setText('\n'.join(lines))


class TeleopNode(Node):
    """The ROS side: parameters, and the cmd_vel publisher."""

    def __init__(self):
        super().__init__('teleop_window')
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
        # A relative name: run the node in the robot's namespace (-r __ns:=/robot1) and it
        # publishes /robot1/cmd_vel. Reliable with depth 10, as teleop_twist_keyboard does.
        self._publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)

    def _parameter(self, name, default, description):
        # Read once at the start; the window would never see a later change, so refuse it.
        descriptor = ParameterDescriptor(description=description, read_only=True)
        return self.declare_parameter(name, default, descriptor).value

    @property
    def topic(self):
        """Return the full name of the command topic, such as /robot1/cmd_vel."""
        return self._publisher.topic_name

    def publish(self, command):
        """Publish one command, stamped now with this node's clock."""
        message = TwistStamped()
        # With use_sim_time:=true this is sim time. diff_drive_controller judges a command's
        # age from its stamp on its own clock, so the two clocks must agree (plan R1).
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = command.linear
        message.twist.angular.z = command.angular
        self._publisher.publish(message)

    def status(self):
        """Return a line for the window: who listens, and whether sim time has started."""
        count = self._publisher.get_subscription_count()
        if count == 0:
            text = f'Nothing subscribes to {self.topic}: is the robot running in this namespace?'
        else:
            text = f'{count} subscriber(s) on {self.topic}.'
        if self.get_clock().now().nanoseconds == 0:
            text += ' No /clock yet: is the simulation running?'
        return text


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
