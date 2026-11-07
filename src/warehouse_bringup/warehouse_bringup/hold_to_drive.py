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
Hold-to-drive keyboard teleop: which command to send, and when.

Plain Python with no Qt and no ROS, so it can be driven with plain function calls. The window
(teleop_window.py) turns Qt key events into key names, drops the desktop's auto-repeat events,
publishes every command these methods return, and calls tick() once per publish period while
``publishing`` is true.

The rules:

- A driving key (arrows, W A S D) drives while it is held. Keys held together combine: forward
  and left drive a curve to the left. Every change is returned at once, so it is published
  without waiting for the next period.
- While a driving key is held, tick() repeats the command every period, because
  diff_drive_controller brakes once the newest command is cmd_vel_timeout (0.15 s) old.
- When the last driving key is released, or Space is pressed, a few zero commands follow, one
  per period, then nothing: the robot stands still and another command source may drive it.
- When the window loses the keyboard (focus lost, window closed, Ctrl-C), keyboard_lost()
  forgets the held keys and returns one zero command if teleop was publishing: a dead-man
  switch. The window never gets the release of a key that was held at that moment.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    """A velocity command: linear.x [m/s] and angular.z [rad/s] of a TwistStamped."""

    linear: float = 0.0
    angular: float = 0.0


STOP = Command()

FORWARD = 'forward'
BACKWARD = 'backward'
LEFT = 'left'
RIGHT = 'right'

# Key names as Qt writes them (QKeySequence(key).toString()). Left always turns the robot
# counter-clockwise (positive angular.z, REP-103), also while it drives backwards, as a
# joystick does.
DRIVING_KEYS = {
    'Up': FORWARD, 'W': FORWARD,
    'Down': BACKWARD, 'S': BACKWARD,
    'Left': LEFT, 'A': LEFT,
    'Right': RIGHT, 'D': RIGHT,
}
# Speed keys: (which speed, up or down). Two columns beside W A S D: Q/Z for driving, E/C for
# turning (teleop_twist_keyboard also uses e/c for the turning speed).
SPEED_KEYS = {
    'Q': ('linear', 1), 'Z': ('linear', -1),
    'E': ('angular', 1), 'C': ('angular', -1),
}
STOP_KEY = 'Space'


def is_bound(key):
    """Return True if the key drives, stops or changes a speed."""
    return key in DRIVING_KEYS or key in SPEED_KEYS or key == STOP_KEY


class HoldToDrive:
    """
    Keyboard state in, velocity commands out.

    The speeds are magnitudes; the driving keys give the sign. The speed keys change a speed in
    steps of ``speed_step``, from ``speed_step`` up to the maximum. ``stop_messages`` is the
    number of zero commands sent after a stop (the first one at once, then one per period).
    More than one, because diff_drive_controller subscribes to cmd_vel best effort (see
    ``ros2 topic info -v``): a lost message is not sent again. If all of them were lost, the
    controller would still brake once the last driving command is cmd_vel_timeout old.
    """

    def __init__(self, linear_speed=0.3, angular_speed=0.6, max_linear_speed=1.2,
                 max_angular_speed=2.0, speed_step=0.1, stop_messages=3):
        if not speed_step > 0.0:
            raise ValueError(f'speed_step must be above 0, not {speed_step}')
        for name, speed, maximum in (('linear_speed', linear_speed, max_linear_speed),
                                     ('angular_speed', angular_speed, max_angular_speed)):
            if not 0.0 < speed <= maximum:
                raise ValueError(
                    f'{name} must be above 0 and at most max_{name} ({maximum}), not {speed}')
        if stop_messages < 1:
            raise ValueError(f'stop_messages must be at least 1, not {stop_messages}')
        self._speed = {'linear': float(linear_speed), 'angular': float(angular_speed)}
        self._max_speed = {'linear': float(max_linear_speed),
                           'angular': float(max_angular_speed)}
        self._step = speed_step
        self._stop_messages = stop_messages
        self._held = {}       # each held driving key -> its direction (FORWARD, LEFT, ...)
        self._zeros_due = 0   # zero commands still to send after a stop

    @property
    def linear_speed(self):
        """Return the driving speed [m/s] of the forward and backward keys."""
        return self._speed['linear']

    @property
    def angular_speed(self):
        """Return the turning speed [rad/s] of the left and right keys."""
        return self._speed['angular']

    @property
    def command(self):
        """Return the command for the driving keys held now; STOP when none is held."""
        directions = set(self._held.values())  # W and Up held together count once
        # A bool minus a bool is 1, 0 or -1: forward and backward held together cancel out.
        linear = (FORWARD in directions) - (BACKWARD in directions)
        angular = (LEFT in directions) - (RIGHT in directions)
        return Command(linear * self._speed['linear'], angular * self._speed['angular'])

    @property
    def driving(self):
        """Return True while at least one driving key is held."""
        return bool(self._held)

    @property
    def publishing(self):
        """Return True while a command is due every publish period (call tick())."""
        return bool(self._held) or self._zeros_due > 0

    def press(self, key):
        """
        Handle a key press; return the command to publish now, or None.

        The window must not pass on auto-repeat presses. A second press of a held key changes
        nothing anyway.
        """
        if key in DRIVING_KEYS:
            if key in self._held:
                return None
            self._held[key] = DRIVING_KEYS[key]
            self._zeros_due = 0  # a new drive ends the zeros of the last stop
            return self.command
        if key in SPEED_KEYS:
            changed = self._change_speed(*SPEED_KEYS[key])
            # While driving, the new speed applies at once; otherwise from the next key press.
            return self.command if changed and self._held else None
        if key == STOP_KEY:
            return self.stop()
        return None

    def release(self, key):
        """Handle a key release; return the command to publish now, or None."""
        if key not in self._held:
            return None  # not a driving key, or already forgotten by stop() or keyboard_lost()
        del self._held[key]
        if self._held:
            return self.command  # the keys still held drive on
        return self._start_stopping()

    def stop(self):
        """Stop (Space): forget the held keys and return the first of the zero commands."""
        self._held.clear()
        return self._start_stopping()

    def keyboard_lost(self):
        """
        Forget the held keys: the window lost the keyboard (a dead-man switch).

        Return one zero command if teleop was publishing, else None. Nothing follows it.
        """
        was_publishing = self.publishing
        self._held.clear()
        self._zeros_due = 0
        return STOP if was_publishing else None

    def tick(self):
        """Return the command due in this publish period, or None when nothing is due."""
        if self._held:
            return self.command
        if self._zeros_due > 0:
            self._zeros_due -= 1
            return STOP
        return None

    def _start_stopping(self):
        self._zeros_due = self._stop_messages - 1  # the first zero command goes out now
        return STOP

    def _change_speed(self, which, direction):
        """Change one speed by one step; return False if it is at its limit."""
        old = self._speed[which]
        # Rounding keeps the speeds on the step grid: 0.1 + 0.2 is 0.30000000000000004.
        new = min(round(old + direction * self._step, 6), self._max_speed[which])
        if new < self._step or new == old:
            return False
        self._speed[which] = new
        return True
