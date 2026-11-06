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

"""Tests of the hold-to-drive logic: key presses and releases in, commands out."""

import pytest
from warehouse_bringup.hold_to_drive import Command
from warehouse_bringup.hold_to_drive import HoldToDrive
from warehouse_bringup.hold_to_drive import is_bound
from warehouse_bringup.hold_to_drive import STOP

FORWARD = Command(0.3, 0.0)
LEFT = Command(0.0, 0.6)


def ticks(teleop, count):
    """Return what ``count`` publish periods send (None where nothing is due)."""
    return [teleop.tick() for _ in range(count)]


def test_idle_publishes_nothing():
    teleop = HoldToDrive()
    assert teleop.command == STOP
    assert not teleop.driving
    assert not teleop.publishing
    assert ticks(teleop, 3) == [None, None, None]


@pytest.mark.parametrize('key, command', [
    ('Up', Command(0.3, 0.0)), ('W', Command(0.3, 0.0)),
    ('Down', Command(-0.3, 0.0)), ('S', Command(-0.3, 0.0)),
    ('Left', Command(0.0, 0.6)), ('A', Command(0.0, 0.6)),
    ('Right', Command(0.0, -0.6)), ('D', Command(0.0, -0.6)),
])
def test_a_press_drives_at_once_and_repeats_while_held(key, command):
    teleop = HoldToDrive()
    assert teleop.press(key) == command
    assert teleop.driving and teleop.publishing
    assert ticks(teleop, 3) == [command] * 3


def test_a_second_press_of_a_held_key_changes_nothing():
    teleop = HoldToDrive()
    teleop.press('Up')
    assert teleop.press('Up') is None
    assert teleop.tick() == FORWARD


def test_release_sends_zero_commands_then_nothing():
    teleop = HoldToDrive(stop_messages=3)
    teleop.press('Up')
    assert teleop.release('Up') == STOP  # the first zero command, at once
    assert teleop.publishing and not teleop.driving
    assert ticks(teleop, 2) == [STOP, STOP]  # two more, one per period
    assert not teleop.publishing
    assert ticks(teleop, 3) == [None, None, None]  # then nothing: others may drive


def test_one_stop_message():
    teleop = HoldToDrive(stop_messages=1)
    teleop.press('Up')
    assert teleop.release('Up') == STOP
    assert not teleop.publishing
    assert teleop.tick() is None


def test_keys_combine_and_a_release_applies_at_once():
    teleop = HoldToDrive()
    teleop.press('Up')
    assert teleop.press('Left') == Command(0.3, 0.6)  # a curve to the left
    assert teleop.tick() == Command(0.3, 0.6)
    assert teleop.release('Left') == FORWARD  # straight on, at once
    assert teleop.tick() == FORWARD


def test_left_turns_counter_clockwise_also_backwards():
    teleop = HoldToDrive()
    teleop.press('Down')
    assert teleop.press('Left') == Command(-0.3, 0.6)


def test_opposite_keys_cancel_out_but_keep_publishing():
    teleop = HoldToDrive()
    teleop.press('Up')
    assert teleop.press('Down') == STOP
    assert teleop.publishing  # still held: the zero command repeats
    assert teleop.tick() == STOP
    assert teleop.release('Down') == FORWARD


def test_two_keys_for_one_direction():
    teleop = HoldToDrive()
    teleop.press('W')
    assert teleop.press('Up') == FORWARD  # not twice as fast
    assert teleop.release('Up') == FORWARD  # W is still held
    assert teleop.release('W') == STOP


def test_a_press_during_the_stop_drives_again():
    teleop = HoldToDrive()
    teleop.press('Up')
    teleop.release('Up')
    assert teleop.press('Left') == LEFT
    assert ticks(teleop, 3) == [LEFT] * 3  # no zero commands left over


def test_space_stops_and_forgets_held_keys():
    teleop = HoldToDrive(stop_messages=3)
    teleop.press('Up')
    teleop.press('Left')
    assert teleop.press('Space') == STOP
    assert not teleop.driving
    assert ticks(teleop, 3) == [STOP, STOP, None]
    # The keys are still down; their releases come later and change nothing.
    assert teleop.release('Up') is None
    assert teleop.release('Left') is None
    assert not teleop.publishing


def test_space_when_idle_sends_the_zero_commands():
    teleop = HoldToDrive(stop_messages=3)
    assert teleop.press('Space') == STOP
    assert ticks(teleop, 3) == [STOP, STOP, None]


def test_keyboard_lost_while_driving_sends_one_zero():
    teleop = HoldToDrive()
    teleop.press('Up')
    assert teleop.keyboard_lost() == STOP
    assert not teleop.publishing
    assert teleop.tick() is None
    assert teleop.release('Up') is None  # if the release arrives after all


def test_keyboard_lost_while_stopping_sends_one_zero_and_ends_the_stop():
    teleop = HoldToDrive(stop_messages=3)
    teleop.press('Up')
    teleop.release('Up')
    assert teleop.keyboard_lost() == STOP
    assert teleop.tick() is None


def test_keyboard_lost_when_idle_sends_nothing():
    teleop = HoldToDrive()
    assert teleop.keyboard_lost() is None
    assert not teleop.publishing


def test_speed_keys_while_driving_apply_at_once():
    teleop = HoldToDrive()
    teleop.press('Up')
    assert teleop.press('Q') == Command(0.4, 0.0)
    assert teleop.press('Z') == FORWARD
    teleop.press('Left')
    assert teleop.press('E') == Command(0.3, 0.7)
    assert teleop.press('C') == Command(0.3, 0.6)


def test_speed_keys_when_idle_publish_nothing():
    teleop = HoldToDrive()
    assert teleop.press('Q') is None
    assert teleop.press('E') is None
    assert (teleop.linear_speed, teleop.angular_speed) == (0.4, 0.7)
    assert not teleop.publishing
    assert teleop.press('Up') == Command(0.4, 0.0)


def test_speeds_stay_on_the_step_grid_and_within_limits():
    teleop = HoldToDrive(linear_speed=0.3, max_linear_speed=1.2)
    for _ in range(9):
        teleop.press('Q')
    assert teleop.linear_speed == 1.2
    assert teleop.press('Q') is None  # at the maximum
    assert teleop.linear_speed == 1.2
    for _ in range(20):
        teleop.press('Z')
    assert teleop.linear_speed == 0.1  # one step is the lowest speed


def test_bound_keys():
    for key in ('Up', 'Down', 'Left', 'Right', 'W', 'A', 'S', 'D', 'Space', 'Q', 'Z', 'E', 'C'):
        assert is_bound(key)
    for key in ('X', 'Esc', 'Return', ''):
        assert not is_bound(key)
    teleop = HoldToDrive()
    assert teleop.press('X') is None
    assert teleop.release('X') is None
    assert not teleop.publishing


@pytest.mark.parametrize('arguments', [
    {'linear_speed': 0.0},
    {'linear_speed': -0.3},
    {'linear_speed': 1.5, 'max_linear_speed': 1.2},
    {'angular_speed': 0.0},
    {'angular_speed': 2.5, 'max_angular_speed': 2.0},
    {'speed_step': 0.0},
    {'stop_messages': 0},
])
def test_bad_values_are_refused(arguments):
    with pytest.raises(ValueError):
        HoldToDrive(**arguments)
