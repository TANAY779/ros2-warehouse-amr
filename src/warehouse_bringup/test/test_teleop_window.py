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
Tests of the teleop window with synthetic Qt events, on Qt's offscreen platform.

They check the thin Qt layer around HoldToDrive: auto-repeat events are ignored, presses and
releases publish at once, the publish timer repeats and then stops, and losing the keyboard
(focus, close, Ctrl-C) sends one zero command. No ROS: commands go into a list.
"""

import os
import signal
import threading

import pytest
from python_qt_binding.QtCore import QEvent
from python_qt_binding.QtCore import QEventLoop
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFocusEvent
from python_qt_binding.QtGui import QKeyEvent
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QLabel
from warehouse_bringup.hold_to_drive import Command
from warehouse_bringup.hold_to_drive import HoldToDrive
from warehouse_bringup.hold_to_drive import STOP
from warehouse_bringup.teleop_window import close_on_signals
from warehouse_bringup.teleop_window import TeleopWindow

PERIOD = 0.05  # s: 20 Hz, the default publish rate
FORWARD = Command(0.3, 0.0)


@pytest.fixture(scope='module')
def app():
    # Qt reads the platform when the application starts: no window on the screen, and no
    # display needed.
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    return QApplication.instance() or QApplication([])


@pytest.fixture
def sent():
    """Every command the window publishes, in order."""
    return []


@pytest.fixture
def window(app, sent):
    window = TeleopWindow(HoldToDrive(), sent.append, PERIOD, '/robot1/cmd_vel')
    window.show()
    yield window
    window.close()


def press(window, key, auto_repeat=False, modifiers=Qt.NoModifier):
    QApplication.sendEvent(window, QKeyEvent(QEvent.KeyPress, key, modifiers, '', auto_repeat))


def release(window, key, auto_repeat=False, modifiers=Qt.NoModifier):
    QApplication.sendEvent(window,
                           QKeyEvent(QEvent.KeyRelease, key, modifiers, '', auto_repeat))


def lose_focus(window):
    QApplication.sendEvent(window, QFocusEvent(QEvent.FocusOut, Qt.ActiveWindowFocusReason))


def run_event_loop(seconds):
    """Let Qt run its timers for ``seconds``."""
    loop = QEventLoop()
    QTimer.singleShot(round(seconds * 1000), loop.quit)
    loop.exec_()


def window_text(window):
    return ' '.join(label.text() for label in window.findChildren(QLabel))


def test_a_press_publishes_at_once(window, sent):
    press(window, Qt.Key_Up)
    assert sent == [FORWARD]  # before Qt runs any timer


def test_a_held_key_repeats_at_the_publish_rate(window, sent):
    press(window, Qt.Key_Up)
    run_event_loop(0.5)
    # 1 at once and 10 in 0.5 s at 20 Hz; a busy machine may fire the timer a little late.
    assert 7 <= len(sent) <= 12
    assert set(sent) == {FORWARD}


def test_auto_repeat_events_change_nothing(window, sent):
    press(window, Qt.Key_Up)
    for _ in range(10):
        # X11 delivers each repeat of a held key as a release and a press, both marked.
        release(window, Qt.Key_Up, auto_repeat=True)
        press(window, Qt.Key_Up, auto_repeat=True)
    assert sent == [FORWARD]
    release(window, Qt.Key_Up)
    assert sent == [FORWARD, STOP]


def test_release_sends_three_zero_commands_then_nothing(window, sent):
    press(window, Qt.Key_Up)
    release(window, Qt.Key_Up)
    assert sent == [FORWARD, STOP]  # the first zero command at once
    run_event_loop(0.4)
    assert sent == [FORWARD, STOP, STOP, STOP]


def test_keys_combine(window, sent):
    press(window, Qt.Key_Up)
    press(window, Qt.Key_Left)
    assert sent[-1] == Command(0.3, 0.6)
    release(window, Qt.Key_Left)
    assert sent[-1] == FORWARD


@pytest.mark.parametrize('how', ['focus', 'close'])
def test_losing_the_keyboard_sends_one_zero_command(window, sent, how):
    press(window, Qt.Key_Up)
    if how == 'focus':
        lose_focus(window)
    else:
        window.close()
    assert sent == [FORWARD, STOP]
    run_event_loop(0.3)
    release(window, Qt.Key_Up)  # if the release arrives after all
    assert sent == [FORWARD, STOP]


def test_losing_the_keyboard_when_idle_sends_nothing(window, sent):
    lose_focus(window)
    window.close()
    assert sent == []


def test_space_stops_a_held_key(window, sent):
    press(window, Qt.Key_Up)
    press(window, Qt.Key_Space)
    assert sent == [FORWARD, STOP]
    press(window, Qt.Key_Up, auto_repeat=True)  # Up is still down
    run_event_loop(0.4)
    release(window, Qt.Key_Up)
    assert sent == [FORWARD, STOP, STOP, STOP]


def test_speed_keys(window, sent):
    press(window, Qt.Key_Q)
    release(window, Qt.Key_Q)
    assert sent == []  # standing still: nothing to publish
    assert 'Driving speed 0.40 m/s' in window_text(window)
    press(window, Qt.Key_Up)
    assert sent == [Command(0.4, 0.0)]
    assert 'linear.x +0.40 m/s' in window_text(window)


def test_shortcuts_do_not_drive_but_their_releases_count(window, sent):
    press(window, Qt.Key_C, modifiers=Qt.ControlModifier)  # Ctrl+C in the window
    press(window, Qt.Key_Up, modifiers=Qt.ControlModifier)
    assert sent == []
    assert 'turning speed 0.60 rad/s' in window_text(window)
    press(window, Qt.Key_Up)
    release(window, Qt.Key_Up, modifiers=Qt.ControlModifier)
    assert sent == [FORWARD, STOP]


@pytest.mark.parametrize('driving', [True, False])
def test_ctrl_c_closes_the_window_after_one_zero_command(window, sent, driving):
    handlers = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    close_on_signals(window)
    try:
        if driving:
            press(window, Qt.Key_Up)
        # Sent from another thread while Qt's event loop runs, as Ctrl-C in the terminal is.
        threading.Timer(0.1, os.kill, (os.getpid(), signal.SIGINT)).start()
        run_event_loop(0.6)
    finally:
        # Python has run the handler by now: it runs pending handlers as soon as the main
        # thread is back in Python code, which it is here.
        for number, handler in handlers.items():
            signal.signal(number, handler)
    assert not window.isVisible()
    if driving:
        assert sent[-1] == STOP
        assert sent.count(STOP) == 1
        assert set(sent[:-1]) == {FORWARD}
    else:
        assert sent == []
