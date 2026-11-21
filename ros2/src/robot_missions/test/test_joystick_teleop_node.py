"""joystick_teleop node in-process: shutdown and startup edges. Skips without ROS.

The driving behaviour itself is the gate's (test_joystick_gate.py) and the
running stack's (robot_sim/test/test_joystick_stack.py); this covers what
only the node does: close() leaves one zero on cmd_vel_joy if the gate still
had the wheels, and a missing python3-evdev stops the node at startup.
"""

import json
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from geometry_msgs.msg import Twist                     # noqa: E402
from rclpy.executors import SingleThreadedExecutor      # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from robot_missions.joystick_teleop import JoystickTeleop   # noqa: E402


@pytest.fixture
def context():
    own = not rclpy.ok()
    if own:
        rclpy.init()
    yield
    if own:
        rclpy.shutdown()


@pytest.fixture
def watcher(context):
    node = rclpy.create_node("joy_close_watcher")
    got = []
    node.create_subscription(Twist, "cmd_vel_joy",
                             lambda m: got.append((m.linear.x, m.angular.z)), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    yield got
    ex.shutdown()
    t.join(timeout=5)
    node.destroy_node()


def _fake_node():
    node = JoystickTeleop(parameter_overrides=[Parameter("fake", value=True)])
    deadline = time.monotonic() + 10
    while node._pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert node._pub.get_subscription_count() > 0, "watcher never matched"
    return node


def _send(node, kind, code="", value=0):
    node._on_fake(String(data=json.dumps({"kind": kind, "code": code, "value": value})))


def _settle(got, n, timeout=3.0):
    deadline = time.monotonic() + timeout
    while len(got) < n and time.monotonic() < deadline:
        time.sleep(0.02)
    time.sleep(0.3)                     # and nothing more after that


def test_close_while_driving_leaves_one_zero(watcher):
    node = _fake_node()                 # not spun: no timer publishes
    try:
        _send(node, "connected", value=1)
        _send(node, "key", "BTN_SIDE", 1)
        _send(node, "rel", "y", -17)
        assert node._gate.output() == pytest.approx((0.2, 0.0))
        node.close()
        _settle(watcher, 1)
        assert watcher == [(0.0, 0.0)]
    finally:
        node.destroy_node()


def test_close_while_silent_publishes_nothing(watcher):
    node = _fake_node()
    try:
        _send(node, "connected", value=1)     # connected, not armed: silent
        assert node._gate.output() is None
        node.close()
        _settle(watcher, 1, timeout=0.5)
        assert watcher == []
    finally:
        node.destroy_node()


def test_missing_evdev_fails_at_startup(context, monkeypatch):
    monkeypatch.setitem(sys.modules, "evdev", None)     # import evdev -> ImportError
    with pytest.raises(ImportError):
        JoystickTeleop(parameter_overrides=[Parameter("fake", value=False)])
