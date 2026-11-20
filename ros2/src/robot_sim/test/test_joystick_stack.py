"""The Bluetooth remote on a running robot stack. Skips without ROS, ~1-2 min.

joystick_teleop runs in fake mode (events on joystick/fake_events, the same
JSON the evdev source would produce), and everything behind it is real:
twist_mux, the bridge, the simulated robot. The claims: the remote drives the
robot through the mux; silence stops it within the deadman (0.25 s) plus the
bridge's margin; a dropped link stops it and the stick must be re-armed; and
arming with the stick already pushed is refused.
"""

import json
import math
import threading
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from geometry_msgs.msg import Twist                     # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.executors import SingleThreadedExecutor      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

QUIET_GPS = ("-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")
EVENT_PERIOD_S = 0.03
DEADMAN_S = 0.25
RELEASE_S = 2.0


class JoyWatcher:
    """Records /cmd_vel, /cmd_vel_joy, /joy/state and /ground_truth with
    wall-clock arrival times, and publishes the fake remote's events."""

    def __init__(self):
        self.node = rclpy.create_node("joystick_watcher")
        self.cmd, self.joy, self.states, self.truth = [], [], [], []
        stamp = time.monotonic
        self.node.create_subscription(
            Twist, "/cmd_vel", lambda m: self.cmd.append((stamp(), m.linear.x)), 50)
        self.node.create_subscription(
            Twist, "/cmd_vel_joy", lambda m: self.joy.append((stamp(), m.linear.x)), 50)
        self.node.create_subscription(
            String, "/joy/state", lambda m: self.states.append((stamp(), json.loads(m.data))), 50)
        self.node.create_subscription(
            Odometry, "/ground_truth",
            lambda m: self.truth.append((stamp(), m.pose.pose.position.x,
                                         m.pose.pose.position.y)), 50)
        self.events = self.node.create_publisher(String, "/joystick/fake_events", 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def wait_ready(self, timeout=60.0):
        """Ground truth flowing AND the joystick node subscribed to our events."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.truth and self.states
                    and self.events.get_subscription_count() > 0):
                time.sleep(0.5)       # let the DDS match settle
                return
            time.sleep(0.1)
        raise AssertionError(
            f"stack not ready: truth {len(self.truth)}, joy/state {len(self.states)}, "
            f"fake_events subscribers {self.events.get_subscription_count()}")

    def send(self, kind, code="", value=0.0):
        self.events.publish(String(data=json.dumps(
            {"kind": kind, "code": code, "value": value})))
        return time.monotonic()

    def stick(self, y, seconds):
        """Stream REL_Y events every 30 ms; returns the time of the last one."""
        end = time.monotonic() + seconds
        last = time.monotonic()
        while time.monotonic() < end:
            last = self.send("rel", "y", y)
            time.sleep(EVENT_PERIOD_S)
        return last

    def arm(self):
        self.send("connected", value=1)
        self.send("battery", value=80)
        time.sleep(0.2)
        self.send("key", "BTN_SIDE", 1)
        self.send("key", "BTN_SIDE", 0)
        time.sleep(0.2)

    def truth_at(self, t):
        before = [p for p in self.truth if p[0] <= t]
        return before[-1] if before else None

    def state(self):
        return self.states[-1][1]

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.node.destroy_node()


@pytest.fixture
def watch(joystick_stack):
    joystick_stack([], *QUIET_GPS)
    w = JoyWatcher()
    try:
        w.wait_ready()
        yield w
    finally:
        w.close()


def test_joystick_drives_and_silence_stops_it(watch):
    watch.arm()
    assert watch.state()["armed"] is True, watch.state()
    start = time.monotonic()
    last = watch.stick(-17, 3.0)        # forward, level 1 (0.2 m/s)
    time.sleep(RELEASE_S + 1.5)

    driving = [c for c in watch.cmd if start < c[0] < last]
    assert driving, "nothing reached /cmd_vel: the joystick did not go through twist_mux"
    assert max(c[1] for c in driving) == pytest.approx(0.2, abs=1e-6)

    p0, p1 = watch.truth_at(start), watch.truth_at(last)
    assert p0 is not None and p1 is not None, "no /ground_truth"
    travel = p1[1] - p0[1]
    print(f"\nMEASURED travel={travel:.3f} m, lateral={abs(p1[2] - p0[2]):.3f} m")
    assert 0.4 < travel < 0.8 and abs(p1[2] - p0[2]) < 0.1, (p0, p1)

    zero = next((c for c in watch.cmd if c[0] > last and c[1] == 0.0), None)
    assert zero is not None, "the robot was never told to stop"
    first_zero = zero[0] - last
    rest = watch.truth_at(last + 1.5)
    coast = math.hypot(rest[1] - p1[1], rest[2] - p1[2])
    print(f"MEASURED first_zero={first_zero:.3f} s, coast={coast:.3f} m")
    assert first_zero < DEADMAN_S + 0.15, f"stop came {first_zero:.2f} s late"
    assert coast < 0.12, f"the robot rolled {coast:.2f} m after the remote went quiet"

    late = [t for t in watch.joy if t[0] > last + RELEASE_S + 0.2]
    assert not late, f"/cmd_vel_joy still published {len(late)} times after the release"


def test_disconnect_mid_drive_stops_and_reconnect_needs_rearm(watch):
    watch.arm()
    watch.stick(-17, 1.5)
    t_drop = watch.send("connected", value=0)
    time.sleep(0.5)
    zero = next((c for c in watch.cmd if c[0] > t_drop and c[1] == 0.0), None)
    assert zero is not None and zero[0] - t_drop < 0.15, \
        f"no zero within 0.15 s of the link dropping ({zero})"
    assert watch.state()["armed"] is False, watch.state()

    # The gate holds zeros for release_s after a disarm (the wheels stay
    # commanded to 0, not handed back at once). Let that tail end first.
    time.sleep(RELEASE_S)
    assert all(x == 0.0 for t, x in watch.joy if t > t_drop), "drove after the link dropped"

    # Back on the air, stick pushed, but nobody clicked: it must stay silent.
    watch.send("connected", value=1)
    time.sleep(0.3)
    t0 = time.monotonic()
    p0 = watch.truth_at(t0)
    watch.stick(-17, 1.5)
    time.sleep(0.5)
    assert not [t for t in watch.joy if t[0] > t0], "cmd_vel_joy spoke without a re-arm"
    p1 = watch.truth[-1]
    assert math.hypot(p1[1] - p0[1], p1[2] - p0[2]) < 0.02, (p0, p1)


def test_arming_with_the_stick_pushed_is_refused(watch):
    watch.send("connected", value=1)
    watch.send("battery", value=80)
    time.sleep(0.2)
    t0 = time.monotonic()
    p0 = watch.truth_at(t0)
    # Stream stick events from a thread while the arm button goes down.
    stop = threading.Event()

    def stream():
        while not stop.is_set():
            watch.send("rel", "y", -17)
            time.sleep(EVENT_PERIOD_S)

    streamer = threading.Thread(target=stream, daemon=True)
    streamer.start()
    try:
        time.sleep(0.3)
        watch.send("key", "BTN_SIDE", 1)
        watch.send("key", "BTN_SIDE", 0)
        time.sleep(1.0)
    finally:
        stop.set()
        streamer.join(timeout=2)
    state = watch.state()
    assert state["armed"] is False, state
    # Exactly the stick-pushed refusal, with the link and battery events
    # having landed — otherwise "not connected" / "low battery" would pass too.
    assert state["connected"] is True and state["battery"] == 80, state
    assert state["refusal"] == "çubuk itiliyken etkinleştirilemez", state
    assert not [t for t in watch.joy if t[0] > t0]
    p1 = watch.truth[-1]
    assert math.hypot(p1[1] - p0[1], p1[2] - p0[2]) < 0.02, (p0, p1)
