"""joystick_source: event mapping, device choice, BlueZ battery (no evdev, no ROS)."""

import errno
import math
import subprocess
import sys
import threading
import types

import pytest

from robot_missions.joystick_gate import JoyEvent
from robot_missions.joystick_gate import Buttons, JoystickGate, apply_event
from robot_missions.joystick_source import (BatteryPoller, EvdevSource, bluez_battery,
                                           evdev_key_name, parse_fake_event,
                                           pick_device, to_event)


def _name(code):
    return f"KEY_{code}"


def test_to_event_rel_axes():
    assert to_event(2, 0, 17, _name) == JoyEvent("rel", "x", 17)
    assert to_event(2, 1, -3, _name) == JoyEvent("rel", "y", -3)


def test_to_event_wheel_and_abs_are_other():
    assert to_event(2, 8, 1, _name).kind == "other"
    assert to_event(3, 16, -1, _name).kind == "other"


def test_to_event_key_uses_name():
    assert to_event(1, 275, 1, lambda c: "BTN_SIDE") == JoyEvent("key", "BTN_SIDE", 1)


def test_to_event_syn_and_msc_dropped():
    assert to_event(0, 0, 0, _name) is None
    assert to_event(4, 4, 0x90004, _name) is None


def test_pick_device_prefers_the_node_with_rel():
    cands = [("/dev/input/event5", "VR-PARK", "", False),
             ("/dev/input/event6", "VR-PARK", "", True)]
    assert pick_device(cands, "VR-PARK", "") == "/dev/input/event6"


def test_pick_device_first_match_wins():
    cands = [("/dev/input/event7", "VR-PARK", "", True),
             ("/dev/input/event6", "VR-PARK", "", True)]
    assert pick_device(cands, "VR-PARK", "") == "/dev/input/event7"


def test_pick_device_respects_mac():
    cands = [("/dev/input/event6", "VR-PARK", "7f:ab:6e:93:c4:31", True)]
    assert pick_device(cands, "VR-PARK", "7F:AB:6E:93:C4:30") is None
    cands = [("/dev/input/event6", "VR-PARK", "7f:ab:6e:93:c4:30", True)]
    assert pick_device(cands, "VR-PARK", "7F:AB:6E:93:C4:30") == "/dev/input/event6"


def test_pick_device_name_must_match():
    cands = [("/dev/input/event6", "VR-PARK", "", True)]
    assert pick_device(cands, "VR-PARK2", "") is None


def test_bluez_battery_parses():
    seen = []

    def run(cmd, **kw):
        seen.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, stdout="y 49\n", stderr="")

    assert bluez_battery("7F:AB:6E:93:C4:30", run=run) == 49.0
    assert any(str(p).endswith("dev_7F_AB_6E_93_C4_30") for p in seen[0])


def test_bluez_battery_failures_are_none():
    def bad_rc(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="x")

    def garbage(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, stdout="garbage", stderr="")

    def missing(cmd, **kw):
        raise FileNotFoundError

    def slow(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, 2)

    for run in (bad_rc, garbage, missing, slow):
        assert bluez_battery("7F:AB:6E:93:C4:30", run=run) is None


# --- parse_fake_event --------------------------------------------------------

def test_parse_fake_event_accepts():
    assert parse_fake_event('{"kind":"rel","code":"x","value":3}') == JoyEvent("rel", "x", 3.0)
    assert math.isnan(parse_fake_event('{"kind":"battery","code":"","value":NaN}').value)


@pytest.mark.parametrize("text", [
    "garbage", '{"kind":"rel"}', '{"kind":"rel","code":"x"}', '[1,2]', 'null',
    '{"kind":"nope","code":"","value":1}',
    '{"kind":"rel","code":1,"value":1}',
    '{"kind":"rel","code":"x","value":"5"}',
    '{"kind":"rel","code":"x","value":true}',
    '{"kind":"rel","code":"x","value":null}',
    '{"kind":"rel","code":"x","value":Infinity}',
    '{"kind":"rel","code":"x","value":NaN}',
    '{"kind":"connected","code":"","value":NaN}',
])
def test_parse_fake_event_rejects(text):
    with pytest.raises((ValueError, TypeError, KeyError)):
        parse_fake_event(text)


# --- EvdevSource with a fake evdev module ------------------------------------

class _Ev:
    def __init__(self, t, c, v):
        self.type, self.code, self.value = t, c, v


class _Dev:
    def __init__(self, script, grab_exc=None, path="/dev/input/event6"):
        self.path, self.name, self.uniq, self.fd = path, "VR-PARK", "7f:ab:6e:93:c4:30", -1
        self.script = list(script)
        self.grab_exc = grab_exc
        self.closed = 0

    def capabilities(self):
        return {2: [0, 1]}

    def grab(self):
        if self.grab_exc:
            raise self.grab_exc

    def read_one(self):
        if not self.script:
            return None
        item = self.script.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.closed += 1


@pytest.fixture
def fake_evdev(monkeypatch):
    devices = []                                    # a list of lists of _Dev, one per scan
    scans = []
    mod = types.ModuleType("evdev")
    ecodes = types.SimpleNamespace(EV_REL=2, KEY={}, BTN={275: "BTN_SIDE"})
    mod.ecodes = ecodes

    def list_devices():
        scans.append(1)
        return [d.path for d in (devices.pop(0) if len(devices) > 1 else devices[0])]

    cur = {}

    def input_device(path):
        return cur[path]

    mod.list_devices = list_devices
    mod.InputDevice = input_device
    monkeypatch.setitem(sys.modules, "evdev", mod)
    monkeypatch.setitem(sys.modules, "evdev.ecodes", ecodes)
    import robot_missions.joystick_source as js
    monkeypatch.setattr(js.select, "select", lambda *a, **k: ([], [], []))

    def add(*devs):
        for d in devs:
            cur[d.path] = d
        devices.append(list(devs))
    return add, scans


def _run(source, until, timeout=3.0):
    import time
    source.start()
    end = time.monotonic() + timeout
    while time.monotonic() < end and not until():
        time.sleep(0.01)
    source.stop()


def test_evdev_source_session_loss_and_rescan(fake_evdev):
    add, scans = fake_evdev
    d1 = _Dev([_Ev(2, 0, 5), _Ev(0, 0, 0), _Ev(1, 275, 1),
               OSError(errno.ENODEV, "gone")])
    add(d1)
    got, logs = [], []
    src = EvdevSource("VR-PARK", "", got.append, retry_s=0.01, log=logs.append)
    _run(src, lambda: len(scans) >= 2 and JoyEvent("connected", "", 0) in got)
    assert got[:4] == [JoyEvent("connected", "", 1), JoyEvent("rel", "x", 5),
                       JoyEvent("key", "BTN_SIDE", 1), JoyEvent("connected", "", 0)]
    assert got[3] == JoyEvent("connected", "", 0) and got[4] != JoyEvent("connected", "", 0)
    assert d1.closed >= 1
    assert len(scans) >= 2                          # rescanned after the loss
    assert any("lost" in m for m in logs)
    assert src._thread is not None and not src._thread.is_alive()


def test_evdev_source_bad_event_keeps_reader_alive(fake_evdev):
    add, _ = fake_evdev
    add(_Dev([_Ev(1, 275, 1), _Ev(2, 0, 7)]))
    got, logs = [], []

    def emit(ev):
        if ev == JoyEvent("key", "BTN_SIDE", 1):
            raise RuntimeError("boom")
        got.append(ev)

    src = EvdevSource("VR-PARK", "", emit, retry_s=0.01, log=logs.append)
    _run(src, lambda: JoyEvent("rel", "x", 7) in got)
    assert JoyEvent("rel", "x", 7) in got
    assert any("dropped" in m for m in logs)


def test_evdev_source_grab_failure_logged_no_connected_0(fake_evdev):
    add, _ = fake_evdev
    d = _Dev([], grab_exc=OSError(errno.EBUSY, "busy"))
    add(d)
    got, logs = [], []
    src = EvdevSource("VR-PARK", "", got.append, retry_s=0.01, log=logs.append)
    _run(src, lambda: d.closed >= 2)
    assert got == []                                # never connected 1, so never connected 0
    assert any("cannot open" in m for m in logs)
    assert d.closed >= 2                            # closed and retried


def test_evdev_source_clears_mac_before_connected_0(fake_evdev):
    """A stale MAC after a loss would be polled while BlueZ has no Battery1."""
    add, _ = fake_evdev
    add(_Dev([_Ev(2, 0, 5), OSError(errno.ENODEV, "gone")]))
    seen = []                                       # (event, src.mac at emit time)
    src = EvdevSource("VR-PARK", "", lambda ev: seen.append((ev, src.mac)), retry_s=5.0)
    _run(src, lambda: any(ev == JoyEvent("connected", "", 0) for ev, _ in seen))
    assert seen[0] == (JoyEvent("connected", "", 1), "7f:ab:6e:93:c4:30")
    assert (JoyEvent("connected", "", 0), None) in seen
    assert src.mac is None


def test_evdev_source_start_fails_fast_without_evdev(monkeypatch):
    """The node imports evdev at start, not inside the reader thread."""
    monkeypatch.setitem(sys.modules, "evdev", None)     # import evdev -> ImportError
    src = EvdevSource("VR-PARK", "", lambda ev: None)
    with pytest.raises(ImportError):
        src.start()
    assert src._thread is None


# --- key aliases ---------------------------------------------------------------

_ECODES = types.SimpleNamespace(
    KEY={28: "KEY_ENTER"},
    BTN={275: "BTN_SIDE", 304: ["BTN_A", "BTN_GAMEPAD", "BTN_SOUTH"]})


def test_evdev_key_name_matches_any_alias():
    assert evdev_key_name(304, _ECODES, ("BTN_SOUTH",)) == "BTN_SOUTH"
    assert evdev_key_name(304, _ECODES, ("BTN_GAMEPAD", "BTN_SIDE")) == "BTN_GAMEPAD"
    assert evdev_key_name(304, _ECODES, ()) == "BTN_A"          # unconfigured: first alias
    assert evdev_key_name(275, _ECODES, ("BTN_SOUTH",)) == "BTN_SIDE"
    assert evdev_key_name(28, _ECODES, ()) == "KEY_ENTER"
    assert evdev_key_name(999, _ECODES, ()) == "KEY_999"


def test_alias_configured_button_arms_the_gate():
    """A button configured by any alias name is the same button."""
    now = [100.0]
    gate = JoystickGate(clock=lambda: now[0])
    buttons = Buttons(arm="BTN_SOUTH")
    apply_event(gate, JoyEvent("connected", "", 1), buttons)
    ev = to_event(1, 304, 1, lambda c: evdev_key_name(c, _ECODES, (buttons.arm,)))
    assert ev == JoyEvent("key", "BTN_SOUTH", 1)
    apply_event(gate, ev, buttons)
    assert gate.state()["armed"] is True


def test_evdev_source_reports_the_configured_alias(fake_evdev):
    add, _ = fake_evdev
    import robot_missions.joystick_source  # noqa: F401  (fixture patched evdev)
    sys.modules["evdev"].ecodes.BTN[304] = ["BTN_A", "BTN_GAMEPAD", "BTN_SOUTH"]
    add(_Dev([_Ev(1, 304, 1)]))
    got = []
    src = EvdevSource("VR-PARK", "", got.append, retry_s=0.01, buttons=("BTN_SOUTH",))
    _run(src, lambda: JoyEvent("key", "BTN_SOUTH", 1) in got)
    assert JoyEvent("key", "BTN_SOUTH", 1) in got


# --- BatteryPoller -------------------------------------------------------------

class _Reads:
    """Fake bluez_battery: records MACs, answers from a script (default 80)."""

    def __init__(self, *answers):
        self.macs, self.answers = [], list(answers)

    def __call__(self, mac):
        self.macs.append(mac)
        a = self.answers.pop(0) if self.answers else 80.0
        if isinstance(a, Exception):
            raise a
        return a


def _wait(cond, timeout=2.0):
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.005)
    return cond()


def _batteries(got):
    return [ev.value for ev in got if ev.kind == "battery"]


def test_poller_polls_right_after_connect():
    """Remote off/on must not leave the battery unknown for battery_poll_s."""
    mac = {"v": "7F:AB:6E:93:C4:30"}
    reads, got = _Reads(20.0), []
    p = BatteryPoller(lambda: mac["v"], got.append, period_s=60.0, read=reads)
    p.start()
    try:
        p.on_event(JoyEvent("connected", "", 1))
        assert _wait(lambda: _batteries(got) == [20.0]), got
        assert reads.macs == ["7F:AB:6E:93:C4:30"]
    finally:
        p.stop()


def test_poller_ignores_other_events_and_disconnects():
    reads, got = _Reads(), []
    p = BatteryPoller(lambda: "AA", got.append, period_s=60.0, read=reads)
    p.start()
    try:
        for ev in (JoyEvent("rel", "x", 3), JoyEvent("connected", "", 0),
                   JoyEvent("key", "BTN_SIDE", 1)):
            p.on_event(ev)
        assert not _wait(lambda: reads.macs, timeout=0.3)
    finally:
        p.stop()


def test_poller_never_polls_without_a_mac():
    """Disconnected (the source cleared the MAC): no poll, no NaN, the gate
    keeps its last known battery."""
    reads, got = _Reads(), []
    p = BatteryPoller(lambda: None, got.append, period_s=0.02, read=reads)
    p.start()
    try:
        p.kick()
        assert not _wait(lambda: reads.macs or got, timeout=0.3)
    finally:
        p.stop()


def test_poller_periodic():
    reads, got = _Reads(70.0, 69.0, 68.0), []
    p = BatteryPoller(lambda: "AA", got.append, period_s=0.02, read=reads)
    p.start()
    try:
        assert _wait(lambda: _batteries(got)[:3] == [70.0, 69.0, 68.0]), got
    finally:
        p.stop()


def test_poller_periodic_failure_keeps_the_last_reading():
    """BlueZ drops Battery1 as the link goes down, maybe before the evdev node
    (and the MAC) is gone: a failed periodic read must not turn a known-low
    battery into "unknown, allowed"."""
    reads, got, logs = _Reads(18.0, None, None, 17.0), [], []
    p = BatteryPoller(lambda: "AA", got.append, period_s=0.02, read=reads,
                      log=logs.append)
    p.start()
    try:
        assert _wait(lambda: len(reads.macs) >= 4), reads.macs
        assert _batteries(got)[:2] == [18.0, 17.0], got
        assert not any(math.isnan(b) for b in _batteries(got)), got
        assert any("keeping the last reading" in s for s in logs), logs
    finally:
        p.stop()


def test_poller_does_not_lose_a_kick_that_lands_as_the_wait_times_out():
    """kick() between wait() timing out and the clear must still poll now,
    not a whole period_s later."""

    class KickInWindow(threading.Event):
        def __init__(self):
            super().__init__()
            self.first = True

        def wait(self, timeout=None):
            if self.first:
                self.first = False
                self.set()                      # the connect's kick lands here
                return False                    # ...after wait() timed out
            return super().wait(timeout)

    reads, got = _Reads(None, 20.0), []
    p = BatteryPoller(lambda: "AA", got.append, period_s=60.0, read=reads,
                      retry_s=60.0)
    p._wake = KickInWindow()
    p.start()
    try:
        assert _wait(lambda: _batteries(got) == [20.0], timeout=1.0), got
    finally:
        p.stop()


def test_poller_retries_a_failed_read_after_connect_before_unknown():
    """Battery1 can lag the input device: a failed first read must not turn a
    known-low battery into "unknown, allowed" — retry first."""
    reads, got = _Reads(None, None, 18.0), []
    p = BatteryPoller(lambda: "AA", got.append, period_s=60.0, read=reads,
                      retry_s=0.01, retries=3)
    p.start()
    try:
        p.on_event(JoyEvent("connected", "", 1))
        assert _wait(lambda: _batteries(got)), got
        assert _batteries(got) == [18.0] and len(reads.macs) == 3
    finally:
        p.stop()


def test_poller_gives_up_after_retries_as_unknown():
    reads, got = _Reads(None, None, None, None, 50.0), []
    p = BatteryPoller(lambda: "AA", got.append, period_s=60.0, read=reads,
                      retry_s=0.01, retries=2)
    p.start()
    try:
        p.on_event(JoyEvent("connected", "", 1))
        assert _wait(lambda: _batteries(got)), got
        b = _batteries(got)
        assert len(b) == 1 and math.isnan(b[0]) and len(reads.macs) == 3
    finally:
        p.stop()


def test_poller_drops_a_read_whose_mac_went_away():
    """Disconnect during busctl: the stale result (usually a failure = NaN)
    must not reach the gate."""
    mac = {"v": "AA"}

    def read(m):
        mac["v"] = None                         # the link drops mid-read
        return None

    got = []
    p = BatteryPoller(lambda: mac["v"], got.append, period_s=60.0, read=read,
                      retry_s=0.01, retries=0)
    p.start()
    try:
        p.kick()
        assert not _wait(lambda: got, timeout=0.3)
    finally:
        p.stop()


def test_poller_survives_exceptions():
    logs, got = [], []
    reads = _Reads(RuntimeError("boom"), 55.0)

    def bad_emit(ev):
        got.append(ev)

    p = BatteryPoller(lambda: "AA", bad_emit, period_s=60.0, read=reads,
                      retry_s=0.01, retries=0, log=logs.append)
    p.start()
    try:
        p.kick()
        assert _wait(lambda: logs), logs
        p.kick()
        assert _wait(lambda: _batteries(got) == [55.0]), got
        assert p._thread.is_alive()
    finally:
        p.stop()
    assert not p._thread.is_alive()
