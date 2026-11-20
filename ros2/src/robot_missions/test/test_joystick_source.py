"""joystick_source: event mapping, device choice, BlueZ battery (no evdev, no ROS)."""

import errno
import math
import subprocess
import sys
import types

import pytest

from robot_missions.joystick_gate import JoyEvent
from robot_missions.joystick_source import (EvdevSource, bluez_battery, parse_fake_event,
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
