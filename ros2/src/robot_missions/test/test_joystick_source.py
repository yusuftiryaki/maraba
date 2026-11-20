"""joystick_source: event mapping, device choice, BlueZ battery (no evdev, no ROS)."""

import subprocess

from robot_missions.joystick_gate import JoyEvent
from robot_missions.joystick_source import bluez_battery, pick_device, to_event


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
