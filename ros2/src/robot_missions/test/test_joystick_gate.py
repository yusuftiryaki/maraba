"""joystick_gate.JoystickGate: arming, deadman, release window, speed levels and
the idle/battery/disconnect disarms, on a stepped clock. ROS-free. The evdev
reader and the ROS node are tested separately."""

import math
import time

import pytest

from robot_missions.joystick_gate import Buttons, JoyEvent, JoystickGate, apply_event


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make(**kw):
    clock = Clock()
    g = JoystickGate(clock=clock, **kw)
    g.clock = clock
    return g


@pytest.fixture
def gate():
    return make()


def armed(**kw):
    g = make(**kw)
    g.connected(True)
    assert g.toggle_arm() is True
    return g


def test_starts_disarmed_disconnected_and_silent(gate):
    assert gate.output() is None
    s = gate.state()
    assert s["armed"] is False
    assert s["connected"] is False
    assert s["speed_level"] == 1


def test_rel_while_disarmed_never_drives(gate):
    gate.connected(True)
    for _ in range(10):
        gate.rel("y", -17)
        gate.clock.advance(0.03)
        assert gate.output() is None


def test_arm_needs_connection(gate):
    assert gate.toggle_arm() is False
    assert gate.state()["refusal"] is not None
    assert gate.state()["armed"] is False


def test_arm_refused_while_stick_pushed(gate):
    gate.connected(True)
    gate.rel("y", -17)
    gate.clock.advance(0.1)
    assert gate.toggle_arm() is False
    assert gate.state()["refusal"] is not None


def test_arm_refused_on_low_battery(gate):
    gate.connected(True)
    gate.battery(20.0)
    assert gate.toggle_arm() is False
    assert gate.state()["refusal"] is not None


def test_unknown_battery_does_not_block(gate):
    gate.connected(True)
    gate.battery(None)
    assert gate.toggle_arm() is True
    assert gate.state()["refusal"] is None


def test_forward_is_lowest_level_after_arming():
    g = armed()
    g.rel("y", -17)
    assert g.output() == (0.2, 0.0)


def test_reverse_is_capped():
    g = armed()
    g.speed(+2)
    g.rel("y", 17)
    assert g.output() == (-0.2, 0.0)


def test_right_stick_turns_clockwise_and_invert_flips():
    g = armed()
    g.rel("x", 17)
    assert g.output() == (0.0, -0.4)
    g = armed(invert_x=True)
    g.rel("x", 17)
    assert g.output() == (0.0, 0.4)


def test_arc_uses_both_axes():
    g = armed()
    g.rel("y", -17)
    g.rel("x", -17)
    assert g.output() == (0.2, 0.4)


def test_stale_axis_drops_out_of_an_arc():
    g = armed()
    g.rel("x", -17)
    for _ in range(10):
        g.clock.advance(0.03)
        g.rel("y", -17)
    g.clock.advance(0.0)
    assert g.output() == (0.2, 0.0)


def test_zero_value_zeroes_that_axis():
    g = armed()
    g.rel("y", -17)
    g.rel("x", -17)
    g.rel("x", 0)
    assert g.output() == (0.2, 0.0)


def test_silence_gives_zero_then_nothing():
    g = armed()
    g.rel("y", -17)
    assert g.output() == (0.2, 0.0)
    g.clock.advance(0.25)
    assert g.output() == (0.0, 0.0)
    g.clock.advance(1.74)
    assert g.output() == (0.0, 0.0)
    g.clock.advance(0.01)
    assert g.output() is None


def test_armed_without_any_rel_is_silent():
    g = armed()
    assert g.output() is None


def test_disconnect_mid_drive_disarms_with_zero_window():
    g = armed()
    g.rel("y", -17)
    g.connected(False)
    assert g.output() == (0.0, 0.0)
    assert g.state()["armed"] is False
    assert g.state()["refusal"] is not None
    g.clock.advance(2.0)
    assert g.output() is None


def test_disarm_click_mid_drive_zero_window():
    g = armed()
    g.rel("y", -17)
    assert g.toggle_arm() is False
    assert g.output() == (0.0, 0.0)
    g.clock.advance(1.9)
    assert g.output() == (0.0, 0.0)
    g.clock.advance(0.1)
    assert g.output() is None


def test_disarm_while_silent_stays_silent():
    g = armed()
    assert g.toggle_arm() is False
    assert g.output() is None


def test_low_battery_mid_drive_disarms():
    g = armed()
    g.rel("y", -17)
    g.battery(10.0)
    assert g.state()["armed"] is False
    assert g.output() == (0.0, 0.0)


def test_idle_disarms_after_arm_idle_s():
    g = armed()
    g.clock.advance(59.9)
    assert g.state()["armed"] is True
    g.clock.advance(0.1)
    assert g.state()["armed"] is False
    assert g.state()["refusal"] is not None


def test_reconnect_needs_rearm():
    g = armed()
    g.connected(False)
    g.connected(True)
    g.rel("y", -17)
    assert g.output() is None


def test_short_gap_without_disconnect_resumes_while_armed():
    g = armed()
    g.rel("y", -17)
    g.clock.advance(1.0)
    assert g.output() == (0.0, 0.0)
    g.rel("y", -17)
    assert g.output() == (0.2, 0.0)


def test_speed_levels_clamp():
    g = armed()
    assert g.speed(+5) == 2
    assert g.state()["speed_level"] == 3
    assert g.speed(-9) == 0
    g.speed(+1)
    g.toggle_arm()                      # disarm
    g.connected(True)
    assert g.toggle_arm() is True       # arm resets to the lowest level
    assert g.state()["speed_level"] == 1


def test_mode_hint_when_only_non_rel_events(gate):
    gate.connected(True)
    assert gate.state()["mode_hint"] is None
    gate.other()
    assert gate.state()["mode_hint"] == "@+D?"
    gate.rel("y", -17)
    assert gate.state()["mode_hint"] is None


def test_apply_event_press_toggles_release_ignored(gate):
    b = Buttons()
    apply_event(gate, JoyEvent("connected", "", 1), b)
    apply_event(gate, JoyEvent("key", "BTN_SIDE", 1), b)
    assert gate.state()["armed"] is True
    apply_event(gate, JoyEvent("key", "BTN_SIDE", 0), b)
    apply_event(gate, JoyEvent("key", "BTN_SIDE", 2), b)
    assert gate.state()["armed"] is True


def test_apply_event_unmapped_key_is_other_and_empty_button_matches_nothing(gate):
    b = Buttons(speed_up="")
    apply_event(gate, JoyEvent("connected", "", 1), b)
    apply_event(gate, JoyEvent("key", "", 1), b)
    assert gate.state()["speed_level"] == 1
    apply_event(gate, JoyEvent("key", "KEY_PREVIOUSSONG", 1), b)
    assert gate.state()["mode_hint"] == "@+D?"


def test_apply_event_speed_buttons_and_rel(gate):
    b = Buttons(speed_up="BTN_A", speed_down="BTN_B")
    apply_event(gate, JoyEvent("connected", "", 1), b)
    apply_event(gate, JoyEvent("key", "BTN_SIDE", 1), b)
    apply_event(gate, JoyEvent("key", "BTN_A", 1), b)
    assert gate.state()["speed_level"] == 2
    apply_event(gate, JoyEvent("key", "BTN_B", 1), b)
    assert gate.state()["speed_level"] == 1
    apply_event(gate, JoyEvent("rel", "y", -17), b)
    assert gate.output() == (0.2, 0.0)
    apply_event(gate, JoyEvent("other", "", 0), b)
    apply_event(gate, JoyEvent("connected", "", 0), b)
    assert gate.state()["armed"] is False


def test_apply_event_battery_nan_is_unknown(gate):
    apply_event(gate, JoyEvent("battery", "", 80.0), Buttons())
    assert gate.state()["battery"] == 80.0
    apply_event(gate, JoyEvent("battery", "", math.nan), Buttons())
    assert gate.state()["battery"] is None


@pytest.mark.parametrize("kw", [
    {"linear_levels": ()},
    {"linear_levels": (0.2, 0.4), "angular_levels": (0.4,)},
    {"linear_levels": (0.0, 0.4, 0.6)},
    {"angular_levels": (0.4, -0.7, 1.0)},
    {"linear_levels": (0.2, math.inf, 0.6)},
    {"linear_levels": (0.2, math.nan, 0.6)},
    {"reverse_max": 0.0},
    {"deadman_s": 0.0},
    {"deadman_s": 2.0, "release_s": 2.0},
    {"arm_idle_s": 2.0},
])
def test_bad_parameters_raise(kw):
    with pytest.raises(ValueError):
        JoystickGate(**kw)


def test_default_clock_is_monotonic():
    assert JoystickGate()._clock is time.monotonic


def test_idle_disarm_cannot_be_skipped_by_a_late_rel():
    g = armed()
    g.clock.advance(61.0)
    g.rel("y", -17)                     # no output()/state() poll in between
    assert g.state()["armed"] is False
    assert g.output() is None


def test_rearm_inside_zero_window_keeps_the_hold():
    g = armed()
    g.rel("y", -17)
    assert g.toggle_arm() is False
    assert g.output() == (0.0, 0.0)
    g.clock.advance(0.25)
    assert g.toggle_arm() is True       # stick released, allowed
    assert g.output() == (0.0, 0.0)
    g.clock.advance(1.74)
    assert g.output() == (0.0, 0.0)
    g.clock.advance(0.01)
    assert g.output() is None
