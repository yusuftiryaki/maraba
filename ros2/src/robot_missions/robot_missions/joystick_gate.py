"""Driving with a Bluetooth remote (docs/superpowers/specs/2026-11-20-bluetooth-kumanda-design.md).
ROS-free; the joystick node feeds it events and ticks it.

The remote is a mouse-mode HID pad: its stick arrives as relative motion
(REL_X / REL_Y) while pushed and as silence when let go, so the pad has no
"stick centred" message. Silence is the stop signal, which makes the rules
below the whole safety story of this input:

    state / since the last REL        output()
    disarmed                          None (publish nothing), except the
                                      zero window after a disarm (below)
    armed, no REL since arming        None
    armed, an axis younger than       that axis' command at the current speed
      deadman_s                       level; a stale axis counts as 0
    armed, all axes stale, within     ZERO — a stick released or a radio
      release_s of the last REL       hiccup stops the robot now
    armed, beyond release_s           None — the mux hands the wheels back

A short radio gap while armed is accepted by design: after a gap the next
REL resumes driving without re-arming. Anything longer than that is a
disconnect, and a disconnect disarms.

Arming is a deliberate click and is REFUSED when: the remote is not
connected, the battery is known and below min_battery_pct, or the stick is
being pushed right now (a REL younger than deadman_s). Arming resets the
speed to the lowest level, so every session starts slow.

Disarming (click, disconnect, low battery, arm_idle_s without any REL) holds
ZERO for release_s if the robot was being driven at that instant, then goes
silent. A disarm while silent stays silent.

Reverse is capped at reverse_max however high the speed level is. The pad
only gives a direction; the magnitude always comes from the level tables
here, so no input can exceed them. Everything below (twist_mux, the bridge
cmd_timeout, the ESP32 watchdog, E-stop, the bumper veto) is untouched.

The clock is injected (monotonic by default) so the tests can step it. The
node's reader thread and its timer both call in, hence the lock.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

_NEVER = -math.inf
_HINT_WINDOW_S = 2.0
_HINT = "@+D?"          # the pad is probably not in mouse mode


@dataclass(frozen=True)
class JoyEvent:
    kind: str      # "rel" | "key" | "other" | "connected" | "battery"
    code: str      # "x"/"y" for rel; evdev key name for key; "" otherwise
    value: float   # rel delta | key 1/0/2 | connected 1/0 | battery % (nan = unknown)


@dataclass(frozen=True)
class Buttons:
    arm: str = "BTN_SIDE"
    speed_up: str = ""
    speed_down: str = ""


def _levels(values: Sequence[float], what: str) -> Tuple[float, ...]:
    out = tuple(float(v) for v in values)
    if not out or not all(math.isfinite(v) and v > 0 for v in out):
        raise ValueError(f"{what} must be a non-empty list of positive finite numbers")
    return out


class JoystickGate:
    def __init__(self, linear_levels: Sequence[float] = (0.2, 0.4, 0.6),
                 angular_levels: Sequence[float] = (0.4, 0.7, 1.0),
                 reverse_max: float = 0.2, deadman_s: float = 0.25,
                 release_s: float = 2.0, arm_idle_s: float = 60.0,
                 min_battery_pct: float = 25.0, invert_x: bool = False,
                 clock: Callable[[], float] = time.monotonic):
        self.linear_levels = _levels(linear_levels, "linear_levels")
        self.angular_levels = _levels(angular_levels, "angular_levels")
        if len(self.linear_levels) != len(self.angular_levels):
            raise ValueError("linear_levels and angular_levels must be the same length")
        if not (math.isfinite(reverse_max) and reverse_max > 0):
            raise ValueError("reverse_max must be positive")
        if not 0 < deadman_s < release_s:
            raise ValueError("need 0 < deadman_s < release_s")
        if not arm_idle_s > release_s:
            raise ValueError("need arm_idle_s > release_s")
        self.reverse_max = float(reverse_max)
        self.deadman_s = float(deadman_s)
        self.release_s = float(release_s)
        self.arm_idle_s = float(arm_idle_s)
        self.min_battery_pct = float(min_battery_pct)
        self.invert_x = bool(invert_x)
        self._clock = clock
        self._lock = threading.Lock()
        self._connected = False
        self._armed = False
        self._armed_at = _NEVER
        self._level = 0
        self._battery: Optional[float] = None
        self._last_rel = _NEVER                 # any REL, armed or not
        self._last_other = _NEVER
        self._axis: Dict[str, Tuple[float, float]] = {}   # axis -> (time, sign)
        self._zero_until = _NEVER
        self._refusal: Optional[str] = None

    # --- events -----------------------------------------------------------

    def rel(self, axis: str, value: int) -> None:
        with self._lock:
            now = self._clock()
            self._last_rel = now
            if self._armed and axis in ("x", "y"):
                sign = (value > 0) - (value < 0)
                self._axis[axis] = (now, float(sign))

    def other(self) -> None:
        with self._lock:
            self._last_other = self._clock()

    def connected(self, up: bool) -> None:
        with self._lock:
            now = self._clock()
            if not up and self._armed:
                self._disarm(now, "bağlantı koptu")
            self._connected = bool(up)

    def battery(self, pct: Optional[float]) -> None:
        with self._lock:
            now = self._clock()
            self._battery = pct
            if self._armed and self._low_battery():
                self._disarm(now, "pil düşük")

    def toggle_arm(self) -> bool:
        """Click on the arm button. Returns the armed state afterwards."""
        with self._lock:
            now = self._clock()
            self._idle_check(now)
            if self._armed:
                self._disarm(now, "kapatıldı")
                return False
            if not self._connected:
                self._refusal = "bağlı değil"
            elif self._low_battery():
                self._refusal = "pil düşük"
            elif now - self._last_rel < self.deadman_s:
                self._refusal = "çubuk itiliyken etkinleştirilemez"
            else:
                self._armed, self._armed_at = True, now
                self._level, self._axis = 0, {}
                self._zero_until = _NEVER
                self._refusal = None
            return self._armed

    def speed(self, delta: int) -> int:
        """Change the speed level (clamped); returns the 0-based level."""
        with self._lock:
            self._level = max(0, min(len(self.linear_levels) - 1, self._level + int(delta)))
            return self._level

    # --- outputs ----------------------------------------------------------

    def output(self) -> Optional[Tuple[float, float]]:
        """(linear m/s, angular rad/s) to publish now, or None for nothing."""
        with self._lock:
            return self._output(self._clock())

    def state(self) -> Dict[str, Any]:
        with self._lock:
            now = self._clock()
            out = self._output(now)
            rel_age = None if self._last_rel == _NEVER else round(now - self._last_rel, 2)
            hint = (self._connected and now - self._last_other < _HINT_WINDOW_S
                    and now - self._last_rel >= _HINT_WINDOW_S)
            return {"connected": self._connected, "armed": self._armed,
                    "mode_hint": _HINT if hint else None,
                    "speed_level": self._level + 1, "battery": self._battery,
                    "rel_age_s": rel_age,
                    "linear": None if out is None else out[0],
                    "angular": None if out is None else out[1],
                    "refusal": self._refusal}

    # --- internals (lock held) -------------------------------------------

    def _low_battery(self) -> bool:
        return self._battery is not None and self._battery < self.min_battery_pct

    def _idle_check(self, now: float) -> None:
        if self._armed and now - max(self._armed_at, self._last_rel) >= self.arm_idle_s:
            self._disarm(now, "boşta kaldı")

    def _disarm(self, now: float, reason: str) -> None:
        if self._armed_output(now) is not None:
            self._zero_until = now + self.release_s
        self._armed, self._axis = False, {}
        self._refusal = reason

    def _armed_output(self, now: float) -> Optional[Tuple[float, float]]:
        if not self._axis:
            return None
        lin = ang = 0.0
        fresh = False
        for axis, (t, sign) in self._axis.items():
            if now - t >= self.deadman_s:
                continue
            fresh = True
            if axis == "y":
                v = self.linear_levels[self._level]
                lin = v if sign < 0 else -min(v, self.reverse_max) if sign > 0 else 0.0
            else:
                w = self.angular_levels[self._level]
                ang = -sign * w * (-1.0 if self.invert_x else 1.0)
        if fresh:
            return lin, ang + 0.0           # + 0.0 turns a -0.0 into 0.0
        last = max(t for t, _ in self._axis.values())
        return (0.0, 0.0) if now - last < self.release_s else None

    def _output(self, now: float) -> Optional[Tuple[float, float]]:
        self._idle_check(now)
        if self._armed:
            return self._armed_output(now)
        return (0.0, 0.0) if now < self._zero_until else None


def apply_event(gate: JoystickGate, event: JoyEvent, buttons: Buttons) -> None:
    """Route one reader event into the gate."""
    if event.kind == "rel":
        gate.rel(event.code, int(event.value))
    elif event.kind == "key":
        if event.value != 1:
            return                                   # release / auto-repeat
        if buttons.arm and event.code == buttons.arm:
            gate.toggle_arm()
        elif buttons.speed_up and event.code == buttons.speed_up:
            gate.speed(+1)
        elif buttons.speed_down and event.code == buttons.speed_down:
            gate.speed(-1)
        else:
            gate.other()
    elif event.kind == "other":
        gate.other()
    elif event.kind == "connected":
        gate.connected(event.value >= 1)
    elif event.kind == "battery":
        gate.battery(None if math.isnan(event.value) else float(event.value))
