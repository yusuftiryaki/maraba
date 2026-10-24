"""The policy's gate (docs/learning.md, 6d). ROS-free; policy_gate ticks it.

A learned policy is the least trusted thing that will ever move this robot.
Whatever it outputs, this decides what reaches twist_mux (priority 20: under
teleop 100 and the battery lock 50, over Nav2 10):

    disarmed (the default)  nothing — the mux never hears from the policy
    armed                   the policy's command, clamped (max_linear,
                            max_angular) and rate-limited (max_accel,
                            max_alpha), while ALL of these hold:
      - the command is fresh (cmd_timeout_s since it arrived)
      - its observation is fresh (obs_timeout_s since the camera frame /
        state it was computed from — a policy acting on an old picture is
        driving blind)
      - the robot is inside the armed area, grown by margin_m (`map`)
      - no jam (jam_monitor) since arming
      - the arm's time budget has not run out
    A broken freshness rule gives ZERO (stop, still holding the mux); a
    broken fence, jam or budget DISARMS (and says why); a policy silent for
    lease_s releases the mux to Nav2.

Below this, untouched: twist_mux, the bridge's deadman and limits, the
ESP32 watchdog, the E-stop and the bumper veto.

⚠️ The fence is in `map`, so it is only as good as the GPS: with a NEO-6M the
robot's own estimate can be metres off (docs/missions.md decision 1). It
catches a policy driving away, not a policy cutting 50 cm too wide.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon


@dataclass(frozen=True)
class GateConfig:
    max_linear: float = 0.3        # m/s — below teleop's 0.6 and the bridge's 1.0
    max_angular: float = 0.8       # rad/s
    max_accel: float = 0.5         # m/s^2
    max_alpha: float = 2.0         # rad/s^2
    cmd_timeout_s: float = 0.3
    obs_timeout_s: float = 0.5
    lease_s: float = 2.0
    margin_m: float = 1.0


class PolicyGate:
    def __init__(self, config: GateConfig = GateConfig()):
        self.config = config
        self._armed = False
        self._area: Optional[Polygon] = None
        self._until = 0.0
        self._cmd: Optional[Tuple[float, float, float, float]] = None  # v, w, arrived, obs stamp
        self._out = (0.0, 0.0)
        self._last_tick: Optional[float] = None
        self.reason = "disarmed"

    # --- arming ---------------------------------------------------------------

    def arm(self, area_map: Sequence[Tuple[float, float]], budget_s: float, now: float) -> None:
        area = Polygon(area_map)
        if not area.is_valid or area.area <= 0:
            raise ValueError("the armed area must be a valid polygon in map")
        if not (budget_s > 0 and math.isfinite(budget_s)):
            raise ValueError("budget_s must be positive")
        self._armed, self._area, self._until = True, area, now + budget_s
        self._cmd, self._out, self._last_tick = None, (0.0, 0.0), now
        self.reason = "armed"

    def disarm(self, why: str) -> None:
        self._armed, self._area, self._cmd = False, None, None
        self._out = (0.0, 0.0)
        self.reason = why

    @property
    def armed(self) -> bool:
        return self._armed

    # --- inputs -----------------------------------------------------------------

    def command(self, v: float, w: float, obs_stamp: float, now: float) -> None:
        if not (math.isfinite(v) and math.isfinite(w) and math.isfinite(obs_stamp)):
            return                          # a NaN from a network is no command
        self._cmd = (v, w, now, obs_stamp)

    def jam(self, why: str) -> None:
        if self._armed:
            self.disarm(f"jammed: {why}")

    # --- output -----------------------------------------------------------------

    def tick(self, robot_xy: Optional[Tuple[float, float]], now: float
             ) -> Optional[Tuple[float, float]]:
        """(v, w) to publish now, or None for silence."""
        c = self.config
        if not self._armed:
            return None
        dt = max(0.0, now - (self._last_tick if self._last_tick is not None else now))
        self._last_tick = now
        if now > self._until:
            self.disarm("time budget used up")
            return (0.0, 0.0)
        if robot_xy is None:
            target = (0.0, 0.0)             # no pose: cannot check the fence
            self.reason = "no robot pose"
        elif not self._area.buffer(c.margin_m).contains(Point(robot_xy)):
            self.disarm(f"left the area (+{c.margin_m} m) at "
                        f"({robot_xy[0]:.1f}, {robot_xy[1]:.1f})")
            return (0.0, 0.0)
        elif self._cmd is None:
            target = (0.0, 0.0)
            self.reason = "armed, no command yet"
        else:
            v, w, arrived, obs = self._cmd
            if now - arrived > c.lease_s:
                self.reason = "policy silent: releasing the mux"
                self._out = (0.0, 0.0)
                return None
            if now - arrived > c.cmd_timeout_s:
                target = (0.0, 0.0)
                self.reason = "command stale"
            elif now - obs > c.obs_timeout_s:
                target = (0.0, 0.0)
                self.reason = f"observation {now - obs:.2f} s old"
            else:
                target = (max(-c.max_linear, min(c.max_linear, v)),
                          max(-c.max_angular, min(c.max_angular, w)))
                self.reason = "driving"
        # Slew toward the target: no jumps, in either direction. Stopping is
        # also rate-limited — the bridge's own limits and the ESP32 brake
        # harder if they must.
        v0, w0 = self._out
        dv = max(-c.max_accel * dt, min(c.max_accel * dt, target[0] - v0))
        dw = max(-c.max_alpha * dt, min(c.max_alpha * dt, target[1] - w0))
        self._out = (v0 + dv, w0 + dw)
        return self._out
