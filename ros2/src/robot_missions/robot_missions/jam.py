"""Is the robot jammed — are its wheels lying? ROS-free; jam_monitor ticks it.

⚠️ Why this exists (docs/missions.md, "Patinaj sonrası alan dışında biçme"):
in CI a robot pressed against a rock turned its wheels while it did not move.
Wheel odometry reported motion that never happened, the EKF swung map->odom
to keep `map` right, and mission_runner — which drives rows in `odom` —
mowed 3 m outside the lawn. A guard on map->odom alone was tried and backed
out: with a NEO-6M map->odom moves that much anyway, it fired 273 times in one
coverage run. The signal has to be specific to the WHEELS disagreeing with
something that is not the wheels:

  turn   wheel yaw rate vs the gyro (imu/data_raw). ekf_local fuses both, so a
         sustained disagreement is exactly what corrupts odom's heading.
  stall  wheel velocity vs the GPS's DOPPLER velocity (gps/vel — the NEO-6M's
         VTG, which nmea_serial_driver publishes as `vel`), both averaged as
         VECTORS over window_s: the wheels' with odom's heading, the GPS's in
         ENU. Only magnitudes are compared (the frames differ by a rotation),
         a window counts only while the wheels claim min_speed, and the
         verdict must hold for stall_hold_s.

         ⚠️ The first version compared wheel distance with GPS POSITION
         displacement over 10 s. Under sim_node's NEO-6M (OU wander 2 m /
         60 s + 1 m white) the wander alone moves ~1 m per axis in 8 s — as
         much as the robot drives — and it fired in CI on an honest row
         ("wheels 2.00 m, GPS 0.23 m in 8 s"). Longer windows did not save
         it: rows here are 4 m, and a window over a reversal cancels. Doppler
         velocity does not carry the position's wander; the vector average
         makes a reversal cancel on BOTH sides alike.

Nothing is converted into `map` here (decision 4).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple


@dataclass(frozen=True)
class JamConfig:
    turn_rad_s: float = 0.3        # |wheel - gyro| yaw rate that counts as a slip
    turn_min_rad_s: float = 0.2    # ...only while the wheels claim at least this
    turn_hold_s: float = 1.0       # ...for this long
    window_s: float = 3.0          # stall: velocities averaged over this
    min_speed: float = 0.15        # the wheels must claim this much (m/s, averaged)
    min_gps: int = 3               # GPS velocities needed in the window (a NEO-6M
                                   # sends 1 Hz until UBX-CFG-RATE sets 5 Hz)
    stall_ratio: float = 0.35      # GPS speed below this fraction of the wheels'
    stall_hold_s: float = 2.0      # ...for this long


class JamDetector:
    TURN = "turn"
    STALL = "stall"

    def __init__(self, config: JamConfig = JamConfig()):
        self.config = config
        self._wheel_w: Optional[Tuple[float, float]] = None     # (t, rad/s)
        self._gyro_w: Optional[Tuple[float, float]] = None
        self._turn_since: Optional[float] = None
        self._stall_since: Optional[float] = None
        self._wheel: Deque[Tuple[float, float, float]] = deque()   # (t, v signed, yaw)
        self._gps: Deque[Tuple[float, float, float]] = deque()     # (t, east, north) m/s
        self.last: Optional[Dict] = None                        # the latest verdict

    # --- inputs ---------------------------------------------------------------

    def wheel(self, t: float, v: float, w: float, yaw: float) -> None:
        """v, w: odom's twist; yaw: odom's heading (any frame — magnitudes only)."""
        self._wheel_w = (t, w)
        self._wheel.append((t, v, yaw))
        self._trim(t)

    def gyro(self, t: float, w: float) -> None:
        self._gyro_w = (t, w)

    def gps(self, t: float, east: float, north: float) -> None:
        """The receiver's Doppler velocity, m/s."""
        self._gps.append((t, east, north))
        self._trim(t)

    # --- the verdict ----------------------------------------------------------

    def check(self, now: float) -> Optional[Dict]:
        """A dict {"kind", "detail"} while jammed, else None."""
        verdict = self._turn(now) or self._stall(now)
        self.last = verdict
        return verdict

    def _turn(self, now: float) -> Optional[Dict]:
        c = self.config
        if self._wheel_w is None or self._gyro_w is None:
            return None
        (tw, ww), (tg, wg) = self._wheel_w, self._gyro_w
        if now - tw > 0.5 or now - tg > 0.5:                   # stale: no claim
            self._turn_since = None
            return None
        if abs(ww) >= c.turn_min_rad_s and abs(ww - wg) > c.turn_rad_s:
            if self._turn_since is None:
                self._turn_since = now
            if now - self._turn_since >= c.turn_hold_s:
                return {"kind": self.TURN,
                        "detail": f"wheels turn {ww:+.2f} rad/s, gyro {wg:+.2f} rad/s"}
            return None
        self._turn_since = None
        return None

    def _stall(self, now: float) -> Optional[Dict]:
        verdict = self._stall_now(now)
        if verdict is None:
            self._stall_since = None
            return None
        if self._stall_since is None:
            self._stall_since = now
        return verdict if now - self._stall_since >= self.config.stall_hold_s else None

    def _stall_now(self, now: float) -> Optional[Dict]:
        c = self.config
        if not self._wheel or not self._gps:
            return None
        begin = now - c.window_s
        if self._wheel[0][0] > begin + 0.5 or now - self._wheel[-1][0] > 0.5:
            return None                                         # window not full / stale
        gps = [(e, n) for t, e, n in self._gps if t >= begin]
        if len(gps) < c.min_gps or now - self._gps[-1][0] > 1.5:
            return None
        wheel = [(t, v, yaw) for t, v, yaw in self._wheel if t >= begin]
        wheel_v = math.hypot(sum(v * math.cos(yaw) for _, v, yaw in wheel) / len(wheel),
                             sum(v * math.sin(yaw) for _, v, yaw in wheel) / len(wheel))
        if wheel_v < c.min_speed:
            return None
        gps_v = math.hypot(sum(e for e, _ in gps) / len(gps), sum(n for _, n in gps) / len(gps))
        if gps_v < c.stall_ratio * wheel_v:
            # forward: the wheels were pushing AHEAD — something is in front
            # (site_map_server records it like a bumper press).
            signed = sum(v for _, v, _ in wheel) / len(wheel)
            return {"kind": self.STALL, "forward": signed > 0.0,
                    "detail": f"wheels {wheel_v:.2f} m/s, GPS {gps_v:.2f} m/s "
                              f"over {c.window_s:.0f} s"}
        return None

    def _trim(self, now: float) -> None:
        horizon = now - self.config.window_s
        while self._wheel and self._wheel[0][0] < horizon:
            self._wheel.popleft()
        while self._gps and self._gps[0][0] < horizon:
            self._gps.popleft()

