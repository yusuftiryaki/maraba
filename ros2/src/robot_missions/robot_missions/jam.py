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
  stall  wheel distance vs raw GPS displacement over a window. The GPS end
         points are averaged (first/last avg_s of the window) so a NEO-6M's
         white noise does not fake a stall, and the wheel distance is taken
         over the SAME baseline (between the two averages' centres); a
         window only counts when the wheels claim real distance
         (min_wheel_m), and the verdict must hold for hold_s. Measured on
         synthetic NEO-6M (1 m noise + 2 m wander): a 6 s window with the
         wheel distance over the whole window fired 37-73 times in ten
         minutes of honest rows — the GPS baseline was only 4 s of it.

Distances from lat/lon use site.distance_m (haversine) — a magnitude check
only; nothing is converted into `map` here (decision 4).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

from robot_missions.site import LatLon, distance_m


@dataclass(frozen=True)
class JamConfig:
    turn_rad_s: float = 0.3        # |wheel - gyro| yaw rate that counts as a slip
    turn_min_rad_s: float = 0.2    # ...only while the wheels claim at least this
    turn_hold_s: float = 1.0       # ...for this long
    window_s: float = 10.0         # stall window
    avg_s: float = 2.0             # GPS averaged over this much at each end
    min_wheel_m: float = 1.0       # the wheels must claim this much in the window
    stall_ratio: float = 0.25      # GPS moved less than this fraction of it
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
        self._wheel: Deque[Tuple[float, float]] = deque()      # (t, |v|) samples
        self._gps: Deque[Tuple[float, LatLon]] = deque()
        self.last: Optional[Dict] = None                        # the latest verdict

    # --- inputs ---------------------------------------------------------------

    def wheel(self, t: float, v: float, w: float) -> None:
        self._wheel_w = (t, w)
        self._wheel.append((t, abs(v)))
        self._trim(t)

    def gyro(self, t: float, w: float) -> None:
        self._gyro_w = (t, w)

    def gps(self, t: float, fix: LatLon) -> None:
        self._gps.append((t, fix))
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
        if self._gps[0][0] > begin + 0.5 or self._wheel[0][0] > begin + 0.5:
            return None                                         # window not full yet
        start = [(t, f) for t, f in self._gps if t <= begin + c.avg_s]
        end = [(t, f) for t, f in self._gps if t >= now - c.avg_s]
        if not start or not end:
            return None
        t_a = sum(t for t, _ in start) / len(start)             # the averages' centres
        t_b = sum(t for t, _ in end) / len(end)
        wheel_m = 0.0
        samples = list(self._wheel)
        for (t0, v0), (t1, _) in zip(samples, samples[1:]):
            overlap = min(t1, t_b) - max(t0, t_a)
            if overlap > 0:
                wheel_m += v0 * overlap
        if wheel_m < c.min_wheel_m:
            return None
        gps_m = distance_m(_mean([f for _, f in start]), _mean([f for _, f in end]))
        if gps_m < c.stall_ratio * wheel_m:
            return {"kind": self.STALL,
                    "detail": f"wheels {wheel_m:.2f} m, GPS {gps_m:.2f} m in {t_b - t_a:.0f} s"}
        return None

    def _trim(self, now: float) -> None:
        horizon = now - self.config.window_s
        while self._wheel and self._wheel[0][0] < horizon:
            self._wheel.popleft()
        while self._gps and self._gps[0][0] < horizon:
            self._gps.popleft()


def _mean(fixes) -> LatLon:
    return LatLon(sum(f.lat for f in fixes) / len(fixes), sum(f.lon for f in fixes) / len(fixes))
