"""A pretend ZED-F9P: true antenna position in, UBX-NAV-PVT out. ROS-free.

robot_sim's sim_node feeds it the kit antenna's true position (through the TF
the kit_manager published from the identity chip — so a wrong calibration
chain shows up as a wrong fix) and writes its frames to a pty; fake_kit uses
it with a fixed position. What it models, and what it does NOT:

  - the three solutions, chosen by the corrections it is fed: no RTCM -> a
    single-point fix (wanders ~1 m); RTCM arriving -> RTK float, and after
    `time_to_fix_s` of uninterrupted corrections -> RTK fixed (cm). Corrections
    stopping for `rtcm_timeout_s` drops it back to single.
  - hAcc that matches its own noise, as a healthy receiver's does.
  - ⚠️ every UBX-CFG written to it is COUNTED (`cfg_writes`): the robot must
    never configure the kit's receiver (ubx.py), and the stack test asserts
    this stays zero.

NOT modelled: multipath under canopy, cycle slips, the real convergence curve,
or what a VRS caster does with the GGA. These numbers are a plausible F9P on a
good day — enough to test the plumbing and the mode switch, not to predict
what the orchard will do to RTK (that is a field measurement).
"""

from __future__ import annotations

import datetime as dt
import math
import random
from dataclasses import dataclass
from typing import Optional

from robot_kit import rtcm, ubx

EARTH_RADIUS_M = 6371000.0
GPS_EPOCH = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc)
GPS_UTC_LEAP_S = 18


@dataclass(frozen=True)
class Solution:
    carr: int
    diff: bool
    white_m: float          # per-fix noise, 1 sigma per axis
    wander_m: float         # steady-state slow error, 1 sigma per axis
    tau_s: float            # its correlation time
    h_acc_m: float          # what the receiver reports
    v_acc_m: float


SINGLE = Solution(ubx.CARR_NONE, False, 0.3, 0.8, 60.0, 1.2, 2.0)
FLOAT = Solution(ubx.CARR_FLOAT, True, 0.10, 0.20, 20.0, 0.35, 0.5)
FIXED = Solution(ubx.CARR_FIXED, True, 0.008, 0.005, 10.0, 0.014, 0.025)


class F9pSim:
    def __init__(self, seed: int = 0, time_to_fix_s: float = 8.0,
                 rtcm_timeout_s: float = 10.0, num_sv: int = 24) -> None:
        self._rng = random.Random(seed)
        self.time_to_fix_s = time_to_fix_s
        self.rtcm_timeout_s = rtcm_timeout_s
        self.num_sv = num_sv
        self._rtcm = rtcm.RtcmParser()
        self._ubx = ubx.UbxParser()
        self._rtcm_first: Optional[float] = None    # start of the current unbroken run
        self._rtcm_last: Optional[float] = None
        self._bias = [0.0, 0.0]
        self._last_t: Optional[float] = None
        self._solution = SINGLE
        self.rtcm_frames = 0
        self.cfg_writes = 0

    # --- what the robot writes to the receiver ---------------------------------

    def feed(self, data: bytes, t: float) -> None:
        for _type, _whole in self._rtcm.feed(data):
            self.rtcm_frames += 1
            if self._rtcm_last is None or t - self._rtcm_last > self.rtcm_timeout_s:
                self._rtcm_first = t
            self._rtcm_last = t
        for cls, _mid, _payload in self._ubx.feed(data):
            if cls == ubx.CFG_CLASS:
                self.cfg_writes += 1

    def solution_at(self, t: float) -> Solution:
        if self._rtcm_last is None or t - self._rtcm_last > self.rtcm_timeout_s:
            return SINGLE
        if t - self._rtcm_first >= self.time_to_fix_s:
            return FIXED
        return FLOAT

    # --- what it sends ---------------------------------------------------------

    def nav_pvt(self, t: float, lat: float, lon: float, height_m: float,
                vel_e: float = 0.0, vel_n: float = 0.0) -> ubx.NavPvt:
        """t: seconds since the Unix epoch (wall or sim clock)."""
        solution = self.solution_at(t)
        if solution is not self._solution:
            # A better solution does not inherit the old one's metres of error.
            scale = solution.wander_m / self._solution.wander_m
            self._bias = [b * scale for b in self._bias]
            self._solution = solution
        step = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        tau = solution.tau_s
        drive = solution.wander_m * math.sqrt(2.0 / tau)
        for i in range(2):
            self._bias[i] += (-self._bias[i] / tau * step
                              + drive * math.sqrt(step) * self._rng.gauss(0.0, 1.0))
        east = self._bias[0] + self._rng.gauss(0.0, solution.white_m)
        north = self._bias[1] + self._rng.gauss(0.0, solution.white_m)
        up = self._rng.gauss(0.0, solution.white_m * 1.5)
        lat_out = lat + math.degrees(north / EARTH_RADIUS_M)
        lon_out = lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
        utc = dt.datetime.fromtimestamp(t, tz=dt.timezone.utc)
        gps_s = (utc - GPS_EPOCH).total_seconds() + GPS_UTC_LEAP_S
        return ubx.NavPvt(
            itow_ms=int(round(gps_s * 1000.0)) % (7 * 86400 * 1000), fix_type=ubx.FIX_3D,
            gnss_fix_ok=True, diff_soln=solution.diff, carr_soln=solution.carr,
            num_sv=self.num_sv, lat_deg=lat_out, lon_deg=lon_out,
            height_m=height_m + up, hmsl_m=height_m + up - 36.0,
            h_acc_m=solution.h_acc_m, v_acc_m=solution.v_acc_m,
            vel_n=vel_n + self._rng.gauss(0.0, 0.02), vel_e=vel_e + self._rng.gauss(0.0, 0.02),
            vel_d=0.0,
            utc=(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second))

    def frame(self, *args, **kwargs) -> bytes:
        return ubx.encode_nav_pvt(self.nav_pvt(*args, **kwargs))
