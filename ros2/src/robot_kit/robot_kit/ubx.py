"""UBX framing and UBX-NAV-PVT, the one message the robot reads from the F9P.

ROS-free. Reference: u-blox F9 HPG interface description, UBX-NAV-PVT
(class 0x01, id 0x07, 92 bytes).

⚠️ THE ROBOT NEVER WRITES UBX TO THE KIT'S RECEIVER. Its configuration belongs
to orchadOS: the kit's ESP32 logger takes the raw measurements (RAWX/SFRBX) for
PPK from UART1, and a CFG message from the robot could switch them off — the
scan would still look fine and the post-processing would have nothing to work
with. So this module has a NAV-PVT ENCODER (for the fake receiver) and no CFG
encoder at all; the only bytes the robot sends the F9P are RTCM corrections.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

SYNC = b"\xb5\x62"
NAV_PVT = (0x01, 0x07)
CFG_CLASS = 0x06

_PVT = struct.Struct("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH4siHH")
assert _PVT.size == 92

# fixType
NO_FIX, DEAD_RECKONING, FIX_2D, FIX_3D, GNSS_DR, TIME_ONLY = range(6)
# flags bits 6-7: carrier-phase solution
CARR_NONE, CARR_FLOAT, CARR_FIXED = 0, 1, 2


def checksum(body: bytes) -> bytes:
    """8-bit Fletcher over class, id, length and payload."""
    a = b = 0
    for byte in body:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    body = struct.pack("<BBH", msg_class, msg_id, len(payload)) + payload
    return SYNC + body + checksum(body)


@dataclass(frozen=True)
class NavPvt:
    itow_ms: int
    fix_type: int
    gnss_fix_ok: bool
    diff_soln: bool
    carr_soln: int
    num_sv: int
    lat_deg: float
    lon_deg: float
    height_m: float          # above the WGS84 ellipsoid (NavSatFix.altitude's meaning)
    hmsl_m: float
    h_acc_m: float
    v_acc_m: float
    vel_n: float             # m/s
    vel_e: float
    vel_d: float
    invalid_llh: bool = False
    utc: Tuple[int, int, int, int, int, int] = (2026, 1, 1, 0, 0, 0)
    utc_valid: bool = True

    def has_position(self) -> bool:
        return (self.gnss_fix_ok and not self.invalid_llh
                and self.fix_type in (FIX_2D, FIX_3D, GNSS_DR))


def encode_nav_pvt(pvt: NavPvt) -> bytes:
    flags = ((1 if pvt.gnss_fix_ok else 0) | (2 if pvt.diff_soln else 0)
             | (pvt.carr_soln & 0x3) << 6)
    year, month, day, hour, minute, second = pvt.utc
    valid = 0x07 if pvt.utc_valid else 0x00
    payload = _PVT.pack(
        pvt.itow_ms, year, month, day, hour, minute, second, valid,
        50, 0,                                   # tAcc ns, nano
        pvt.fix_type, flags, 0, pvt.num_sv,
        round(pvt.lon_deg * 1e7), round(pvt.lat_deg * 1e7),
        round(pvt.height_m * 1000), round(pvt.hmsl_m * 1000),
        round(pvt.h_acc_m * 1000), round(pvt.v_acc_m * 1000),
        round(pvt.vel_n * 1000), round(pvt.vel_e * 1000), round(pvt.vel_d * 1000),
        round((pvt.vel_n ** 2 + pvt.vel_e ** 2) ** 0.5 * 1000),
        0, 100, 0, 120,                          # headMot, sAcc, headAcc, pDOP
        1 if pvt.invalid_llh else 0, b"\0" * 4, 0, 0, 0)
    return frame(*NAV_PVT, payload)


def decode_nav_pvt(payload: bytes) -> NavPvt:
    if len(payload) != _PVT.size:
        raise ValueError(f"NAV-PVT is {_PVT.size} bytes, got {len(payload)}")
    (itow, year, month, day, hour, minute, second, valid, _tacc, _nano, fix_type, flags,
     _flags2, num_sv, lon, lat, height, hmsl, hacc, vacc, vel_n, vel_e, vel_d, _gspeed,
     _head, _sacc, _headacc, _pdop, flags3, _res, _headveh, _magdec, _magacc) = \
        _PVT.unpack(payload)
    return NavPvt(
        itow_ms=itow, fix_type=fix_type, gnss_fix_ok=bool(flags & 1),
        diff_soln=bool(flags & 2), carr_soln=(flags >> 6) & 0x3, num_sv=num_sv,
        lat_deg=lat * 1e-7, lon_deg=lon * 1e-7, height_m=height / 1000.0,
        hmsl_m=hmsl / 1000.0, h_acc_m=hacc / 1000.0, v_acc_m=vacc / 1000.0,
        vel_n=vel_n / 1000.0, vel_e=vel_e / 1000.0, vel_d=vel_d / 1000.0,
        invalid_llh=bool(flags3 & 1), utc=(year, month, day, hour, minute, second),
        utc_valid=(valid & 0x03) == 0x03)


class UbxParser:
    """Bytes in, (class, id, payload) out. NMEA, RTCM and line noise on the same
    port are skipped: the F9P's USB port carries whatever orchadOS enabled on it,
    and only UBX frames with a good checksum count."""

    MAX_PAYLOAD = 4096

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> Iterator[Tuple[int, int, bytes]]:
        self._buf.extend(data)
        while True:
            start = self._buf.find(SYNC)
            if start < 0:
                # keep a trailing 0xB5: it may be half a sync word
                del self._buf[:-1 if self._buf.endswith(SYNC[:1]) else len(self._buf)]
                return
            del self._buf[:start]
            if len(self._buf) < 6:
                return
            msg_class, msg_id, length = struct.unpack_from("<BBH", self._buf, 2)
            if length > self.MAX_PAYLOAD:
                del self._buf[:2]
                continue
            total = 6 + length + 2
            if len(self._buf) < total:
                return
            body = bytes(self._buf[2:6 + length])
            if checksum(body) != bytes(self._buf[6 + length:total]):
                self.bad_checksums += 1
                del self._buf[:2]
                continue
            del self._buf[:total]
            yield msg_class, msg_id, body[4:]


def nav_pvts(parser: UbxParser, data: bytes) -> List[NavPvt]:
    return [decode_nav_pvt(payload) for cls, mid, payload in parser.feed(data)
            if (cls, mid) == NAV_PVT and len(payload) == _PVT.size]


# --- NavSatFix mapping (ROS-free numbers; kit_gnss_node builds the message) ---

# sensor_msgs/NavSatStatus
STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX = -1, 0, 1, 2


def fix_status(pvt: NavPvt) -> int:
    """RTK float or fixed -> GBAS (ground-based augmentation, which is what RTK
    is); other differential -> SBAS; else a plain fix."""
    if not pvt.has_position():
        return STATUS_NO_FIX
    if pvt.carr_soln in (CARR_FLOAT, CARR_FIXED):
        return STATUS_GBAS_FIX
    if pvt.diff_soln:
        return STATUS_SBAS_FIX
    return STATUS_FIX


def covariance_enu(pvt: NavPvt) -> Tuple[float, float, float]:
    """Diagonal variances (east, north, up), m². hAcc is the receiver's own
    horizontal accuracy estimate; it is used for EACH axis — conservative by up
    to 2x, which the EKF tolerates better than the reverse."""
    return (pvt.h_acc_m ** 2, pvt.h_acc_m ** 2, pvt.v_acc_m ** 2)


def solution_name(pvt: Optional[NavPvt]) -> str:
    if pvt is None or not pvt.has_position():
        return "none"
    return {CARR_FIXED: "rtk_fixed", CARR_FLOAT: "rtk_float"}.get(pvt.carr_soln, "single")
