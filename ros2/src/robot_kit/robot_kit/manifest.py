"""The kit's identity chip (DS2431, 128 bytes): what is plugged into the socket.

ROS-free. docs/kit-soketi.md ("Kimlik çipinin içeriği") is the contract; the
orchadOS repo has the WRITER of this format and the two share the test vectors
in test/vectors/ — change one side, change the other, run both suites (the same
rule as the serial protocol's two copies).

Layout, little-endian, written from offset 0 of the EEPROM:

    off  size  field
    0    4     magic  b"HKIT"
    4    1     schema (this module reads 1)
    5    1     body length: bytes from offset 6 up to the CRC
    6    1     kind   (1 = orchados-scan-kit)
    7    8     serial, ASCII, NUL-padded
    15   2     hw_rev major, minor
    17   2     caps   bit field (CAPS)
    19   1     gnss_link (GNSS_LINKS)
    20   4     calib_id
    24   1     frame count k (<= 3)
    25   25*k  per frame: sensor id (SENSORS) + x y z roll pitch yaw (float32),
               the sensor in the kit's own mounting frame `kit_mount`
    ..   2     CRC-16/CCITT-FALSE over everything before it

The rest of the chip stays erased (0xFF).

⚠️ AN UNKNOWN SCHEMA IS REFUSED, NEVER GUESSED. A newer kit read with this
reader's idea of the layout would hand the robot calibration numbers from the
wrong offsets — plausible floats, wrong frames, and a GNSS antenna placed a
metre from where it is. The robot stays in kitless mode and says why.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Dict, Tuple

MAGIC = b"HKIT"
SCHEMA = 1
EEPROM_SIZE = 128

KINDS = {1: "orchados-scan-kit"}
# What the kit HAS installed — not whether it works (that is the data stream).
CAPS = {"gnss_rtk": 0, "pps": 1, "cam360": 2, "cam360_ctrl": 3, "lidar": 4}
GNSS_LINKS = {0: "none", 1: "usb", 2: "ros"}
# sensor id -> the TF frame the robot publishes it as
SENSORS = {1: "kit_gnss", 2: "kit_cam360", 3: "kit_lidar"}
# Each capability needs its sensor's calibration: a camera with no pose is a
# picture of nowhere.
CAP_NEEDS_FRAME = {"gnss_rtk": "kit_gnss", "cam360": "kit_cam360", "lidar": "kit_lidar"}
# A frame further than this from the mount is a typo, not a mast.
MAX_OFFSET_M = 5.0

_HEAD = struct.Struct("<4sBB")             # magic, schema, body length
_FIXED = struct.Struct("<B8sBBHBIB")       # kind .. frame count
_FRAME = struct.Struct("<B6f")
_CRC = struct.Struct("<H")


class ManifestError(ValueError):
    """The chip does not hold a manifest this robot can trust. The message says why."""


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Manifest:
    kind: str
    serial: str
    hw_rev: Tuple[int, int]
    caps: frozenset
    gnss_link: str
    calib_id: int
    frames: Dict[str, Pose] = field(default_factory=dict)   # frame name -> pose in kit_mount
    schema: int = SCHEMA

    def has(self, cap: str) -> bool:
        return cap in self.caps


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def validate(manifest: Manifest) -> None:
    if manifest.kind not in KINDS.values():
        raise ManifestError(f"unknown kind {manifest.kind!r}")
    unknown = set(manifest.caps) - set(CAPS)
    if unknown:
        raise ManifestError(f"unknown capabilities {sorted(unknown)}")
    if manifest.gnss_link not in GNSS_LINKS.values():
        raise ManifestError(f"unknown gnss_link {manifest.gnss_link!r}")
    if manifest.has("gnss_rtk") and manifest.gnss_link == "none":
        raise ManifestError("gnss_rtk installed but gnss_link is 'none'")
    if manifest.has("cam360_ctrl") and not manifest.has("cam360"):
        raise ManifestError("cam360_ctrl without cam360")
    for name, pose in manifest.frames.items():
        if name not in SENSORS.values():
            raise ManifestError(f"unknown frame {name!r}")
        values = (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw)
        if not all(math.isfinite(v) for v in values):
            raise ManifestError(f"{name}: non-finite calibration")
        if max(abs(pose.x), abs(pose.y), abs(pose.z)) > MAX_OFFSET_M:
            raise ManifestError(f"{name}: more than {MAX_OFFSET_M} m from kit_mount")
        if max(abs(pose.roll), abs(pose.pitch), abs(pose.yaw)) > math.pi + 1e-6:
            raise ManifestError(f"{name}: angle outside [-pi, pi]")
    for cap, frame in CAP_NEEDS_FRAME.items():
        if manifest.has(cap) and frame not in manifest.frames:
            raise ManifestError(f"{cap} installed but {frame} has no calibration")
    serial = manifest.serial.encode("ascii", errors="strict")
    if not serial or len(serial) > 8 or b"\0" in serial:
        raise ManifestError(f"serial must be 1-8 ASCII characters, got {manifest.serial!r}")
    if not 0 <= manifest.calib_id <= 0xFFFFFFFF:
        raise ManifestError("calib_id out of range")


def encode(manifest: Manifest) -> bytes:
    """The full 128-byte chip image."""
    validate(manifest)
    kind = {v: k for k, v in KINDS.items()}[manifest.kind]
    link = {v: k for k, v in GNSS_LINKS.items()}[manifest.gnss_link]
    sensor_id = {v: k for k, v in SENSORS.items()}
    caps = sum(1 << CAPS[c] for c in manifest.caps)
    body = _FIXED.pack(kind, manifest.serial.encode("ascii"), manifest.hw_rev[0],
                       manifest.hw_rev[1], caps, link, manifest.calib_id,
                       len(manifest.frames))
    for name in sorted(manifest.frames, key=lambda n: sensor_id[n]):
        p = manifest.frames[name]
        body += _FRAME.pack(sensor_id[name], p.x, p.y, p.z, p.roll, p.pitch, p.yaw)
    used = _HEAD.pack(MAGIC, SCHEMA, len(body)) + body
    used += _CRC.pack(crc16_ccitt(used))
    return used + b"\xff" * (EEPROM_SIZE - len(used))


def decode(image: bytes) -> Manifest:
    """Raises ManifestError for anything but a valid schema-1 manifest."""
    if len(image) < EEPROM_SIZE:
        raise ManifestError(f"chip read returned {len(image)} bytes, expected {EEPROM_SIZE}")
    if set(image) == {0xFF}:
        raise ManifestError("the chip is blank (never written)")
    magic, schema, length = _HEAD.unpack_from(image)
    if magic != MAGIC:
        raise ManifestError(f"not a kit manifest (magic {magic!r})")
    if schema != SCHEMA:
        raise ManifestError(f"manifest schema {schema}; this robot reads only {SCHEMA}")
    end = _HEAD.size + length
    if length < _FIXED.size or end + _CRC.size > len(image):
        raise ManifestError(f"body length {length} does not fit the chip")
    (stored,) = _CRC.unpack_from(image, end)
    if stored != crc16_ccitt(image[:end]):
        raise ManifestError("CRC mismatch (corrupt or half-written chip)")
    kind, serial, major, minor, caps_bits, link, calib_id, count = _FIXED.unpack_from(
        image, _HEAD.size)
    if _FIXED.size + count * _FRAME.size != length:
        raise ManifestError(f"{count} frames do not match body length {length}")
    if kind not in KINDS:
        raise ManifestError(f"unknown kind {kind}")
    if link not in GNSS_LINKS:
        raise ManifestError(f"unknown gnss_link {link}")
    known_bits = sum(1 << b for b in CAPS.values())
    if caps_bits & ~known_bits:
        raise ManifestError(f"unknown capability bits {caps_bits & ~known_bits:#x}")
    frames = {}
    offset = _HEAD.size + _FIXED.size
    for _ in range(count):
        sensor, *values = _FRAME.unpack_from(image, offset)
        offset += _FRAME.size
        if sensor not in SENSORS:
            raise ManifestError(f"unknown sensor id {sensor}")
        if SENSORS[sensor] in frames:
            raise ManifestError(f"{SENSORS[sensor]} calibrated twice")
        frames[SENSORS[sensor]] = Pose(*values)
    try:
        serial_text = serial.rstrip(b"\0").decode("ascii")
    except UnicodeDecodeError as exc:
        raise ManifestError("serial is not ASCII") from exc
    manifest = Manifest(
        kind=KINDS[kind], serial=serial_text, hw_rev=(major, minor),
        caps=frozenset(c for c, bit in CAPS.items() if caps_bits >> bit & 1),
        gnss_link=GNSS_LINKS[link], calib_id=calib_id, frames=frames)
    validate(manifest)
    return manifest


def to_dict(manifest: Manifest) -> dict:
    """JSON-friendly form (the test vectors, /kit/state, the web page)."""
    return {
        "kind": manifest.kind, "schema": manifest.schema, "serial": manifest.serial,
        "hw_rev": list(manifest.hw_rev), "caps": sorted(manifest.caps),
        "gnss_link": manifest.gnss_link, "calib_id": manifest.calib_id,
        "frames": {name: [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]
                   for name, p in sorted(manifest.frames.items())},
    }


def from_dict(data: dict) -> Manifest:
    try:
        return Manifest(
            kind=data["kind"], serial=data["serial"], hw_rev=tuple(data["hw_rev"]),
            caps=frozenset(data["caps"]), gnss_link=data["gnss_link"],
            calib_id=int(data["calib_id"]),
            frames={name: Pose(*map(float, values))
                    for name, values in data.get("frames", {}).items()})
    except (KeyError, TypeError) as exc:
        raise ManifestError(f"malformed manifest dict: {exc}") from exc
