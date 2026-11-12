"""EFeru hoverboard-firmware-hack-FOC serial frames (VARIANT_USART).

This is the STM32 side of the link, i.e. what firmware/esp32_bridge/src/main.cpp
speaks on Serial2 (HoverCommand / HoverFeedback) — NOT the Pi<->ESP32 protocol in
hoverboard_bridge/protocol.py. Layouts follow EFeru Src/util.c (SerialCommand)
and Src/main.c (SerialFeedback), little-endian, packed.

The wizard only ever drives wheels that are off the ground, at a crawl:
encode_command() refuses anything above SPEED_CAP.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

START = 0xABCD
START_BYTES = struct.pack('<H', START)          # b'\xcd\xab'

# Hard ceiling for anything this tool sends. In SPD_MODE an input of 1000 maps
# to N_MOT_MAX rpm (1000 by default), so 100 is ~100 rpm — a slow wheel in air.
SPEED_CAP = 100

_CMD = struct.Struct('<HhhH')                     # start, steer, speed, checksum
_FB = struct.Struct('<HhhhhhhHH')                 # 18 B, ENABLE_ODOMETRY off
_FB_ODOM = struct.Struct('<HhhhhhhhhHH')          # 22 B, ENABLE_ODOMETRY on

CMD_SIZE = _CMD.size
FB_SIZE = _FB.size
FB_ODOM_SIZE = _FB_ODOM.size


def _u16(v: int) -> int:
    return v & 0xFFFF


def xor16(*values: int) -> int:
    out = 0
    for v in values:
        out ^= _u16(v)
    return out


def encode_command(steer: int, speed: int, cap: int = SPEED_CAP) -> bytes:
    """One 8-byte command frame. Raises ValueError above the cap."""
    if abs(steer) > cap or abs(speed) > cap:
        raise ValueError(
            f'steer={steer} speed={speed}: sihirbaz sınırı ±{cap} aşılamaz')
    return _CMD.pack(START, steer, speed, xor16(START, steer, speed))


def decode_command(data: bytes):
    """(steer, speed) or None if the frame is not a valid command."""
    if len(data) != CMD_SIZE:
        return None
    start, steer, speed, chk = _CMD.unpack(data)
    if start != START or chk != xor16(start, steer, speed):
        return None
    return steer, speed


@dataclass
class Feedback:
    cmd1: int
    cmd2: int
    speed_r: int          # speedR_meas [rpm]
    speed_l: int          # speedL_meas [rpm]
    bat: int              # batVoltage, ~centivolt (uncalibrated until B4)
    temp: int             # boardTemp [degC * 10]
    led: int
    t: float = 0.0        # host timestamp, filled in by the reader
    odometry: bool = False


def encode_feedback(fb: Feedback, odometry: bool = False,
                    wheel_r: int = 0, wheel_l: int = 0) -> bytes:
    """Used by the fake board; mirrors main.c's checksum."""
    if odometry:
        fields = (START, fb.cmd1, fb.cmd2, fb.speed_r, fb.speed_l,
                  wheel_r, wheel_l, fb.bat, fb.temp)
        return _FB_ODOM.pack(*fields, _u16(fb.led), xor16(*fields, fb.led))
    fields = (START, fb.cmd1, fb.cmd2, fb.speed_r, fb.speed_l, fb.bat, fb.temp)
    return _FB.pack(*fields, _u16(fb.led), xor16(*fields, fb.led))


def _try_decode(buf: bytes, odometry: bool):
    st = _FB_ODOM if odometry else _FB
    vals = st.unpack(buf[:st.size])
    *fields, led, chk = vals
    if fields[0] != START or chk != xor16(*fields, led):
        return None
    if odometry:
        _, c1, c2, sr, sl, _wr, _wl, bat, temp = fields
    else:
        _, c1, c2, sr, sl, bat, temp = fields
    return Feedback(c1, c2, sr, sl, bat, temp, led, odometry=odometry)


class FeedbackParser:
    """Stream parser: feed() raw bytes, get whole frames back.

    Counts 22-byte ENABLE_ODOMETRY frames separately: they are valid EFeru
    frames but the ESP32 bridge only understands the 18-byte layout, so seeing
    one is a configuration error worth naming.
    """

    def __init__(self):
        self._buf = bytearray()
        self.frames = 0
        self.odometry_frames = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[Feedback]:
        self._buf.extend(data)
        out: list[Feedback] = []
        while True:
            i = self._buf.find(START_BYTES)
            if i < 0:
                # keep a trailing 0xCD, it may be half a start word
                keep = 1 if self._buf[-1:] == START_BYTES[:1] else 0
                self.dropped_bytes += len(self._buf) - keep
                del self._buf[:len(self._buf) - keep]
                return out
            if i:
                self.dropped_bytes += i
                del self._buf[:i]
            if len(self._buf) < FB_SIZE:
                return out
            fb = _try_decode(bytes(self._buf), odometry=False)
            if fb is not None:
                del self._buf[:FB_SIZE]
                self.frames += 1
                out.append(fb)
                continue
            if len(self._buf) < FB_ODOM_SIZE:
                return out          # could still be a 22-byte frame
            fb = _try_decode(bytes(self._buf), odometry=True)
            if fb is not None:
                del self._buf[:FB_ODOM_SIZE]
                self.odometry_frames += 1
                out.append(fb)
                continue
            self.dropped_bytes += 1
            del self._buf[:1]
