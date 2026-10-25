"""RTCM 3 framing: what comes from the NTRIP caster and goes to the F9P.

ROS-free. A frame is 0xD3, 6 reserved bits + a 10-bit length, the payload, and
a 24-bit CRC (CRC-24Q). The message type is the payload's first 12 bits.

Why the robot frames corrections it only forwards: a caster that answers with
an HTML error page, or a TCP stream that resyncs mid-frame, would otherwise be
written into the receiver as if it were corrections. The F9P would ignore it —
and the web page would show "corrections flowing" while the receiver sat in a
single-point solution. Only frames with a good CRC count, and only those are
forwarded.
"""

from __future__ import annotations

import struct
from typing import Iterator, List, Tuple

PREAMBLE = 0xD3


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def frame(payload: bytes) -> bytes:
    if len(payload) > 1023:
        raise ValueError("RTCM payload is at most 1023 bytes")
    head = bytes((PREAMBLE, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    body = head + payload
    return body + crc24q(body).to_bytes(3, "big")


def message_type(payload: bytes) -> int:
    if len(payload) < 2:
        return 0
    return struct.unpack(">H", payload[:2])[0] >> 4


def make_message(msg_type: int, size: int = 20) -> bytes:
    """A syntactically valid frame of the given type (filler payload) — for the
    fakes and tests; real content comes from the caster."""
    payload = bytes(((msg_type >> 4) & 0xFF, (msg_type & 0x0F) << 4)) + bytes(max(0, size - 2))
    return frame(payload)


class RtcmParser:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_crcs = 0

    def feed(self, data: bytes) -> Iterator[Tuple[int, bytes]]:
        """Yields (message type, the whole frame) for every frame with a good CRC."""
        self._buf.extend(data)
        while True:
            start = self._buf.find(bytes((PREAMBLE,)))
            if start < 0:
                self._buf.clear()
                return
            del self._buf[:start]
            if len(self._buf) < 3:
                return
            if self._buf[1] & 0xFC:          # reserved bits must be zero
                del self._buf[:1]
                continue
            length = ((self._buf[1] & 0x03) << 8) | self._buf[2]
            total = 3 + length + 3
            if len(self._buf) < total:
                return
            whole = bytes(self._buf[:total])
            if crc24q(whole[:-3]) != int.from_bytes(whole[-3:], "big"):
                self.bad_crcs += 1
                del self._buf[:1]
                continue
            del self._buf[:total]
            yield message_type(whole[3:-3]), whole


def frames(data: bytes) -> List[Tuple[int, bytes]]:
    return list(RtcmParser().feed(data))
