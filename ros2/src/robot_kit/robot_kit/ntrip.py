"""NTRIP client pieces: the request, the caster's answer, chunked bodies, GGA.

ROS-free; ntrip_client_node does the socket. TUSAGA-Aktif (the corrections
orchadOS subscribes to) is an NTRIP caster with network (VRS) mountpoints.

⚠️ A VRS MOUNTPOINT SENDS NOTHING UNTIL IT HEARS WHERE YOU ARE. The caster
computes a virtual reference station near the rover from the NMEA GGA the
client sends; without it the connection stays open and silent, which looks
exactly like a caster that is merely slow. So the client sends a GGA right
after connecting and then every `gga_interval_s`.

⚠️ "SOURCETABLE 200 OK" IS NOT SUCCESS. A caster answers an unknown mountpoint
with its source table and a 200 — the connection "works" and carries no
corrections, forever. It is an error here, named as one.
"""

from __future__ import annotations

import base64
import math
from dataclasses import dataclass
from typing import Optional, Tuple


class NtripError(RuntimeError):
    """The caster refused; the message is what to fix."""


@dataclass(frozen=True)
class Caster:
    host: str
    port: int = 2101
    mountpoint: str = ""
    user: str = ""
    password: str = ""
    version: int = 2                   # NTRIP 2 (HTTP/1.1); 1 for old casters


USER_AGENT = "NTRIP robot_kit/0.1"


def request(caster: Caster) -> bytes:
    if not caster.mountpoint:
        raise NtripError("no mountpoint configured")
    lines = []
    if caster.version == 1:
        lines.append(f"GET /{caster.mountpoint} HTTP/1.0")
    else:
        lines += [f"GET /{caster.mountpoint} HTTP/1.1", f"Host: {caster.host}:{caster.port}",
                  "Ntrip-Version: Ntrip/2.0"]
    lines.append(f"User-Agent: {USER_AGENT}")
    if caster.user:
        token = base64.b64encode(f"{caster.user}:{caster.password}".encode()).decode()
        lines.append(f"Authorization: Basic {token}")
    if caster.version != 1:
        lines.append("Connection: close")
    return ("\r\n".join(lines) + "\r\n\r\n").encode("ascii")


@dataclass(frozen=True)
class Response:
    chunked: bool
    body_start: bytes                  # bytes after the header that already arrived


def parse_response(head: bytes) -> Optional[Response]:
    """None until the whole header has arrived; raises NtripError on refusal.

    NTRIP 1 casters answer "ICY 200 OK" with no further header (the body may
    follow right after one CRLF); NTRIP 2 is plain HTTP."""
    if head.startswith(b"ICY 200 OK"):
        end = head.find(b"\r\n")
        if end < 0:
            return None
        rest = head[end + 2:]
        if rest.startswith(b"\r\n"):
            rest = rest[2:]
        return Response(chunked=False, body_start=rest)
    end = head.find(b"\r\n\r\n")
    if end < 0:
        if head.startswith(b"SOURCETABLE"):
            raise NtripError("the caster sent its source table: the mountpoint does not exist")
        return None
    text = head[:end].decode("latin-1")
    status = text.split("\r\n", 1)[0]
    if status.startswith("SOURCETABLE"):
        raise NtripError("the caster sent its source table: the mountpoint does not exist")
    parts = status.split(" ", 2)
    if len(parts) < 2 or not parts[0].startswith("HTTP/"):
        raise NtripError(f"not an NTRIP answer: {status!r}")
    code = parts[1]
    if code == "401":
        raise NtripError("401: wrong user name or password")
    if code == "404":
        raise NtripError("404: no such mountpoint")
    if code != "200":
        raise NtripError(f"caster said {status!r}")
    headers = {line.split(":", 1)[0].strip().lower(): line.split(":", 1)[1].strip()
               for line in text.split("\r\n")[1:] if ":" in line}
    if headers.get("content-type", "").startswith("gnss/sourcetable"):
        raise NtripError("the caster sent its source table: the mountpoint does not exist")
    chunked = headers.get("transfer-encoding", "").lower() == "chunked"
    return Response(chunked=chunked, body_start=head[end + 4:])


class Dechunker:
    """HTTP/1.1 chunked transfer decoding, incremental (NTRIP 2 casters use it)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._left = 0                  # payload bytes left in the current chunk
        self._need_crlf = False
        self.finished = False

    def feed(self, data: bytes) -> bytes:
        self._buf.extend(data)
        out = bytearray()
        while not self.finished:
            if self._need_crlf:
                if len(self._buf) < 2:
                    break
                del self._buf[:2]
                self._need_crlf = False
            if self._left:
                take = min(self._left, len(self._buf))
                if not take:
                    break
                out += self._buf[:take]
                del self._buf[:take]
                self._left -= take
                if not self._left:
                    self._need_crlf = True
                continue
            end = self._buf.find(b"\r\n")
            if end < 0:
                break
            size_text = self._buf[:end].split(b";")[0].strip()
            del self._buf[:end + 2]
            try:
                self._left = int(size_text, 16)
            except ValueError as exc:
                raise NtripError(f"bad chunk size {bytes(size_text)!r}") from exc
            if self._left == 0:
                self.finished = True
        return bytes(out)


def nmea_checksum(body: str) -> str:
    value = 0
    for char in body:
        value ^= ord(char)
    return f"{value:02X}"


def gga(lat: float, lon: float, alt_m: float, utc_hms: Tuple[int, int, float],
        quality: int = 1, num_sv: int = 12, hdop: float = 1.0) -> bytes:
    """$GPGGA for the caster. quality: 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 float."""
    hour, minute, second = utc_hms

    def dm(value: float, width: int) -> str:
        degrees = int(abs(value))
        minutes = round((abs(value) - degrees) * 60.0, 4)
        if minutes >= 60.0:                 # 59.99999 rounds up to 60.0000
            degrees, minutes = degrees + 1, 0.0
        return f"{degrees:0{width}d}{minutes:07.4f}"

    lat_text, lon_text = dm(lat, 2), dm(lon, 3)
    body = (f"GPGGA,{hour:02d}{minute:02d}{second:05.2f},{lat_text},{'N' if lat >= 0 else 'S'},"
            f"{lon_text},{'E' if lon >= 0 else 'W'},{quality},{num_sv:02d},{hdop:.1f},"
            f"{alt_m:.1f},M,0.0,M,,")
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def gga_quality(status: int) -> int:
    """NavSatFix status (ubx.fix_status) -> GGA quality."""
    return {-1: 0, 0: 1, 1: 2, 2: 5}.get(status, 0)


def backoff_s(attempt: int, base: float = 2.0, cap: float = 60.0) -> float:
    """Reconnect delay: 2, 4, 8 ... 60 s. A caster that drops us is not hammered."""
    return min(cap, base * math.pow(2.0, max(0, attempt)))
