"""Talking to the freshly flashed board over its USART2 cable.

Three things, in the order the wizard needs them:
  listen()        — receive only; identifies the STM32 TX wire without driving
                    anything into the board
  spin_test()     — a slow, short, capped command; did the wheels turn and
                    did the hall feedback see it
  timeout_test()  — stop sending mid-spin and time how long until the
                    firmware drops the command (SERIAL_TIMEOUT, ~0.8 s). The
                    ESP32 watchdog is the first layer of "link lost -> stop";
                    this is the second, and it is measured, not assumed.

Every function that sends a non-zero command ends by sending zeros, also
when interrupted (Ctrl-C) — and if this process dies outright, the firmware's
own timeout is exactly what timeout_test() just proved.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol

from .frames import Feedback, FeedbackParser, encode_command

TEST_SPEED = 50          # of 1000; ~50 rpm in SPD_MODE with N_MOT_MAX 1000
RATE_HZ = 50             # the ESP32 bridge also sends at a steady rate
TIMEOUT_LIMIT_S = 1.2    # SERIAL_TIMEOUT 160 * 5 ms = 0.8 s, plus margin


class Port(Protocol):
    def write(self, data: bytes) -> int: ...
    def read_available(self) -> bytes: ...
    def close(self) -> None: ...


class Clock(Protocol):
    def now(self) -> float: ...
    def sleep(self, dt: float) -> None: ...


class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, dt: float) -> None:
        time.sleep(dt)


class SerialPort:
    """pyserial behind the Port interface; imported only when needed."""

    def __init__(self, device: str, baud: int = 115200):
        import serial  # python3-serial is in the dev container image
        self._s = serial.Serial(device, baud, timeout=0)

    def write(self, data: bytes) -> int:
        return self._s.write(data)

    def read_available(self) -> bytes:
        n = self._s.in_waiting
        return self._s.read(n) if n else b''

    def close(self) -> None:
        self._s.close()


def _pump(port: Port, parser: FeedbackParser, clock: Clock, out: list[Feedback]):
    data = port.read_available()
    if data:
        t = clock.now()
        for fb in parser.feed(data):
            fb.t = t
            out.append(fb)


@dataclass
class ListenResult:
    frames: list[Feedback]
    odometry_frames: int
    dropped_bytes: int

    @property
    def valid(self) -> int:
        return len(self.frames) - self.odometry_frames


def listen(port: Port, clock: Clock, seconds: float) -> ListenResult:
    parser, frames = FeedbackParser(), []
    end = clock.now() + seconds
    while clock.now() < end:
        _pump(port, parser, clock, frames)
        clock.sleep(0.01)
    _pump(port, parser, clock, frames)
    return ListenResult(frames, parser.odometry_frames, parser.dropped_bytes)


def drive(port: Port, clock: Clock, seconds: float, speed: int, steer: int = 0,
          parser: FeedbackParser | None = None,
          frames: list[Feedback] | None = None) -> float:
    """Send the command at RATE_HZ for `seconds`. Returns the last write time."""
    parser = parser or FeedbackParser()
    frames = frames if frames is not None else []
    frame = encode_command(steer, speed)
    end = clock.now() + seconds
    last = clock.now()
    while clock.now() < end:
        port.write(frame)
        last = clock.now()
        _pump(port, parser, clock, frames)
        clock.sleep(1.0 / RATE_HZ)
    return last


def stop(port: Port, clock: Clock, seconds: float = 0.5):
    drive(port, clock, seconds, 0)


@dataclass
class SpinResult:
    ok: bool
    frames: int
    cmd_seen: bool
    mean_left: float
    mean_right: float
    diagnosis: list[str] = field(default_factory=list)


def spin_test(port: Port, clock: Clock, speed: int = TEST_SPEED,
              seconds: float = 3.0) -> SpinResult:
    parser, frames = FeedbackParser(), []
    try:
        t_end = drive(port, clock, seconds, speed, parser=parser, frames=frames)
    finally:
        stop(port, clock, 1.0)
    late = [f for f in frames if f.t >= t_end - 1.0]
    if not frames:
        return SpinResult(False, 0, False, 0.0, 0.0,
                          ['Komut gönderilirken hiç feedback gelmedi — '
                           'RX teli / kart açık mı?'])
    cmd_seen = any(abs(f.cmd2 - speed) <= 2 for f in late)
    ml = sum(f.speed_l for f in late) / max(len(late), 1)
    mr = sum(f.speed_r for f in late) / max(len(late), 1)
    diag = []
    if not cmd_seen:
        diag.append('Feedback geliyor ama firmware komutu görmüyor (cmd2 ≠ '
                    f'{speed}): USB-TTL TX → STM32 RX teli bağlı mı, doğru tel mi?')
    moving = abs(ml) >= 0.3 * speed, abs(mr) >= 0.3 * speed
    if cmd_seen and not moving[0]:
        diag.append(f'Sol tekerlek dönmüyor (ortalama {ml:.0f} rpm): hall '
                    'sırası / motor fazları / hall kablosu (hasar vermez)')
    if cmd_seen and not moving[1]:
        diag.append(f'Sağ tekerlek dönmüyor (ortalama {mr:.0f} rpm): hall '
                    'sırası / motor fazları / hall kablosu (hasar vermez)')
    return SpinResult(cmd_seen and all(moving), len(frames), cmd_seen, ml, mr, diag)


@dataclass
class TimeoutResult:
    ok: bool
    cmd_seen: bool                    # the command was applied before the silence
    cmd_zero_after_s: float | None    # last write -> feedback cmd1 = cmd2 = 0
    slowed_after_s: float | None      # last write -> both wheels under 20 %
    frames: int


def timeout_test(port: Port, clock: Clock, speed: int = TEST_SPEED,
                 drive_s: float = 2.0, watch_s: float = 3.0) -> TimeoutResult:
    parser, frames = FeedbackParser(), []
    try:
        t_last = drive(port, clock, drive_s, speed, parser=parser, frames=frames)
        end = clock.now() + watch_s
        while clock.now() < end:              # silence: nothing written
            _pump(port, parser, clock, frames)
            clock.sleep(0.01)
        _pump(port, parser, clock, frames)
    finally:
        stop(port, clock, 0.5)
    # Without this a board that never took the command would "time out" at 0 s.
    cmd_seen = any(abs(f.cmd2 - speed) <= 2 for f in frames if f.t <= t_last)
    after = [f for f in frames if f.t > t_last]
    zero = next((f.t - t_last for f in after if f.cmd1 == 0 and f.cmd2 == 0), None)
    slow = next((f.t - t_last for f in after
                 if abs(f.speed_l) < 0.2 * speed and abs(f.speed_r) < 0.2 * speed),
                None)
    ok = cmd_seen and zero is not None and zero <= TIMEOUT_LIMIT_S
    return TimeoutResult(ok, cmd_seen, zero, slow, len(frames))
