"""The kit's 360° camera through Insta360's Camera SDK — the robot's side. ROS-free.

The SDK is a C++ library (libCameraSDK.so, USB only, Ubuntu 22.04 x86-64 and
aarch64 per Insta360's docs) that comes by application and does NOT go into
this repo. So the robot talks to it through a small separate program, the
SHIM, over JSON lines on stdin/stdout:

    request   {"id": 7, "cmd": "status"}
    response  {"id": 7, "ok": true, "data": {...}}   or  {"id": 7, "ok": false, "error": "..."}
    event     {"event": "temperature_high"}  (also storage_full, battery_low, capture_stopped)

    cmd          data in / out
    connect      -> {"model", "serial", "firmware"}
    status       -> {"recording": bool, "battery_pct": int, "on_adapter": bool,
                     "storage": "ok"|"no_card"|"full"|..., "free_bytes": int}
    start        {"resolution": "5.7K30"|...} -> {}
    stop         -> {"files": [...]}               (the SD card's paths)
    sync_time    {"utc_ms": int, "tz_offset_s": int} -> {}
    media_time   -> {"media_time": int}            (GetCameraMediaTime)

Why a separate process: the SDK needs root or a udev rule for USB, runs its
own HTTP server (port 9099), and is closed source — a crash in it must not
take a ROS node down with it, and a robot built without it must still build.
`fake_camera_shim` speaks the same protocol with a simulated camera; the real
shim (C++, linked against the SDK) is written once the SDK's headers are on
the build machine (docs/kit-soketi.md, "360° kamera").
"""

from __future__ import annotations

import json
import queue
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

EVENTS = ("temperature_high", "storage_full", "battery_low", "capture_stopped")


class CameraError(RuntimeError):
    """The shim refused or is gone; the message says which."""


@dataclass
class TimePair:
    """One (robot clock, camera media clock) sample — orchadOS lines the video
    up with the GNSS log from these (the QR step's replacement on the robot)."""
    robot_ms: int
    camera_media: int
    round_trip_ms: float


class ShimClient:
    def __init__(self, command: Sequence[str], timeout_s: float = 5.0) -> None:
        self.timeout_s = timeout_s
        self._proc = subprocess.Popen(list(command), stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      text=True, bufsize=1)
        self._next = 0
        self._replies: "queue.Queue[dict]" = queue.Queue()
        self._events: "queue.Queue[str]" = queue.Queue()
        self._lock = threading.Lock()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        for line in self._proc.stdout:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if "event" in message:
                self._events.put(str(message["event"]))
            else:
                self._replies.put(message)
        self._replies.put({"id": None, "ok": False, "error": "the camera shim exited"})

    def alive(self) -> bool:
        return self._proc.poll() is None

    def call(self, cmd: str, **args) -> Dict[str, Any]:
        with self._lock:
            if not self.alive():
                raise CameraError("the camera shim is not running")
            self._next += 1
            ident = self._next
            self._proc.stdin.write(json.dumps({"id": ident, "cmd": cmd, **args}) + "\n")
            self._proc.stdin.flush()
            deadline = time.monotonic() + self.timeout_s
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise CameraError(f"{cmd}: no answer in {self.timeout_s:.0f} s")
                try:
                    reply = self._replies.get(timeout=left)
                except queue.Empty:
                    continue
                if reply.get("id") is None and not reply.get("ok"):
                    raise CameraError(reply.get("error", "the camera shim exited"))
                if reply.get("id") != ident:
                    continue            # a late answer to a call that timed out
                if not reply.get("ok"):
                    raise CameraError(f"{cmd}: {reply.get('error', 'refused')}")
                return reply.get("data") or {}

    def events(self) -> List[str]:
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                return out

    def time_pair(self, now_ms) -> TimePair:
        """now_ms(): the robot's clock (PPS-locked on the robot), ms."""
        before = now_ms()
        media = int(self.call("media_time")["media_time"])
        after = now_ms()
        return TimePair(robot_ms=(before + after) // 2, camera_media=media,
                        round_trip_ms=float(after - before))

    def close(self) -> None:
        try:
            self._proc.stdin.close()
        except OSError:
            pass
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()


class FakeCamera:
    """What fake_camera_shim simulates: a camera that records to a card, warms
    up while recording and stops itself when too hot (the Gaziantep-summer
    risk in docs/kit-soketi.md) — and which never really records anything."""

    def __init__(self, overheat_after_s: float = 0.0, free_bytes: int = 400 * 10 ** 9,
                 bytes_per_s: int = 12_500_000) -> None:
        self.overheat_after_s = overheat_after_s     # 0 = never
        self.free_bytes = free_bytes
        self.bytes_per_s = bytes_per_s
        self.recording_since: Optional[float] = None
        self.clock_offset_ms = 0
        self.files: List[str] = []
        self.pending_events: List[str] = []
        self.connected = False

    def handle(self, cmd: str, args: Dict[str, Any], now: float) -> Dict[str, Any]:
        self.tick(now)
        if cmd == "connect":
            self.connected = True
            return {"model": "fake X5", "serial": "FAKE0001", "firmware": "0.0"}
        if not self.connected:
            raise CameraError("not connected")
        if cmd == "status":
            return {"recording": self.recording_since is not None, "battery_pct": 100,
                    "on_adapter": True, "storage": "ok" if self.free_bytes > 0 else "full",
                    "free_bytes": self.free_bytes}
        if cmd == "start":
            if self.recording_since is not None:
                raise CameraError("already recording")
            self.recording_since = now
            return {}
        if cmd == "stop":
            if self.recording_since is None:
                raise CameraError("not recording")
            self.recording_since = None
            name = f"/DCIM/Camera01/VID_{len(self.files) + 1:03d}.insv"
            self.files.append(name)
            return {"files": [name]}
        if cmd == "sync_time":
            self.clock_offset_ms = int(args["utc_ms"]) - int(now * 1000)
            return {}
        if cmd == "media_time":
            return {"media_time": int(now * 1000) + self.clock_offset_ms}
        raise CameraError(f"unknown command {cmd!r}")

    def tick(self, now: float) -> None:
        last, self._last = getattr(self, "_last", None), now
        if self.recording_since is None:
            return
        if last is not None:
            self.free_bytes = max(0, self.free_bytes - int(self.bytes_per_s * (now - last)))
        if self.free_bytes == 0:
            self.recording_since = None
            self.pending_events += ["storage_full", "capture_stopped"]
        elif self.overheat_after_s and now - self.recording_since >= self.overheat_after_s:
            self.recording_since = None
            self.pending_events += ["temperature_high", "capture_stopped"]
