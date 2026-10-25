"""Kit mode: is a kit plugged in, what is it, is it working, which GPS to use.

ROS-free; kit_manager_node feeds it and publishes what it says
(docs/kit-soketi.md, "Mod durum makinesi").

    NONE ──(plug stable + chip read)──► READY ◄──(streams)──► DEGRADED
      ▲                                   │                      │
      └──────────────── plug gone ────────┴──────────────────────┘

- The plug is DEBOUNCED: it must hold for `debounce_s` before anything moves.
  A connector being screwed on makes and breaks the presence pin many times.
- The chip is read once the plug is stable, and re-read every `chip_retry_s`
  while that fails (dusty contacts). A chip that cannot be trusted
  (manifest.ManifestError) keeps the robot in NONE, with the reason.
- A stream (GNSS fixes, camera status, LiDAR clouds) is EXPECTED only when the
  chip says that hardware is installed. Missing after `grace_s` from attaching,
  or silent for longer than its timeout -> DEGRADED, naming what is missing.
- ⚠️ THE GPS SOURCE CHANGES ONLY WHILE THE ROBOT IS IDLE (no mission running,
  wheels still). The NEO-6M and the F9P disagree by metres; switching under a
  moving robot would jump its map pose mid-row. The switch is deferred, not
  dropped: it happens at the first idle moment. A kit unplugged mid-mission
  leaves the selector on a source that has gone quiet until the robot is
  stopped — mission_runner pauses the mission for exactly that reason.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from robot_kit.manifest import Manifest

NONE, READY, DEGRADED = "NONE", "READY", "DEGRADED"
NEO6M, KIT = "neo6m", "kit"

# stream -> how long it may be silent once it has started, s
STREAM_TIMEOUT_S = {"gnss": 2.0, "camera": 5.0, "lidar": 1.0}


def expected_streams(manifest: Optional[Manifest]) -> Tuple[str, ...]:
    if manifest is None:
        return ()
    streams = []
    if manifest.has("gnss_rtk") and manifest.gnss_link in ("usb", "ros"):
        streams.append("gnss")
    # Without SDK control the robot has no line to the camera at all: it records
    # on its own (started by hand), and its silence is not a fault.
    if manifest.has("cam360_ctrl"):
        streams.append("camera")
    if manifest.has("lidar"):
        streams.append("lidar")
    return tuple(streams)


@dataclass
class Status:
    state: str
    present: bool                       # the debounced plug
    manifest: Optional[Manifest]
    error: str                          # why a present kit is not in use ("" if none)
    missing: Tuple[str, ...]            # expected streams that are silent
    gps_source: str
    gps_wanted: str                     # where it goes at the next idle moment
    read_chip: bool                     # the node should read the chip now
    events: List[str] = field(default_factory=list)


class KitMonitor:
    def __init__(self, debounce_s: float = 0.5, grace_s: float = 15.0,
                 chip_retry_s: float = 2.0,
                 timeouts: Optional[Dict[str, float]] = None) -> None:
        self.debounce_s = debounce_s
        self.grace_s = grace_s
        self.chip_retry_s = chip_retry_s
        self.timeouts = dict(STREAM_TIMEOUT_S, **(timeouts or {}))
        self._raw: Optional[bool] = None
        self._raw_since = 0.0
        self._present = False
        self._attached_at: Optional[float] = None
        self._manifest: Optional[Manifest] = None
        self._error = ""
        self._chip_due: Optional[float] = None
        self._seen: Dict[str, float] = {}
        self._source = NEO6M
        self._state = NONE
        self._missing: Tuple[str, ...] = ()
        self._events: List[str] = []

    # --- inputs ----------------------------------------------------------------

    def presence(self, present: bool, t: float) -> None:
        if present != self._raw:
            self._raw = present
            self._raw_since = t

    def chip(self, manifest: Optional[Manifest], error: str, t: float) -> None:
        """The result of a chip read the monitor asked for (Status.read_chip)."""
        if not self._present:
            return
        if manifest is not None:
            self._manifest, self._error, self._chip_due = manifest, "", None
            self._events.append(f"attached {manifest.kind} {manifest.serial}")
        else:
            if error != self._error:
                self._events.append(f"chip: {error}")
            self._error = error
            self._chip_due = t + self.chip_retry_s

    def seen(self, stream: str, t: float) -> None:
        self._seen[stream] = t

    # --- the step --------------------------------------------------------------

    def tick(self, t: float, idle: bool) -> Status:
        self._debounce(t)
        expected = expected_streams(self._manifest)
        if not self._present or self._manifest is None:
            state, missing = NONE, ()
        else:
            missing = tuple(s for s in expected if self._silent(s, t))
            state = DEGRADED if missing else READY
        if state != self._state or missing != self._missing:
            if state == DEGRADED:
                self._events.append(f"degraded: {', '.join(missing)} silent")
            elif state == READY and self._state != NONE:
                self._events.append("ready")
            self._state, self._missing = state, missing

        # The kit's fixes must be ARRIVING, not merely not-yet-overdue: inside
        # the grace period a silent receiver is not "missing", and switching to
        # it would hand navsat_transform a source that says nothing.
        wanted = KIT if (state != NONE and "gnss" in expected and self._fresh("gnss", t)) else NEO6M
        if wanted != self._source and idle:
            self._events.append(f"gps source {self._source} -> {wanted}")
            self._source = wanted

        read_chip = self._chip_due is not None and t >= self._chip_due
        events, self._events = self._events, []
        return Status(state=state, present=self._present, manifest=self._manifest,
                      error=self._error if self._present else "", missing=missing,
                      gps_source=self._source, gps_wanted=wanted, read_chip=read_chip,
                      events=events)

    def _debounce(self, t: float) -> None:
        if self._raw is None or self._raw == self._present:
            return
        if t - self._raw_since < self.debounce_s:
            return
        self._present = self._raw
        if self._present:
            self._attached_at = t
            self._chip_due = t
            self._seen.clear()
        else:
            if self._manifest is not None:
                self._events.append(f"detached {self._manifest.serial}")
            else:
                self._events.append("detached")
            self._manifest, self._error, self._chip_due = None, "", None
            self._attached_at = None

    def _fresh(self, stream: str, t: float) -> bool:
        last = self._seen.get(stream)
        return last is not None and t - last <= self.timeouts.get(stream, 2.0)

    def _silent(self, stream: str, t: float) -> bool:
        last = self._seen.get(stream)
        if last is None:
            return t - self._attached_at > self.grace_s
        return t - last > self.timeouts.get(stream, 2.0)
