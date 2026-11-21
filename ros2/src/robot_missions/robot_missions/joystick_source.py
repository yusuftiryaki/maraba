"""Event source for the Bluetooth remote: evdev reader + BlueZ battery.

ROS-free. `evdev` is imported by EvdevSource.start() (not at module import)
so tests and the node's fake mode work on machines without python3-evdev,
while the real node still fails at startup, not later in a thread.
"""

from __future__ import annotations

import json
import math
import re
import select
import subprocess
import threading
import time
from typing import Callable, Optional, Sequence, Tuple

from robot_missions.joystick_gate import JoyEvent

EV_KEY, EV_REL, EV_ABS = 1, 2, 3            # linux/input-event-codes.h
_REL_X, _REL_Y = 0, 1
_LOG_EVERY_S = 30.0


def to_event(ev_type: int, code: int, value: int,
             key_name: Callable[[int], str]) -> Optional[JoyEvent]:
    """Map a raw input event to a JoyEvent; None for SYN/MSC and friends."""
    if ev_type == EV_REL:
        if code == _REL_X:
            return JoyEvent("rel", "x", value)
        if code == _REL_Y:
            return JoyEvent("rel", "y", value)
        return JoyEvent("other", "", 0)
    if ev_type == EV_KEY:
        return JoyEvent("key", key_name(code), value)
    if ev_type == EV_ABS:
        return JoyEvent("other", "", 0)
    return None


_KINDS = ("rel", "key", "other", "connected", "battery")


def parse_fake_event(text: str) -> JoyEvent:
    """Parse a fake_events message; raises ValueError/TypeError/KeyError if bad."""
    d = json.loads(text)
    kind, code, value = d["kind"], d["code"], d["value"]
    if kind not in _KINDS or not isinstance(code, str):
        raise ValueError("bad kind/code")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("value must be a number")
    value = float(value)
    if math.isinf(value) or (math.isnan(value) and kind != "battery"):
        raise ValueError("value must be finite")
    return JoyEvent(kind, code, value)


def pick_device(candidates: Sequence[Tuple[str, str, str, bool]],
                name: str, mac: str) -> Optional[str]:
    """candidates: (path, name, uniq, has_rel). First path that matches wins."""
    for path, dev_name, uniq, has_rel in candidates:
        if dev_name != name or not has_rel:
            continue
        if mac and (uniq or "").lower() != mac.lower():
            continue
        return path
    return None


def evdev_key_name(code: int, ecodes, prefer: Sequence[str] = ()) -> str:
    """evdev name of a key code. python-evdev gives some codes a LIST of
    aliases (304 -> ['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH']); a configured button
    name matches ANY of them, so the configured alias is the one returned."""
    n = ecodes.KEY.get(code) or ecodes.BTN.get(code) or f"KEY_{code}"
    names = list(n) if isinstance(n, (list, tuple)) else [n]
    for name in names:
        if name in prefer:
            return name
    return names[0]


def bluez_battery(mac: str, adapter: str = "hci0",
                  run: Callable = subprocess.run) -> Optional[float]:
    """Battery percentage from BlueZ over busctl, or None. Never raises."""
    try:
        obj = f"/org/bluez/{adapter}/dev_{mac.upper().replace(':', '_')}"
        res = run(["busctl", "get-property", "org.bluez", obj,
                   "org.bluez.Battery1", "Percentage"],
                  capture_output=True, text=True, timeout=2.0)
        if res.returncode != 0:
            return None
        m = re.fullmatch(r"\s*y\s+(\d+)\s*", res.stdout)
        return float(m.group(1)) if m else None
    except Exception:
        return None


class EvdevSource:
    """Reader thread: find the remote, grab it, emit events, rescan on loss."""

    def __init__(self, name: str, mac: str, emit: Callable[[JoyEvent], None],
                 retry_s: float = 1.0, log: Callable[[str], None] = lambda s: None,
                 buttons: Sequence[str] = ()):
        self._name, self._want_mac = name, mac
        self._emit = emit
        self._retry_s = retry_s
        self._log = log
        self._buttons = tuple(b for b in buttons if b)  # configured names (aliases)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._evdev = self._ecodes = None
        # The connected remote's MAC (evdev uniq); None while disconnected,
        # cleared BEFORE "connected 0" is emitted so nothing polls a stale one.
        self.mac: Optional[str] = None

    def start(self) -> None:
        """Raises ImportError right here when python3-evdev is missing."""
        import evdev                                # only on the Pi
        from evdev import ecodes
        self._evdev, self._ecodes = evdev, ecodes
        self._thread = threading.Thread(target=self._run, name="evdev", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        evdev, ecodes = self._evdev, self._ecodes
        last_log = {}                               # message key -> last log time

        def log_limited(key: str, msg: str) -> None:
            now = time.monotonic()
            if now - last_log.get(key, -_LOG_EVERY_S) >= _LOG_EVERY_S:
                last_log[key] = now
                self._log(msg)

        while not self._stop.is_set():
            dev = None
            up = False                              # connected 1 was emitted
            try:
                devs = [evdev.InputDevice(p) for p in evdev.list_devices()]
                cands = [(d.path, d.name, d.uniq or "",
                          ecodes.EV_REL in d.capabilities()) for d in devs]
                path = pick_device(cands, self._name, self._want_mac)
                for d in devs:
                    if d.path == path:
                        dev = d
                    else:
                        d.close()
                if dev is None:
                    log_limited("missing", f"remote '{self._name}' not found, retrying")
                else:
                    dev.grab()
                    self.mac = dev.uniq or None
                    self._emit(JoyEvent("connected", "", 1))
                    up = True
                    self._read(dev, ecodes)
            except OSError as e:
                where = getattr(dev, "path", "?")
                if up:
                    self._log(f"remote lost on {where}: {e!r}")
                else:
                    log_limited("oserror", f"cannot open remote on {where}: {e!r}")
            except Exception as e:                  # keep the reader alive
                log_limited("error", f"evdev reader error: {e!r}")
            finally:
                self.mac = None
                if up:
                    self._emit(JoyEvent("connected", "", 0))
                if dev is not None:
                    try:
                        dev.close()
                    except Exception:
                        pass
            self._stop.wait(self._retry_s)

    def _read(self, dev, ecodes) -> None:
        def key_name(code: int) -> str:
            return evdev_key_name(code, ecodes, self._buttons)

        while not self._stop.is_set():
            ready = dev.read_one()                  # non-blocking
            if ready is None:
                select.select([dev.fd], [], [], 0.2)
                continue
            try:
                ev = to_event(ready.type, ready.code, ready.value, key_name)
                if ev is not None:
                    self._emit(ev)
            except Exception as e:                  # a bad event must not kill the reader
                self._log(f"event dropped: {e!r}")


class BatteryPoller:
    """Reads the remote's battery on its own thread (busctl can take 2 s,
    which must not stall the node's 20 Hz tick).

    Polls right after every connect, then every period_s. Without the
    connect poll, a remote switched off and on (BlueZ drops Battery1 while it
    is away) could be armed on an unknown battery for up to period_s even
    though the last reading was low.

    Only polls while get_mac() returns a MAC; the source clears it on a
    disconnect, so a known-low reading is kept rather than overwritten by
    "unknown". A read whose MAC changed while busctl ran is dropped. A failed
    read right after a connect is retried every retry_s, up to `retries`
    times, before it is reported as unknown (NaN). A failed PERIODIC read is
    only logged: the gate keeps its last reading. BlueZ drops Battery1 as the
    link goes down, possibly before the evdev node (and so the MAC) is gone —
    reporting NaN there would turn a known-low battery into "unknown,
    allowed". Never raises.
    """

    def __init__(self, get_mac: Callable[[], Optional[str]],
                 emit: Callable[[JoyEvent], None], period_s: float = 30.0,
                 read: Callable[[str], Optional[float]] = bluez_battery,
                 retry_s: float = 1.0, retries: int = 3,
                 log: Callable[[str], None] = lambda s: None):
        self._get_mac, self._emit, self._read = get_mac, emit, read
        self._period_s, self._retry_s = float(period_s), float(retry_s)
        self._retries = max(0, int(retries))
        self._log = log
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="joy-battery",
                                        daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def kick(self) -> None:
        """Poll now (and retry a failure), e.g. right after a connect."""
        self._wake.set()

    def on_event(self, event: JoyEvent) -> None:
        if event.kind == "connected" and event.value >= 1:
            self.kick()

    def _run(self) -> None:
        attempts = 0                        # > 0: attempts left after a kick
        while True:
            kicked = self._wake.wait(self._retry_s if attempts else self._period_s)
            if kicked:
                # Only clear what we saw: clearing after a timeout would eat a
                # kick() that landed between wait() returning and the clear.
                self._wake.clear()
            if self._stop.is_set():
                return
            if kicked:
                attempts = self._retries + 1
            after_connect = attempts > 0
            final = attempts <= 1
            try:
                ok = self._poll_once(after_connect, final)
            except Exception as e:          # keep the poller alive
                self._log(f"battery poll failed: {e!r}")
                ok = False
            attempts = 0 if ok or final else attempts - 1

    def _poll_once(self, after_connect: bool, final: bool) -> bool:
        """True when done (read, or nothing to read); False to retry."""
        mac = self._get_mac()
        if not mac:
            return True
        pct = self._read(mac)
        if self._get_mac() != mac:
            return True                     # disconnected / replaced meanwhile
        if pct is None:
            if not after_connect:
                self._log("periodic battery read failed; keeping the last reading")
                return True
            if not final:
                return False
            self._emit(JoyEvent("battery", "", math.nan))
            return False
        self._emit(JoyEvent("battery", "", float(pct)))
        return True
