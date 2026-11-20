"""Event source for the Bluetooth remote: evdev reader + BlueZ battery.

ROS-free. `evdev` is imported lazily inside EvdevSource so tests and the
node's fake mode work on machines without python3-evdev.
"""

from __future__ import annotations

import re
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
                 retry_s: float = 1.0, log: Callable[[str], None] = lambda s: None):
        self._name, self._want_mac = name, mac
        self._emit = emit
        self._retry_s = retry_s
        self._log = log
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.mac: Optional[str] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="evdev", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        import evdev                                # lazy: only on the Pi
        from evdev import ecodes
        last_log = -_LOG_EVERY_S
        while not self._stop.is_set():
            dev = None
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
                    now = time.monotonic()
                    if now - last_log >= _LOG_EVERY_S:
                        last_log = now
                        self._log(f"remote '{self._name}' not found, retrying")
                    self._stop.wait(self._retry_s)
                    continue
                dev.grab()
                self.mac = dev.uniq or None
                self._emit(JoyEvent("connected", "", 1))
                self._read(dev, ecodes)
            except OSError:
                pass
            except Exception as e:                  # keep the reader alive
                self._log(f"evdev reader error: {e!r}")
            finally:
                if dev is not None:
                    self._emit(JoyEvent("connected", "", 0))
                    try:
                        dev.close()
                    except Exception:
                        pass
            self._stop.wait(self._retry_s)

    def _read(self, dev, ecodes) -> None:
        def key_name(code: int) -> str:
            n = ecodes.KEY.get(code) or ecodes.BTN.get(code) or f"KEY_{code}"
            return n[0] if isinstance(n, (list, tuple)) else n

        while not self._stop.is_set():
            ready = dev.read_one()                  # non-blocking
            if ready is None:
                import select
                select.select([dev.fd], [], [], 0.2)
                continue
            try:
                ev = to_event(ready.type, ready.code, ready.value, key_name)
                if ev is not None:
                    self._emit(ev)
            except Exception as e:                  # a bad event must not kill the reader
                self._log(f"event dropped: {e!r}")
