"""The socket's two sense lines: the presence pin and the identity chip.

ROS-free. Each has a real reader (the Pi) and a file-backed fake (fake_kit, the
tests) with the same interface, so kit_manager_node never knows which it has.

    presence:  real  GpioPresence  — libgpiod v2, pin pulled up, LOW = plug in
               fake  FilePresence  — a file holding "1" (plugged) or "0"
    chip:      real  W1Eeprom      — /sys/bus/w1/devices/2d-*/eeprom (DS2431,
                                     family 0x2d, the kernel's w1_ds2431 driver)
               fake  FileEeprom    — a file holding the 128-byte image

⚠️ NEITHER REAL READER HAS RUN ON HARDWARE. They are written from the libgpiod
2.x API and the w1 sysfs layout; the Pi needs `dtoverlay=w1-gpio,gpiopin=N` and
the presence line's pull-up (docs/kit-soketi.md, deployment notes). What is
verified is the logic above them.
"""

from __future__ import annotations

import glob
import os
from typing import Optional, Tuple


class ChipReadError(OSError):
    """The chip could not be read at all (missing, bus fault) — not a bad image."""


class FilePresence:
    def __init__(self, path: str) -> None:
        self.path = path

    def read(self) -> bool:
        try:
            with open(self.path, encoding="ascii") as handle:
                return handle.read().strip() == "1"
        except OSError:
            return False


class GpioPresence:
    """The presence pin through libgpiod 2.x. The kit ties it to GND in the plug;
    the robot pulls it up, so LOW means plugged (and a broken wire reads as
    'no kit' — safe, because nothing safety-related depends on it: speed limits
    do not change with the kit, docs/kit-soketi.md)."""

    def __init__(self, chip: str = "/dev/gpiochip0", line: int = 17) -> None:
        import gpiod                                   # python3-libgpiod, Pi only
        from gpiod.line import Bias, Direction

        self._request = gpiod.request_lines(
            chip, consumer="kit_manager",
            config={line: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)})
        self._line = line

    def read(self) -> bool:
        from gpiod.line import Value

        return self._request.get_value(self._line) == Value.INACTIVE


class FileEeprom:
    def __init__(self, path: str) -> None:
        self.path = path

    def read(self) -> bytes:
        try:
            with open(self.path, "rb") as handle:
                return handle.read()
        except OSError as exc:
            raise ChipReadError(f"no chip image at {self.path}") from exc


class W1Eeprom:
    FAMILY_GLOB = "/sys/bus/w1/devices/2d-*/eeprom"

    def __init__(self, pattern: str = FAMILY_GLOB) -> None:
        self.pattern = pattern

    def read(self) -> bytes:
        found = sorted(glob.glob(self.pattern))
        if not found:
            raise ChipReadError("no DS2431 on the 1-Wire bus")
        if len(found) > 1:
            raise ChipReadError(f"{len(found)} DS2431 chips on the bus; expected one")
        try:
            with open(found[0], "rb") as handle:
                return handle.read(128)
        except OSError as exc:
            raise ChipReadError(f"reading {found[0]}: {exc}") from exc


def make_readers(fake_dir: str) -> Tuple[object, object]:
    """fake_dir set -> the file fakes in it (what fake_kit writes); else the Pi's."""
    if fake_dir:
        return (FilePresence(os.path.join(fake_dir, "present")),
                FileEeprom(os.path.join(fake_dir, "eeprom")))
    return GpioPresence(), W1Eeprom()


def write_fake(fake_dir: str, present: bool, image: Optional[bytes]) -> None:
    """What fake_kit does to 'plug' a kit in: the chip first, then the pin, so a
    reader never sees the pin before there is a chip behind it."""
    os.makedirs(fake_dir, exist_ok=True)
    eeprom = os.path.join(fake_dir, "eeprom")
    if image is not None:
        tmp = eeprom + ".tmp"
        with open(tmp, "wb") as handle:
            handle.write(image)
        os.replace(tmp, eeprom)
    elif os.path.exists(eeprom):
        os.remove(eeprom)
    tmp = os.path.join(fake_dir, "present.tmp")
    with open(tmp, "w", encoding="ascii") as handle:
        handle.write("1" if present else "0")
    os.replace(tmp, os.path.join(fake_dir, "present"))
