"""Pure parsers for tool output and bench measurements.

Every decision the wizard makes about hardware goes through one of these, so
each can be tested against captured output without a board on the desk.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

# ST-Link USB ids: V2 (the purple clone), V2-1 (Nucleo), V3.
STLINK_USB_IDS = {'0483:3748': 'ST-Link V2', '0483:374b': 'ST-Link V2-1',
                  '0483:3752': 'ST-Link V2-1', '0483:374f': 'ST-Link V3'}
FTDI_USB_ID = '0403:6001'

# STM32F10x errata (ES0340 & co.) name the core as Cortex-M3 r1p1, CPUID
# 0x411FC231. GD32 / other clones are known to report later revisions. Anything
# else is a stop, not a guess.
STM32F1_CPUID = 0x411FC231
F1_HIGH_DENSITY_CHIPID = 0x414
RCT6_FLASH_BYTES = 256 * 1024


def find_usb(lsusb_out: str, ids: dict[str, str]) -> list[str]:
    found = []
    for line in lsusb_out.splitlines():
        m = re.search(r'ID\s+([0-9a-fA-F]{4}:[0-9a-fA-F]{4})', line)
        if m and m.group(1).lower() in ids:
            found.append(ids[m.group(1).lower()])
    return found


def find_stlink(lsusb_out: str) -> list[str]:
    return find_usb(lsusb_out, STLINK_USB_IDS)


@dataclass
class StInfo:
    programmers: int = 0
    chipid: int | None = None
    flash: int | None = None
    dev_type: str = ''
    libusb_error: bool = False

    @property
    def has_target(self) -> bool:
        return bool(self.chipid)


def parse_st_info(out: str) -> StInfo:
    """`st-info --probe`, both the 1.7 (`descr:`) and 1.8 (`dev-type:`) formats."""
    info = StInfo()
    m = re.search(r'Found\s+(\d+)\s+stlink programmers', out)
    if m:
        info.programmers = int(m.group(1))
    m = re.search(r'chipid:\s*(0x[0-9a-fA-F]+)', out)
    if m:
        info.chipid = int(m.group(1), 16)
    m = re.search(r'flash:\s*(\d+)', out)
    if m:
        info.flash = int(m.group(1))
    m = re.search(r'(?:dev-type|descr):\s*(.+)', out)
    if m:
        info.dev_type = m.group(1).strip()
    info.libusb_error = bool(re.search(r'libusb.*error|LIBUSB_ERROR|error: -4',
                                       out, re.IGNORECASE))
    return info


def parse_mdw(out: str, addr: int) -> list[int] | None:
    """Words OpenOCD printed for `mdw <addr> [n]`, or None if it did not."""
    m = re.search(r'0x%08x:\s*((?:[0-9a-fA-F]{8}\s*)+)' % addr, out, re.IGNORECASE)
    if not m:
        return None
    return [int(w, 16) for w in m.group(1).split()]


def openocd_unexpected_idcode(out: str) -> bool:
    return bool(re.search(r'UNEXPECTED.*idcode', out, re.IGNORECASE))


def openocd_unlocked(out: str) -> bool:
    # stm32f1x.c: "stm32x unlocked.\nINFO: a reset or power cycle is required..."
    return bool(re.search(r'stm32x unlocked', out, re.IGNORECASE))


def classify_flash_words(words: list[int] | None) -> str:
    """After unlock the whole flash is erased: every word 0xFFFFFFFF.

    'bos'      erased and readable — RDP is off
    'kilitli'  no read / all zeros — what a protected part looks like via SWD
    'dolu'     readable data — RDP off and something already flashed
    """
    if not words:
        return 'kilitli'
    if all(w == 0xFFFFFFFF for w in words):
        return 'bos'
    if all(w == 0 for w in words):
        return 'kilitli'
    return 'dolu'


# --- left sensor-board cable ------------------------------------------------

@dataclass
class Wire:
    color: str
    continuity: bool      # beeps against board GND
    volts: float          # DC against board GND, battery connected


@dataclass
class WireMap:
    gnd: Wire | None = None
    supply: Wire | None = None
    signals: list[Wire] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems


def classify_wires(wires: list[Wire]) -> WireMap:
    """Exactly one GND, one ~15 V supply, two ~3.3 V signals — or a stop.

    Bands are wide on purpose (a busy TX line reads a little under 3.3 V on a
    multimeter) but there is no band between 3.6 V and 12 V: a 5 V line here
    is not something the docs predict and nothing gets connected to it.
    """
    wm = WireMap()
    gnds, supplies, signals = [], [], []
    for w in wires:
        if w.continuity and abs(w.volts) < 0.3:
            gnds.append(w)
        elif w.continuity:
            wm.problems.append(
                f'{w.color}: GND ile süreklilik var ama {w.volts:.2f} V ölçülmüş '
                '— ölçümlerden biri yanlış, tekrar ölç')
        elif 12.0 <= w.volts <= 17.0:
            supplies.append(w)
        elif 2.5 <= w.volts <= 3.6:
            signals.append(w)
        else:
            wm.problems.append(
                f'{w.color}: {w.volts:.2f} V beklenen hiçbir banda girmiyor '
                '(GND ≈0, sinyal 2.5–3.6, besleme 12–17) — DUR')
    if len(gnds) != 1:
        wm.problems.append(f'{len(gnds)} GND teli bulundu, tam 1 olmalı')
    if len(supplies) != 1:
        wm.problems.append(f'{len(supplies)} adet ~15 V teli bulundu, tam 1 olmalı')
    if len(signals) != 2:
        wm.problems.append(f'{len(signals)} adet ~3.3 V teli bulundu, tam 2 olmalı')
    wm.gnd = gnds[0] if len(gnds) == 1 else None
    wm.supply = supplies[0] if len(supplies) == 1 else None
    wm.signals = signals
    return wm
