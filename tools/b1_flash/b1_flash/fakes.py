"""Fake bench: ST-Link tools, an EFeru board on a serial line, simulated time.

Same idea as hoverboard_bridge/fake_esp32.py and mpu6050_driver/fake_bus.py:
the wizard's logic runs end to end with no hardware, in tests and in
`b1_flash --prova` (a rehearsal before the real afternoon at the bench).

The fake board follows EFeru's behaviour where the wizard depends on it
(Src/util.c, Src/main.c): feedback every 10 ms whether or not commands come;
a valid command resets the serial timeout; after SERIAL_TIMEOUT the firmware
goes to OPEN_MODE with the commands zeroed (feedback cmd1/cmd2 = 0) and the
wheels coast.
"""
from __future__ import annotations

import math
from pathlib import Path

from .frames import CMD_SIZE, START_BYTES, Feedback, decode_command, encode_feedback
from .runner import CmdResult


class FakeClock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


class FakeBoard:
    """An EFeru VARIANT_USART board as seen from a USB-TTL adapter."""

    PERIOD = 0.01

    def __init__(self, clock: FakeClock, timeout_s: float | None = 0.8,
                 rx_connected: bool = True, tx_connected: bool = True,
                 odometry: bool = False, wheels_ok: tuple[bool, bool] = (True, True),
                 tau: float = 0.15, coast_tau: float = 1.5):
        self.clock = clock
        self.timeout_s = timeout_s          # None = a firmware with no timeout
        self.rx_connected = rx_connected    # adapter TX reaches STM32 RX
        self.tx_connected = tx_connected    # STM32 TX reaches adapter RX
        self.odometry = odometry
        self.wheels_ok = wheels_ok          # (left, right)
        self.tau, self.coast_tau = tau, coast_tau
        self._in = bytearray()
        self._out = bytearray()
        self._steer = self._speed = 0
        self._last_cmd = -math.inf
        self._vl = self._vr = 0.0
        self._t = clock.now()
        self.commands = 0
        self.closed = False

    # --- Port interface
    def write(self, data: bytes) -> int:
        self._advance()
        if self.rx_connected:
            self._in.extend(data)
            self._parse_commands()
        return len(data)

    def read_available(self) -> bytes:
        self._advance()
        out = bytes(self._out)
        self._out.clear()
        return out if self.tx_connected else b''

    def close(self) -> None:
        self.closed = True

    # --- simulation
    def _parse_commands(self):
        while True:
            i = self._in.find(START_BYTES)
            if i < 0:
                self._in.clear()
                return
            del self._in[:i]
            if len(self._in) < CMD_SIZE:
                return
            cmd = decode_command(bytes(self._in[:CMD_SIZE]))
            if cmd is None:
                del self._in[:1]
                continue
            del self._in[:CMD_SIZE]
            self._steer, self._speed = cmd
            self._last_cmd = self.clock.now()
            self.commands += 1

    def _advance(self):
        while self._t + self.PERIOD <= self.clock.now():
            self._t += self.PERIOD
            timed_out = (self.timeout_s is not None
                         and self._t - self._last_cmd > self.timeout_s)
            steer, speed = (0, 0) if timed_out else (self._steer, self._speed)
            if timed_out:                       # OPEN_MODE: no torque, coast
                k = math.exp(-self.PERIOD / self.coast_tau)
                self._vl *= k
                self._vr *= k
            else:
                a = self.PERIOD / self.tau
                self._vl += (speed - self._vl) * a if self.wheels_ok[0] else -self._vl
                self._vr += (speed - self._vr) * a if self.wheels_ok[1] else -self._vr
            fb = Feedback(steer, speed, round(self._vr), round(self._vl),
                          3920, 312, 0)
            self._out.extend(encode_feedback(fb, odometry=self.odometry))


# --- a minimal EFeru tree ------------------------------------------------------
# Same shape as upstream where the checker cares: the PLATFORMIO selection
# block, the TRANSPOTTER #else, variant blocks that #undef global settings,
# nested DUAL_INPUTS, a backslash-continued #if, #ifndef-guarded baud rates.

FAKE_CONFIG_H = r"""// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### VARIANT SELECTION ###############################
// PlatformIO: uncomment desired variant in platformio.ini
#if !defined(PLATFORMIO)
  //#define VARIANT_ADC         // Variant for control via ADC input
  //#define VARIANT_USART       // Variant for Serial control via USART3 input
  //#define VARIANT_HOVERCAR    // Variant for HOVERCAR build
#endif
// ########################### END OF VARIANT SELECTION ############################

// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000     // PWM frequency in Hz / is also used for buzzer
#ifdef VARIANT_TRANSPOTTER
  #define DC_CUR_LIMIT     30         // DC current limit in amps
#else
  #define DC_CUR_LIMIT     15         // DC current limit in amps
#endif
#define TIMEOUT                20     // number of wrong / missing input commands before emergency off
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################

// ############################### BATTERY ###############################
#define BAT_CALIB_REAL_VOLTAGE  3970      // input voltage measured by multimeter (multiplied by 100)
#define BAT_CALIB_ADC           1492      // adc-value measured by mainboard
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s
// ######################## END OF BATTERY ###############################

// ############################### ODOMETRY ######################################
// #define ENABLE_ODOMETRY                   // [-] Enable hall sensor tick counters (adds wheelR_cnt / wheelL_cnt to serial feedback)
// ########################### END OF ODOMETRY ###################################

// ############################### MOTOR CONTROL #########################
#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

// Control selections
#define CTRL_TYP_SEL    FOC_CTRL        // [-] Control type selection: COM_CTRL, SIN_CTRL, FOC_CTRL (default)
#define CTRL_MOD_REQ    VLT_MODE        // [-] Control mode request: OPEN_MODE, VLT_MODE (default), SPD_MODE, TRQ_MODE.
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum single motor current limit
#define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit (I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit
// ########################### END OF MOTOR CONTROL ########################

// ############################ VARIANT_USART SETTINGS ############################
#ifdef VARIANT_USART
  // #define SIDEBOARD_SERIAL_USART2 0
  #define CONTROL_SERIAL_USART2  0    // left sensor board cable, disable if ADC or PPM is used!
  #define FEEDBACK_SERIAL_USART2      // left sensor board cable, disable if ADC or PPM is used!

  // #define CONTROL_SERIAL_USART3  0    // right sensor board cable.
  // #define FEEDBACK_SERIAL_USART3      // right sensor board cable

  // #define DUAL_INPUTS                 //  UART*(Primary) + SIDEBOARD(Auxiliary).
  #define PRI_INPUT1             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND.
  #define PRI_INPUT2             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND.
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY      0x1102  // Flash memory writing key.
    #define SIDEBOARD_SERIAL_USART3 1   // right sideboard
  #else
    #define FLASH_WRITE_KEY      0x1002  // Flash memory writing key.
  #endif

  // #define TANK_STEERING              // use for tank steering, each input controls each wheel
#endif
// ######################## END OF VARIANT_USART SETTINGS #########################

// ############################ VARIANT_HOVERCAR SETTINGS ############################
#ifdef VARIANT_HOVERCAR
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ            TRQ_MODE  // HOVERCAR works best in TORQUE Mode.
  #define CONTROL_ADC           0         // use ADC as input.
  #define TANK_STEERING
#endif
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################

// ########################### UART SETIINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD                  // [-] Start frame definition for serial commands
  #define SERIAL_BUFFER_SIZE      64                      // [bytes] Size of Serial Rx buffer. Make sure it is always larger than the structure size
  #define SERIAL_TIMEOUT          160                     // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec.
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           115200                  // UART2 baud rate (long wired cable)
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
// ########################### UART SETIINGS ############################

#endif // CONFIG_H
"""

FAKE_PLATFORMIO_INI = """\
[platformio]
include_dir = Inc
src_dir = Src

;default_envs = VARIANT_ADC         ; Variant for control via ADC input
;default_envs = VARIANT_USART       ; Variant for Serial control via USART3 input

[env:VARIANT_ADC]
platform = ststm32
framework = stm32cube
board = genericSTM32F103RC
debug_tool = stlink
upload_protocol = stlink
build_flags =
    -DUSE_HAL_DRIVER
    -DSTM32F103xE
    -T./STM32F103RCTx_FLASH.ld
    -D VARIANT_ADC

[env:VARIANT_USART]
platform = ststm32
framework = stm32cube
board = genericSTM32F103RC
debug_tool = stlink
upload_protocol = stlink
build_flags =
    -DUSE_HAL_DRIVER
    -DSTM32F103xE
    -T./STM32F103RCTx_FLASH.ld
    -lc
    -lm
    -g -ggdb        ; to generate correctly the 'firmware.elf' for STM STUDIO vizualization
    -D VARIANT_USART
"""


def write_fake_tree(path: Path, config_h: str = FAKE_CONFIG_H,
                    platformio_ini: str = FAKE_PLATFORMIO_INI):
    (path / 'Inc').mkdir(parents=True, exist_ok=True)
    (path / 'Inc' / 'config.h').write_text(config_h)
    (path / 'platformio.ini').write_text(platformio_ini)


class FakeRunner:
    """Answers the wizard's commands the way a genuine RCT6 bench would."""

    COMMIT = '0123456789abcdef0123456789abcdef01234567'

    def __init__(self, cpuid: int = 0x411FC231, chipid: int = 0x414,
                 locked: bool = True, missing: tuple[str, ...] = (),
                 stlink: bool = True, target: bool = True,
                 unlock_ok: bool = True, upload_ok: bool = True,
                 unexpected_idcode: bool = False):
        self.cpuid, self.chipid = cpuid, chipid
        self.locked, self.erased = locked, False
        self.missing, self.stlink, self.target = missing, stlink, target
        self.unlock_ok, self.upload_ok = unlock_ok, upload_ok
        self.unexpected_idcode = unexpected_idcode
        self.flashed = False
        self.calls: list[list[str]] = []

    def which(self, tool: str):
        return None if tool in self.missing else f'/usr/bin/{tool}'

    def ran(self, needle: str) -> bool:
        return any(needle in ' '.join(c) for c in self.calls)

    def run(self, cmd, cwd=None, timeout=60, stream=False) -> CmdResult:
        self.calls.append(list(cmd))
        line = ' '.join(cmd)
        tool = cmd[0]
        if tool in self.missing:
            return CmdResult(127, f'{tool}: komut bulunamadı')
        if tool == 'lsusb':
            out = 'Bus 001 Device 002: ID 8087:0024 Intel Corp. Hub\n'
            if self.stlink:
                out += 'Bus 001 Device 005: ID 0483:3748 STMicroelectronics ST-LINK/V2\n'
            out += 'Bus 001 Device 006: ID 0403:6001 Future Technology Devices FT232 Serial (UART) IC\n'
            return CmdResult(0, out)
        if tool == 'st-info':
            if not self.stlink:
                return CmdResult(0, 'Found 0 stlink programmers\n')
            out = ('Found 1 stlink programmers\n  version:    V2J37S7\n'
                   '  serial:     303030303030303030303031\n')
            if self.target:
                out += ('  flash:      262144 (pagesize: 2048)\n  sram:       65536\n'
                        f'  chipid:     0x{self.chipid:03x}\n'
                        '  dev-type:   STM32F1xx_HD\n')
            else:
                out += '  flash:      0 (pagesize: 0)\n  sram:       0\n  chipid:     0x000\n'
            return CmdResult(0, out)
        if tool == 'openocd':
            pre = 'Open On-Chip Debugger 0.12.0\nInfo : STLINK V2J37S7 (API v2) VID:PID 0483:3748\n'
            if self.unexpected_idcode:
                pre += 'Warn : UNEXPECTED idcode: 0x2ba01477\nError: expected 1 of 1: 0x1ba01477\n'
                return CmdResult(1, pre)
            if not self.target:
                return CmdResult(1, pre + 'Error: init mode failed (unable to connect to the target)\n')
            if 'mdw 0xE000ED00' in line:
                return CmdResult(0, pre + f'0xe000ed00: {self.cpuid:08x} \n')
            if 'stm32f1x unlock 0' in line:
                if not self.unlock_ok:
                    return CmdResult(1, pre + 'Error: stm32x device protected\n')
                self.locked, self.erased = False, True
                return CmdResult(0, pre + 'stm32x unlocked.\nINFO: a reset or power '
                                 'cycle is required for the new settings to take effect.\n')
            if 'mdw 0x08000000' in line:
                if self.locked:
                    return CmdResult(1, pre + 'Error: Failed to read memory at 0x08000000\n')
                if self.erased:
                    return CmdResult(0, pre + '0x08000000: ffffffff ffffffff ffffffff ffffffff \n')
                return CmdResult(0, pre + '0x08000000: 20005000 08000d2d 08000c91 08000c95 \n')
            return CmdResult(1, pre + 'Error: fake: bilinmeyen komut\n')
        if tool == 'git' and 'clone' in cmd:
            write_fake_tree(Path(cmd[-1]))
            return CmdResult(0, f"Cloning into '{cmd[-1]}'...\n")
        if tool == 'git' and 'rev-parse' in cmd:
            return CmdResult(0, self.COMMIT + '\n')
        if tool == 'git' and 'status' in cmd:
            return CmdResult(0, '')
        if tool == 'pio' and '-t' in cmd and 'upload' in cmd:
            if not self.upload_ok:
                return CmdResult(1, 'Error: init mode failed (unable to connect to the target)\n')
            self.flashed, self.erased = True, False
            return CmdResult(0, '** Programming Finished **\n** Verified OK **\n'
                             '** Resetting Target **\n=== [SUCCESS] Took 6.12 seconds ===\n')
        if tool == 'pio':
            b = Path(cwd) / '.pio' / 'build' / 'VARIANT_USART'
            b.mkdir(parents=True, exist_ok=True)
            (b / 'firmware.bin').write_bytes(b'\0' * 48_000)
            return CmdResult(0, 'Building in release mode\n=== [SUCCESS] Took 41.0 seconds ===\n')
        return CmdResult(127, f'fake: bilinmeyen komut {line}')


# Answers that walk the whole rehearsal without typing (`--prova --otomatik`
# and the flow tests): a four-wire cable measured as GND / 15 V / 3.3 V / 3.3 V.
PROVA_RULES: list[tuple[str, object]] = [
    ('kaç tel', '4'),
    ('Tel 1 rengi', 'siyah'), ('Tel 2 rengi', 'kırmızı'),
    ('Tel 3 rengi', 'yeşil'), ('Tel 4 rengi', 'mavi'),
    ('siyah: kartın GND', True), ('kırmızı: kartın GND', False),
    ('yeşil: kartın GND', False), ('mavi: kartın GND', False),
    ('siyah: GND\'ye karşı', 0.0), ('kırmızı: GND\'ye karşı', 15.2),
    ('yeşil: GND\'ye karşı', 3.3), ('mavi: GND\'ye karşı', 3.2),
    ('NRST teli', False),
    ('Sol tekerlek', 'i'), ('Sağ tekerlek', 'i'),
]
