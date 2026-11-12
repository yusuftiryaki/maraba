"""python3 -m b1_flash — normalde ./scripts/b1_flash.sh ile çağrılır."""
from __future__ import annotations

import argparse
import glob
import shutil
import sys
from pathlib import Path

from . import wizard
from .fakes import PROVA_RULES, FakeBoard, FakeClock, FakeRunner
from .runner import RealRunner
from .ui import ConsoleUI, ScriptedUI
from .wheel import RealClock, SerialPort

REPO = Path(__file__).resolve().parents[3]
STATE_DIR = REPO / '.b1_flash'


class _EchoUI(ScriptedUI):
    """Scripted answers, but printed — so `--prova --otomatik` reads like a run."""

    def __init__(self, rules):
        super().__init__(rules)
        self._console = ConsoleUI()

    def __getattribute__(self, name):
        if name in ('header', 'say', 'ok', 'warn', 'err', 'cmd'):
            console = object.__getattribute__(self, '_console')
            return getattr(console, name)
        return object.__getattribute__(self, name)

    def confirm(self, q):
        a = super().confirm(q)
        self._console.say(f'  ? {q} → {"e" if a else "h"}')
        return a

    def ask(self, q):
        a = super().ask(q)
        self._console.say(f'  ? {q} → {a}')
        return a

    def choose(self, q, options):
        a = super().choose(q, options)
        self._console.say(f'  ? {q} → {a} ({options[a]})')
        return a


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog='b1_flash',
        description='Hoverboard anakartını (STM32F103RCT6, EFeru FOC) adım adım '
                    'flash\'lama sihirbazı. Tek kaynak: docs/bringup-checklist.md')
    ap.add_argument('--fw-dir', type=Path,
                    default=Path.home() / 'hoverboard-firmware-hack-FOC',
                    help='EFeru kaynağının yeri (yoksa klonlanır)')
    ap.add_argument('--adim', choices=[s.id for s in wizard.STEPS],
                    help='bu adımdan başla')
    ap.add_argument('--liste', action='store_true', help='adımları listele ve çık')
    ap.add_argument('--sifirla', action='store_true',
                    help='kayıtlı ilerlemeyi sil (raporlar kalır)')
    ap.add_argument('--prova', action='store_true',
                    help='sahte donanımla prova: hiçbir komut gerçekten koşmaz')
    ap.add_argument('--otomatik', action='store_true',
                    help='--prova ile: soruları hazır cevaplarla geç')
    a = ap.parse_args(argv)

    if a.liste:
        for i, s in enumerate(wizard.STEPS, 1):
            print(f'{i:2}. {s.id:<11} {s.title}' + (f'  ({s.ref})' if s.ref else ''))
        return 0
    if a.otomatik and not a.prova:
        ap.error('--otomatik yalnız --prova ile')

    if a.prova:
        state_dir = STATE_DIR / 'prova'
        shutil.rmtree(state_dir, ignore_errors=True)
        clock = FakeClock()
        board = FakeBoard(clock)
        ctx = wizard.Ctx(
            ui=_EchoUI(PROVA_RULES + [('Ne yapalım', 'c')]) if a.otomatik else ConsoleUI(),
            runner=FakeRunner(), clock=clock,
            state=wizard.State(state_dir / 'state.json'),
            fw_dir=state_dir / 'hoverboard-firmware-hack-FOC',
            open_serial=lambda _p: board, list_ports=lambda: ['/dev/ttyPROVA0'],
            report_dir=state_dir, prova=True)
        print('*** PROVA — sahte ST-Link, sahte kart, sahte zaman. Donanıma dokunulmaz. ***')
    else:
        if a.sifirla:
            (STATE_DIR / 'state.json').unlink(missing_ok=True)
        ctx = wizard.Ctx(
            ui=ConsoleUI(), runner=RealRunner(), clock=RealClock(),
            state=wizard.State(STATE_DIR / 'state.json'),
            fw_dir=a.fw_dir.expanduser().resolve(),
            open_serial=SerialPort,
            list_ports=lambda: sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')),
            report_dir=STATE_DIR)
    out = wizard.run(ctx, a.adim)
    return out.code


if __name__ == '__main__':
    sys.exit(main())
