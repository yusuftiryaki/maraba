"""B1: the guided flash of the hoverboard mainboard, step by step.

docs/bringup-checklist.md is the single source; each step here names the
checklist section it carries out. A step either completes, fails (the user
fixes something and retries, or quits and resumes later), or STOPS the wizard
— the last is reserved for things that must not be pushed through: a chip that
is not the STM32 we expect, or a firmware that does not stop when the link goes
quiet.
"""
from __future__ import annotations

import datetime as _dt
import difflib
import json
import shlex
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import config_check, parsers
from .frames import FB_ODOM_SIZE, FB_SIZE
from .wheel import TEST_SPEED, TIMEOUT_LIMIT_S, listen, spin_test, timeout_test

EFERU_URL = 'https://github.com/EFeru/hoverboard-firmware-hack-FOC'
ENV = config_check.ENV

OPENOCD = ['openocd', '-f', 'interface/stlink.cfg', '-f', 'target/stm32f1x.cfg']
NRST_CFG = ['-c', 'reset_config srst_only srst_nogate connect_assert_srst']


class StepFailed(Exception):
    def __init__(self, message: str, hint: str = ''):
        super().__init__(message)
        self.hint = hint


class Abort(Exception):
    """Stop the wizard: continuing would be unsafe or meaningless."""


class State:
    def __init__(self, path: Path):
        self.path = path
        self.data = {'tamamlanan': [], 'kayit': {}, 'gecmis': []}
        if path.exists():
            self.data.update(json.loads(path.read_text()))

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.data, indent=2, ensure_ascii=False))

    def record(self, key: str, value):
        self.data['kayit'][key] = value
        self.save()

    def get(self, key: str, default=None):
        return self.data['kayit'].get(key, default)

    def event(self, step: str, what: str):
        stamp = _dt.datetime.now().isoformat(timespec='seconds')
        self.data['gecmis'].append(f'{stamp} [{step}] {what}')
        self.save()

    def done(self, step: str):
        if step not in self.data['tamamlanan']:
            self.data['tamamlanan'].append(step)
        self.event(step, 'tamamlandı')

    @property
    def completed(self) -> list[str]:
        return self.data['tamamlanan']


@dataclass
class Ctx:
    ui: object
    runner: object
    clock: object
    state: State
    fw_dir: Path
    open_serial: Callable[[str], object]
    list_ports: Callable[[], list[str]]
    report_dir: Path
    prova: bool = False

    @property
    def nrst(self) -> bool:
        return bool(self.state.get('nrst', False))

    def run(self, cmd: list[str], **kw):
        self.ui.cmd(shlex.join(cmd))
        return self.runner.run(cmd, **kw)

    def openocd(self, *commands: str, reset_halt: bool = False, timeout: float = 60):
        cmd = list(OPENOCD)
        if self.nrst:
            cmd += NRST_CFG
        cmd += ['-c', 'init']
        if reset_halt or self.nrst:
            cmd += ['-c', 'reset halt']
        for c in commands:
            cmd += ['-c', c]
        cmd += ['-c', 'shutdown']
        return self.run(cmd, timeout=timeout)


def _checklist(ctx: Ctx, items: list[str]):
    for item in items:
        if not ctx.ui.confirm(item):
            raise StepFailed(f'Sağlanmadı: "{item}" — bu olmadan devam edilmez.')


# ----------------------------------------------------------------------------
# steps
# ----------------------------------------------------------------------------

def s_guvenlik(ctx: Ctx):
    ctx.ui.say('Bu kart 36 V Li-ion paketle motor sürüyor. Aşağıdakilerin hepsi '
               '"evet" olmadan sihirbaz ilerlemez.')
    _checklist(ctx, [
        'Tekerlekler yerden kesik mi (mengene / sehpa)?',
        'Batarya çıkışında 20 A / 60 V sigorta takılı mı?',
        'Fiziksel E-stop (mantar buton) 36 V hattını kesiyor ve elinin altında mı?',
        'Yangın söndürücü ya da bir kova kum yakında mı?',
    ])


def s_araclar(ctx: Ctx):
    missing = [t for t in ('st-info', 'openocd', 'pio', 'git', 'lsusb')
               if not ctx.runner.which(t)]
    if missing:
        raise StepFailed(
            f'Eksik araç: {", ".join(missing)}',
            'Dev container\'da hepsi hazır (docs/devcontainer.md). Değilse: '
            'sudo apt install stlink-tools openocd usbutils git; '
            'pip install platformio')
    ctx.ui.ok('st-info, openocd, pio, git, lsusb bulundu')

    r = ctx.run(['lsusb'])
    sl = parsers.find_stlink(r.output)
    if not sl:
        raise StepFailed(
            'lsusb\'de ST-Link (0483:3748) görünmüyor.',
            'USB kablosunu/portu değiştir. Container\'dasın ve host görüyorsa '
            'runArgs\'taki /dev/bus/usb bind mount\'u kontrol et.')
    ctx.ui.ok(f'Programlayıcı: {sl[0]}')
    if parsers.find_usb(r.output, {parsers.FTDI_USB_ID: 'FT232RL'}):
        ctx.ui.ok('FT232RL USB-TTL de takılı (UART adımları için)')

    r = ctx.run(['st-info', '--probe'], timeout=30)
    info = parsers.parse_st_info(r.output)
    if info.programmers < 1:
        hint = ('lsusb görüyor ama st-info açamıyor: container\'da /dev/bus/usb '
                'yok ya da izin sorunu (docs/devcontainer.md "İzinler"; geçici '
                'çözüm sudo st-info --probe).')
        raise StepFailed('st-info programlayıcıyı açamadı.\n' + r.tail(), hint)
    ctx.state.record('stlink', sl[0])
    ctx.ui.ok('st-info programlayıcıyı açabiliyor')
    ctx.ui.warn('ST-Link klonunun kasa yazısına güvenme — sonraki adımda '
                'GND pinini multimetreyle doğrulayacağız.')


J4_TABLE = """\
    J4 pad (çipe bakarken) │ Çip bacağı │ Sinyal       │ ST-Link pini
    ───────────────────────┼────────────┼──────────────┼─────────────
    sol                    │ 49         │ SWCLK (PA14) │ 6
    orta                   │ —          │ GND          │ 3 veya 4
    sağ                    │ 46         │ SWDIO (PA13) │ 2
    (3V3 BAĞLANMAZ — J4'te zaten yok)"""


def s_baglanti(ctx: Ctx):
    ctx.ui.say('Kart 1 (sarı kasa, TXTY 6052MAIN_V2.1), SWD = J4 — 2026-10-16\'da '
               'multimetreyle ölçüldü:')
    ctx.ui.say(J4_TABLE)
    _checklist(ctx, [
        'ST-Link\'in pin 3/4\'ünün (GND) USB soketinin metal gövdesiyle '
        'sürekliliğini multimetreyle doğruladın mı?',
        'ST-Link\'in 3.3V / 5V pinleri hiçbir yere bağlı DEĞİL mi?',
        'Üç tel tablodaki gibi bağlandı mı (sol=SWCLK→6, orta=GND→3/4, sağ=SWDIO→2)?',
        'Batarya sigorta + E-stop üzerinden karta bağlı mı?',
    ])
    nrst = ctx.ui.confirm('NRST teli de bağlı mı? (J4\'te yok; bacak 7\'nin izini '
                          'bulup ST-Link pin 1\'e bağladıysan "e", bilmiyorsan "h")')
    ctx.state.record('nrst', nrst)
    if nrst:
        ctx.ui.ok('OpenOCD "connect_assert_srst" ile bağlanacak (Connect Under Reset)')
    else:
        ctx.ui.say('Normal bağlantı denenecek; çoğu zaman yeter.')


def s_kimlik(ctx: Ctx):
    ctx.ui.say('Güç butonuna bas ve BASILI TUT (self-latch: bırakırsan MCU kapanır).')
    ctx.ui.pause('Butona basılıyken Enter')
    r = ctx.run(['st-info', '--probe'], timeout=30)
    info = parsers.parse_st_info(r.output)
    if info.programmers < 1:
        raise StepFailed('Programlayıcı kayboldu.\n' + r.tail())
    if not info.has_target:
        raise StepFailed(
            'Programlayıcı var ama hedef çip görünmüyor (chipid 0).',
            'Kontrol: butona basılı mı, batarya bağlı mı, J4 telleri tablodaki '
            'yerde mi? Olmazsa NRST gerekebilir: LQFP64 bacak 7\'nin izini '
            'multimetreyle bul, ST-Link pin 1\'e bağla ve "baglanti" adımında '
            'NRST = evet de. Pad sırasını TAHMİNLE değiştirme.')
    ctx.ui.ok(f'chipid 0x{info.chipid:03x}  {info.dev_type}  flash {info.flash}')
    ctx.state.record('chipid', f'0x{info.chipid:03x}')
    ctx.state.record('st_info_flash', info.flash)
    if info.chipid != parsers.F1_HIGH_DENSITY_CHIPID:
        raise Abort(f'chipid 0x{info.chipid:03x} — STM32F103RCT6 için 0x414 '
                    '(F1 high-density) beklenir. Çip yazısını ve bu değeri not et; '
                    'bu kart için yol ayrı değerlendirilmeli.')
    if info.flash and info.flash != parsers.RCT6_FLASH_BYTES:
        ctx.ui.warn(f'Flash {info.flash} B (RCT6 = 262144). RDP kilitliyken '
                    'yanlış okunabilir; kilit kalktıktan sonra tekrar bakılacak.')

    r = ctx.openocd('mdw 0xE000ED00')
    if parsers.openocd_unexpected_idcode(r.output):
        raise Abort('OpenOCD "UNEXPECTED idcode" dedi — SWD hata ayıklama '
                    'portu STM32\'ninki değil: büyük olasılıkla klon (GD32/CKS32). '
                    'DUR. Unlock ve hatta baud farklı olabilir.\n' + r.tail())
    words = parsers.parse_mdw(r.output, 0xE000ED00)
    if not words:
        raise StepFailed('CPUID okunamadı.\n' + r.tail(),
                         'Buton basılı mı? Olmazsa NRST (bacak 7) gerekebilir.')
    cpuid = words[0]
    ctx.state.record('cpuid', f'0x{cpuid:08X}')
    if cpuid != parsers.STM32F1_CPUID:
        raise Abort(f'CPUID 0x{cpuid:08X} — gerçek STM32F103 0x411FC231 '
                    '(Cortex-M3 r1p1) verir. Başka revizyon = klon olabilir '
                    '(GD32?). DUR: değeri ve çip yazısını not et.')
    ctx.ui.ok('CPUID 0x411FC231 — Cortex-M3 r1p1, gerçek STM32F10x ile uyumlu')


def _flash_state(ctx: Ctx) -> str:
    r = ctx.openocd('mdw 0x08000000 4', reset_halt=True)
    if parsers.openocd_unexpected_idcode(r.output):
        raise Abort('OpenOCD "UNEXPECTED idcode" — klon çip. DUR.\n' + r.tail())
    return parsers.classify_flash_words(parsers.parse_mdw(r.output, 0x08000000))


def s_kilit(ctx: Ctx):
    st = _flash_state(ctx)
    if st == 'bos':
        ctx.ui.ok('Okuma koruması zaten kapalı ve flash boş — unlock gerekmiyor.')
        ctx.state.record('rdp', 'kapalı')
        return
    if st == 'dolu':
        ctx.ui.ok('Flash okunabiliyor: okuma koruması zaten kapalı (içinde veri '
                  'var — flash adımı üzerine yazacak).')
        ctx.state.record('rdp', 'kapalı')
        return
    ctx.ui.say('Flash okunamıyor → okuma koruması (RDP) açık. Kaldırmak fabrika '
               'firmware\'ini GERİ DÖNÜŞSÜZ siler (RDP yüzünden yedeği de '
               'alınamaz). Kartı bozma riski yok; Kart 2 yedek.')
    if not ctx.ui.typed_confirm('Fabrika firmware\'i silinsin mi?', 'SIL'):
        raise StepFailed('Onay verilmedi — unlock yapılmadı.')
    r = ctx.openocd('reset halt', 'stm32f1x unlock 0', timeout=90)
    if parsers.openocd_unexpected_idcode(r.output):
        raise Abort('OpenOCD "UNEXPECTED idcode" — klon çip. DUR.\n' + r.tail())
    if not parsers.openocd_unlocked(r.output):
        hint = ('Buton basılı mıydı? ' + (
            'NRST bağlı olduğu halde olmadı: NRST izini tekrar ölç.' if ctx.nrst else
            'Stok firmware SWD\'yi kapatıyorsa "Connect Under Reset" gerekir: '
            'LQFP64 bacak 7 (NRST) izini multimetreyle bul, ST-Link pin 1\'e '
            'bağla, "baglanti" adımında NRST = evet de.'))
        raise StepFailed('Unlock başarısız.\n' + r.tail(), hint)
    ctx.ui.ok('stm32x unlocked')
    ctx.ui.say('Yeni ayar ancak güç kesilince geçerli olur: kartın gücünü KES '
               '(E-stop), 3 sn bekle, tekrar ver ve butona basılı tut.')
    ctx.ui.pause('Güç kesilip verildi, buton basılı — Enter')
    st = _flash_state(ctx)
    if st != 'bos':
        raise StepFailed(f'Doğrulama: flash durumu "{st}" — boş (0xFFFFFFFF) beklenirdi.',
                         'Güç gerçekten kesildi mi? Unlock\'u tekrar dene.')
    ctx.ui.ok('Doğrulandı: flash okunabiliyor ve boş (0xFFFFFFFF)')
    ctx.state.record('rdp', 'kapalı')


def _fw_files(ctx: Ctx):
    return ctx.fw_dir / 'platformio.ini', ctx.fw_dir / 'Inc' / 'config.h'


def s_kaynak(ctx: Ctx):
    ini, cfg = _fw_files(ctx)
    if not ctx.fw_dir.exists():
        if not ctx.ui.confirm(f'EFeru kaynağı {ctx.fw_dir} altına klonlansın mı?'):
            raise StepFailed('Kaynak yok.', 'Var olan bir klon için --fw-dir ver.')
        r = ctx.run(['git', 'clone', EFERU_URL, str(ctx.fw_dir)], timeout=600,
                    stream=not ctx.prova)
        if not r.ok:
            raise StepFailed('git clone başarısız.\n' + r.tail())
    if not ini.exists() or not cfg.exists():
        raise StepFailed(f'{ctx.fw_dir} bir EFeru ağacı değil (platformio.ini / '
                         'Inc/config.h yok).')
    if config_check.env_defines(ini.read_text()) is None:
        raise StepFailed(f'platformio.ini\'de [env:{ENV}] yok — EFeru sürümü '
                         'beklenenden farklı; checklist\'e göre elle bak.')
    r = ctx.run(['git', '-C', str(ctx.fw_dir), 'rev-parse', 'HEAD'])
    commit = r.output.strip() if r.ok else 'bilinmiyor'
    ctx.state.record('eferu_commit', commit)
    ctx.ui.ok(f'EFeru {commit[:12]} — env {ENV} mevcut')
    r = ctx.run(['git', '-C', str(ctx.fw_dir), 'status', '--porcelain'])
    if r.ok and r.output.strip():
        ctx.ui.warn('Ağaçta yerel değişiklik var:\n' + r.output.rstrip())


def _check_config(ctx: Ctx):
    ini, cfg = _fw_files(ctx)
    pre = config_check.env_defines(ini.read_text())
    if pre is None:
        raise StepFailed(f'[env:{ENV}] bulunamadı')
    text = cfg.read_text()
    findings, pp = config_check.check(text, pre)
    return text, findings, pp


def _show_findings(ctx: Ctx, findings):
    for f in findings:
        line = f'{f.name}: {f.message}'
        {'ok': ctx.ui.ok, 'uyari': ctx.ui.warn, 'hata': ctx.ui.err}[f.level](line)


def s_yapilandir(ctx: Ctx):
    _, cfg = _fw_files(ctx)
    text, findings, _ = _check_config(ctx)
    ctx.ui.say(f'{cfg} — {ENV} derlemesinin göreceği değerler:')
    _show_findings(ctx, findings)
    unfixable = [f for f in findings if f.level == 'hata' and not f.fix]
    if unfixable:
        raise StepFailed('Sihirbazın düzeltemeyeceği hata var: '
                         + ', '.join(f.name for f in unfixable),
                         'config.h\'yi elle düzelt, sonra bu adımı tekrar dene.')
    fixable = [f for f in findings if f.fix and f.level in ('hata', 'uyari')]
    if fixable:
        new = config_check.apply_fixes(text, fixable)
        diff = difflib.unified_diff(text.splitlines(), new.splitlines(),
                                    'config.h', 'config.h (önerilen)', n=0, lineterm='')
        ctx.ui.say('Önerilen değişiklik:')
        ctx.ui.say('\n'.join('    ' + d for d in diff))
        if not ctx.ui.confirm('Bu değişiklikler config.h\'ye yazılsın mı?'):
            if any(f.level == 'hata' for f in fixable):
                raise StepFailed('config.h hatalı kaldı — bu haliyle flash edilmez.')
            ctx.ui.warn('Uyarılar olduğu gibi bırakıldı.')
        else:
            backup = cfg.with_name(cfg.name + '.b1_flash.orig')
            if not backup.exists():
                backup.write_text(text)
                ctx.ui.ok(f'Yedek: {backup}')
            cfg.write_text(new)
            text, findings, _ = _check_config(ctx)
            if any(f.level == 'hata' for f in findings):
                _show_findings(ctx, findings)
                raise StepFailed('Düzeltme sonrası hâlâ hata var (sihirbaz hatası?).')
            ctx.ui.ok('config.h yazıldı ve yeniden doğrulandı')
    _record_config(ctx)


def _record_config(ctx: Ctx):
    _, _, pp = _check_config(ctx)
    vals = {}
    for k in ('CTRL_TYP_SEL', 'CTRL_MOD_REQ', 'I_MOT_MAX', 'I_DC_MAX', 'N_MOT_MAX',
              'BAT_CELLS', 'SERIAL_TIMEOUT', 'USART2_BAUD'):
        m = pp.macros.get(k)
        vals[k] = m.value.split()[0] if m and m.value else None
    ctx.state.record('config', vals)


def _require_valid_config(ctx: Ctx, what: str):
    _, findings, _ = _check_config(ctx)
    bad = [f for f in findings if f.level == 'hata']
    if bad:
        _show_findings(ctx, bad)
        raise StepFailed(f'config.h doğrulamadan geçmiyor — {what} yapılmaz.',
                         '"yapilandir" adımını tekrar koş.')


def _bin(ctx: Ctx) -> Path:
    return ctx.fw_dir / '.pio' / 'build' / ENV / 'firmware.bin'


def s_derle(ctx: Ctx):
    _require_valid_config(ctx, 'derleme')
    r = ctx.run(['pio', 'run', '-e', ENV], cwd=str(ctx.fw_dir), timeout=1200,
                stream=not ctx.prova)
    if not r.ok:
        raise StepFailed('Derleme başarısız.\n' + r.tail(25))
    b = _bin(ctx)
    if not b.exists():
        raise StepFailed(f'Derleme bitti ama {b} yok.')
    ctx.ui.ok(f'{b} ({b.stat().st_size} bayt)')
    ctx.state.record('firmware_bin_bytes', b.stat().st_size)


def s_flash(ctx: Ctx):
    _require_valid_config(ctx, 'flash')       # nothing unvalidated gets written
    if not _bin(ctx).exists():
        raise StepFailed('firmware.bin yok — önce "derle".')
    if ctx.state.get('rdp') != 'kapalı':
        raise StepFailed('Okuma korumasının kalktığı doğrulanmadı — önce "kilit".')
    _checklist(ctx, [
        'Tekerlekler hâlâ havada mı? (flash biter bitmez firmware çalışır)',
        'Batarya bağlı, ST-Link J4\'e bağlı mı?',
    ])
    ctx.ui.say('Güç butonuna bas ve flash BİTENE kadar basılı tut.')
    ctx.ui.pause('Butona basılıyken Enter')
    cmd = ['pio', 'run', '-e', ENV, '-t', 'upload']
    r = ctx.run(cmd, cwd=str(ctx.fw_dir), timeout=600, stream=not ctx.prova)
    if not r.ok:
        raise StepFailed('Flash başarısız.\n' + r.tail(20),
                         'Buton flash boyunca basılı kaldı mı? Olmazsa NRST '
                         '(bacak 7) ile "Connect Under Reset" gerekebilir.')
    if 'Verified OK' in r.output:
        ctx.ui.ok('Yazıldı ve doğrulandı (Verified OK)')
    else:
        ctx.ui.warn('Komut başarılı ama "Verified OK" satırı görünmedi.')
    ctx.state.record('flashlandi', _dt.datetime.now().isoformat(timespec='seconds'))
    ctx.state.record('flashlanan_commit', ctx.state.get('eferu_commit'))
    ctx.ui.say('Butonu bırak. Firmware güç kilidini kendisi tutmalı.')
    stayed = ctx.ui.confirm('Kart açık kaldı mı (LED / bip)?')
    ctx.state.record('acik_kaldi', stayed)
    if not stayed:
        ctx.ui.warn('Kapanırsa butona ~1 sn basıp aç. UART adımında feedback '
                    'gelmezse ilk şüpheli bu.')
    ctx.ui.ok('ST-Link\'i artık çıkarabilirsin.')


def _pick_port(ctx: Ctx) -> str:
    ports = ctx.list_ports()
    if not ports:
        raise StepFailed(
            'Seri port yok (/dev/ttyUSB*).',
            'FT232RL takılı mı (lsusb 0403:6001)? Dev container\'da '
            'devcontainer.json\'daki --device=/dev/ttyUSB0 satırını aç; cihaz '
            'container başlamadan ÖNCE takılı olmalı (docs/devcontainer.md).')
    last = ctx.state.get('port')
    if len(ports) == 1:
        port = ports[0]
    else:
        opts = {str(i + 1): p for i, p in enumerate(ports)}
        port = opts[ctx.ui.choose('Hangi port USB-TTL?', opts)]
    if port != last:
        ctx.state.record('port', port)
    ctx.ui.ok(f'Port: {port} @ 115200')
    return port


def s_uart(ctx: Ctx):
    ctx.ui.say('EFeru: sol kablo = USART2 (bizim varyant). Uyarı kaynağın '
               'kendisinden: "kırmızı tel 15 V — her şeyi yakar." Bu kartta tel '
               'sırası belgesiz, tahmin yok: ölçüyoruz.')
    _checklist(ctx, [
        'Yan kart (BLB) sökülü ve SOL kablo demeti açıkta mı?',
        'Batarya bağlı ve kart açık mı (voltaj ölçümü için)?',
        'USB-TTL\'in VCCIO jumper\'ı 3.3V\'ta mı?',
    ])
    while True:
        try:
            n = int(ctx.ui.ask('Sol kablo demetinde kaç tel var?'))
            break
        except ValueError:
            ctx.ui.warn('sayı gir')
    wires = []
    for i in range(n):
        color = ctx.ui.ask(f'Tel {i + 1} rengi?') or f'tel{i + 1}'
        cont = ctx.ui.confirm(f'{color}: kartın GND\'sine (batarya eksi) '
                              'süreklilik (bip) var mı?')
        volts = ctx.ui.ask_float(f'{color}: GND\'ye karşı DC voltaj kaç V?')
        wires.append(parsers.Wire(color, cont, volts))
    wm = parsers.classify_wires(wires)
    if not wm.ok:
        raise StepFailed('Ölçümler beklenen kalıba uymuyor:\n  - '
                         + '\n  - '.join(wm.problems),
                         'Hiçbir şey bağlama. Ölçümü tekrarla; yine uymuyorsa DUR.')
    ctx.ui.ok(f'GND = {wm.gnd.color}, 15 V = {wm.supply.color}, '
              f'sinyal adayları = {wm.signals[0].color} / {wm.signals[1].color}')
    _checklist(ctx, [f'{wm.supply.color} (~15 V) telinin ucunu bantladın mı? '
                     'Bu tel HİÇBİR yere bağlanmayacak.'])

    port_name = _pick_port(ctx)
    ctx.ui.say('TX/RX voltajla ayırt edilemez (ikisi de boşta ~3.3 V). Güvenli '
               'yol: yalnız DİNLE. Firmware 10 ms\'de bir feedback gönderiyor; '
               'çerçeve gelen tel STM32 TX\'tir. USB-TTL TX\'i bu sırada BOŞ — '
               'karta hiçbir şey sürülmüyor.')
    port = ctx.open_serial(port_name)
    tx = None
    try:
        for cand in wm.signals:
            _checklist(ctx, [f'USB-TTL GND → {wm.gnd.color}, USB-TTL RX → '
                             f'{cand.color} bağlı ve USB-TTL TX boşta mı?'])
            res = listen(port, ctx.clock, 1.5)
            if res.odometry_frames:
                raise StepFailed(
                    f'{FB_ODOM_SIZE} baytlık çerçeveler geliyor: ENABLE_ODOMETRY '
                    f'açık flash\'lanmış. ESP32 {FB_SIZE} bayt bekliyor.',
                    '"yapilandir" → "derle" → "flash" adımlarını tekrarla.')
            if res.valid >= 20:
                ctx.ui.ok(f'{cand.color}: {res.valid} geçerli çerçeve → STM32 TX')
                tx = cand
                break
            ctx.ui.say(f'{cand.color}: çerçeve yok ({res.dropped_bytes} bayt çöp)')
    finally:
        port.close()
    if tx is None:
        raise StepFailed(
            'İki adaydan da feedback gelmedi.',
            'Kart açık mı (butona bas)? GND bağlı mı? Port doğru mu? Çöp bayt '
            'geliyorsa baud/çip (GD32: farklı baud raporları var) şüpheli.')
    rx = next(w for w in wm.signals if w is not tx)
    table = {'gnd': wm.gnd.color, 'besleme_15v': wm.supply.color,
             'stm32_tx': tx.color, 'stm32_rx': rx.color}
    ctx.state.record('sol_kablo', table)
    ctx.ui.say(f"""\
    Tel rengi   │ Rol       │ ESP32'ye
    ────────────┼───────────┼──────────────
    {wm.gnd.color:<11} │ GND       │ GND
    {wm.supply.color:<11} │ 15 V      │ BAĞLAMA
    {tx.color:<11} │ STM32 TX  │ RX2 (GPIO16)
    {rx.color:<11} │ STM32 RX  │ TX2 (GPIO17)""")


def s_tekerlek(ctx: Ctx):
    t = ctx.state.get('sol_kablo')
    if not t:
        raise StepFailed('Tel tablosu yok — önce "uart".')
    _checklist(ctx, [
        'Tekerlekler hâlâ havada mı?',
        f'USB-TTL: GND → {t["gnd"]}, RX → {t["stm32_tx"]}, TX → {t["stm32_rx"]} '
        f'bağlı mı ({t["besleme_15v"]} bantlı ve boşta)?',
        'E-stop elinin altında mı?',
    ])
    port = ctx.open_serial(_pick_port(ctx))
    try:
        res = listen(port, ctx.clock, 1.0)
        if res.valid < 20:
            raise StepFailed(f'Feedback yok ({res.valid} çerçeve).',
                             'Kart açık mı? Kablolar "uart" tablosundaki gibi mi?')
        last = res.frames[-1]
        ctx.ui.ok(f'Feedback: {res.valid} çerçeve/sn, batarya ~{last.bat / 100:.1f} V '
                  f'(kalibresiz, B4), kart {last.temp / 10:.1f} °C')
        ctx.state.record('ilk_feedback', {'bat_raw': last.bat, 'temp_raw': last.temp})

        if not ctx.ui.confirm(f'Tekerlekler {TEST_SPEED}/1000 hız komutuyla (~{TEST_SPEED} '
                              'rpm) 3 sn dönecek. Başlatılsın mı?'):
            raise StepFailed('Test başlatılmadı.')
        for i in (3, 2, 1):
            ctx.ui.say(f'    {i}…')
            ctx.clock.sleep(1.0)
        sp = spin_test(port, ctx.clock)
        ctx.ui.say(f'Ortalama ölçülen hız: sol {sp.mean_left:+.0f} rpm, '
                   f'sağ {sp.mean_right:+.0f} rpm ({sp.frames} çerçeve)')
        if not sp.ok:
            raise StepFailed('Tekerlek testi geçmedi:\n  - ' + '\n  - '.join(sp.diagnosis),
                             'Yön ters/titreme: INVERT_*_DIRECTION ya da hall '
                             'kombinasyonu (hasar vermez).')
        ctx.ui.ok('Tekerlekler dönüyor ve hall feedback\'i görüyor ✅')
        dirs = {}
        for side in ('Sol', 'Sağ'):
            dirs[side] = ctx.ui.choose(
                f'{side} tekerlek robotun önüne göre hangi yöne döndü?',
                {'i': 'ileri', 'g': 'geri', '?': 'emin değilim'})
        ctx.state.record('donus', {'komut': TEST_SPEED, 'sol_rpm': round(sp.mean_left),
                                   'sag_rpm': round(sp.mean_right),
                                   'sol_yon': dirs['Sol'], 'sag_yon': dirs['Sağ']})
        for side, flag in (('Sol', 'INVERT_L_DIRECTION'), ('Sağ', 'INVERT_R_DIRECTION')):
            if dirs[side] == 'g':
                ctx.ui.warn(f'{side} geri döndü → {flag} gerekebilir (B4 '
                            'kalibrasyonunda kesinleşir; şimdi şart değil).')

        ctx.ui.say('Son test GÜVENLİK testi: dönerken komut göndermeyi keseceğim. '
                   f'Firmware ~0.8 sn (SERIAL_TIMEOUT) içinde komutu düşürmeli.')
        if not ctx.ui.confirm('Timeout testi başlatılsın mı?'):
            raise StepFailed('Timeout testi yapılmadı — bu test atlanmaz.')
        to = timeout_test(port, ctx.clock)
    finally:
        port.close()
    if not to.cmd_seen:
        raise StepFailed('Timeout testinde komut hiç uygulanmadı — ölçüm anlamsız.',
                         'Bağlantıyı kontrol edip tekrar dene.')
    if to.cmd_zero_after_s is None or to.cmd_zero_after_s > TIMEOUT_LIMIT_S:
        got = ('hiç düşmedi' if to.cmd_zero_after_s is None
               else f'{to.cmd_zero_after_s:.2f} sn')
        ctx.state.record('timeout', {'sonuc': 'BAŞARISIZ', 'olcum': got})
        raise Abort(f'GÜVENLİK: komut kesildiği halde firmware komutu {got} '
                    f'(sınır {TIMEOUT_LIMIT_S} sn). Bu firmware robota takılmaz. '
                    'SERIAL_TIMEOUT ve varyantı kontrol et.')
    ms = round(to.cmd_zero_after_s * 1000)
    slow = (f', tekerlekler {to.slowed_after_s:.1f} sn\'de yavaşladı (serbest dönüş)'
            if to.slowed_after_s is not None else ', tekerlek serbest dönüyor')
    ctx.ui.ok(f'Komut kesilince firmware {ms} ms sonra komutu sıfırladı{slow}')
    ctx.state.record('timeout', {'sonuc': 'GEÇTİ', 'komut_sifir_ms': ms,
                                 'yavaslama_s': to.slowed_after_s})


def write_report(ctx: Ctx, outcome: str) -> Path:
    k = ctx.state.data['kayit']
    stamp = _dt.datetime.now().strftime('%Y%m%d-%H%M')
    path = ctx.report_dir / f'rapor-{stamp}.md'
    lines = [f'# B1 flash raporu — {stamp}' + (' (PROVA, sahte donanım)' if ctx.prova else ''),
             '', f'Sonuç: **{outcome}**', '',
             'Tamamlanan adımlar: ' + ', '.join(ctx.state.completed), '',
             '## Kayıtlar', '']
    for key, val in k.items():
        lines.append(f'- `{key}`: {json.dumps(val, ensure_ascii=False)}')
    lines += ['', '## Geçmiş', ''] + [f'- {e}' for e in ctx.state.data['gecmis']]
    lines += ['', '## Sonra', '',
              '- Tel tablosunu `docs/wiring-map.md`\'ye ve checklist adım 5\'e işle.',
              '- `docs/handoff.md`: İz B adım 1 (B1) durumu + chipid/CPUID + EFeru commit.',
              '- Sıradaki: B2 — ESP32 tezgah testi (`firmware/esp32_bridge`).']
    ctx.report_dir.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')
    return path


def s_rapor(ctx: Ctx):
    path = write_report(ctx, 'B1 tamamlandı')
    ctx.ui.ok(f'Rapor: {path}')
    ctx.ui.say('🎉 Projenin en riskli adımı bitti. Tel tablosunu wiring-map.md\'ye, '
               'sonucu handoff.md\'ye işlemeyi unutma.')


@dataclass
class Step:
    id: str
    title: str
    ref: str
    fn: Callable[[Ctx], None]


STEPS = [
    Step('guvenlik', 'Güvenlik ön koşulları', 'checklist adım 0', s_guvenlik),
    Step('araclar', 'Araçlar ve ST-Link', 'checklist adım 0 — Araçlar', s_araclar),
    Step('baglanti', 'SWD bağlantısı (J4)', 'checklist adım 1', s_baglanti),
    Step('kimlik', 'Çip kimliği (chipid + CPUID)', 'checklist adım 1', s_kimlik),
    Step('kilit', 'Okuma korumasını kaldır (RDP)', 'checklist adım 2', s_kilit),
    Step('kaynak', 'EFeru kaynağı', 'checklist adım 3', s_kaynak),
    Step('yapilandir', 'config.h doğrula / düzelt', 'checklist adım 3', s_yapilandir),
    Step('derle', 'Derle', 'checklist adım 3', s_derle),
    Step('flash', 'Flash', 'checklist adım 4', s_flash),
    Step('uart', 'UART tellerini ölç, TX\'i bul', 'checklist adım 5', s_uart),
    Step('tekerlek', 'Tekerlek + timeout testi', 'checklist adım 6', s_tekerlek),
    Step('rapor', 'Rapor', '', s_rapor),
]


@dataclass
class Outcome:
    code: int                 # 0 done, 1 user quit, 2 stopped (Abort), 130 Ctrl-C
    last_step: str = ''
    messages: list[str] = field(default_factory=list)


def run(ctx: Ctx, start: str | None = None) -> Outcome:
    ids = [s.id for s in STEPS]
    if start is None:
        pending = [s for s in ids if s not in ctx.state.completed]
        start = pending[0] if pending else ids[0]
        if ctx.state.completed and pending and start != ids[0]:
            ctx.ui.say(f'Önceki oturumda tamamlanan: {", ".join(ctx.state.completed)}')
            if not ctx.ui.confirm(f'"{start}" adımından devam edilsin mi? '
                                  '(hayır = baştan)'):
                start = ids[0]
    i = ids.index(start)
    while i < len(STEPS):
        step = STEPS[i]
        ctx.ui.header(i + 1, len(STEPS), step.title, step.ref)
        try:
            step.fn(ctx)
        except StepFailed as e:
            ctx.state.event(step.id, f'başarısız: {e}')
            ctx.ui.err(str(e))
            if e.hint:
                ctx.ui.say('  → ' + e.hint)
            if ctx.ui.choose('Ne yapalım?', {'t': 'tekrar dene',
                                             'c': 'çık (sonra kaldığın yerden)'}) == 't':
                continue
            return Outcome(1, step.id)
        except Abort as e:
            ctx.state.event(step.id, f'DURDURULDU: {e}')
            ctx.ui.err('DUR — ' + str(e))
            path = write_report(ctx, f'DURDURULDU ({step.id})')
            ctx.ui.say(f'Rapor: {path}')
            return Outcome(2, step.id, [str(e)])
        except (KeyboardInterrupt, EOFError):
            ctx.state.event(step.id, 'kullanıcı kesti')
            ctx.ui.warn('Kesildi. Kaldığın yerden devam edebilirsin.')
            return Outcome(130, step.id)
        ctx.state.done(step.id)
        i += 1
    return Outcome(0, STEPS[-1].id)
