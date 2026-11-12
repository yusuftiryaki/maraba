"""The whole wizard against the fake bench: what runs, and above all what does NOT."""
import pytest

from b1_flash import wizard
from b1_flash.fakes import PROVA_RULES, FakeBoard, FakeClock, FakeRunner, write_fake_tree
from b1_flash.ui import ScriptedUI

QUIT = [('Ne yapalım', 'c')]


def make(tmp_path, runner=None, board=None, rules=(), fw_exists=False, clock=None):
    clock = clock or FakeClock()
    board = board or FakeBoard(clock)
    fw = tmp_path / 'fw'
    if fw_exists:
        write_fake_tree(fw)
    ctx = wizard.Ctx(
        ui=ScriptedUI(list(rules) + QUIT + PROVA_RULES),
        runner=runner or FakeRunner(), clock=clock,
        state=wizard.State(tmp_path / 'state.json'), fw_dir=fw,
        open_serial=lambda _p: board, list_ports=lambda: ['/dev/ttyUSB0'],
        report_dir=tmp_path, prova=True)
    return ctx


def test_happy_path_runs_every_step_and_writes_a_report(tmp_path):
    ctx = make(tmp_path)
    out = wizard.run(ctx)
    assert out.code == 0, ctx.ui.text()
    assert ctx.state.completed == [s.id for s in wizard.STEPS]
    k = ctx.state.data['kayit']
    assert k['cpuid'] == '0x411FC231' and k['rdp'] == 'kapalı'
    assert k['config']['CTRL_MOD_REQ'] == 'SPD_MODE'
    assert k['config']['I_DC_MAX'] == '10'
    assert k['sol_kablo'] == {'gnd': 'siyah', 'besleme_15v': 'kırmızı',
                              'stm32_tx': 'yeşil', 'stm32_rx': 'mavi'}
    assert k['timeout']['sonuc'] == 'GEÇTİ'
    assert 750 <= k['timeout']['komut_sifir_ms'] <= 850
    assert list(tmp_path.glob('rapor-*.md'))
    assert (ctx.fw_dir / 'Inc' / 'config.h.b1_flash.orig').exists()


def test_safety_no_stops_the_wizard_before_any_command(tmp_path):
    runner = FakeRunner()
    ctx = make(tmp_path, runner, rules=[('Tekerlekler yerden kesik', False)])
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'guvenlik'
    assert runner.calls == []


def test_gd32_cpuid_stops_before_unlock(tmp_path):
    runner = FakeRunner(cpuid=0x412FC231)
    out = wizard.run(make(tmp_path, runner))
    assert out.code == 2 and out.last_step == 'kimlik'
    assert not runner.ran('stm32f1x unlock') and not runner.ran('-t upload')
    assert list(tmp_path.glob('rapor-*.md'))          # the stop is written down


def test_unexpected_idcode_stops(tmp_path):
    runner = FakeRunner(unexpected_idcode=True)
    out = wizard.run(make(tmp_path, runner))
    assert out.code == 2 and out.last_step == 'kimlik'
    assert not runner.ran('stm32f1x unlock')


def test_wrong_chipid_stops(tmp_path):
    out = wizard.run(make(tmp_path, FakeRunner(chipid=0x410)))
    assert out.code == 2 and out.last_step == 'kimlik'


def test_unlock_needs_the_typed_word(tmp_path):
    runner = FakeRunner()
    ctx = make(tmp_path, runner, rules=[('Fabrika firmware', 'evet')])
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'kilit'
    assert not runner.ran('stm32f1x unlock')


def test_failed_unlock_points_at_nrst(tmp_path):
    ctx = make(tmp_path, FakeRunner(unlock_ok=False))
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'kilit'
    assert 'bacak 7' in ctx.ui.text()


def test_already_unlocked_board_is_not_erased_again(tmp_path):
    runner = FakeRunner(locked=False)
    out = wizard.run(make(tmp_path, runner, rules=[('Ne yapalım', 'c')]))
    assert out.code == 0
    assert not runner.ran('stm32f1x unlock')


def test_declined_config_fix_means_nothing_is_built_or_flashed(tmp_path):
    runner = FakeRunner()
    ctx = make(tmp_path, runner, rules=[('config.h\'ye yazılsın', False)])
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'yapilandir'
    assert not runner.ran('pio')


def test_config_edited_after_check_blocks_the_flash(tmp_path):
    runner = FakeRunner()
    ctx = make(tmp_path, runner, rules=[('Tekerlekler hâlâ havada', True)])
    # run up to and including the build
    for s in wizard.STEPS[:8]:
        s.fn(ctx)
        ctx.state.done(s.id)
    cfg = ctx.fw_dir / 'Inc' / 'config.h'
    cfg.write_text(cfg.read_text().replace('I_DC_MAX        10', 'I_DC_MAX        17'))
    with pytest.raises(wizard.StepFailed):
        wizard.s_flash(ctx)
    assert not runner.ran('-t upload')


def test_firmware_without_timeout_is_a_hard_stop(tmp_path):
    clock = FakeClock()
    ctx = make(tmp_path, board=FakeBoard(clock, timeout_s=None), clock=clock)
    out = wizard.run(ctx)
    assert out.code == 2 and out.last_step == 'tekerlek'
    assert ctx.state.get('timeout')['sonuc'] == 'BAŞARISIZ'


def test_odometry_firmware_is_caught_at_uart(tmp_path):
    clock = FakeClock()
    ctx = make(tmp_path, board=FakeBoard(clock, odometry=True), clock=clock)
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'uart'
    assert 'ENABLE_ODOMETRY' in ctx.ui.text()


def test_tx_is_found_on_the_second_candidate(tmp_path):
    clock = FakeClock()
    board = FakeBoard(clock)
    ctx = make(tmp_path, board=board, clock=clock)
    connected = {'n': 0}

    real_confirm = ctx.ui.confirm

    def confirm(q):
        if 'USB-TTL RX →' in q:
            connected['n'] += 1
            board.tx_connected = connected['n'] == 2     # first guess is the RX wire
        return real_confirm(q)

    ctx.ui.confirm = confirm
    out = wizard.run(ctx)
    assert out.code == 0, ctx.ui.text()
    assert ctx.state.get('sol_kablo')['stm32_tx'] == 'mavi'
    assert ctx.state.get('sol_kablo')['stm32_rx'] == 'yeşil'


def test_bad_wire_measurements_stop_before_anything_is_connected(tmp_path):
    clock = FakeClock()
    board = FakeBoard(clock)
    opened = []
    ctx = make(tmp_path, board=board, clock=clock,
               rules=[('mavi: GND\'ye karşı', 5.0)])
    ctx.open_serial = lambda p: opened.append(p) or board
    out = wizard.run(ctx)
    assert out.code == 1 and out.last_step == 'uart'
    assert opened == []


def test_resume_offers_the_first_pending_step(tmp_path):
    ctx = make(tmp_path)
    for s in ('guvenlik', 'araclar'):
        ctx.state.done(s)
    ctx.ui.rules.insert(0, ('adımından devam', True))
    wizard.run(ctx)
    assert ctx.ui.log[0] != '## Güvenlik ön koşulları'
    assert '## SWD bağlantısı (J4)' in ctx.ui.log


def test_nrst_answer_changes_openocd_invocation(tmp_path):
    runner = FakeRunner()
    ctx = make(tmp_path, runner, rules=[('NRST teli', True)])
    wizard.run(ctx)
    oc = [c for c in runner.calls if c[0] == 'openocd']
    assert oc and all('reset_config srst_only srst_nogate connect_assert_srst' in c
                      for c in oc)
