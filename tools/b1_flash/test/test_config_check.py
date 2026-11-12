from b1_flash import config_check as cc
from b1_flash.fakes import FAKE_CONFIG_H, FAKE_PLATFORMIO_INI

PRE = cc.env_defines(FAKE_PLATFORMIO_INI)


def levels(findings):
    return {f.name: f.level for f in findings}


def test_env_flags_come_from_platformio_ini():
    assert PRE['VARIANT_USART'] == '1' and 'PLATFORMIO' in PRE
    assert 'VARIANT_ADC' not in PRE
    assert cc.env_defines(FAKE_PLATFORMIO_INI, 'VARIANT_NOPE') is None


def test_upstream_defaults_fail_on_mode_and_current():
    f, pp = cc.check(FAKE_CONFIG_H, PRE)
    lv = levels(f)
    assert lv['CTRL_MOD_REQ'] == 'hata'                 # VLT_MODE out of the box
    assert lv['I_DC_MAX'] == 'hata'                     # 2 x 17 A > 20 A fuse
    assert lv['CTRL_TYP_SEL'] == lv['BAT_CELLS'] == 'ok'
    assert lv['CONTROL_SERIAL_USART2'] == lv['FEEDBACK_SERIAL_USART2'] == 'ok'
    assert lv['USART2_BAUD'] == lv['SERIAL_TIMEOUT'] == 'ok'   # #ifndef / #if \ blocks
    assert lv['ENABLE_ODOMETRY'] == lv['TANK_STEERING'] == lv['DUAL_INPUTS'] == 'ok'
    assert not pp.warnings


def test_fixes_make_it_pass_and_only_touch_those_lines():
    f, _ = cc.check(FAKE_CONFIG_H, PRE)
    new = cc.apply_fixes(FAKE_CONFIG_H, f)
    f2, pp = cc.check(new, PRE)
    assert all(x.level == 'ok' for x in f2), [(x.name, x.message) for x in f2 if x.level != 'ok']
    assert pp.macros['I_MOT_MAX'].value.split()[0] == '8'
    assert pp.macros['I_DC_MAX'].value.split()[0] == '10'
    changed = [(a, b) for a, b in zip(FAKE_CONFIG_H.split('\n'), new.split('\n')) if a != b]
    assert len(changed) == 3
    for old, new_line in changed:                       # comments survive, same column
        assert old.index('//') == new_line.index('//')


def test_other_variants_redefinitions_do_not_leak_into_usart():
    # VARIANT_HOVERCAR #undefs CTRL_MOD_REQ and defines TANK_STEERING
    text = FAKE_CONFIG_H.replace('CTRL_MOD_REQ    VLT_MODE', 'CTRL_MOD_REQ    SPD_MODE')
    f, pp = cc.check(text, PRE)
    assert pp.macros['CTRL_MOD_REQ'].value.startswith('SPD_MODE')
    assert levels(f)['TANK_STEERING'] == 'ok'
    hovercar = cc.preprocess(text, {'PLATFORMIO': '1', 'VARIANT_HOVERCAR': '1'})
    assert hovercar.macros['CTRL_MOD_REQ'].value.startswith('TRQ_MODE')
    assert 'TANK_STEERING' in hovercar.macros


def test_override_inside_usart_block_is_what_counts_and_what_gets_fixed():
    text = FAKE_CONFIG_H.replace(
        '#ifdef VARIANT_USART\n',
        '#ifdef VARIANT_USART\n  #undef CTRL_MOD_REQ\n  #define CTRL_MOD_REQ TRQ_MODE\n')
    text = text.replace('CTRL_MOD_REQ    VLT_MODE', 'CTRL_MOD_REQ    SPD_MODE')
    f, pp = cc.check(text, PRE)
    assert pp.macros['CTRL_MOD_REQ'].value == 'TRQ_MODE'
    new = cc.apply_fixes(text, f)
    assert '  #define CTRL_MOD_REQ SPD_MODE' in new


def test_enabled_odometry_is_an_error_and_is_commented_out():
    text = FAKE_CONFIG_H.replace('// #define ENABLE_ODOMETRY', '#define ENABLE_ODOMETRY')
    f, _ = cc.check(text, PRE)
    assert levels(f)['ENABLE_ODOMETRY'] == 'hata'
    f2, _ = cc.check(cc.apply_fixes(text, f), PRE)
    assert levels(f2)['ENABLE_ODOMETRY'] == 'ok'


def test_dual_inputs_switches_the_nested_branch():
    text = FAKE_CONFIG_H.replace('// #define DUAL_INPUTS', '#define DUAL_INPUTS')
    pp = cc.preprocess(text, PRE)
    assert pp.macros['FLASH_WRITE_KEY'].value == '0x1102'
    assert levels(cc.check(text, PRE)[0])['DUAL_INPUTS'] == 'hata'
    assert cc.preprocess(FAKE_CONFIG_H, PRE).macros['FLASH_WRITE_KEY'].value == '0x1002'


def test_missing_usart_variant_has_no_serial_control():
    pre = {'PLATFORMIO': '1', 'VARIANT_ADC': '1'}
    lv = levels(cc.check(FAKE_CONFIG_H, pre)[0])
    assert lv['CONTROL_SERIAL_USART2'] == 'hata'
    assert lv['SERIAL_TIMEOUT'] == 'hata'       # the UART block is never entered


def test_limits_rules():
    def lim(mot, dc):
        t = (FAKE_CONFIG_H.replace('I_MOT_MAX       15', f'I_MOT_MAX       {mot}')
             .replace('I_DC_MAX        17', f'I_DC_MAX        {dc}'))
        return [x for x in cc.check(t, PRE)[0] if x.name.startswith('I_')][0].level
    assert lim(8, 10) == 'ok'
    assert lim(6, 8) == 'ok'
    assert lim(7, 10) == 'uyari'                # not I_MOT_MAX + 2
    assert lim(9, 11) == 'hata'                 # 22 A > fuse
    assert lim(10, 9) == 'hata'


def test_long_serial_timeout_is_an_error():
    t = FAKE_CONFIG_H.replace('SERIAL_TIMEOUT          160', 'SERIAL_TIMEOUT          400')
    f = cc.check(t, PRE)[0]
    assert levels(f)['SERIAL_TIMEOUT'] == 'hata'
    assert levels(cc.check(cc.apply_fixes(t, f), PRE)[0])['SERIAL_TIMEOUT'] == 'ok'


def test_if_evaluator():
    m = {'A': cc.Macro('1', 0), 'B': cc.Macro('2', 0)}
    assert cc._eval_if('defined(A) && !defined(C)', m)
    assert cc._eval_if('defined A || defined C', m)
    assert cc._eval_if('(defined(A) && B == 2)', m)
    assert not cc._eval_if('defined(C) && C == 1', m)
    assert cc._eval_if('UNKNOWN == 0', m)
