from b1_flash import parsers as p
from b1_flash.parsers import Wire

ST_INFO_18 = """Found 1 stlink programmers
  version:    V2J37S7
  serial:     303030303030303030303031
  flash:      262144 (pagesize: 2048)
  sram:       65536
  chipid:     0x414
  dev-type:   STM32F1xx_HD
"""
ST_INFO_17 = """Found 1 stlink programmers
 serial:     303030303030303030303031
 flash:      262144 (pagesize: 2048)
 sram:       65536
 chipid:     0x0414
 descr:      F1xx High-density
"""


def test_st_info_both_formats():
    for out in (ST_INFO_18, ST_INFO_17):
        i = p.parse_st_info(out)
        assert i.programmers == 1 and i.chipid == 0x414 and i.flash == 262144
        assert i.has_target
    assert p.parse_st_info(ST_INFO_17).dev_type == 'F1xx High-density'


def test_st_info_no_programmer_and_no_target():
    i = p.parse_st_info('libusb: error [_get_usbfs_fd] libusb couldn\'t open USB device '
                        '/dev/bus/usb/001/005, errno=13\nFound 0 stlink programmers\n')
    assert i.programmers == 0 and i.libusb_error
    i = p.parse_st_info('Found 1 stlink programmers\n  flash:      0\n  chipid:     0x000\n')
    assert i.programmers == 1 and not i.has_target


def test_lsusb():
    out = ('Bus 001 Device 005: ID 0483:3748 STMicroelectronics ST-LINK/V2\n'
           'Bus 001 Device 006: ID 0403:6001 Future Technology Devices FT232\n')
    assert p.find_stlink(out) == ['ST-Link V2']
    assert p.find_stlink('Bus 001 Device 002: ID 8087:0024 Intel\n') == []


def test_mdw_and_cpuid():
    out = 'Info : STLINK V2J37S7\n0xe000ed00: 411fc231 \n'
    assert p.parse_mdw(out, 0xE000ED00) == [0x411FC231]
    assert p.parse_mdw('Error: Failed to read memory', 0xE000ED00) is None
    words = p.parse_mdw('0x08000000: ffffffff ffffffff ffffffff ffffffff \n', 0x08000000)
    assert words == [0xFFFFFFFF] * 4


def test_flash_classification():
    assert p.classify_flash_words(None) == 'kilitli'
    assert p.classify_flash_words([0, 0, 0, 0]) == 'kilitli'
    assert p.classify_flash_words([0xFFFFFFFF] * 4) == 'bos'
    assert p.classify_flash_words([0x20005000, 0x08000D2D, 1, 2]) == 'dolu'


def test_openocd_markers():
    assert p.openocd_unlocked('stm32x unlocked.\nINFO: a reset or power cycle is required')
    assert not p.openocd_unlocked('Error: stm32x device protected')
    assert p.openocd_unexpected_idcode('Warn : UNEXPECTED idcode: 0x2ba01477')
    assert not p.openocd_unexpected_idcode('Info : SWD DPIDR 0x1ba01477')


def test_wires_expected_pattern():
    wm = p.classify_wires([Wire('siyah', True, 0.0), Wire('kırmızı', False, 15.1),
                           Wire('yeşil', False, 3.3), Wire('mavi', False, 3.05)])
    assert wm.ok
    assert wm.gnd.color == 'siyah' and wm.supply.color == 'kırmızı'
    assert {w.color for w in wm.signals} == {'yeşil', 'mavi'}


def test_wires_stop_on_anything_unexpected():
    five_volt = p.classify_wires([Wire('a', True, 0.0), Wire('b', False, 15.0),
                                  Wire('c', False, 5.0), Wire('d', False, 3.3)])
    assert not five_volt.ok
    assert any('5.00 V' in x for x in five_volt.problems)
    two_supplies = p.classify_wires([Wire('a', True, 0.0), Wire('b', False, 15.0),
                                     Wire('c', False, 14.8), Wire('d', False, 3.3)])
    assert not two_supplies.ok
    contradictory = p.classify_wires([Wire('a', True, 3.3), Wire('b', False, 15.0),
                                      Wire('c', False, 3.3), Wire('d', False, 3.3)])
    assert not contradictory.ok
