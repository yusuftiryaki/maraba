import pytest

from b1_flash.fakes import FakeBoard, FakeClock
from b1_flash.frames import decode_command
from b1_flash.wheel import TEST_SPEED, listen, spin_test, timeout_test


class RecordingBoard(FakeBoard):
    def __init__(self, clock, **kw):
        super().__init__(clock, **kw)
        self.sent = []

    def write(self, data):
        self.sent.append(decode_command(data))
        return super().write(data)


def test_listen_counts_frames_without_writing():
    clock = FakeClock()
    board = RecordingBoard(clock)
    res = listen(board, clock, 1.0)
    assert 95 <= res.valid <= 101                     # 10 ms period
    assert board.sent == []                           # receive-only


def test_listen_flags_odometry_firmware():
    clock = FakeClock()
    res = listen(FakeBoard(clock, odometry=True), clock, 0.5)
    assert res.odometry_frames > 0 and res.valid == 0


def test_spin_passes_on_a_healthy_board_and_ends_with_zeros():
    clock = FakeClock()
    board = RecordingBoard(clock)
    r = spin_test(board, clock)
    assert r.ok and r.cmd_seen
    assert r.mean_left == pytest.approx(TEST_SPEED, abs=3)
    assert board.sent[-1] == (0, 0)
    assert max(abs(s[1]) for s in board.sent) == TEST_SPEED


def test_spin_names_a_dead_rx_wire():
    clock = FakeClock()
    r = spin_test(FakeBoard(clock, rx_connected=False), clock)
    assert not r.ok and not r.cmd_seen
    assert any('TX → STM32 RX' in d for d in r.diagnosis)


def test_spin_names_the_wheel_that_does_not_turn():
    clock = FakeClock()
    r = spin_test(FakeBoard(clock, wheels_ok=(True, False)), clock)
    assert not r.ok and r.cmd_seen
    assert any(d.startswith('Sağ') for d in r.diagnosis)
    assert not any(d.startswith('Sol') for d in r.diagnosis)


def test_timeout_measured_close_to_serial_timeout():
    clock = FakeClock()
    r = timeout_test(FakeBoard(clock), clock)
    assert r.ok and r.cmd_seen
    assert r.cmd_zero_after_s == pytest.approx(0.8, abs=0.05)


def test_firmware_without_timeout_fails():
    clock = FakeClock()
    r = timeout_test(FakeBoard(clock, timeout_s=None), clock)
    assert not r.ok and r.cmd_zero_after_s is None


def test_slow_timeout_fails():
    clock = FakeClock()
    r = timeout_test(FakeBoard(clock, timeout_s=2.0), clock)
    assert not r.ok and r.cmd_zero_after_s == pytest.approx(2.0, abs=0.05)


def test_timeout_is_not_passed_by_a_board_that_never_took_the_command():
    # cmd is 0 all along -> "zero after 0 s" must not count as a pass
    clock = FakeClock()
    r = timeout_test(FakeBoard(clock, rx_connected=False), clock)
    assert not r.ok and not r.cmd_seen


def test_interrupt_still_sends_zeros():
    clock = FakeClock()

    class Interrupting(RecordingBoard):
        def read_available(self):
            if len(self.sent) == 10:
                self.sent.append('INTERRUPT')
                raise KeyboardInterrupt
            return super().read_available()

    board = Interrupting(clock)
    with pytest.raises(KeyboardInterrupt):
        spin_test(board, clock)
    i = board.sent.index('INTERRUPT')
    assert board.sent[i + 1:] and all(s == (0, 0) for s in board.sent[i + 1:])
