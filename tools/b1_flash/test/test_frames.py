import struct

import pytest

from b1_flash.frames import (CMD_SIZE, FB_ODOM_SIZE, FB_SIZE, SPEED_CAP, Feedback,
                             FeedbackParser, decode_command, encode_command,
                             encode_feedback)


def test_command_layout_matches_the_esp32_bridge():
    # firmware/esp32_bridge/src/main.cpp HoverCommand: start, steer, speed, checksum
    raw = encode_command(0, 50)
    assert len(raw) == CMD_SIZE == 8
    start, steer, speed, chk = struct.unpack('<HhhH', raw)
    assert (start, steer, speed) == (0xABCD, 0, 50)
    assert chk == 0xABCD ^ 50


def test_negative_values_xor_as_uint16_like_the_firmware():
    # util.c: (uint16_t)(start ^ steer ^ speed) with int16 operands
    raw = encode_command(-7, -50)
    assert struct.unpack('<HhhH', raw)[3] == (0xABCD ^ (-7 & 0xFFFF) ^ (-50 & 0xFFFF))
    assert decode_command(raw) == (-7, -50)


def test_cap_refuses_anything_faster():
    encode_command(0, SPEED_CAP)
    with pytest.raises(ValueError):
        encode_command(0, SPEED_CAP + 1)
    with pytest.raises(ValueError):
        encode_command(-SPEED_CAP - 1, 0)


def test_decode_rejects_bad_checksum():
    raw = bytearray(encode_command(0, 50))
    raw[-1] ^= 1
    assert decode_command(bytes(raw)) is None


def fb(speed=10):
    return Feedback(0, speed, speed, -speed, 3920, 312, 0)


def test_feedback_is_18_bytes_and_round_trips():
    raw = encode_feedback(fb(33))
    assert len(raw) == FB_SIZE == 18
    out = FeedbackParser().feed(raw)
    assert len(out) == 1
    assert (out[0].cmd2, out[0].speed_r, out[0].speed_l) == (33, 33, -33)


def test_parser_resyncs_across_garbage_and_split_reads():
    stream = b'\x00\xcd\x12' + encode_feedback(fb(1)) + b'\xff' * 5 + encode_feedback(fb(2))
    p = FeedbackParser()
    got = []
    for i in range(0, len(stream), 3):               # arrives in tiny pieces
        got += p.feed(stream[i:i + 3])
    assert [f.cmd2 for f in got] == [1, 2]
    assert p.dropped_bytes > 0


def test_corrupted_frame_is_dropped_not_misread():
    bad = bytearray(encode_feedback(fb(5)))
    bad[6] ^= 0x40
    p = FeedbackParser()
    got = p.feed(bytes(bad) + encode_feedback(fb(6)) + encode_feedback(fb(7)))
    assert [f.cmd2 for f in got] == [6, 7]


def test_odometry_frames_are_recognised_and_flagged():
    raw = encode_feedback(fb(4), odometry=True, wheel_r=100, wheel_l=-100)
    assert len(raw) == FB_ODOM_SIZE == 22
    p = FeedbackParser()
    got = p.feed(raw * 3)
    assert len(got) == 3 and all(f.odometry for f in got)
    assert p.odometry_frames == 3 and p.frames == 0
