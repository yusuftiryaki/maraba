"""UBX, RTCM, NTRIP and the fake F9P (ubx.py, rtcm.py, ntrip.py, f9p_sim.py). ROS-free."""

import math

import pytest

from robot_kit import f9p_sim, ntrip, rtcm, ubx


def pvt(**changes):
    base = dict(itow_ms=123456, fix_type=ubx.FIX_3D, gnss_fix_ok=True, diff_soln=True,
                carr_soln=ubx.CARR_FIXED, num_sv=22, lat_deg=37.0662123, lon_deg=37.3833456,
                height_m=880.123, hmsl_m=844.0, h_acc_m=0.014, v_acc_m=0.021,
                vel_n=0.25, vel_e=-0.5, vel_d=0.0)
    base.update(changes)
    return ubx.NavPvt(**base)


# --- UBX -----------------------------------------------------------------------

def test_nav_pvt_round_trip():
    parser = ubx.UbxParser()
    [back] = ubx.nav_pvts(parser, ubx.encode_nav_pvt(pvt()))
    assert back.lat_deg == pytest.approx(37.0662123, abs=1e-7)
    assert back.lon_deg == pytest.approx(37.3833456, abs=1e-7)
    assert back.height_m == pytest.approx(880.123, abs=1e-3)
    assert back.h_acc_m == pytest.approx(0.014) and back.carr_soln == ubx.CARR_FIXED
    assert (back.vel_n, back.vel_e) == (pytest.approx(0.25), pytest.approx(-0.5))


def test_parser_survives_noise_split_frames_and_bad_checksums():
    good = ubx.encode_nav_pvt(pvt())
    corrupt = bytearray(good)
    corrupt[30] ^= 0xFF
    nmea = b"$GNGGA,123519,3704.0,N,03723.0,E,4,22,0.6,880.1,M,36.0,M,,*47\r\n"
    stream = b"\xb5" + nmea + bytes(corrupt) + rtcm.make_message(1005) + good + good[:40]
    parser = ubx.UbxParser()
    found = []
    for i in range(0, len(stream), 7):            # arbitrary chunking, as a pty delivers
        found += ubx.nav_pvts(parser, stream[i:i + 7])
    assert len(found) == 1 and parser.bad_checksums == 1
    found += ubx.nav_pvts(parser, good[40:])
    assert len(found) == 2


@pytest.mark.parametrize("changes, status, name", [
    ({}, ubx.STATUS_GBAS_FIX, "rtk_fixed"),
    ({"carr_soln": ubx.CARR_FLOAT}, ubx.STATUS_GBAS_FIX, "rtk_float"),
    ({"carr_soln": ubx.CARR_NONE}, ubx.STATUS_SBAS_FIX, "single"),
    ({"carr_soln": ubx.CARR_NONE, "diff_soln": False}, ubx.STATUS_FIX, "single"),
    ({"gnss_fix_ok": False}, ubx.STATUS_NO_FIX, "none"),
    ({"fix_type": ubx.TIME_ONLY}, ubx.STATUS_NO_FIX, "none"),
    ({"invalid_llh": True}, ubx.STATUS_NO_FIX, "none"),
])
def test_fix_status(changes, status, name):
    assert ubx.fix_status(pvt(**changes)) == status
    assert ubx.solution_name(pvt(**changes)) == name


def test_covariance_is_the_receivers_own_estimate():
    east, north, up = ubx.covariance_enu(pvt(h_acc_m=0.5, v_acc_m=1.0))
    assert (east, north, up) == (0.25, 0.25, 1.0)


def test_there_is_no_way_to_configure_the_receiver():
    # The robot must never write CFG to the kit's F9P (ubx.py docstring).
    assert not [name for name in dir(ubx) if "cfg" in name.lower() and name != "CFG_CLASS"]


# --- RTCM ----------------------------------------------------------------------

def test_crc24q_reference_value():
    assert rtcm.crc24q(b"123456789") == 0xCDE703


def test_rtcm_frames_and_types():
    stream = (b"<html>503</html>" + rtcm.make_message(1005) + rtcm.make_message(1077, 200)
              + rtcm.make_message(1230))
    assert [t for t, _ in rtcm.frames(stream)] == [1005, 1077, 1230]


def test_rtcm_corrupt_frame_is_dropped_and_parser_resyncs():
    bad = bytearray(rtcm.make_message(1077, 100))
    bad[50] ^= 0x10
    parser = rtcm.RtcmParser()
    got = list(parser.feed(bytes(bad) + rtcm.make_message(1087)))
    assert [t for t, _ in got] == [1087] and parser.bad_crcs == 1


# --- NTRIP ---------------------------------------------------------------------

def test_request_v2_with_auth():
    text = ntrip.request(ntrip.Caster("tusaga.example", 2101, "VRSRTCM31", "user", "pw")).decode()
    assert text.startswith("GET /VRSRTCM31 HTTP/1.1\r\n")
    assert "Ntrip-Version: Ntrip/2.0" in text and "Authorization: Basic dXNlcjpwdw==" in text
    assert text.endswith("\r\n\r\n")


def test_request_v1():
    text = ntrip.request(ntrip.Caster("c", mountpoint="M", version=1)).decode()
    assert text.startswith("GET /M HTTP/1.0\r\n") and "Ntrip-Version" not in text


def test_responses():
    assert ntrip.parse_response(b"ICY 200") is None
    v1 = ntrip.parse_response(b"ICY 200 OK\r\n\xd3\x00")
    assert v1 and v1.body_start == b"\xd3\x00" and not v1.chunked
    assert ntrip.parse_response(b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data") is None
    v2 = ntrip.parse_response(b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n5\r\n")
    assert v2.chunked and v2.body_start == b"5\r\n"


@pytest.mark.parametrize("answer, why", [
    (b"SOURCETABLE 200 OK\r\nServer: x\r\n\r\nSTR;...", "source table"),
    (b"HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\n", "source table"),
    (b"HTTP/1.1 401 Unauthorized\r\n\r\n", "password"),
    (b"HTTP/1.1 404 Not Found\r\n\r\n", "mountpoint"),
    (b"<html><body>nope</body></html>\r\n\r\n", "not an NTRIP answer"),
])
def test_refusals_are_named(answer, why):
    with pytest.raises(ntrip.NtripError, match=why):
        ntrip.parse_response(answer)


def test_dechunker_across_arbitrary_splits():
    payload = rtcm.make_message(1005) + rtcm.make_message(1077, 300)
    body = b""
    for i in range(0, len(payload), 100):
        piece = payload[i:i + 100]
        body += f"{len(piece):x}\r\n".encode() + piece + b"\r\n"
    body += b"0\r\n\r\n"
    dechunker = ntrip.Dechunker()
    out = b"".join(dechunker.feed(body[i:i + 3]) for i in range(0, len(body), 3))
    assert out == payload and dechunker.finished


def test_gga_sentence():
    line = ntrip.gga(37.0662123, 37.3833456, 880.1, (9, 5, 7.25), quality=1, num_sv=22)
    text = line.decode()
    assert text.startswith("$GPGGA,090507.25,3703.9727,N,03723.0007,E,1,22,1.0,880.1,M,")
    body, checksum = text[1:].strip().split("*")
    assert ntrip.nmea_checksum(body) == checksum
    south = ntrip.gga(-33.5, -70.25, 0.0, (0, 0, 0.0)).decode()
    assert ",3330.0000,S,07015.0000,W," in south


def test_gga_minutes_never_print_as_sixty():
    text = ntrip.gga(36.9999999999, 37.0, 0.0, (0, 0, 0.0)).decode()
    assert "3700.0000,N" in text


def test_backoff_grows_and_caps():
    assert [ntrip.backoff_s(i) for i in range(7)] == [2, 4, 8, 16, 32, 60, 60]


# --- the fake F9P --------------------------------------------------------------

def horizontal_error_m(fix, lat, lon):
    north = math.radians(fix.lat_deg - lat) * f9p_sim.EARTH_RADIUS_M
    east = math.radians(fix.lon_deg - lon) * f9p_sim.EARTH_RADIUS_M * math.cos(math.radians(lat))
    return math.hypot(east, north)


def test_f9p_solution_follows_the_corrections():
    sim = f9p_sim.F9pSim(seed=1, time_to_fix_s=8.0, rtcm_timeout_s=10.0)
    lat, lon, t0 = 37.0, 37.0, 1.8e9
    errors = {"single": [], "rtk_fixed": []}
    for i in range(600):                                   # 120 s at 5 Hz
        t = t0 + i * 0.2
        if 150 <= i < 450 and i % 5 == 0:                  # corrections from 30 s to 90 s
            sim.feed(rtcm.make_message(1077), t)
        fix = sim.nav_pvt(t, lat, lon, 880.0)
        name = ubx.solution_name(fix)
        if i == 160:
            assert name == "rtk_float"
        if name in errors:
            errors[name].append(horizontal_error_m(fix, lat, lon))
    assert ubx.solution_name(fix) == "single", "corrections stopped 30 s ago"
    fixed_rms = math.sqrt(sum(e * e for e in errors["rtk_fixed"]) / len(errors["rtk_fixed"]))
    assert fixed_rms < 0.03
    assert max(errors["single"]) > 0.3
    assert sim.cfg_writes == 0 and sim.rtcm_frames == 60


def test_f9p_counts_configuration_attempts():
    sim = f9p_sim.F9pSim()
    sim.feed(ubx.frame(0x06, 0x8A, b"\x00\x01\x00\x00" + b"\x01\x00\x91\x20\x01"), 0.0)
    assert sim.cfg_writes == 1


def test_f9p_garbage_is_not_corrections():
    sim = f9p_sim.F9pSim()
    for i in range(100):
        sim.feed(b"<html>503 Service Unavailable</html>", float(i))
    assert sim.solution_at(100.0) is f9p_sim.SINGLE and sim.rtcm_frames == 0
