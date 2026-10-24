"""jam.JamDetector on synthetic signals. ROS-free, stepped clock.

Both directions matter equally: the jams must be caught, and a NEO-6M's
noise must NEVER look like one — the previous attempt at this guard
(map->odom watching) fired 273 times in one honest coverage run.
"""

import math
import random

import pytest

from robot_missions.jam import JamConfig, JamDetector
from robot_missions.site import LatLon

LAT0, LON0 = 41.0, 29.0
M_LAT = 1 / 111_195.0                                  # degrees per metre north
M_LON = 1 / (111_195.0 * math.cos(math.radians(LAT0)))


def at(east, north):
    return LatLon(LAT0 + north * M_LAT, LON0 + east * M_LON)


def run(detector, seconds, v, w, gyro, pos, vel=(0.0, 0.0), gps_noise=0.0, wander=None,
        t0=0.0, seed=1):
    """Feed 50 Hz wheels/gyro and 5 Hz GPS for `seconds`. pos/vel: the TRUE
    motion (metres, m/s). Returns the verdicts, one per 0.2 s."""
    rng = random.Random(seed)
    verdicts = []
    x, y = pos
    for k in range(int(seconds * 50)):
        t = t0 + k / 50.0
        detector.wheel(t, v, w)
        detector.gyro(t, gyro)
        x, y = x + vel[0] / 50.0, y + vel[1] / 50.0
        if k % 10 == 0:
            ex, ey = wander(t) if wander else (0.0, 0.0)
            detector.gps(t, at(x + ex + rng.gauss(0, gps_noise), y + ey + rng.gauss(0, gps_noise)))
            verdicts.append(detector.check(t))
    return verdicts


def test_driving_honestly_is_not_a_jam():
    d = JamDetector()
    out = run(d, 30.0, v=0.3, w=0.0, gyro=0.0, pos=(0, 0), vel=(0.3, 0.0))
    assert not any(out)


def test_wheels_turning_while_the_robot_does_not_is_a_turn_jam():
    d = JamDetector()
    out = run(d, 3.0, v=0.0, w=0.6, gyro=0.0, pos=(0, 0))
    first = next(i for i, v in enumerate(out) if v)
    assert out[first]["kind"] == JamDetector.TURN
    assert 1.0 <= first * 0.2 <= 1.4, "after turn_hold_s, not before"


def test_a_real_turn_on_the_spot_is_not_a_jam():
    d = JamDetector()
    assert not any(run(d, 10.0, v=0.0, w=0.6, gyro=0.58, pos=(0, 0)))


def test_wheels_driving_while_the_gps_stands_still_is_a_stall():
    d = JamDetector()
    out = run(d, 20.0, v=0.3, w=0.0, gyro=0.0, pos=(0, 0), vel=(0.0, 0.0))
    first = next(i for i, v in enumerate(out) if v)
    assert out[first]["kind"] == JamDetector.STALL
    assert first * 0.2 <= 13.0, "window (10 s) + hold (2 s)"


@pytest.mark.parametrize("seed", range(6))
def test_a_stall_under_neo6m_noise_is_still_caught_within_40_s(seed):
    """The price of zero false alarms: with 1 m white noise the averaged GPS
    end points scatter ~0.45 m, so a true stall reads as one only part of the
    time. MEASURED first detection 12-38 s (seeds 0-5); mission_runner acts
    on the first. A turn slip (the CI incident) is caught by the gyro in 1 s."""
    d = JamDetector()
    out = run(d, 40.0, v=0.3, w=0.0, gyro=0.0, pos=(0, 0), gps_noise=1.0, seed=seed)
    assert any(v and v["kind"] == JamDetector.STALL for v in out)


def test_standing_still_or_turning_on_the_spot_never_stalls():
    d = JamDetector()
    assert not any(run(d, 30.0, v=0.0, w=0.0, gyro=0.0, pos=(0, 0), gps_noise=1.0))
    assert not any(run(d, 30.0, v=0.02, w=0.5, gyro=0.5, pos=(0, 0), gps_noise=1.0, t0=100))


@pytest.mark.parametrize("seed", range(10))
def test_neo6m_noise_and_wander_on_an_honest_drive_never_fire(seed):
    """sim_node's NEO-6M: 1 m white noise + 2 m wander (tau 60 s). Ten
    minutes of rows, back and forth at 0.3 m/s, with turns."""
    d = JamDetector()
    phase = random.Random(seed).uniform(0, 6.28)
    wander = lambda t: (2.0 * math.sin(t / 60.0 + phase), 2.0 * math.cos(t / 47.0 + phase))
    fired, t, x = 0, 0.0, 0.0
    for leg in range(40):
        direction = 1 if leg % 2 == 0 else -1
        fired += sum(1 for v in run(d, 12.0, v=0.3, w=0.0, gyro=0.0, pos=(x, leg * 0.4),
                                    vel=(0.3 * direction, 0.0), gps_noise=1.0, wander=wander,
                                    t0=t, seed=seed * 100 + leg) if v)
        t += 12.0
        x += 3.6 * direction
        fired += sum(1 for v in run(d, 6.0, v=0.0, w=0.52, gyro=0.5, pos=(x, leg * 0.4),
                                    gps_noise=1.0, wander=wander, t0=t, seed=seed * 100 + leg + 50)
                     if v)
        t += 6.0
    assert fired == 0


def test_stale_signals_claim_nothing():
    d = JamDetector()
    d.wheel(0.0, 0.0, 0.6)
    d.gyro(0.0, 0.0)
    assert d.check(5.0) is None


def test_a_stall_says_which_way_the_wheels_were_pushing():
    ahead = JamDetector()
    out = run(ahead, 20.0, v=0.3, w=0.0, gyro=0.0, pos=(0, 0))
    assert next(v for v in out if v)["forward"] is True
    behind = JamDetector()
    out = run(behind, 20.0, v=-0.3, w=0.0, gyro=0.0, pos=(0, 0))
    assert next(v for v in out if v)["forward"] is False
