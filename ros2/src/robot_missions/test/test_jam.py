"""jam.JamDetector on synthetic signals. ROS-free, stepped clock.

Both directions matter equally: the jams must be caught, and a NEO-6M's
noise must NEVER look like one — the previous attempt at this guard
(map->odom watching) fired 273 times in one honest coverage run.
"""

import math
import random

import pytest

from robot_missions.jam import JamDetector

VEL_NOISE = 0.1                                        # sim_node's gps_vel_noise_sigma


def run(detector, seconds, v, w, gyro, yaw=0.0, vel=None, vel_noise=0.0, gps_hz=5,
        t0=0.0, seed=1):
    """Feed 50 Hz wheels/gyro and gps_hz Doppler velocity for `seconds`. v, w:
    what the wheels claim, from heading `yaw` (turning at w). vel: the TRUE
    ground velocity (east, north m/s); None = the wheels are honest. Returns
    the verdicts, one per 0.2 s. Ends with the heading it reached."""
    rng = random.Random(seed)
    verdicts = []
    for k in range(int(seconds * 50)):
        t = t0 + k / 50.0
        detector.wheel(t, v, w, yaw)
        detector.gyro(t, gyro)
        if k % (50 // gps_hz) == 0:
            east, north = vel if vel is not None else (v * math.cos(yaw), v * math.sin(yaw))
            detector.gps(t, east + rng.gauss(0, vel_noise), north + rng.gauss(0, vel_noise))
        if k % 10 == 0:
            verdicts.append(detector.check(t))
        yaw += w / 50.0
    return verdicts


def test_driving_honestly_is_not_a_jam():
    d = JamDetector()
    out = run(d, 30.0, v=0.3, w=0.0, gyro=0.0)
    assert not any(out)


def test_wheels_turning_while_the_robot_does_not_is_a_turn_jam():
    d = JamDetector()
    out = run(d, 3.0, v=0.0, w=0.6, gyro=0.0)
    first = next(i for i, v in enumerate(out) if v)
    assert out[first]["kind"] == JamDetector.TURN
    assert 1.0 <= first * 0.2 <= 1.4, "after turn_hold_s, not before"


def test_a_real_turn_on_the_spot_is_not_a_jam():
    d = JamDetector()
    assert not any(run(d, 10.0, v=0.0, w=0.6, gyro=0.58))


def test_wheels_driving_while_the_gps_stands_still_is_a_stall():
    d = JamDetector()
    out = run(d, 20.0, v=0.3, w=0.0, gyro=0.0, vel=(0.0, 0.0))
    first = next(i for i, v in enumerate(out) if v)
    assert out[first]["kind"] == JamDetector.STALL
    assert first * 0.2 <= 5.4, "window (3 s) + hold (2 s)"


@pytest.mark.parametrize("seed", range(6))
def test_a_stall_under_neo6m_noise_is_caught_within_6_s(seed):
    d = JamDetector()
    out = run(d, 20.0, v=0.3, w=0.0, gyro=0.0, vel=(0.0, 0.0), vel_noise=VEL_NOISE, seed=seed)
    first = next(i for i, v in enumerate(out) if v)
    assert out[first]["kind"] == JamDetector.STALL and first * 0.2 <= 6.0


def test_a_stall_is_caught_at_the_neo6ms_factory_1_hz_too():
    d = JamDetector()
    out = run(d, 30.0, v=0.3, w=0.0, gyro=0.0, vel=(0.0, 0.0), vel_noise=VEL_NOISE, gps_hz=1)
    assert any(v and v["kind"] == JamDetector.STALL for v in out)


def test_standing_still_or_turning_on_the_spot_never_stalls():
    d = JamDetector()
    assert not any(run(d, 30.0, v=0.0, w=0.0, gyro=0.0, vel_noise=VEL_NOISE))
    assert not any(run(d, 30.0, v=0.02, w=0.5, gyro=0.5, vel_noise=VEL_NOISE, t0=100))


def test_slipping_on_wet_grass_is_not_a_stall():
    """30 % of the wheels' speed lost: odometry error, not an obstacle."""
    d = JamDetector()
    assert not any(run(d, 30.0, v=0.3, w=0.0, gyro=0.0, vel=(0.21, 0.0), vel_noise=VEL_NOISE))


@pytest.mark.parametrize("seed", range(10))
def test_neo6m_noise_on_an_honest_drive_never_fires(seed):
    """Ten minutes of 4 m rows, back and forth at 0.3 m/s, with turns on the
    spot and arcs. ⚠️ The position-based check this replaced fired here under
    sim_node's wander (jam.py); Doppler velocity carries none of it."""
    d = JamDetector()
    fired, t, yaw = 0, 0.0, 0.0
    for leg in range(40):
        fired += sum(1 for v in run(d, 13.0, v=0.3, w=0.0, gyro=0.0, yaw=yaw,
                                    vel_noise=VEL_NOISE, t0=t, seed=seed * 100 + leg) if v)
        t += 13.0
        w = 0.52 if leg % 2 else 0.3          # on the spot, or an arc
        out = run(d, math.pi / w, v=0.0 if leg % 2 else 0.15, w=w, gyro=w, yaw=yaw,
                  vel_noise=VEL_NOISE, t0=t, seed=seed * 100 + leg + 50)
        fired += sum(1 for v in out if v)
        t += math.pi / w
        yaw += math.pi
    assert fired == 0


def test_stale_signals_claim_nothing():
    d = JamDetector()
    d.wheel(0.0, 0.0, 0.6, 0.0)
    d.gyro(0.0, 0.0)
    assert d.check(5.0) is None


def test_a_stall_says_which_way_the_wheels_were_pushing():
    ahead = JamDetector()
    out = run(ahead, 20.0, v=0.3, w=0.0, gyro=0.0, vel=(0.0, 0.0))
    assert next(v for v in out if v)["forward"] is True
    behind = JamDetector()
    out = run(behind, 20.0, v=-0.3, w=0.0, gyro=0.0, vel=(0.0, 0.0))
    assert next(v for v in out if v)["forward"] is False
