"""gate.PolicyGate: what of a policy's output may reach the wheels. ROS-free,
stepped clock. Every rule in the module's docstring has a test here."""

import math

import pytest

from robot_learning.gate import GateConfig, PolicyGate

AREA = [(0, 0), (6, 0), (6, 4), (0, 4)]
DT = 0.05                                       # policy_gate ticks at 20 Hz


def armed(budget=60.0, now=0.0, **config):
    gate = PolicyGate(GateConfig(**config))
    gate.arm(AREA, budget, now)
    return gate


def drive(gate, v, w, seconds, t0, robot=(3.0, 2.0), obs_lag=0.05, send=True):
    """Policy sends (v, w) every tick; returns the outputs."""
    out, t = [], t0
    for _ in range(int(round(seconds / DT))):
        if send:
            gate.command(v, w, t - obs_lag, t)
        out.append(gate.tick(robot, t))
        t += DT
    return out, t


def test_disarmed_is_silent_whatever_the_policy_says():
    gate = PolicyGate()
    gate.command(1.0, 1.0, 0.0, 0.0)
    assert gate.tick((3, 2), 0.0) is None and gate.reason == "disarmed"


def test_armed_output_is_clamped_and_ramped():
    gate = armed()
    out, _ = drive(gate, 5.0, -5.0, 2.0, t0=0.0)
    v, w = zip(*out)
    assert max(v) == pytest.approx(0.3) and min(w) == pytest.approx(-0.8)
    steps = [b - a for a, b in zip(v, v[1:])]
    assert max(steps) <= 0.5 * DT + 1e-9, "no faster than max_accel"
    assert v[0] <= 0.5 * DT + 1e-9


def test_a_stale_command_or_observation_brings_it_to_zero_holding_the_mux():
    gate = armed()
    _, t = drive(gate, 0.3, 0.0, 2.0, t0=0.0)
    out, t = drive(gate, 0.3, 0.0, 1.5, t0=t, send=False)        # policy went quiet
    assert out[-1] == (0.0, 0.0) and gate.reason == "command stale"
    out, t = drive(gate, 0.3, 0.0, 2.0, t0=t, obs_lag=0.8)       # acting on old pictures
    assert out[-1] == pytest.approx((0.0, 0.0)) and "observation" in gate.reason


def test_a_policy_silent_for_the_lease_releases_the_mux():
    gate = armed()
    _, t = drive(gate, 0.3, 0.0, 1.0, t0=0.0)
    out, _ = drive(gate, 0.0, 0.0, 2.5, t0=t, send=False)
    assert out[-1] is None and gate.armed, "released, not disarmed"


def test_leaving_the_area_disarms():
    gate = armed()
    assert gate.tick((6.9, 2.0), 0.0) is not None                  # within the 1 m margin
    assert gate.tick((7.1, 2.0), 0.05) == (0.0, 0.0)
    assert not gate.armed and "left the area" in gate.reason
    assert gate.tick((3, 2), 0.1) is None


def test_a_jam_disarms():
    gate = armed()
    drive(gate, 0.3, 0.0, 1.0, t0=0.0)
    gate.jam("stall: wheels 2 m, GPS 0 m")
    assert not gate.armed and gate.reason.startswith("jammed")


def test_the_budget_runs_out():
    gate = armed(budget=1.0)
    out, _ = drive(gate, 0.3, 0.0, 1.5, t0=0.0)
    assert not gate.armed and gate.reason == "time budget used up"
    assert out[-1] is None


def test_no_pose_means_no_motion():
    gate = armed()
    out, _ = drive(gate, 0.3, 0.0, 1.0, t0=0.0, robot=None)
    assert all(o == (0.0, 0.0) for o in out)


def test_nan_commands_are_ignored():
    gate = armed()
    gate.command(float("nan"), 0.0, 0.0, 0.0)
    gate.command(0.2, float("inf"), 0.0, 0.0)
    assert gate.tick((3, 2), 0.0) == (0.0, 0.0) and gate.reason == "armed, no command yet"


@pytest.mark.parametrize("area, budget", [([(0, 0), (1, 0)], 10.0),
                                          ([(0, 0), (2, 2), (2, 0), (0, 2)], 10.0),
                                          (AREA, 0.0), (AREA, math.inf)])
def test_nonsense_arming_is_refused(area, budget):
    with pytest.raises(ValueError):
        PolicyGate().arm(area, budget, 0.0)
