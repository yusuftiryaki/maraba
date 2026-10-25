"""dagger_batch.Takeover: when the expert takes the wheel. Needs ROS (imports)."""

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from robot_sim.dagger_batch import Takeover   # noqa: E402

SPIN, AHEAD = (0.0, 1.0), (0.27, 0.1)


def feed(takeover, start, seconds, label, hz=20):
    changes = []
    for k in range(int(seconds * hz)):
        if takeover.label(start + k / hz, *label):
            changes.append((round(start + k / hz, 2), takeover.expert_drives))
    return changes


def test_the_expert_takes_over_a_long_spin_and_hands_back_after_driving():
    t = Takeover(3.0, 2.0)
    assert feed(t, 0.0, 2.9, SPIN) == [], "a corner turn is not a takeover"
    assert feed(t, 2.9, 1.0, AHEAD) == []
    assert feed(t, 10.0, 4.0, SPIN) == [(13.0, True)]
    assert feed(t, 14.0, 3.0, AHEAD) == [(16.0, False)]
    assert t.count == 1


def test_off_means_never():
    t = Takeover(0.0, 2.0)
    assert feed(t, 0.0, 60.0, SPIN) == []
