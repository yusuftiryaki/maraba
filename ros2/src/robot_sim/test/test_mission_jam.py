"""A rock met SIDE-ON: the front bumper never presses, the wheels turn, the
robot does not move. jam_monitor must say so, mission_runner must stop the
leg, and nothing may be mowed outside the lawn. Skips without ROS, ~6 min.

docs/missions.md, "Patinaj sonrası alan dışında biçme": in CI a robot whose
wheels turned against a rock let odometry drift, and the runner — driving rows
in `odom` — mowed 3 m outside the lawn. Before jam.py the only net under a
side-on jam was the leg's time budget (60 s + 10 s/m): minutes of spinning
wheels, odometry drifting all the while.

Geometry: rows at y = -1.2 ... 1.2, 0.4 apart (CUT_WIDTH 0.5, overlap 0.2),
driven from the south. Rock at (4.0, 0.55), radius 0.2: the row at y = 0
passes 0.55 m from its centre — inside contact (0.4 + 0.2 = 0.6) but at
asin(0.55 / 0.6) = 66 deg off the heading, outside the front bumper's
+-60 deg. So: no press, a jam. Quiet GPS: this measures the detector and the
runner, not the NEO-6M (that is test_mission_coverage's job: zero alarms).
"""

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from test_mission_coverage import QUIET_GPS, run_mission   # noqa: E402

ROCK_X, ROCK_Y, ROCK_R = 4.0, 0.55, 0.2
SIM_ARGS = (*QUIET_GPS,
            "-p", f"unsurveyed_obstacle_centers:=[{ROCK_X}, {ROCK_Y}]",
            "-p", f"unsurveyed_obstacle_radii:=[{ROCK_R}]",
            "-p", "bumper_from_obstacles:=true")


def test_a_side_on_jam_is_caught_and_nothing_is_mown_outside(site_nav2_stack, tmp_path):
    coverage, overrun, status = run_mission(site_nav2_stack, tmp_path, SIM_ARGS,
                                            {"overlap": 0.2})
    assert status["jams"], "the side-on rock held the robot, and nobody noticed"
    assert overrun <= 0.15, f"cut {overrun:.2f} m outside the lawn after the jam"
    # A rock costs its neighbourhood (test_mission_rock: ~78%); a jam must not
    # cost much more than that.
    assert coverage >= 0.6, f"only {coverage:.1%} cut"
