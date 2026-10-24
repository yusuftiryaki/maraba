"""An obstacle the bumper does not register: the wheels turn, the robot does
not move. jam_monitor must say so, mission_runner must stop the leg, and
nothing may be mowed outside the lawn. Skips without ROS, ~6 min.

docs/missions.md, "Patinaj sonrası alan dışında biçme": in CI a robot whose
wheels turned against a rock let odometry drift, and the runner — driving rows
in `odom` — mowed 3 m outside the lawn. Before jam.py the only net under a
jam the bumper misses was the leg's time budget (60 s + 10 s/m): minutes of
spinning wheels, odometry drifting all the while.

The obstacle sits ON a row (rows at y = -1.24 ... 0.36 ... 0.4 apart; one at
-0.04) with bumper_from_obstacles off — a stone under the bumper's edge, a
root. ⚠️ The first version put a rock SIDE-ON to a row instead (outside the
bumper's +-60 deg): that band is ~8 cm wide for a round robot, the row passed
0.59 m from the rock against 0.60 m contact, and in CI the robot never
touched it. Head-on contact is certain; only the missing bumper makes it a
jam. Quiet GPS: this measures the detector and the runner, not the NEO-6M
(test_mission_coverage's job: zero alarms).
"""

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from test_mission_coverage import QUIET_GPS, run_mission   # noqa: E402

ROCK_X, ROCK_Y, ROCK_R = 4.0, -0.04, 0.2
SIM_ARGS = (*QUIET_GPS,
            "-p", f"unsurveyed_obstacle_centers:=[{ROCK_X}, {ROCK_Y}]",
            "-p", f"unsurveyed_obstacle_radii:=[{ROCK_R}]",
            "-p", "bumper_from_obstacles:=false")


def test_a_jam_the_bumper_misses_is_caught_and_nothing_is_mown_outside(site_nav2_stack,
                                                                       tmp_path):
    coverage, overrun, status = run_mission(site_nav2_stack, tmp_path, SIM_ARGS,
                                            {"overlap": 0.2})
    assert status["state"] == "DONE", status
    assert status["jams"], "the rock held the robot, and nobody noticed"
    assert overrun <= 0.15, f"cut {overrun:.2f} m outside the lawn after the jam"
    # Unlike the bumper, a jam leaves no detection on the map: every row
    # through the rock is tried and skipped. MEASURED below; bound loose.
    assert coverage >= 0.5, f"only {coverage:.1%} cut"
