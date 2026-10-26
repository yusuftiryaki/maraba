"""Long rows in `odom` and the gyro's drift (mission_runner._cut_pieces).

ekf_local's yaw is integrated gyro, so odom turns against the world at the
gyro's residual bias (sim_node: 0.1°/s, imu_gyro_residual_bias_dps) and a
straight line in odom is an arc on the ground — lateral error growing with
the square of the row's length. The orchard scan measured 0.95 m at the end of
16.6 m before its rows moved to `map`; mowing with the NEO-6M still drives in
odom. The runner now drives a row in chunks of max_leg_m, re-aimed between
them by map->odom's yaw (the compass).

Same 18 m lawn, quiet GPS (so what is measured is the drift, not the GPS),
whole rows (max_leg_m 0, the old behaviour) against 4 m chunks: the true
track's distance from the row lines while cutting.

CI stage `mission`, ~6 min.
"""

import json
import os
import subprocess


import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")



from robot_missions.planner import row_offsets          # noqa: E402
from test_mission_coverage import (QUIET_GPS, Watcher, lonlat,  # noqa: E402
                                   start_mission)

LAWN = [(1.0, -0.75), (19.0, -0.75), (19.0, 0.75), (1.0, 0.75)]     # 18 x 1.5 m
CUT_WIDTH = 0.5


def long_lawn():
    ring = [lonlat(*c) for c in LAWN] + [lonlat(*LAWN[0])]
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": "area", "name": "uzun"}}


def drive(site_nav2_stack, tmp_path, max_leg):
    stack = site_nav2_stack([long_lawn()], *QUIET_GPS)
    runner = subprocess.Popen(
        ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
         "-p", f"site_file:={stack.site_file}",
         "-p", f"state_file:={tmp_path / f'state_{max_leg}.json'}",
         "-p", f"cut_width:={CUT_WIDTH}", "-p", f"max_leg_m:={max_leg}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    stack._procs.append(runner)
    watcher = Watcher()
    try:
        assert watcher.spin_for(20.0, lambda: watcher.status is not None)
        start_mission(watcher, {"type": "coverage", "area": "uzun",
                                "params": {"overlap": 0.0, "perimeter_pass": False,
                                           "angle_deg": 90.0}})
        assert watcher.spin_for(900.0, lambda: watcher.status
                                and watcher.status["state"] in ("DONE", "FAILED", "PAUSED"))
        assert watcher.status["state"] == "DONE", json.dumps(watcher.status)
        if os.environ.get("LONG_ROWS_DUMP"):      # for looking at a run by hand
            with open(f"{os.environ['LONG_ROWS_DUMP']}_{max_leg}.json", "w") as handle:
                json.dump(watcher.track, handle)
        rows = row_offsets(LAWN[0][1] + CUT_WIDTH / 2, LAWN[2][1] - CUT_WIDTH / 2, CUT_WIDTH)
        off = sorted(min(abs(y - r) for r in rows) for x, y, c in watcher.track if c)
        return off[int(0.95 * (len(off) - 1))], off[-1]
    finally:
        watcher.destroy_node()
        stack.close()


def test_long_rows_hold_their_line_in_chunks(site_nav2_stack, tmp_path):
    whole = drive(site_nav2_stack, tmp_path, 0.0)
    chunked = drive(site_nav2_stack, tmp_path, 4.0)
    print(f"off the row lines on an 18 m lawn: whole rows p95 {whole[0]:.3f} m "
          f"(max {whole[1]:.3f}), 4 m chunks p95 {chunked[0]:.3f} m (max {chunked[1]:.3f})")
    # measured (2026-10-26, two runs): whole rows p95 0.86-1.26 m (the arc, and
    # the next row laid out from its drifted end), 4 m chunks 0.39-0.49 m.
    # What is left is the compass's own error (±1°, test_map_heading) and a
    # few cm per chunk.
    assert chunked[0] < 0.7 * whole[0], "chunking did not help"
    assert chunked[0] < 0.6
