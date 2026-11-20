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


def row_errors(track, rows):
    """Each cutting point's distance from ITS OWN row line.

    ⚠️ Not from the NEAREST row line: on this lawn the two rows are 0.5 m apart,
    so that distance tops out at 0.25 m between them — a row that drifted
    onto its neighbour read as on its line. CI, 2026-11-12: whole rows "p95
    0.233 m, max 0.250 m" (the drift measured locally on the same code:
    1.18 m) and "chunking did not help" failed a run where it had.

    Rows run along x in boustrophedon order, so each row is driven the other
    way from the one before: the cutting stretches are grouped by direction
    (a row driven in chunks, or a blip of the trimmer, stays one group) and
    the k-th group is the k-th row from the edge row the first one started on."""
    stretches, current = [], []
    for x, y, cutting in track:
        if cutting:
            current.append((x, y))
        elif current:
            stretches.append(current)
            current = []
    if current:
        stretches.append(current)
    groups = []                                     # [direction, points]
    for stretch in stretches:
        if abs(stretch[-1][0] - stretch[0][0]) < 0.3:
            continue                                # a blip, not a row
        direction = 1 if stretch[-1][0] > stretch[0][0] else -1
        if groups and groups[-1][0] == direction:
            groups[-1][1].extend(stretch)
        else:
            groups.append([direction, list(stretch)])
    assert len(groups) == len(rows), (
        f"{len(groups)} cutting runs for {len(rows)} rows — cannot tell which row is which")
    first = min(rows, key=lambda r: abs(r - groups[0][1][0][1]))
    order = sorted(rows, reverse=first != min(rows))
    return [abs(y - row) for (_, points), row in zip(groups, order) for _, y in points]


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
        off = sorted(row_errors(watcher.track, rows))
        return off[int(0.95 * (len(off) - 1))], off[-1]
    finally:
        watcher.destroy_node()
        stack.close()


def test_long_rows_hold_their_line_in_chunks(site_nav2_stack, tmp_path):
    whole = drive(site_nav2_stack, tmp_path, 0.0)
    chunked = drive(site_nav2_stack, tmp_path, 4.0)
    print(f"off their own row lines on an 18 m lawn: whole rows p95 {whole[0]:.3f} m "
          f"(max {whole[1]:.3f}), 4 m chunks p95 {chunked[0]:.3f} m (max {chunked[1]:.3f})")
    # measured (2026-10-26, two runs): whole rows p95 0.86-1.26 m (the arc, and
    # the next row laid out from its drifted end), 4 m chunks 0.39-0.49 m —
    # those against the NEAREST row line; row_errors() measures against the
    # row's own line, which no longer hides a drift onto the neighbour.
    # What is left is the compass's own error (±1°, test_map_heading) and a
    # few cm per chunk.
    assert chunked[0] < 0.7 * whole[0], "chunking did not help"
    assert chunked[0] < 0.6
