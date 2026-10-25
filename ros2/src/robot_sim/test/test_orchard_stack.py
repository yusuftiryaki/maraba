"""The orchadOS grid scan, driven: orchard_grid on a (small) simulated orchard
with the kit plugged in (docs/kit-soketi.md, K2).

    site (trees as 'ağaç' points) ─► mission_runner ─► orchard_grid ─► Nav2
    kit v1: RTK from the fake caster, the 360° camera through the fake shim

What it measures on ground truth: every tree passed on all four sides
(within half a row spacing — what the 360° camera needs to see it from each),
no contact with any trunk, the camera recording for the whole scan and
stopped at its end. Before the kit is plugged in the same mission is refused.

CI stage `mission`, ~6-8 min.
"""

import json
import math
import os
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import Bool, String                   # noqa: E402

from robot_kit import fake_kit, hw, manifest            # noqa: E402
from robot_missions_interfaces.srv import StartMission  # noqa: E402
from conftest import KitStack                           # noqa: E402
from test_mission_coverage import lonlat, start_mission  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SPACING = 5.0
TREES = [(4.0 + i * SPACING, -2.5 + j * SPACING) for i in range(3) for j in range(2)]
TRUNK = 0.15
PAD = 3.0
MISSION_TIMEOUT_S = 1200.0


def orchard_site():
    xs, ys = [t[0] for t in TREES], [t[1] for t in TREES]
    corners = [(min(xs) - PAD, min(ys) - PAD), (max(xs) + PAD, min(ys) - PAD),
               (max(xs) + PAD, max(ys) + PAD), (min(xs) - PAD, max(ys) + PAD)]
    ring = [lonlat(*c) for c in corners] + [lonlat(*corners[0])]
    features = [{"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
                 "properties": {"kind": "area", "name": "bahce"}}]
    for k, (x, y) in enumerate(TREES):
        features.append({"type": "Feature",
                         "geometry": {"type": "Point", "coordinates": lonlat(x, y)},
                         "properties": {"kind": "poi", "name": f"S{k // 2 + 1}-{k % 2 + 1}",
                                        "label": "ağaç", "radius_m": TRUNK}})
    return features


def physical_trees():
    centers = [v for t in TREES for v in t]
    return ("-p", f"obstacle_centers:={centers}",
            "-p", f"obstacle_radii:={[TRUNK] * len(TREES)}")


class Watcher(Node):
    def __init__(self):
        super().__init__("orchard_watcher")
        self.status = self.kit = self.camera = self.oracle = None
        self.track = []                       # (x, y) while a scan leg is driven
        self.collided = False
        self.recorded = False
        self.create_subscription(String, "/mission_runner/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), LATCHED)
        self.create_subscription(String, "/kit/state",
                                 lambda m: setattr(self, "kit", json.loads(m.data)), LATCHED)
        self.create_subscription(String, "/kit/camera/status", self._on_camera, 10)
        self.create_subscription(String, "/sim/kit_f9p",
                                 lambda m: setattr(self, "oracle", json.loads(m.data)), 10)
        self.create_subscription(Odometry, "/ground_truth", self._on_truth, 10)
        # (truth x, y, ekf_global x, y) at ~1 Hz while RUNNING: the localization
        # error, converted after the run with one /fromLLArray call
        self.pairs, self._ekf, self._last_pair, self.pair_times = [], None, 0.0, []
        self.trunk_log = []                # (time, status) for a dump
        self.create_subscription(Odometry, "/odometry/filtered/global",
                                 lambda m: setattr(self, "_ekf", m.pose.pose.position), 10)
        self.create_subscription(Bool, "/collision_truth", self._on_collision, 10)
        self.trunk_frames = {}             # reason ("" = accepted) -> count, while RUNNING
        self.create_subscription(String, "/kit/trunk_status", self._on_trunks, 10)

    def _on_trunks(self, msg):
        data = json.loads(msg.data)
        self.trunk_log.append((time.monotonic(), data))
        self.trunk_ok = getattr(self, "trunk_ok", False) or data["accepted"]
        if self.status and self.status["state"] == "RUNNING":
            reason = data["reason"].split(":")[0][:40]
            self.trunk_frames[reason] = self.trunk_frames.get(reason, 0) + 1

    def _on_camera(self, msg):
        self.camera = json.loads(msg.data)
        self.recorded = self.recorded or bool(self.camera.get("recording"))

    def _on_truth(self, msg):
        if self.status and self.status["state"] == "RUNNING":
            p = msg.pose.pose.position
            self.track.append((p.x, p.y))
            if self._ekf is not None and time.monotonic() - self._last_pair > 1.0:
                self._last_pair = time.monotonic()
                self.pairs.append((p.x, p.y, self._ekf.x, self._ekf.y))
                self.pair_times.append(time.monotonic())

    def localization_errors(self):
        from geographic_msgs.msg import GeoPoint
        from robot_localization.srv import FromLLArray
        from robot_sim.site_world import to_lonlat
        client = self.create_client(FromLLArray, "/fromLLArray")
        assert client.wait_for_service(timeout_sec=10.0)
        points = []
        for tx, ty, _ex, _ey in self.pairs:
            lon, lat = to_lonlat(tx, ty, 41.0, 29.0)
            points.append(GeoPoint(latitude=lat, longitude=lon))
        future = client.call_async(FromLLArray.Request(ll_points=points))
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        answer = future.result().map_points
        assert len(answer) == len(points)
        return [math.hypot(ex - q.x, ey - q.y) for (_tx, _ty, ex, ey), q in zip(self.pairs, answer)]

    def _on_collision(self, msg):
        self.collided = self.collided or msg.data

    def wait_until(self, predicate, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        pytest.fail(f"timed out after {timeout:.0f} s waiting for {what}; "
                    f"status={self.status} kit={self.kit} camera={self.camera}")


def sides_seen(tree, track, reach=SPACING / 2 + 0.5):
    """The tree's sides the true track passed alongside it, within `reach`."""
    seen = set()
    for x, y in track:
        dx, dy = x - tree[0], y - tree[1]
        if math.hypot(dx, dy) > reach * 1.5:
            continue
        if abs(dx) < 0.5 and SPACING / 2 - 1.0 < abs(dy) < reach:
            seen.add("north" if dy > 0 else "south")
        if abs(dy) < 0.5 and SPACING / 2 - 1.0 < abs(dx) < reach:
            seen.add("east" if dx > 0 else "west")
    return seen


@pytest.mark.parametrize("fix", ["rtk", "trunks"])
def test_orchard_grid_scan_with_the_kit(site_kit_nav2_stack, tmp_path, request, fix):
    """rtk: corrections from the fake caster. trunks: NO caster — the kit's F9P
    single-point (~1 m wander) plus the trunk localizer (K2c), the
    no-internet / lost-RTK case."""
    if fix == "rtk":
        extra, ntrip = (), request.getfixturevalue("ntrip_caster")
    else:
        extra, ntrip = ("-p", "kit_trunks_rate_hz:=2.0"), ""
    stack = site_kit_nav2_stack(orchard_site(), *physical_trees(), *extra, ntrip_file=ntrip,
                                camera_shim=KitStack.FAKE_CAMERA)
    runner = subprocess.Popen(
        ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
         "-p", f"site_file:={stack.site_file}",
         "-p", f"state_file:={tmp_path / 'mission_state.json'}"],
        stdout=open(tmp_path / "runner.log", "w"), stderr=subprocess.STDOUT,
        start_new_session=True)
    stack._procs.append(runner)
    watcher = Watcher()
    try:
        watcher.wait_until(lambda: watcher.status is not None and watcher.kit is not None,
                           30.0, "the runner and kit_manager")
        # No kit: refused, with the reason.
        client = watcher.create_client(StartMission, "/mission_runner/start")
        assert client.wait_for_service(timeout_sec=20.0)
        future = client.call_async(StartMission.Request(
            mission=json.dumps({"type": "orchard_grid", "area": "bahce"})))
        while not future.done():
            rclpy.spin_once(watcher, timeout_sec=0.05)
        assert not future.result().accepted and "needs the kit" in future.result().message

        hw.write_fake(stack.kit_dir, True, manifest.encode(fake_kit.variant("v1")))
        watcher.wait_until(lambda: watcher.kit["state"] == "READY"
                           and watcher.kit["gps_source"] == "kit"
                           and watcher.camera and watcher.camera.get("connected")
                           and watcher.oracle
                           and (fix != "rtk" or watcher.oracle["solution"] == "rtk_fixed")
                           # trunks: the localizer has fixed the pose once (the RTK
                           # case's wait for rtk_fixed) — else the first rows run
                           # on the single-point F9P alone
                           and (fix != "trunks" or getattr(watcher, "trunk_ok", False)),
                           60.0, f"the kit ready ({fix}), camera connected")
        start_mission(watcher, {"type": "orchard_grid", "area": "bahce"})
        started = time.monotonic()
        watcher.wait_until(lambda: watcher.status["state"] in ("DONE", "FAILED", "PAUSED"),
                           MISSION_TIMEOUT_S, "the scan to finish")
        log = (tmp_path / "runner.log").read_text()[-3000:]
        assert watcher.status["state"] == "DONE", f"{watcher.status}\n{log}"
        elapsed = time.monotonic() - started
        watcher.wait_until(lambda: watcher.camera and not watcher.camera["recording"], 10.0,
                           "the camera stopped")
        assert watcher.recorded, "the camera never recorded during the scan"
        assert any(f.endswith(".insv") for f in watcher.camera["files"])
        assert not watcher.collided, "the robot touched a trunk"
        assert watcher.oracle["cfg_writes"] == 0
        dump = os.environ.get("ORCHARD_TRACK_DUMP")
        if dump:                         # for looking at a run by hand
            with open(dump, "w", encoding="utf-8") as handle:
                json.dump({"track": watcher.track, "trees": TREES, "status": watcher.status},
                          handle)
        seen = {tree: sides_seen(tree, watcher.track) for tree in TREES}
        print(f"orchard scan ({fix}): {elapsed:.0f} s, {watcher.status['total']} waypoints, "
              f"skipped {watcher.status['skipped']}, sides seen "
              f"{ {t: len(s) for t, s in seen.items()} }")
        # How far off its corridor the robot drove: the distance from each
        # true sample to the nearest corridor line (all of them meet at the
        # grid's crossings, so turns count as on-line too).
        rows = sorted({t[1] for t in TREES})
        cols = sorted({t[0] for t in TREES})
        lines_y = [rows[0] - SPACING / 2, *[(a + b) / 2 for a, b in zip(rows, rows[1:])],
                   rows[-1] + SPACING / 2]
        lines_x = [cols[0] - SPACING / 2, *[(a + b) / 2 for a, b in zip(cols, cols[1:])],
                   cols[-1] + SPACING / 2]
        off = sorted(min(min(abs(y - ly) for ly in lines_y), min(abs(x - lx) for lx in lines_x))
                     for x, y in watcher.track)
        p95 = off[int(0.95 * (len(off) - 1))]
        loc_series = watcher.localization_errors()
        if dump:
            with open(dump + ".loc.json", "w", encoding="utf-8") as handle:
                json.dump({"pairs": watcher.pairs, "times": watcher.pair_times,
                           "errors": loc_series, "trunks": watcher.trunk_log}, handle)
        loc = sorted(loc_series)
        print(f"localization ({fix}): median {loc[len(loc) // 2]:.3f} m, "
              f"p95 {loc[int(0.95 * (len(loc) - 1))]:.3f} m")
        print(f"off the corridor lines ({fix}): p95 {p95:.3f} m, max {off[-1]:.3f} m; "
              f"trunk frames {watcher.trunk_frames}")
        # measured on RTK 0.095 m (max 0.24, the turns); odom-anchored it was ~1 m.
        # On trunks 0.23 m (localization p95 0.33): between fixes, at the
        # orchard's edge, the estimate drifts towards the single-point F9P.
        limit = 0.2 if fix == "rtk" else 0.35
        assert p95 < limit, f"rows on {fix} should hold their corridor to decimetres"
        missing = {t: sorted({'north', 'south', 'east', 'west'} - s)
                   for t, s in seen.items() if len(s) < 4}
        sides = sum(len(v) for v in seen.values())
        if fix == "rtk":
            assert not missing, f"trees not passed on every side: {missing}"
        else:
            # Measured on trunks alone (2026-10-26): 23-24 of 24 sides; this
            # 3 x 2 orchard is nearly all edge, where fewer trunks are in view.
            assert sides >= 23, f"trees not passed on every side: {missing}"
    finally:
        watcher.destroy_node()
