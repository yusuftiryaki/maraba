"""Localizing on the orchard's trunks (docs/kit-soketi.md, K2c).

    sim_node's trunk detector (robot_kit/trunk_sim.py) ─► kit/trunks
    site.geojson trees ─/fromLLArray─► trunk_localizer ─► kit/trunk_pose ─► ekf_global
                                                      └─► kit/tree_audit

The kit is plugged in WITHOUT RTK (no caster): its F9P runs single-point and
wanders ~1 m, which is the case the trunks are for — no internet, or RTK
lost under the canopy. The same drive is measured twice, trunk detection off
and on: the robot's `map` pose against where its true lat/lon lands in `map`
(test_kit_stack's measure). The site is deliberately wrong, as an inventory
gets: one mapped tree has died, one tree planted since is not in it. Neither
may pull the pose, and the audit must name both.

Spawns two stacks, ~4 min. Skips without ROS.
"""

import json
import math
import time

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from conftest import KitStack                           # noqa: E402
from test_kit_stack import Probe, plug                  # noqa: E402
from robot_sim.site_world import to_world               # noqa: E402
from test_mission_coverage import lonlat                # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
COLS = (3.0, 9.0, 15.0, 21.0)
ROWS = (-3.0, 3.0, 9.0)
TRUNK = 0.15
DEAD = (15.0, -3.0)              # in the site, not in the world
PLANTED = (12.0, 3.0)            # in the world, not in the site


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.destroy_node()


def tree_name(x, y):
    return f"S{ROWS.index(y) + 1}-{COLS.index(x) + 1}"


def site_trees():
    return [{"type": "Feature", "geometry": {"type": "Point", "coordinates": lonlat(x, y)},
             "properties": {"kind": "poi", "name": tree_name(x, y), "label": "ağaç",
                            "radius_m": TRUNK}}
            for x in COLS for y in ROWS]


def world_trunks():
    trunks = [(x, y) for x in COLS for y in ROWS if (x, y) != DEAD] + [PLANTED]
    # unsurveyed: the robot's survey map is the SITE (site_map_server); these
    # are the physical trunks the simulated camera sees and the robot could hit
    return ("-p", f"unsurveyed_obstacle_centers:={[v for t in trunks for v in t]}",
            "-p", f"unsurveyed_obstacle_radii:={[TRUNK] * len(trunks)}")


def drive_the_corridor(stack_factory, probe, trunks_hz):
    stack = stack_factory(site_trees(), *world_trunks(), "-p", f"kit_trunks_rate_hz:={trunks_hz}",
                          camera_shim=KitStack.FAKE_CAMERA)
    probe.wait_until(lambda: probe.state is not None and probe.ekf_global is not None
                     and probe.truth is not None, 40.0, "kit/state and ekf_global")
    plug(stack, "v1")
    probe.wait_until(lambda: probe.state["gps_source"] == "kit", 30.0, "the GPS on the kit")
    probe.spin_for(8.0)
    errors = []
    probe.cmd = (0.3, 0.0)
    end = time.monotonic() + 60.0                       # 18 m along y = 0, east
    while time.monotonic() < end:
        probe.spin_for(0.5)
        errors.append(probe.map_error())
    probe.cmd = (0.0, 0.0)
    probe.spin_for(1.0)
    return stack, np.array(errors)


def test_trunks_hold_the_pose_without_rtk(site_kit_stack, probe):
    _stack, off = drive_the_corridor(site_kit_stack, probe, 0.0)
    _stack.close()
    probe.state = probe.ekf_global = probe.truth = None
    audit, status = {}, {}
    probe.create_subscription(String, "/kit/tree_audit",
                              lambda m: audit.update(json.loads(m.data)), LATCHED)
    probe.create_subscription(String, "/kit/trunk_status",
                              lambda m: status.update(json.loads(m.data)), 10)
    _stack, on = drive_the_corridor(site_kit_stack, probe, 2.0)
    probe.wait_until(lambda: any("lat" in n for n in audit.get("new", [])), 15.0,
                     "the new tree's lat/lon (/toLL)")
    print("trunks-on error series (m):", " ".join(f"{e:.2f}" for e in on))
    print(f"map error on the kit's single-point F9P: trunks off mean {off.mean():.2f} m "
          f"(p95 {np.percentile(off, 95):.2f}), trunks on mean {on.mean():.3f} m "
          f"(p95 {np.percentile(on, 95):.3f}); last trunk frame {status}; audit {audit}")
    # measured (3 runs): mean 0.048-0.054 m, p95 0.14-0.18 (before the outlier
    # rejection one run reached p95 0.45: the planted tree, matched to a neighbour)
    assert np.percentile(on, 95) < 0.3, "the trunks should hold the pose to decimetres"
    assert on.mean() < off.mean() / 2, "trunk localization made no difference"
    assert tree_name(*DEAD) in audit.get("missing", []), audit
    # The planted tree is reported in `map` and in lat/lon (via /toLL); its
    # true place is its sim_world position, which lat/lon carries directly.
    found = [to_world(n["lon"], n["lat"], 41.0, 29.0) for n in audit.get("new", []) if "lat" in n]
    assert found, f"the new tree was not reported (with a lat/lon): {audit}"
    nearest = min(math.hypot(x - PLANTED[0], y - PLANTED[1]) for x, y in found)
    assert nearest < 0.6, f"the new tree reported {nearest:.2f} m from where it is: {audit}"


def test_trunks_through_a_u_turn(site_kit_stack, probe):
    """The corridor, an in-place U-turn, back: the fixes hold through the turn
    (the hypothesis search's ±6° yaw window round the EKF's estimate)."""
    stack = site_kit_stack(site_trees(), *world_trunks(), "-p", "kit_trunks_rate_hz:=2.0",
                           camera_shim=KitStack.FAKE_CAMERA)
    probe.wait_until(lambda: probe.state is not None and probe.ekf_global is not None
                     and probe.truth is not None, 40.0, "kit/state and ekf_global")
    plug(stack, "v1")
    probe.wait_until(lambda: probe.state["gps_source"] == "kit", 30.0, "the GPS on the kit")
    probe.spin_for(8.0)
    errors = []
    for cmd, secs in (((0.3, 0.0), 30.0), ((0.0, 0.6), 5.24), ((0.3, 0.0), 30.0)):
        probe.cmd = cmd
        end = time.monotonic() + secs
        while time.monotonic() < end:
            probe.spin_for(0.5)
            errors.append(probe.map_error())
    probe.cmd = (0.0, 0.0)
    errors = np.array(errors)
    print(f"through the U-turn: mean {errors.mean():.3f} m, p95 {np.percentile(errors, 95):.3f} m")
    # measured: mean ~0.06, p95 ~0.2 (the sawtooth between 2 Hz fixes)
    assert np.percentile(errors, 95) < 0.35
