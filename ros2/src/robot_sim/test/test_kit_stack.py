"""The kit socket through the whole localization stack (docs/kit-soketi.md, K1).

    fake_kit's chip + pin ─► kit_manager ─► kit/state, tf kit_mount -> kit_gnss
    sim_node's F9P (pty) ─► kit_gnss ─► kit/gnss/fix ─┐
    sim_node's NEO-6M ─────────────► gps/neo6m/fix ──┴► gps_selector ─► gps/fix ─► navsat ─► ekf_global
    fake caster ─► ntrip_client ─► kit/gnss/rtcm ─► kit_gnss ─► the F9P (RTK)

What "right" means here: the robot's position in `map` against where its TRUE
lat/lon lands in `map` (navsat_transform's own /fromLLArray). `map` is anchored
on the first NEO-6M fix and carries that fix's error, but /fromLLArray carries
it too, so it cancels — this is the error that matters for driving a site drawn
in lat/lon (site_map_stack's reasoning).

Spawns the stack, ~2-3 min. Skips without ROS.
"""

import json
import math
import os
import threading
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from geographic_msgs.msg import GeoPoint                # noqa: E402
from geometry_msgs.msg import Twist                     # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from robot_localization.srv import FromLLArray          # noqa: E402
from sensor_msgs.msg import NavSatFix                   # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from robot_kit import fake_kit, hw, manifest            # noqa: E402
from robot_sim.site_world import EARTH_RADIUS_M         # noqa: E402

DATUM_LAT, DATUM_LON = 41.0, 29.0          # sim_node's defaults
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CASTER_PORT = 12101


class Probe(Node):
    def __init__(self):
        super().__init__("kit_probe")
        self.state = None
        self.fix_frames = []
        self.truth = None
        self.ekf_global = None
        self.oracle = None
        self.ntrip = None
        self.cmd = (0.0, 0.0)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/kit/state",
                                 lambda m: setattr(self, "state", json.loads(m.data)), LATCHED)
        self.create_subscription(NavSatFix, "/gps/fix",
                                 lambda m: self.fix_frames.append(m.header.frame_id), 10)
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        self.create_subscription(Odometry, "/odometry/filtered/global",
                                 lambda m: setattr(self, "ekf_global", m), 10)
        self.create_subscription(String, "/sim/kit_f9p",
                                 lambda m: setattr(self, "oracle", json.loads(m.data)), 10)
        self.create_subscription(String, "/kit/ntrip/status",
                                 lambda m: setattr(self, "ntrip", json.loads(m.data)), LATCHED)
        self._from_ll = self.create_client(FromLLArray, "/fromLLArray")
        self.create_timer(0.05, lambda: self._publish_cmd())

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.cmd
        self._pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        pytest.fail(f"timed out after {timeout:.0f} s waiting for {what}; "
                    f"kit/state={self.state} oracle={self.oracle} ntrip={self.ntrip}")

    def latest_fix_frame(self, seconds=1.5):
        self.fix_frames.clear()
        self.spin_for(seconds)
        assert self.fix_frames, "no /gps/fix at all"
        return self.fix_frames[-1]

    def map_error(self):
        """|ekf_global - where the true base_link lat/lon lands in map|, m."""
        p = self.truth.pose.pose.position
        lat = DATUM_LAT + math.degrees(p.y / EARTH_RADIUS_M)
        lon = DATUM_LON + math.degrees(
            p.x / (EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT))))
        assert self._from_ll.wait_for_service(timeout_sec=10.0), "no /fromLLArray"
        request = FromLLArray.Request(ll_points=[GeoPoint(latitude=lat, longitude=lon)])
        future = self._from_ll.call_async(request)
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        points = future.result().map_points
        assert len(points) == 1, "fromLLArray not ready (short answer, decision 5)"
        e = self.ekf_global.pose.pose.position
        return math.hypot(e.x - points[0].x, e.y - points[0].y)

    def mean_map_error(self, seconds):
        errors = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin_for(0.5)
            errors.append(self.map_error())
        return sum(errors) / len(errors), max(errors)


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.destroy_node()


@pytest.fixture
def caster(tmp_path):
    stop, ready = threading.Event(), threading.Event()
    thread = threading.Thread(
        target=fake_kit.serve_caster,
        kwargs=dict(port=CASTER_PORT, mountpoint="TEST", rate_hz=2.0, chunked=True,
                    stop=stop, ready=ready, log=lambda *_: None),
        daemon=True)
    thread.start()
    assert ready.wait(5.0), "fake caster did not start"
    path = os.path.join(str(tmp_path), "ntrip.yaml")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(f"host: 127.0.0.1\nport: {CASTER_PORT}\nmountpoint: TEST\n")
    yield path
    stop.set()
    thread.join(timeout=5.0)


def plug(stack, variant="v1-nocam"):
    hw.write_fake(stack.kit_dir, True, manifest.encode(fake_kit.variant(variant)))


def test_plug_rtk_unplug(kit_stack, probe, caster):
    """The whole K1 chain on one stack: no kit -> plug -> RTK -> unplug."""
    stack = kit_stack(ntrip_file=caster)
    probe.wait_until(lambda: probe.state is not None and probe.ekf_global is not None
                     and probe.truth is not None, 40.0, "kit/state and ekf_global")
    assert probe.state["state"] == "NONE" and probe.state["gps_source"] == "neo6m"
    assert probe.latest_fix_frame() == "gps_link", "without a kit, gps/fix is the NEO-6M"
    probe.spin_for(5.0)
    neo_mean, _ = probe.mean_map_error(5.0)

    plug(stack)
    probe.wait_until(lambda: probe.state["state"] == "READY", 15.0, "READY")
    assert probe.state["kit"]["serial"] == "K001"
    probe.wait_until(lambda: probe.state["gps_source"] == "kit", 10.0, "the GPS on the kit")
    assert probe.latest_fix_frame() == "kit_gnss"
    # corrections: the caster -> ntrip_client -> kit_gnss -> the F9P
    probe.wait_until(lambda: probe.oracle and probe.oracle["solution"] == "rtk_fixed",
                     40.0, "RTK fixed")
    probe.spin_for(5.0)                         # ekf_global absorbs the jump
    kit_mean, kit_worst = probe.mean_map_error(5.0)
    print(f"map error still: NEO-6M {neo_mean:.2f} m, kit RTK {kit_mean:.3f} m "
          f"(worst {kit_worst:.3f})")

    # and while driving
    probe.cmd = (0.3, 0.0)
    probe.spin_for(2.0)
    moving_mean, moving_worst = probe.mean_map_error(6.0)
    probe.cmd = (0.0, 0.0)
    print(f"map error at 0.3 m/s on RTK: mean {moving_mean:.3f} m, worst {moving_worst:.3f}")
    assert kit_worst < 0.15 and moving_worst < 0.25, (
        "on RTK the robot should sit within centimetres of its true lat/lon in map — "
        "suspect the kit_gnss frame (its offset from base_link) or the fix covariance")
    assert probe.oracle["cfg_writes"] == 0, "the robot wrote UBX-CFG to the kit's receiver"

    hw.write_fake(stack.kit_dir, False, None)
    probe.wait_until(lambda: probe.state["state"] == "NONE", 5.0, "NONE after unplug")
    probe.wait_until(lambda: probe.state["gps_source"] == "neo6m", 10.0, "back on the NEO-6M")
    assert probe.latest_fix_frame() == "gps_link"


def test_source_switch_waits_until_the_robot_stops(kit_stack, probe):
    stack = kit_stack()
    probe.wait_until(lambda: probe.state is not None, 40.0, "kit/state")
    probe.cmd = (0.3, 0.2)
    probe.spin_for(2.0)
    plug(stack)
    probe.wait_until(lambda: probe.state["state"] == "READY"
                     and probe.state["gps_wanted"] == "kit", 15.0, "READY, wanting the kit")
    probe.spin_for(3.0)
    assert probe.state["gps_source"] == "neo6m", "the GPS switched under a moving robot"
    assert probe.latest_fix_frame() == "gps_link"
    probe.cmd = (0.0, 0.0)
    probe.wait_until(lambda: probe.state["gps_source"] == "kit", 10.0, "the switch once stopped")


def test_a_blank_chip_is_not_a_kit(kit_stack, probe):
    stack = kit_stack()
    probe.wait_until(lambda: probe.state is not None, 40.0, "kit/state")
    hw.write_fake(stack.kit_dir, True, b"\xff" * manifest.EEPROM_SIZE)
    probe.wait_until(lambda: probe.state["present"], 5.0, "the plug")
    probe.spin_for(3.0)
    assert probe.state["state"] == "NONE" and "blank" in probe.state["error"]
    assert probe.state["gps_source"] == "neo6m"
