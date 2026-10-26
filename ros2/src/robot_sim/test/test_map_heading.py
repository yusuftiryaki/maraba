"""Does `map`'s heading agree with `map`'s positions? (docs/missions.md, orchard_grid)

ekf_global takes its yaw from the compass (true north, through madgwick) and
its x, y from the GPS through navsat_transform. If the two disagree by an
angle, every pattern driven open-loop in `odom` ("chain" anchoring: rows laid
out from the first waypoint with the compass's heading) comes out rotated by
that angle — 3° was measured as 0.8-1 m at the end of a 16 m row.

The measurement: drive straight, take the direction the robot TRULY moved,
expressed in `map` (navsat_transform's own /fromLLArray of the true lat/lons,
so the datum's error cancels), and compare it with ekf_global's yaw. Quiet GPS,
so the GPS's wander does not swamp the degree being looked for.

Spawns a GlobalStack, ~1 min. Skips without ROS.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from geographic_msgs.msg import GeoPoint                # noqa: E402
from geometry_msgs.msg import Twist                     # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.node import Node                             # noqa: E402
from robot_localization.srv import FromLLArray          # noqa: E402

from robot_sim.site_world import to_lonlat              # noqa: E402

QUIET_GPS = ("--ros-args", "-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("map_heading_probe")
        self.truth = self.ekf = None
        self.cmd = (0.0, 0.0)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ground_truth", lambda m: setattr(self, "truth", m), 10)
        self.create_subscription(Odometry, "/odometry/filtered/global",
                                 lambda m: setattr(self, "ekf", m), 10)
        self._from_ll = self.create_client(FromLLArray, "/fromLLArray")
        self.create_timer(0.05, self._tick)

    def _tick(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.cmd
        self._pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def to_map(self, points):
        assert self._from_ll.wait_for_service(timeout_sec=10.0)
        lls = []
        for x, y in points:
            lon, lat = to_lonlat(x, y, 41.0, 29.0)
            lls.append(GeoPoint(latitude=lat, longitude=lon))
        future = self._from_ll.call_async(FromLLArray.Request(ll_points=lls))
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        answer = future.result().map_points
        assert len(answer) == len(points), "fromLLArray not ready"
        return [(p.x, p.y) for p in answer]


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.destroy_node()


def test_maps_heading_matches_maps_positions(global_stack, probe):
    global_stack(*QUIET_GPS)
    end = time.monotonic() + 40.0
    while (probe.truth is None or probe.ekf is None) and time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.05)
    assert probe.ekf is not None, "no ekf_global"
    probe.spin_for(8.0)
    disagreements = []
    for turn in (0.0, 1.6, 1.6, 1.6):           # four headings, ~90° apart
        if turn:
            probe.cmd = (0.0, 0.5)
            probe.spin_for(turn / 0.5 * 1.0)
            probe.cmd = (0.0, 0.0)
            probe.spin_for(2.0)
        a = probe.truth.pose.pose.position
        probe.cmd = (0.3, 0.0)
        yaws = []
        stop = time.monotonic() + 20.0
        while time.monotonic() < stop:
            probe.spin_for(0.2)
            yaws.append(yaw_of(probe.ekf.pose.pose.orientation))
        probe.cmd = (0.0, 0.0)
        b = probe.truth.pose.pose.position
        probe.spin_for(1.0)
        (ax, ay), (bx, by) = probe.to_map([(a.x, a.y), (b.x, b.y)])
        moved = math.atan2(by - ay, bx - ax)
        believed = math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))
        disagreements.append(math.degrees(math.atan2(math.sin(believed - moved),
                                                      math.cos(believed - moved))))
    print("map heading - map motion, per heading (deg):",
          ", ".join(f"{d:+.2f}" for d in disagreements))
    # measured (2026-10-26): +1.02, +0.69, -1.23, -0.98 — heading-dependent,
    # the compass's residual hard iron; no constant offset (navsat_transform
    # handles the UTM grid convergence, ~1.3° here, itself). A constant
    # offset of a degree or two is what a convention error would look like.
    worst = max(abs(d) for d in disagreements)
    mean = sum(disagreements) / len(disagreements)
    assert worst < 3.0 and abs(mean) < 1.0, disagreements
