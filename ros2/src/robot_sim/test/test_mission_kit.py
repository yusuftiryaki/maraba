"""Missions and the kit socket (docs/kit-soketi.md, "Mod durum makinesi").

The kit is pulled out mid-mission while the robot localizes on its RTK GPS:
the mission must PAUSE (kit_gate.lost), the robot must STOP, and the GPS must
go back to the NEO-6M once it has — in that order. Without the pause the
robot would carry on mowing rows on dead reckoning, its fixes gone.

CI stage `mission` (a GPS + Nav2 stack, a few minutes).
"""

import json
import math
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from robot_kit import fake_kit, hw, manifest            # noqa: E402
from test_mission_coverage import CUT_WIDTH, lawn_feature, start_mission  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Watcher(Node):
    def __init__(self):
        super().__init__("kit_mission_watcher")
        self.status = None
        self.kit = None
        self.speed = None
        self.create_subscription(String, "/mission_runner/status",
                                 lambda m: setattr(self, "status", json.loads(m.data)), LATCHED)
        self.create_subscription(String, "/kit/state",
                                 lambda m: setattr(self, "kit", json.loads(m.data)), LATCHED)
        self.create_subscription(Odometry, "/ground_truth", self._on_truth, 10)

    def _on_truth(self, msg):
        t = msg.twist.twist
        self.speed = math.hypot(t.linear.x, t.linear.y) + abs(t.angular.z) * 0.25

    def wait_until(self, predicate, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return time.monotonic()
        pytest.fail(f"timed out after {timeout:.0f} s waiting for {what}; "
                    f"status={self.status} kit={self.kit}")


def test_unplugging_the_kit_mid_mission_pauses_and_stops(site_kit_nav2_stack, tmp_path,
                                                         ntrip_caster):
    stack = site_kit_nav2_stack([lawn_feature()], ntrip_file=ntrip_caster)
    runner = subprocess.Popen(
        ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
         "-p", f"site_file:={stack.site_file}",
         "-p", f"state_file:={tmp_path / 'mission_state.json'}",
         "-p", f"cut_width:={CUT_WIDTH}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    stack._procs.append(runner)
    watcher = Watcher()
    try:
        watcher.wait_until(lambda: watcher.status is not None and watcher.kit is not None,
                           30.0, "the runner and kit_manager")
        hw.write_fake(stack.kit_dir, True, manifest.encode(fake_kit.variant("v1-nocam")))
        watcher.wait_until(lambda: watcher.kit["gps_source"] == "kit", 20.0,
                           "the GPS on the kit")
        start_mission(watcher, {"type": "coverage", "area": "cim", "params": {}})
        watcher.wait_until(lambda: watcher.status["state"] == "RUNNING"
                           and (watcher.speed or 0.0) > 0.1, 120.0, "the robot driving")
        end = time.monotonic() + 8.0
        while time.monotonic() < end:            # well into the mission
            rclpy.spin_once(watcher, timeout_sec=0.05)

        hw.write_fake(stack.kit_dir, False, None)
        unplugged = time.monotonic()
        paused = watcher.wait_until(lambda: watcher.status["state"] == "PAUSED", 5.0,
                                    "PAUSED")
        assert "unplugged" in watcher.status["reason"], watcher.status
        stopped = watcher.wait_until(lambda: (watcher.speed or 0.0) < 0.02, 5.0, "the stop")
        watcher.wait_until(lambda: watcher.kit["gps_source"] == "neo6m", 10.0,
                           "the NEO-6M again")
        print(f"unplug -> PAUSED {paused - unplugged:.2f} s, -> stopped "
              f"{stopped - unplugged:.2f} s")
    finally:
        watcher.destroy_node()
