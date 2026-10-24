"""policy_gate on a running robot (docs/learning.md 6d). SKIPs without ROS, ~1.5 min.

A stand-in "policy" publishes policy/cmd as fast and as hard as it likes; the
robot (twist_mux, bridge, ESP32 simulator, kinematic world — /ground_truth)
must do only what the gate allows: nothing while disarmed, at most the gate's
speed while armed, a stop when the policy's observations go stale, and a stop
plus a disarm when it leaves the armed area.
"""

import json
import math
import threading
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped       # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.executors import SingleThreadedExecutor      # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from robot_missions_interfaces.srv import PolicyArm     # noqa: E402
from test_learning_stack import wait_status             # noqa: E402
from test_web_stack import LAWN, QUIET_GPS              # noqa: E402

# map ~ sim_world with a quiet GPS; the robot starts at (0, 0) facing east.
AREA = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]


class Policy:
    """A reckless stand-in: always full ahead, stamped `lag` s in the past."""

    def __init__(self):
        self.node = rclpy.create_node("fake_policy")
        self.pub = self.node.create_publisher(TwistStamped, "/policy/cmd", 10)
        self.cmd, self.truth, self.state = [], [], []
        stamp = time.monotonic
        self.node.create_subscription(Twist, "/cmd_vel",
                                      lambda m: self.cmd.append((stamp(), m.linear.x)), 50)
        self.node.create_subscription(
            Odometry, "/ground_truth",
            lambda m: self.truth.append((stamp(), m.pose.pose.position.x)), 50)
        self.node.create_subscription(
            String, "/policy_gate/state", lambda m: self.state.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lag = 0.05
        self.sending = True
        self.node.create_timer(0.1, self._send)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _send(self):
        if not self.sending:
            return
        msg = TwistStamped()
        now = self.node.get_clock().now().nanoseconds * 1e-9 - self.lag
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(now), int((now % 1) * 1e9)
        msg.twist.linear.x = 1.0                       # far over the gate's 0.3
        self.pub.publish(msg)

    def arm(self, area, budget):
        client = self.node.create_client(PolicyArm, "/policy_gate/arm")
        assert client.wait_for_service(timeout_sec=20.0)
        request = PolicyArm.Request()
        request.area, request.budget_s = json.dumps(area), float(budget)
        future = client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert future.done() and future.result().success, future.result()

    def x_now(self):
        return self.truth[-1][1]

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()


def test_the_gate_lets_through_only_what_it_should(learning_stack):
    learning_stack([LAWN], *QUIET_GPS)
    wait_status(lambda s: s["robot"] is not None, 60.0, "a robot")
    policy = Policy()
    try:
        time.sleep(3.0)
        x0 = policy.x_now()
        assert abs(x0) < 0.1 and not any(c[1] > 0 for c in policy.cmd), \
            "disarmed, the policy must not move the robot"

        policy.arm(AREA, 60.0)
        time.sleep(4.0)
        moving = [c[1] for c in policy.cmd if c[0] > time.monotonic() - 2.0]
        assert max(moving) == pytest.approx(0.3, abs=1e-6), "capped at the gate's 0.3 m/s"
        assert policy.x_now() > x0 + 0.5

        policy.lag = 1.0                                # acting on stale observations
        time.sleep(2.0)
        x_stale = policy.x_now()
        time.sleep(1.5)
        assert abs(policy.x_now() - x_stale) < 0.02, "a stale observation must stop it"
        assert policy.state[-1]["armed"] and "observation" in policy.state[-1]["reason"]

        policy.lag = 0.05                               # on again, to the fence (x = 2 + 1)
        deadline = time.monotonic() + 30.0
        while policy.state[-1]["armed"] and time.monotonic() < deadline:
            time.sleep(0.2)
        assert not policy.state[-1]["armed"] and "left the area" in policy.state[-1]["reason"]
        time.sleep(2.0)
        x_out = policy.x_now()
        print(f"\nMEASURED gate: disarmed at x = {x_out:.2f} (fence 3.0)")
        assert 2.9 < x_out < 3.4
        time.sleep(1.5)
        assert abs(policy.x_now() - x_out) < 0.02, "disarmed, it must stay stopped"
    finally:
        policy.close()
