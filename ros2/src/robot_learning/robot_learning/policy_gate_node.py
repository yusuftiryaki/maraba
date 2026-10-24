"""policy_gate — what of a learned policy reaches the wheels (gate.py;
docs/learning.md 6d).

    policy/cmd (TwistStamped)     in  — header.stamp = the OBSERVATION's
                                        stamp (camera frame / state it came from)
    jam_monitor/state (String)    in  — a jam disarms
    map -> base_link (tf)         in  — the fence
    cmd_vel_policy (Twist, 20 Hz) out — twist_mux priority 20; only while armed
    ~/arm (PolicyArm), ~/disarm (Trigger)
    ~/state (String JSON, latched)    — armed?, why

Disarmed (the default) it publishes nothing at all. The clock is the node's
(sim time in the simulator), the same one the observation stamps are in.
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_learning.gate import GateConfig, PolicyGate
from robot_missions_interfaces.srv import PolicyArm


class PolicyGateNode(Node):
    def __init__(self, **kwargs):
        super().__init__("policy_gate", **kwargs)
        defaults = GateConfig()
        for name in ("max_linear", "max_angular", "max_accel", "max_alpha", "cmd_timeout_s",
                     "obs_timeout_s", "lease_s", "margin_m"):
            self.declare_parameter(name, getattr(defaults, name))
        self.gate = PolicyGate(GateConfig(**{
            name: float(self.get_parameter(name).value) for name in defaults.__dataclass_fields__}))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._out = self.create_publisher(Twist, "cmd_vel_policy", 10)
        self._state = self.create_publisher(String, "~/state", latched)
        self.create_subscription(TwistStamped, "policy/cmd", self._on_cmd, 10)
        self.create_subscription(String, "jam_monitor/state", self._on_jam, latched)
        self.create_service(PolicyArm, "~/arm", self._on_arm)
        self.create_service(Trigger, "~/disarm", self._on_disarm)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._said = None
        self.create_timer(0.05, self._tick)
        self._publish_state()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: TwistStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gate.command(msg.twist.linear.x, msg.twist.angular.z, stamp, self._now())

    def _on_jam(self, msg: String) -> None:
        state = json.loads(msg.data)
        if state.get("jammed"):
            self.gate.jam(f"{state.get('kind')}: {state.get('detail')}")

    def _on_arm(self, request, response):
        try:
            area = [tuple(map(float, p)) for p in json.loads(request.area)]
            self.gate.arm(area, request.budget_s, self._now())
            response.success, response.message = True, f"armed for {request.budget_s:.0f} s"
            self.get_logger().warn(f"ARMED: {response.message}")
        except (ValueError, TypeError) as exc:
            response.success, response.message = False, str(exc)
        self._publish_state()
        return response

    def _on_disarm(self, _request, response):
        self.gate.disarm("disarmed on request")
        self._out.publish(Twist())
        response.success, response.message = True, "disarmed"
        self._publish_state()
        return response

    def _robot_xy(self):
        try:
            t = self._tf.lookup_transform("map", "base_link", Time()).transform.translation
        except TransformException:
            return None
        return (t.x, t.y)

    def _tick(self) -> None:
        out = self.gate.tick(self._robot_xy(), self._now())
        if out is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = out
            self._out.publish(msg)
        self._publish_state()

    def _publish_state(self) -> None:
        said = (self.gate.armed, self.gate.reason)
        if said == self._said:
            return
        if self._said is not None and self._said[0] and not said[0]:
            self.get_logger().warn(f"DISARMED: {self.gate.reason}")
        self._said = said
        self._state.publish(String(data=json.dumps({"armed": said[0], "reason": said[1]})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyGateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
