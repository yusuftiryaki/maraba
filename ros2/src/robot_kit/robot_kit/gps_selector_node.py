"""gps_selector — one GPS into navsat_transform: the NEO-6M or the kit's F9P.

    gps/neo6m/fix, gps/neo6m/vel     in   the robot's own receiver (sensors.launch.py
                                          remaps it here when use_kit:=true)
    kit/gnss/fix, kit/gnss/vel       in   the kit's (kit_gnss)
    kit/state                        in   gps_source, decided by kit_manager
    gps/fix, gps/vel                 out  what navsat_transform and jam_monitor read

Only a relay: WHEN to switch is kit_manager's decision (state.py — only while
the robot is idle). With no kit/state at all — kit_manager not running, or
dead — it relays the NEO-6M: the robot's own receiver is the one that is
always there.

navsat_transform needs nothing else to follow a switch: each fix carries its
antenna's frame (gps_link or kit_gnss), and the antenna offset is looked up
from TF by that name (sim_node's "antenna, not base_link" lesson).
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from robot_kit.state import KIT, NEO6M


class GpsSelector(Node):
    def __init__(self, **kwargs):
        super().__init__("gps_selector", **kwargs)
        self._source = NEO6M
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self._vel_pub = self.create_publisher(TwistStamped, "gps/vel", 10)
        self._source_pub = self.create_publisher(String, "gps_selector/source", latched)
        self.create_subscription(String, "kit/state", self._on_state, latched)
        for source, prefix in ((NEO6M, "gps/neo6m"), (KIT, "kit/gnss")):
            self.create_subscription(NavSatFix, f"{prefix}/fix",
                                     lambda m, s=source: self._relay(s, self._fix_pub, m), 10)
            self.create_subscription(TwistStamped, f"{prefix}/vel",
                                     lambda m, s=source: self._relay(s, self._vel_pub, m), 10)
        self._source_pub.publish(String(data=self._source))

    def _on_state(self, msg: String) -> None:
        try:
            source = json.loads(msg.data).get("gps_source", NEO6M)
        except (ValueError, AttributeError):
            return
        if source not in (NEO6M, KIT) or source == self._source:
            return
        self.get_logger().info(f"GPS source: {self._source} -> {source}")
        self._source = source
        self._source_pub.publish(String(data=source))

    def _relay(self, source: str, pub, msg) -> None:
        if source == self._source:
            pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GpsSelector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
