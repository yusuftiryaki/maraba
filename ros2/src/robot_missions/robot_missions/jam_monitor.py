"""jam_monitor — are the wheels lying? (jam.py; docs/missions.md, "Patinaj
sonrası alan dışında biçme").

    /odom (wheel odometry), imu/data_raw (gyro), gps/fix (raw GPS)   in
    jam_monitor/state (String JSON, latched, on change)             out
        {"jammed": bool, "kind": "turn"|"stall"|null, "forward": bool, "detail": str}

A forward stall is an obstacle ahead the bumper missed: site_map_server
records it like a press, so mission_runner backs off it and cuts round it.

mission_runner stops the leg on a jam and restarts its odom chain from `map`;
the policy gate (6d) will listen too. Raw GPS on purpose: /gps/filtered is the
EKF, which fuses the very wheel odometry being checked.
"""

from __future__ import annotations

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

from robot_missions.jam import JamDetector
from robot_missions.site import LatLon


class JamMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__("jam_monitor", **kwargs)
        self._detector = JamDetector()
        self._state = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, "~/state", latched)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(Imu, "imu/data_raw", self._on_imu, 20)
        self.create_subscription(NavSatFix, "gps/fix", self._on_fix, 10)
        self.create_timer(0.2, self._tick)
        self._publish(False, None, "")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self._detector.wheel(self._now(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_imu(self, msg: Imu) -> None:
        self._detector.gyro(self._now(), msg.angular_velocity.z)

    def _on_fix(self, msg: NavSatFix) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and msg.status.status >= 0:
            self._detector.gps(self._now(), LatLon(msg.latitude, msg.longitude))

    def _tick(self) -> None:
        verdict = self._detector.check(self._now())
        if verdict is None:
            self._publish(False, None, "")
        else:
            self._publish(True, verdict["kind"], verdict["detail"],
                          bool(verdict.get("forward", False)))

    def _publish(self, jammed: bool, kind, detail: str, forward: bool = False) -> None:
        if self._state == (jammed, kind):
            return
        if jammed:
            self.get_logger().warn(f"JAMMED ({kind}): {detail}")
        elif self._state is not None and self._state[0]:
            self.get_logger().info("no longer jammed")
        self._state = (jammed, kind)
        self._pub.publish(String(data=json.dumps({"jammed": jammed, "kind": kind,
                                                  "forward": forward, "detail": detail})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JamMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
