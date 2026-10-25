"""kit_manager — is a kit plugged into the socket, and which mode is the robot in.

    presence pin + identity chip (hw.py)      in   (fake_dir: fake_kit's files)
    kit/gnss/fix, kit/camera/status,
    kit/lidar/points                          in   the streams the chip promises
    odom, mission_runner/status               in   "idle": no mission running, wheels still
    kit/state  (String JSON, latched, on change + 1 Hz)   out
        {"state": "NONE"|"READY"|"DEGRADED", "present": bool, "kit": {manifest}|null,
         "error": str, "missing": [...], "gps_source": "neo6m"|"kit",
         "gps_wanted": ..., "gnss": {"solution": ..., "h_acc_m": ...}}
    tf_static  kit_mount -> kit_gnss / kit_cam360 / kit_lidar, from the chip

The decisions are in state.py (ROS-free, tested event by event); this node
only reads the lines, listens and publishes. docs/kit-soketi.md.

⚠️ The kit's frames come ONLY from the chip. Nothing in the robot's URDF knows
where the kit's antenna or camera are: a kit re-calibrated in orchadOS carries
its new numbers with it, and a robot that hard-coded them would quietly place
every fix where the old antenna used to be.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from robot_kit import hw, manifest as kit_manifest
from robot_kit.state import KitMonitor

# Still for this long (and no mission running) = idle: the GPS source may change.
IDLE_HOLD_S = 1.0
STILL_V = 0.02          # m/s
STILL_W = 0.05          # rad/s


def quaternion(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class KitManager(Node):
    def __init__(self, **kwargs):
        super().__init__("kit_manager", **kwargs)
        self.declare_parameter("fake_dir", "")
        self.declare_parameter("gpio_chip", "/dev/gpiochip0")
        self.declare_parameter("gpio_line", 17)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("debounce_s", 0.5)
        self.declare_parameter("grace_s", 15.0)
        p = self.get_parameter
        fake_dir = p("fake_dir").value
        if fake_dir:
            self._presence, self._eeprom = hw.make_readers(fake_dir)
        else:
            self._presence = hw.GpioPresence(p("gpio_chip").value, p("gpio_line").value)
            self._eeprom = hw.W1Eeprom()
        self._monitor = KitMonitor(debounce_s=p("debounce_s").value, grace_s=p("grace_s").value)

        self._mission_running = False
        self._moving_at = self._now()
        self._gnss = {"solution": "none", "h_acc_m": None}
        self._published = None
        self._frames_of = None              # calib_id + serial the TF was sent for

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, "kit/state", latched)
        self._tf = StaticTransformBroadcaster(self)
        self.create_subscription(NavSatFix, "kit/gnss/fix", self._on_fix, 10)
        self.create_subscription(String, "kit/gnss/status", self._on_gnss_status, 10)
        self.create_subscription(String, "kit/camera/status", self._on_camera, 10)
        self.create_subscription(PointCloud2, "kit/lidar/points",
                                 lambda _m: self._monitor.seen("lidar", self._now()),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(String, "mission_runner/status", self._on_mission, latched)
        self.create_timer(1.0 / p("poll_hz").value, self._tick)
        self.create_timer(1.0, lambda: self._publish(force=True))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ----------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status != NavSatStatus.STATUS_NO_FIX:
            self._monitor.seen("gnss", self._now())

    def _on_camera(self, msg: String) -> None:
        # kit_camera publishes while disconnected too (with the reason): only a
        # connected camera is a camera that works.
        try:
            if json.loads(msg.data).get("connected"):
                self._monitor.seen("camera", self._now())
        except (ValueError, AttributeError):
            pass

    def _on_gnss_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._gnss = {"solution": data.get("solution", "none"),
                          "h_acc_m": data.get("h_acc_m")}
        except (ValueError, AttributeError):
            pass

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        if abs(twist.linear.x) > STILL_V or abs(twist.angular.z) > STILL_W:
            self._moving_at = self._now()

    def _on_mission(self, msg: String) -> None:
        try:
            self._mission_running = json.loads(msg.data).get("state") == "RUNNING"
        except (ValueError, AttributeError):
            self._mission_running = False

    # --- the step --------------------------------------------------------------

    def _tick(self) -> None:
        now = self._now()
        self._monitor.presence(self._presence.read(), now)
        idle = not self._mission_running and now - self._moving_at >= IDLE_HOLD_S
        status = self._monitor.tick(now, idle)
        if status.read_chip:
            self._read_chip(now)
            status = self._monitor.tick(now, idle)
        for event in status.events:
            self.get_logger().info(event)
        if status.manifest is not None:
            self._send_frames(status.manifest)
        self._status = status
        self._publish()

    def _read_chip(self, now: float) -> None:
        try:
            manifest = kit_manifest.decode(self._eeprom.read())
        except (hw.ChipReadError, kit_manifest.ManifestError) as exc:
            self._monitor.chip(None, str(exc), now)
            return
        self._monitor.chip(manifest, "", now)

    def _send_frames(self, manifest) -> None:
        key = (manifest.serial, manifest.calib_id)
        if key == self._frames_of:
            return
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for name, pose in sorted(manifest.frames.items()):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = "kit_mount"
            tf.child_frame_id = name
            tf.transform.translation.x = pose.x
            tf.transform.translation.y = pose.y
            tf.transform.translation.z = pose.z
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = quaternion(
                pose.roll, pose.pitch, pose.yaw)
            transforms.append(tf)
        self._tf.sendTransform(transforms)
        self._frames_of = key
        self.get_logger().info(
            f"kit frames from the chip (calib {manifest.calib_id}): {sorted(manifest.frames)}")

    def _publish(self, force: bool = False) -> None:
        status = getattr(self, "_status", None)
        if status is None:
            return
        body = {
            "state": status.state, "present": status.present,
            "kit": kit_manifest.to_dict(status.manifest) if status.manifest else None,
            "error": status.error, "missing": list(status.missing),
            "gps_source": status.gps_source, "gps_wanted": status.gps_wanted,
            "gnss": self._gnss if status.manifest else None,
        }
        text = json.dumps(body, sort_keys=True)
        if text == self._published and not force:
            return
        self._published = text
        self._pub.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KitManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
