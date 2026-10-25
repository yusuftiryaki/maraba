"""trunk_localizer — the robot's pose from the trunks the kit's camera sees.

    kit/trunks  (robot_missions_interfaces/TrunkDetections)   in
    odometry/filtered/global (Odometry)                       in   the prior
    site_file: the trees (poi "ağaç" + radius) -> /fromLLArray -> `map`
    kit/trunk_pose  (PoseWithCovarianceStamped, map)  out  -> ekf_global pose0 (ekf.yaml)
    kit/trunk_status (String JSON)                    out  per frame: accepted / why not
    kit/tree_audit  (String JSON, latched, 2 s)       out  {"checked", "missing": [codes],
                                                            "new": [{"x", "y", "lat", "lon", "views"}]}

The decisions are in trunk_localizer.py (ROS-free). The tree map is orchadOS's
inventory and only a PRIOR: the audit reports what disagrees with it, and
nothing here edits the site (docs/kit-soketi.md, "Ağaç haritası").

The camera's place comes from TF (base_link -> kit_cam360), i.e. from the
kit's identity chip — its height is the z of that transform, base_link being
at ground level (robot.urdf.xacro).

⚠️ A WRONG FIX IS WORSE THAN NONE: it would put the robot a row over, next to
the trunks. So the localizer refuses ambiguous or thin frames, and ekf_global
also rejects a fix too far from its own estimate (pose0_rejection_threshold).
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.srv import FromLLArray, ToLL
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_missions_interfaces.msg import TrunkDetections
from robot_kit.trunk_localizer import TreeAudit, TrunkLocalizer

TREE_LABEL = "ağaç"


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrunkLocalizerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("trunk_localizer", **kwargs)
        self.declare_parameter("site_file", "")
        self.declare_parameter("max_rate_hz", 2.0)
        self.declare_parameter("camera_frame", "kit_cam360")
        p = self.get_parameter
        self._site_file = p("site_file").value
        self._period = 1.0 / max(0.1, p("max_rate_hz").value)
        self._camera_frame = p("camera_frame").value
        self._trees = None               # [(name, x, y, r)] in map
        self._site_mtime = None
        self._loc = None
        self._audit = None
        self._camera = None
        self._prior = None
        self._last = 0.0
        self._converting = False
        self._new_ll = {}                # (x, y) -> (lat, lon)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "kit/trunk_pose", 10)
        self._status_pub = self.create_publisher(String, "kit/trunk_status", 10)
        self._audit_pub = self.create_publisher(String, "kit/tree_audit", latched)
        self._from_ll = self.create_client(FromLLArray, "fromLLArray")
        self._to_ll = self.create_client(ToLL, "toLL")
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(Odometry, "odometry/filtered/global", self._on_prior, 10)
        self.create_subscription(TrunkDetections, "kit/trunks", self._on_trunks, 5)
        self.create_timer(2.0, self._housekeeping)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- the tree map ------------------------------------------------------------

    def _housekeeping(self) -> None:
        if self._site_file and os.path.exists(self._site_file):
            mtime = os.path.getmtime(self._site_file)
            if mtime != self._site_mtime and not self._converting:
                self._convert_trees(mtime)
        if self._audit is not None:
            self._publish_audit()

    def _convert_trees(self, mtime) -> None:
        from robot_missions.site import SiteError, load_site

        try:
            site = load_site(self._site_file, missing_ok=True)
        except (SiteError, OSError) as exc:
            self.get_logger().warn(f"site: {exc}")
            self._site_mtime = mtime
            return
        trees = [t for t in site.pois if t.label == TREE_LABEL and t.radius_m is not None]
        if not trees:
            self._site_mtime = mtime
            self._trees, self._loc, self._audit = [], None, None
            return
        if not self._from_ll.service_is_ready():
            return
        request = FromLLArray.Request(ll_points=[
            GeoPoint(latitude=t.position.lat, longitude=t.position.lon) for t in trees])
        self._converting = True

        def done(future):
            self._converting = False
            result = future.result()
            # ⚠️ docs/missions.md decision 5: a short answer is "not ready", never trees.
            if result is None or len(result.map_points) != len(trees):
                return
            self._trees = [(t.name, pt.x, pt.y, t.radius_m)
                           for t, pt in zip(trees, result.map_points)]
            self._site_mtime = mtime
            self._loc = None              # rebuilt with the camera on the next frame
            self.get_logger().info(f"{len(self._trees)} trees from the site")
        self._from_ll.call_async(request).add_done_callback(done)

    # --- the frames --------------------------------------------------------------

    def _on_prior(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._prior = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _camera_in_base(self):
        try:
            tf = self._tf.lookup_transform("base_link", self._camera_frame, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z, yaw_of(tf.transform.rotation))

    def _on_trunks(self, msg: TrunkDetections) -> None:
        now = time.monotonic()
        if now - self._last < self._period or self._prior is None or not self._trees:
            return
        self._last = now
        camera = self._camera_in_base()
        if camera is None:
            return self._status("no TF base_link -> kit camera (kit chip not read?)")
        if self._loc is None or camera != self._camera:
            self._camera = camera
            self._loc = TrunkLocalizer(self._trees, camera)
            self._audit = TreeAudit(self._loc)
        detections = list(zip(msg.bearing, msg.depression))
        verdict = self._loc.locate(self._prior, detections)
        if verdict.fix is None:
            return self._status(verdict.reason, seen=len(detections))
        fix = verdict.fix
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.pose.pose.position.x, out.pose.pose.position.y = fix.x, fix.y
        out.pose.pose.orientation.z = math.sin(fix.yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(fix.yaw / 2.0)
        cov = [0.0] * 36
        axes = (0, 1, 5)                                   # x, y, yaw of the 6x6
        for i, a in enumerate(axes):
            for j, b in enumerate(axes):
                cov[a * 6 + b] = float(fix.cov[i, j])
        cov[14] = cov[21] = cov[28] = 1e6                 # z, roll, pitch: not measured
        out.pose.covariance = cov
        self._pose_pub.publish(out)
        self._audit.observe(fix)
        self._status("", seen=len(detections), matched=len(fix.matches),
                     rms_deg=round(fix.rms_deg, 3),
                     sigma_m=round(math.sqrt(max(fix.cov[0, 0], fix.cov[1, 1])), 3))

    def _status(self, reason: str, **extra) -> None:
        self._status_pub.publish(String(data=json.dumps(
            {"accepted": not reason, "reason": reason, **extra})))

    def _publish_audit(self) -> None:
        report = self._audit.report()
        for item in report["new"]:
            key = (item["x"], item["y"])
            if key not in self._new_ll and self._to_ll.service_is_ready():
                self._new_ll[key] = None
                request = ToLL.Request(map_point=Point(x=float(item["x"]), y=float(item["y"])))

                def done(future, key=key):
                    result = future.result()
                    # (0, 0) is /toLL's "not ready", never a place (missions.md decision 5)
                    ll = (result.ll_point.latitude, result.ll_point.longitude) if result else None
                    if ll is not None and ll != (0.0, 0.0):
                        self._new_ll[key] = ll
                    else:
                        self._new_ll.pop(key, None)
                self._to_ll.call_async(request).add_done_callback(done)
            ll = self._new_ll.get(key)
            if ll:
                item["lat"], item["lon"] = round(ll[0], 8), round(ll[1], 8)
        self._audit_pub.publish(String(data=json.dumps(report)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrunkLocalizerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
