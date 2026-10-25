"""ntrip_client — RTK corrections from a caster (TUSAGA-Aktif) to the kit's F9P.

    caster (TCP)                    RTCM in, GGA out (ntrip.py)
    kit/gnss/fix   (NavSatFix)      in   the position the GGA reports
    kit/state      (String JSON)    in   connects only while a kit with gnss_rtk is in
    kit/gnss/rtcm  (UInt8MultiArray) out whole RTCM frames, to kit_gnss
    kit/ntrip/status (String JSON)  out  {"state", "error", "frames", "bytes", "age_s"}

The caster is configured from a file, not launch arguments: `credentials_file`
(YAML: host, port, mountpoint, user, password, version), kept outside the repo
(~/.robot/ntrip.yaml). Empty or missing file: the node idles and says so —
the F9P then runs single-point, and orchadOS's PPK still works from the raw
log, which this does not touch.

Socket work happens on a thread; ROS callbacks only hand data across.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, UInt8MultiArray

from robot_kit import ntrip, rtcm, ubx


def load_caster(path: str) -> Optional[ntrip.Caster]:
    if not path or not os.path.exists(os.path.expanduser(path)):
        return None
    import yaml

    with open(os.path.expanduser(path), encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not data.get("host"):
        return None
    return ntrip.Caster(host=str(data["host"]), port=int(data.get("port", 2101)),
                        mountpoint=str(data.get("mountpoint", "")),
                        user=str(data.get("user", "")), password=str(data.get("password", "")),
                        version=int(data.get("version", 2)))


class NtripClient(Node):
    def __init__(self, **kwargs):
        super().__init__("ntrip_client", **kwargs)
        self.declare_parameter("credentials_file", os.path.expanduser("~/.robot/ntrip.yaml"))
        self.declare_parameter("gga_interval_s", 10.0)
        self.declare_parameter("timeout_s", 15.0)
        p = self.get_parameter
        self._caster = load_caster(p("credentials_file").value)
        self._gga_interval = p("gga_interval_s").value
        self._timeout = p("timeout_s").value
        self._lock = threading.Lock()
        self._fix = None                       # (lat, lon, alt, status)
        self._wanted = False
        self._status = {"state": "idle", "error": "", "frames": 0, "bytes": 0, "age_s": None}
        self._last_frame_at = None
        self._stop = threading.Event()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._rtcm_pub = self.create_publisher(UInt8MultiArray, "kit/gnss/rtcm", 50)
        self._status_pub = self.create_publisher(String, "kit/ntrip/status", latched)
        self.create_subscription(NavSatFix, "kit/gnss/fix", self._on_fix, 10)
        self.create_subscription(String, "kit/state", self._on_state, latched)
        self.create_timer(1.0, self._publish_status)
        if self._caster is None:
            self._set(state="unconfigured",
                      error=f"no caster in {p('credentials_file').value}")
            self.get_logger().info("no NTRIP caster configured: the kit runs without RTK")
        else:
            threading.Thread(target=self._loop, daemon=True).start()

    # --- ROS side ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status == ubx.STATUS_NO_FIX:
            return
        with self._lock:
            self._fix = (msg.latitude, msg.longitude, msg.altitude, msg.status.status)

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        kit = data.get("kit") or {}
        with self._lock:
            self._wanted = data.get("state") != "NONE" and "gnss_rtk" in kit.get("caps", [])

    def _set(self, **changes) -> None:
        with self._lock:
            self._status.update(changes)

    def _publish_status(self) -> None:
        with self._lock:
            status = dict(self._status)
            if self._last_frame_at is not None:
                status["age_s"] = round(time.monotonic() - self._last_frame_at, 1)
        self._status_pub.publish(String(data=json.dumps(status)))

    # --- the socket thread ---------------------------------------------------------

    def _gga(self) -> Optional[bytes]:
        with self._lock:
            fix = self._fix
        if fix is None:
            return None
        lat, lon, alt, status = fix
        utc = time.gmtime()
        return ntrip.gga(lat, lon, alt, (utc.tm_hour, utc.tm_min, float(utc.tm_sec)),
                         quality=ntrip.gga_quality(status))

    def _loop(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            with self._lock:
                wanted = self._wanted
            if not wanted:
                self._set(state="waiting for kit")
                time.sleep(1.0)
                continue
            try:
                self._session()
                attempt = 0
            except (OSError, ntrip.NtripError) as exc:
                self._set(state="retrying", error=str(exc))
                self.get_logger().warn(f"NTRIP: {exc}")
            delay = ntrip.backoff_s(attempt)
            attempt += 1
            self._stop.wait(delay)

    def _session(self) -> None:
        caster = self._caster
        self._set(state="connecting", error="")
        with socket.create_connection((caster.host, caster.port), timeout=self._timeout) as sock:
            sock.settimeout(1.0)
            sock.sendall(ntrip.request(caster))
            head = b""
            deadline = time.monotonic() + self._timeout
            response = None
            while response is None:
                if time.monotonic() > deadline:
                    raise ntrip.NtripError("the caster did not answer")
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ntrip.NtripError("the caster closed the connection")
                head += chunk
                response = ntrip.parse_response(head)
            self._set(state="connected")
            self.get_logger().info(f"NTRIP connected to {caster.host}/{caster.mountpoint}")
            dechunker = ntrip.Dechunker() if response.chunked else None
            parser = rtcm.RtcmParser()
            next_gga = 0.0
            last_data = time.monotonic()
            pending = response.body_start
            while not self._stop.is_set():
                with self._lock:
                    wanted = self._wanted
                if not wanted:
                    self._set(state="waiting for kit")
                    return
                now = time.monotonic()
                if now >= next_gga:
                    gga = self._gga()
                    if gga is not None:
                        sock.sendall(gga)
                        next_gga = now + self._gga_interval
                if pending:
                    data, pending = pending, b""
                else:
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        data = None
                    if data == b"":
                        raise ntrip.NtripError("the caster closed the connection")
                if not data:
                    if time.monotonic() - last_data > self._timeout:
                        raise ntrip.NtripError(
                            f"no corrections for {self._timeout:.0f} s (VRS waiting for a GGA?)")
                    continue
                last_data = time.monotonic()
                body = dechunker.feed(data) if dechunker else data
                frames = [frame for _type, frame in parser.feed(body)]
                if frames:
                    self._rtcm_pub.publish(UInt8MultiArray(data=list(b"".join(frames))))
                    with self._lock:
                        self._status["frames"] += len(frames)
                        self._status["bytes"] += sum(len(f) for f in frames)
                        self._last_frame_at = time.monotonic()

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NtripClient()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
