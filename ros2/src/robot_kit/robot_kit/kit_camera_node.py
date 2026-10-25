"""kit_camera — the kit's 360° camera, driven over USB through the SDK shim.

    kit/state            (String JSON)  in   connects only while a kit with cam360_ctrl is in
    kit_camera/record    (SetBool)      srv  true: start recording, false: stop
    kit/camera/status    (String JSON)  out  1 Hz: {"connected", "recording", "battery_pct",
                                              "storage", "free_bytes", "fault", "files", "error"}

The shim (camera.py) is the only thing that touches the SDK. `shim_command`
names it; empty means ~/.robot/sdk/kit_camera_shim if that exists. No shim:
the node says so and publishes connected=false — kit_manager then reports the
kit DEGRADED (camera silent) and scan missions are refused, rather than
driving a scan that records nothing.

On every connect the camera's clock is set from the robot's
(SyncLocalTimeToCamera), and while recording a (robot ms, camera media time)
pair is appended to `time_log` every `pair_every_s` — what lines the video up
with the GNSS log in orchadOS without the QR step. The robot's clock is only
as good as chrony + PPS make it (docs/kit-soketi.md).

`fault` latches the camera's own complaints (temperature_high, storage_full,
battery_low, capture_stopped) until the next start.
"""

from __future__ import annotations

import json
import os
import shlex
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

from robot_kit.camera import CameraError, ShimClient

DEFAULT_SHIM = os.path.expanduser("~/.robot/sdk/kit_camera_shim")


class KitCamera(Node):
    def __init__(self, **kwargs):
        super().__init__("kit_camera", **kwargs)
        self.declare_parameter("shim_command", "")
        self.declare_parameter("resolution", "5.7K30")
        self.declare_parameter("time_log", os.path.expanduser("~/.robot/kit_camera_time.csv"))
        self.declare_parameter("pair_every_s", 10.0)
        p = self.get_parameter
        command = p("shim_command").value
        if not command and os.access(DEFAULT_SHIM, os.X_OK):
            command = DEFAULT_SHIM
        self._command = shlex.split(command) if command else []
        self._resolution = p("resolution").value
        self._time_log = p("time_log").value
        self._pair_every = p("pair_every_s").value
        self._wanted = False
        self._shim = None
        self._info = {}
        self._status = {}
        self._fault = None
        self._files = []
        self._error = "" if self._command else "no camera shim (the SDK is not installed)"
        self._next_pair = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, "kit/camera/status", 10)
        self.create_subscription(String, "kit/state", self._on_kit, latched)
        self.create_service(SetBool, "kit_camera/record", self._on_record)
        self.create_timer(1.0, self._tick)
        if not self._command:
            self.get_logger().warn(self._error)

    def _on_kit(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        kit = data.get("kit") or {}
        self._wanted = data.get("state") != "NONE" and "cam360_ctrl" in kit.get("caps", [])

    def _connect(self) -> None:
        try:
            self._shim = ShimClient(self._command)
            self._info = self._shim.call("connect")
            self._shim.call("sync_time", utc_ms=int(time.time() * 1000), tz_offset_s=0)
            self._error = ""
            self.get_logger().info(f"camera {self._info.get('model')} "
                                   f"{self._info.get('serial')} connected, clock set")
        except (CameraError, OSError) as exc:
            self._error = str(exc)
            self._drop()

    def _drop(self) -> None:
        if self._shim is not None:
            self._shim.close()
        self._shim = None
        self._status = {}

    def _on_record(self, request, response):
        if self._shim is None:
            response.success, response.message = False, self._error or "camera not connected"
            return response
        try:
            if request.data:
                self._shim.call("start", resolution=self._resolution)
                self._fault = None
                self._next_pair = 0.0
                response.message = "recording"
            else:
                files = self._shim.call("stop").get("files", [])
                self._files = files
                response.message = f"stopped: {', '.join(files)}"
            response.success = True
        except CameraError as exc:
            response.success, response.message = False, str(exc)
        self._tick()
        return response

    def _tick(self) -> None:
        if self._wanted and self._command and (self._shim is None or not self._shim.alive()):
            self._drop()
            self._connect()
        elif not self._wanted and self._shim is not None:
            self._drop()
        if self._shim is not None:
            try:
                self._status = self._shim.call("status")
                for event in self._shim.events():
                    self._fault = event if self._fault is None else self._fault
                    self.get_logger().warn(f"camera: {event}")
                if self._status.get("recording") and time.monotonic() >= self._next_pair:
                    self._log_pair()
                    self._next_pair = time.monotonic() + self._pair_every
            except CameraError as exc:
                self._error = str(exc)
                self._drop()
        self._pub.publish(String(data=json.dumps({
            "connected": self._shim is not None,
            "recording": bool(self._status.get("recording")),
            "battery_pct": self._status.get("battery_pct"),
            "storage": self._status.get("storage"),
            "free_bytes": self._status.get("free_bytes"),
            "fault": self._fault, "files": self._files, "error": self._error,
            "camera": self._info or None,
        })))

    def _log_pair(self) -> None:
        pair = self._shim.time_pair(lambda: int(time.time() * 1000))
        os.makedirs(os.path.dirname(self._time_log) or ".", exist_ok=True)
        new = not os.path.exists(self._time_log)
        with open(self._time_log, "a", encoding="utf-8") as handle:
            if new:
                handle.write("robot_utc_ms,camera_media_time,round_trip_ms\n")
            handle.write(f"{pair.robot_ms},{pair.camera_media},{pair.round_trip_ms:.1f}\n")

    def destroy_node(self) -> bool:
        self._drop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KitCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
