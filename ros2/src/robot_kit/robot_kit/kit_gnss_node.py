"""kit_gnss — the kit's ZED-F9P over the socket's USB, read-only but for RTCM.

    port (/dev/kit_gnss, udev)        UBX-NAV-PVT in, RTCM out
    kit/gnss/rtcm   (UInt8MultiArray) in   corrections from ntrip_client
    kit/gnss/fix    (NavSatFix)       out  frame kit_gnss, covariance from hAcc/vAcc
    kit/gnss/vel    (TwistStamped)    out  east/north m/s, like gps/vel
    kit/gnss/status (String JSON)     out  {"solution", "num_sv", "h_acc_m",
                                            "rtcm_frames", "rtcm_age_s", "port"}

⚠️ THIS NODE NEVER CONFIGURES THE RECEIVER (ubx.py): no CFG, no rate change,
no message enabling. orchadOS owns the F9P's configuration — the kit's ESP32
logger depends on it for PPK. If NAV-PVT is not enabled on the USB port, this
node says so (no fix for `silent_warn_s`) and that is fixed in orchadOS's
receiver config, never from here.

The port disappears when the kit is unplugged; the node keeps retrying it
every second rather than dying (kit_manager already knows the plug is gone).
Corrections are written only as whole frames with a good CRC (rtcm.py).
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String, UInt8MultiArray

from robot_kit import rtcm, ubx

SERVICES = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_GALILEO | NavSatStatus.SERVICE_COMPASS)


class KitGnss(Node):
    def __init__(self, **kwargs):
        super().__init__("kit_gnss", **kwargs)
        self.declare_parameter("port", "/dev/kit_gnss")
        self.declare_parameter("baud", 460800)    # ignored by USB CDC; matters on a UART
        self.declare_parameter("frame_id", "kit_gnss")
        self.declare_parameter("silent_warn_s", 10.0)
        p = self.get_parameter
        self._port_name = p("port").value
        self._baud = p("baud").value
        self._frame_id = p("frame_id").value
        self._silent_warn_s = p("silent_warn_s").value
        self._port = None
        self._parser = ubx.UbxParser()
        self._rtcm = rtcm.RtcmParser()
        self._last_pvt = None
        self._last_fix_at = None
        self._opened_at = None
        self._warned_silent = False
        self._rtcm_frames = 0
        self._rtcm_at = None

        self._fix_pub = self.create_publisher(NavSatFix, "kit/gnss/fix", 10)
        self._vel_pub = self.create_publisher(TwistStamped, "kit/gnss/vel", 10)
        self._status_pub = self.create_publisher(String, "kit/gnss/status", 10)
        self.create_subscription(UInt8MultiArray, "kit/gnss/rtcm", self._on_rtcm, 50)
        self.create_timer(0.02, self._read)
        self.create_timer(1.0, self._housekeeping)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- the port ----------------------------------------------------------------

    def _open(self) -> None:
        import serial

        try:
            self._port = serial.Serial(self._port_name, self._baud, timeout=0)
        except (serial.SerialException, OSError):
            self._port = None
            return
        self._opened_at = self._now()
        self._warned_silent = False
        self._parser = ubx.UbxParser()
        self.get_logger().info(f"kit receiver on {self._port_name}")

    def _close(self, why: str) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except OSError:
                pass
            self.get_logger().warn(f"kit receiver lost ({why})")
        self._port = None

    def _read(self) -> None:
        if self._port is None:
            return
        try:
            data = self._port.read(4096)
        except Exception as exc:                 # serial.SerialException on unplug
            self._close(str(exc))
            return
        if data:
            for pvt in ubx.nav_pvts(self._parser, data):
                self._publish(pvt)

    def _on_rtcm(self, msg: UInt8MultiArray) -> None:
        frames = [frame for _type, frame in self._rtcm.feed(bytes(msg.data))]
        whole = b"".join(frames)
        if not whole or self._port is None:
            return
        try:
            self._port.write(whole)
        except Exception as exc:
            self._close(str(exc))
            return
        self._rtcm_frames += len(frames)
        self._rtcm_at = self._now()

    # --- out -----------------------------------------------------------------------

    def _publish(self, pvt: ubx.NavPvt) -> None:
        stamp = self.get_clock().now().to_msg()
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_id
        fix.status.status = ubx.fix_status(pvt)
        fix.status.service = SERVICES
        if pvt.has_position():
            fix.latitude, fix.longitude, fix.altitude = pvt.lat_deg, pvt.lon_deg, pvt.height_m
            east, north, up = ubx.covariance_enu(pvt)
            fix.position_covariance = [east, 0.0, 0.0, 0.0, north, 0.0, 0.0, 0.0, up]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._last_fix_at = self._now()
        else:
            fix.latitude = fix.longitude = fix.altitude = math.nan
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._fix_pub.publish(fix)
        if pvt.has_position():
            vel = TwistStamped()
            vel.header.stamp = stamp
            vel.header.frame_id = self._frame_id
            vel.twist.linear.x, vel.twist.linear.y = pvt.vel_e, pvt.vel_n
            self._vel_pub.publish(vel)
        self._last_pvt = pvt

    def _housekeeping(self) -> None:
        now = self._now()
        if self._port is None:
            self._open()
        elif (not self._warned_silent and now - (self._last_fix_at or self._opened_at)
              > self._silent_warn_s):
            self.get_logger().warn(
                f"no UBX-NAV-PVT on {self._port_name} for {self._silent_warn_s:.0f} s — is "
                "it enabled on the receiver's USB port? (orchadOS owns that config; this "
                "node never changes it)")
            self._warned_silent = True
        pvt = self._last_pvt
        self._status_pub.publish(String(data=json.dumps({
            "solution": ubx.solution_name(pvt),
            "num_sv": pvt.num_sv if pvt else 0,
            "h_acc_m": round(pvt.h_acc_m, 3) if pvt and pvt.has_position() else None,
            "rtcm_frames": self._rtcm_frames,
            "rtcm_age_s": round(now - self._rtcm_at, 1) if self._rtcm_at else None,
            "port": self._port is not None,
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KitGnss()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
