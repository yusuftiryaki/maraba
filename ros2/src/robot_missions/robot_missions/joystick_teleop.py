"""joystick_teleop — drive with the Bluetooth remote (docs/superpowers/specs/2026-11-20-bluetooth-kumanda-design.md).

    cmd_vel_joy (out, 20 Hz)   only while the gate has an output; silence hands
                               the wheels back through twist_mux. Never /cmd_vel.
    joy/state (out, 10 Hz)     JSON state for the web page / recorder
    joystick/fake_events (in)  fake:=true only — JSON {"kind","code","value"};
                               evdev is never touched, battery is not polled

All events go through apply_event. The gate clock is time.monotonic, also
when use_sim_time is true: the deadman lives in the human's time.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_missions.joystick_gate import (Buttons, JoyEvent, JoystickGate,
                                          apply_event)
from robot_missions.joystick_source import EvdevSource, bluez_battery

_KINDS = ("rel", "key", "other", "connected", "battery")


def parse_fake_event(text: str) -> JoyEvent:
    """Parse a fake_events message; raises ValueError/TypeError/KeyError if bad."""
    d = json.loads(text)
    kind, code, value = d["kind"], d["code"], d["value"]
    if kind not in _KINDS or not isinstance(code, str):
        raise ValueError("bad kind/code")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("value must be a number")
    value = float(value)
    if math.isinf(value) or (math.isnan(value) and kind != "battery"):
        raise ValueError("value must be finite")
    return JoyEvent(kind, code, value)


class JoystickTeleop(Node):
    def __init__(self, **kwargs):
        super().__init__("joystick_teleop", **kwargs)
        p = self.declare_parameter
        p("fake", False)
        p("device_name", "VR-PARK")
        p("device_mac", "")
        p("deadman_s", 0.25)
        p("release_s", 2.0)
        p("arm_idle_s", 60.0)
        p("min_battery_pct", 25.0)
        p("reverse_max", 0.2)
        p("linear_levels", [0.2, 0.4, 0.6])
        p("angular_levels", [0.4, 0.7, 1.0])
        p("invert_x", False)
        p("arm_button", "BTN_SIDE")
        p("speed_up_button", "")
        p("speed_down_button", "")
        p("battery_poll_s", 30.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self._fake = bool(g("fake"))
        self._buttons = Buttons(g("arm_button"), g("speed_up_button"),
                                g("speed_down_button"))
        self._gate = JoystickGate(
            linear_levels=g("linear_levels"), angular_levels=g("angular_levels"),
            reverse_max=g("reverse_max"), deadman_s=g("deadman_s"),
            release_s=g("release_s"), arm_idle_s=g("arm_idle_s"),
            min_battery_pct=g("min_battery_pct"), invert_x=g("invert_x"),
            clock=time.monotonic)

        self._pub = self.create_publisher(Twist, "cmd_vel_joy", 10)
        self._state_pub = self.create_publisher(String, "joy/state", 10)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._tick_state)

        self._source = None
        if self._fake:
            self.create_subscription(String, "joystick/fake_events",
                                     self._on_fake, 10)
        else:
            self._source = EvdevSource(g("device_name"), g("device_mac"),
                                       self._emit, log=self.get_logger().warning)
            self._source.start()
            self.create_timer(float(g("battery_poll_s")), self._poll_battery)

    def _emit(self, event: JoyEvent) -> None:
        try:
            apply_event(self._gate, event, self._buttons)
        except Exception as e:
            self.get_logger().error(f"event dropped: {e!r}")

    def _on_fake(self, msg: String) -> None:
        try:
            event = parse_fake_event(msg.data)
        except Exception as e:
            self.get_logger().warning(f"bad fake event dropped: {e!r}")
            return
        self._emit(event)

    def _poll_battery(self) -> None:
        mac = self._source.mac if self._source else None
        if not mac:
            return
        pct = bluez_battery(mac)
        self._emit(JoyEvent("battery", "", math.nan if pct is None else pct))

    def _tick(self) -> None:
        out = self._gate.output()
        if out is None:
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = out
        self._pub.publish(msg)

    def _tick_state(self) -> None:
        self._state_pub.publish(String(data=json.dumps(self._gate.state())))

    def close(self) -> None:
        if self._source is not None:
            self._source.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JoystickTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
