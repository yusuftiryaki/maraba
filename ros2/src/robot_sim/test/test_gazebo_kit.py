"""The kit's mast on the robot, in physics: does it tip? (docs/kit-soketi.md, K2)

robot_sim/kit_model.py computes the margins by hand; this drives the robot
in Gazebo with the mast on its plate and the casters in the URDF's layout
(kit_model.kit_sdf) and measures the chassis pitch:

  - a hard start and a hard stop, 0 <-> 0.5 m/s as a step (the bridge has no
    acceleration limit; DiffDrive applies the step at once and the wheels'
    friction, mu 0.5, is what limits the acceleration);
  - standing on slopes (gravity tilted, kit_model.tilted_world), nose up and
    nose down.

The pitch comes from a 3-D OdometryPublisher (/gazebo/pose_3d) that only the
kit model carries. CI stage `gazebo`. SKIPs without ROS or gz.
"""

import json
import math
import os
import shutil
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")
if shutil.which("gz") is None:
    pytest.skip("gz sim not installed", allow_module_level=True)

import rclpy                                             # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from rclpy.node import Node                              # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile       # noqa: E402
from std_msgs.msg import String                          # noqa: E402

from robot_sim import kit_model                          # noqa: E402
from conftest import GazeboStack                         # noqa: E402

LIFT_DEG = 0.5            # chassis pitch beyond this = it came off a contact


def pitch_of(q):
    """REP-103 pitch, degrees: POSITIVE = NOSE DOWN (x forward rotates toward -z)."""
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.degrees(math.asin(max(-1.0, min(1.0, s))))


class Probe(Node):
    def __init__(self):
        super().__init__("gazebo_kit_probe", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pitch = None
        self.speed = None
        self.accel = []                    # (t, dv/dt) along x, m/s^2
        self.command = (0.0, 0.0)
        self.create_subscription(Odometry, "/gazebo/pose_3d", self._on_pose, 50)
        self.create_timer(0.05, self._republish)

    def _on_pose(self, msg):
        self.pitch = pitch_of(msg.pose.pose.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.twist.linear.x
        if self.speed is not None and t > self.speed[0]:
            self.accel.append((t, (v - self.speed[1]) / (t - self.speed[0])))
        self.speed = (t, v)

    def peak_accel(self):
        """The largest |acceleration| over 0.1 s windows (smooths the 100 Hz
        differences), m/s^2."""
        best, window = 0.0, []
        for t, a in self.accel:
            window.append((t, a))
            while window and t - window[0][0] > 0.1:
                window.pop(0)
            best = max(best, abs(sum(x for _, x in window) / len(window)))
        return best

    def _republish(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.command
        self._pub.publish(msg)

    def run(self, v, seconds):
        """Hold v for `seconds` of sim time; the pitch range seen meanwhile."""
        self.command = (float(v), 0.0)
        start = self.get_clock().now()
        low, high = math.inf, -math.inf
        cap = time.monotonic() + seconds * 6 + 10
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.005)
            assert time.monotonic() < cap, "sim time stalled"
            if self.pitch is not None:
                low, high = min(low, self.pitch), max(high, self.pitch)
        return low, high

    def ready(self, timeout=60.0):
        end = time.monotonic() + timeout
        while self.pitch is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert self.pitch is not None, "no /gazebo/pose_3d — the kit model did not load"


class KitGazeboStack(GazeboStack):
    def __init__(self, workdir, slope_deg=0.0, downhill_deg=0.0, mount=kit_model.KIT_MOUNT,
                 kg=kit_model.KIT_MASS_KG):
        share = get_package_share_directory("robot_sim")
        with open(os.path.join(share, "gazebo", "hoverbot.sdf"), encoding="utf-8") as f:
            model = kit_model.kit_sdf(f.read(), mount=mount, kg=kg)
        with open(os.path.join(share, "gazebo", "empty.sdf"), encoding="utf-8") as f:
            world = kit_model.tilted_world(f.read(), slope_deg, downhill_deg)
        model_path = os.path.join(workdir, "hoverbot_kit.sdf")
        world_path = os.path.join(workdir, "empty_tilted.sdf")
        for path, text in ((model_path, model), (world_path, world)):
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)
        self.GAZEBO_ARGS = (f"model_file:={model_path}", f"world_file:={world_path}")
        super().__init__()


@pytest.fixture
def kit_gazebo(ros, tmp_path):
    made = []

    def _make(**kwargs):
        stack = KitGazeboStack(str(tmp_path), **kwargs)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.command = (0.0, 0.0)
    node.destroy_node()


@pytest.mark.parametrize("kg, ramp, lifts", [(kit_model.KIT_MASS_KG, False, True),
                                             (kit_model.KIT_MASS_KG, True, False),
                                             (0.01, False, False)])
def test_hard_start_and_stop(kit_gazebo, probe, kg, ramp, lifts):
    """A step start lifts the front casters with the mast on and not without
    it — the mast is what does it, not the model (the third case is the
    control). By hand: launch margin 0.29 g with the kit, 0.62 g without; what
    the wheels' friction (mu 0.5) can deliver lies between.

    ramp: the bridge told the kit is on the plate (kit/state present, as
    kit_manager says it), so it limits speeding up (kit_max_linear_accel,
    1.5 m/s^2 ~ 0.15 g): the same step must then NOT lift the nose."""
    masses = (*kit_model.ROBOT, kit_model.kit_mass(kg=kg)) if kg > 0.1 else kit_model.ROBOT
    margins = kit_model.margins(masses)
    kit_gazebo(kg=kg)
    if ramp:
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        kit_state = probe.create_publisher(String, "/kit/state", latched)
        kit_state.publish(String(data=json.dumps({"state": "READY", "present": True})))
    probe.ready()
    probe.run(0.0, 3.0)
    rest = probe.pitch
    probe.accel.clear()
    launch_low, _high = probe.run(0.5, 4.0)
    launch_a = probe.peak_accel()
    probe.accel.clear()
    _low, brake_high = probe.run(0.0, 4.0)
    brake_a = probe.peak_accel()
    nose_up, nose_down = rest - launch_low, brake_high - rest
    print(f"{'kit' if kg > 0.1 else 'no kit'}{' + ramp' if ramp else ''} (URDF casters): hard start {launch_a / kit_model.G:.2f} g "
          f"-> nose up {nose_up:.2f}°; hard stop {brake_a / kit_model.G:.2f} g -> nose down "
          f"{nose_down:.2f}° (by hand: lifts beyond launch {margins['launch_g']:.2f} g, "
          f"brake {margins['brake_g']:.2f} g)")
    assert nose_up < 45.0 and nose_down < 45.0, "it fell over"
    if lifts:
        assert nose_up > LIFT_DEG, "kit_model says a step start lifts the nose; physics disagrees"
    elif ramp:
        assert nose_up < LIFT_DEG, "the speed-up ramp did not keep the nose down"
        assert launch_a < margins["launch_g"] * kit_model.G, "the ramp let a launch past the margin"
    else:
        assert nose_up < LIFT_DEG, "the bare robot lifted its nose: the control failed"


@pytest.mark.parametrize("slope, downhill, holds", [
    (12.0, 180.0, True),     # nose uphill: the weak (rear) side, 16.4° by hand
    (22.0, 180.0, False),
    (20.0, 0.0, True),       # nose downhill: 25.3° by hand
])
def test_standing_on_a_slope(kit_gazebo, probe, slope, downhill, holds):
    kit_gazebo(slope_deg=slope, downhill_deg=downhill)
    probe.ready()
    _low, _high = probe.run(0.0, 6.0)
    # Gravity is tilted, the ground is not: standing, the chassis stays level
    # in the world frame; tipping, it rotates off its contacts.
    tilt = abs(probe.pitch)
    print(f"slope {slope}° (downhill {downhill}°): chassis pitch {probe.pitch:+.1f}°")
    if holds:
        assert tilt < LIFT_DEG, f"came off its contacts on a {slope}° slope"
    else:
        assert tilt > 10.0, f"stood on a {slope}° slope kit_model says it cannot"
