"""The robot with the orchadOS kit's mast on it: how far before it tips. ROS-free.

docs/kit-soketi.md ("Soket — mekanik") took one decision on a hand
calculation: the speed limits do NOT change with the kit, because a mast on
the plate in the middle of the support polygon leaves ~21° of fore-aft
margin. This module is that calculation, written down so it can be re-run
when the robot and the kit are weighed, and the Gazebo half
(test_gazebo_kit.py) checks it against physics.

Two numbers per direction, from the combined centre of mass (CoM) and the
support polygon (the ground contacts):
  - static: the slope it stands on before the CoM leaves the polygon,
    atan(d / h) — d the CoM's horizontal distance to that edge, h its height;
  - dynamic: the braking (or launching) deceleration that lifts it off the
    far contacts, a = g d / h. Braking pivots on the FRONT contacts, a hard
    start on the REAR.

⚠️ THE NUMBERS ARE ONLY AS GOOD AS THE MASSES. Neither the robot nor the kit
has been weighed (robot.urdf.xacro: "TODO: weigh it"); the defaults here are
the URDF's and docs/kit-soketi.md's estimates.

⚠️ THE GAZEBO MODEL IS NOT THE URDF'S CHASSIS. hoverbot.sdf stands on a front
AND a rear caster (±0.28 m, a diamond); the URDF on two front casters
(x = 0.22). The chassis is not built yet (B3), so `kit_sdf` re-seats
the casters in the URDF layout for the kit tests: the risky layout is the one
measured.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

G = 9.80665
XY = Tuple[float, float]


@dataclass(frozen=True)
class Mass:
    name: str
    kg: float
    x: float
    y: float
    z: float


# robot.urdf.xacro: base_mass 12 kg on the chassis (centre x = caster_x / 2,
# z = base_z + base_height / 2), wheels 2.5 kg at the axle. Ground at z = 0.
ROBOT = (Mass("chassis", 12.0, 0.11, 0.0, 0.1425),
         Mass("left_wheel", 2.5, 0.0, 0.25, 0.0825),
         Mass("right_wheel", 2.5, 0.0, -0.25, 0.0825))
# docs/kit-soketi.md: ~4.5 kg, battery at the mast's foot -> CoM ~0.9 m up.
KIT_MASS_KG = 4.5
KIT_COM_Z = 0.9
KIT_MOUNT = (0.11, 0.0)           # robot.urdf.xacro kit_mount
# Ground contacts: the two drive wheels, the two front casters (URDF).
URDF_SUPPORT = ((0.0, 0.25), (0.22, 0.15), (0.22, -0.15), (0.0, -0.25))


def kit_mass(mount: XY = KIT_MOUNT, kg: float = KIT_MASS_KG, com_z: float = KIT_COM_Z) -> Mass:
    return Mass("kit", kg, mount[0], mount[1], com_z)


def centre_of_mass(masses: Sequence[Mass]) -> Tuple[float, float, float, float]:
    total = sum(m.kg for m in masses)
    return (total, sum(m.kg * m.x for m in masses) / total,
            sum(m.kg * m.y for m in masses) / total, sum(m.kg * m.z for m in masses) / total)


def _edge_distance(point: XY, a: XY, b: XY) -> float:
    """Signed distance from `point` to edge a->b, positive inside a CCW polygon."""
    ex, ey = b[0] - a[0], b[1] - a[1]
    return (ex * (point[1] - a[1]) - ey * (point[0] - a[0])) / math.hypot(ex, ey)


def margins(masses: Sequence[Mass], support: Sequence[XY] = URDF_SUPPORT) -> Dict[str, float]:
    """Tip margins of the combined CoM over `support` (a convex polygon, CCW).

    front/rear/left/right: the edge the robot would tip over, as a static
    slope (deg) — "front" is the slope it stands on nose-down. brake_g /
    launch_g: the deceleration / acceleration (in g) that lifts it off its
    rear / front contacts."""
    _total, cx, cy, h = centre_of_mass(masses)
    point = (cx, cy)
    area2 = sum(support[i][0] * support[(i + 1) % len(support)][1]
                - support[(i + 1) % len(support)][0] * support[i][1]
                for i in range(len(support)))
    if area2 < 0:                       # clockwise given: walk it the other way
        support = tuple(reversed(support))
    edges = [(support[i], support[(i + 1) % len(support)]) for i in range(len(support))]
    out = {}
    for name, outward in (("front", (1.0, 0.0)), ("rear", (-1.0, 0.0)),
                          ("left", (0.0, 1.0)), ("right", (0.0, -1.0))):
        # the edge the CoM crosses moving along `outward`
        best = math.inf
        for a, b in edges:
            ex, ey = b[0] - a[0], b[1] - a[1]
            nx, ny = ey / math.hypot(ex, ey), -ex / math.hypot(ex, ey)   # outward normal (CCW)
            if nx * outward[0] + ny * outward[1] <= 1e-9:
                continue
            d = _edge_distance(point, a, b)
            best = min(best, d / (nx * outward[0] + ny * outward[1]))
        out[name] = math.degrees(math.atan2(best, h))
    out["brake_g"] = math.tan(math.radians(out["front"]))
    out["launch_g"] = math.tan(math.radians(out["rear"]))
    out["com_x"], out["com_y"], out["com_z"] = cx, cy, h
    return out


# --- the Gazebo model -----------------------------------------------------------

def kit_sdf(base: str, kg: float = KIT_MASS_KG, com_z: float = KIT_COM_Z,
            mount: XY = KIT_MOUNT, urdf_casters: bool = True,
            mast_height: float = 2.1) -> str:
    """hoverbot.sdf with the kit on its plate: a mast link (its mass at
    `com_z` above the ground, its collision a thin cylinder up to
    `mast_height`), fixed to base_link; the casters re-seated in the URDF's
    front pair if `urdf_casters`; and a 3-D OdometryPublisher
    (/model/hoverbot/pose_3d) — the stock one is 2-D and cannot see pitch."""
    text = base
    if urdf_casters:
        # ...and the URDF's masses, so the physics and `margins` weigh the same
        # robot: the chassis 12 kg at the URDF chassis centre (x 0.11, z
        # 0.1425 — base_link sits at z 0.18 here), the wheels 2.5 kg each.
        # hoverbot.sdf's own 15 kg sit at x = 0, right over the axle: with the
        # front casters only, that robot rests on its tail at the slightest push.
        text = re.sub(r"(<link name=\"base_link\">\s*<pose>[^<]*</pose>\s*<inertial>)\s*<mass>[^<]*</mass>",
                      r"\g<1>\n        <pose>0.11 0 -0.0375 0 0 0</pose>\n        <mass>12</mass>",
                      text, count=1)
        text = re.sub(r"(<link name=\"(left|right)_wheel\">\s*<pose>[^<]*</pose>\s*<inertial>\s*)<mass>1</mass>",
                      r"\g<1><mass>2.5</mass>", text, count=2)
        # base_link sits at z = 0.18; the casters reach the ground at -0.135.
        text = _replace_pose(text, "caster_front", "0.22 0.15 -0.135 0 0 0")
        text = _replace_pose(text, "caster_front_visual", "0.22 0.15 -0.135 0 0 0")
        text = _replace_pose(text, "caster_rear", "0.22 -0.15 -0.135 0 0 0")
        text = _replace_pose(text, "caster_rear_visual", "0.22 -0.15 -0.135 0 0 0")
    # A rod's inertia about its CoM, mast-like: most of the mass low (battery),
    # the rest spread up the mast — a 2 m rod of the whole mass is the
    # pessimistic bound and is what is used.
    ixx = kg * mast_height ** 2 / 12.0
    # a thin rod spins easily about its own axis; and izz must stay within
    # ixx + iyy or the engine refuses the model (it did, for a near-massless kit)
    izz = 0.05 * ixx
    kit = f"""
    <link name="kit">
      <pose>{mount[0]} {mount[1]} {com_z} 0 0 0</pose>
      <inertial>
        <mass>{kg}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><iyy>{ixx:.6f}</iyy><izz>{izz:.6f}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="mast">
        <pose>0 0 {mast_height / 2 + 0.27 - com_z:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>{mast_height - 0.27:.4f}</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="kit_joint" type="fixed">
      <parent>base_link</parent><child>kit</child>
    </joint>
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>sim_world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/model/hoverbot/pose_3d</odom_topic>
      <odom_publish_frequency>100</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
"""
    marker = "  </model>"
    if marker not in text:
        raise ValueError("hoverbot.sdf: no </model> to insert the kit before")
    return text.replace(marker, kit + marker, 1)


def tilted_world(base: str, slope_deg: float, downhill_deg: float = 0.0) -> str:
    """The world with gravity tilted: a uniform slope of `slope_deg` whose
    downhill direction is `downhill_deg` (0 = +x, the robot's nose at spawn).
    Tilting gravity is exactly a tilted plane for a rigid body standing on it,
    and keeps the spawn and the ground plane as they are."""
    s = math.radians(slope_deg)
    d = math.radians(downhill_deg)
    gx, gy, gz = G * math.sin(s) * math.cos(d), G * math.sin(s) * math.sin(d), -G * math.cos(s)
    gravity = f"<gravity>{gx:.5f} {gy:.5f} {gz:.5f}</gravity>"
    if "<gravity>" in base:
        return re.sub(r"<gravity>[^<]*</gravity>", gravity, base, count=1)
    marker = '<physics name="1ms"'
    if marker not in base:
        raise ValueError("empty.sdf: no <physics> to put the gravity next to")
    return base.replace(marker, gravity + "\n    " + marker, 1)


def _replace_pose(text: str, collision_name: str, pose: str) -> str:
    pattern = re.compile(rf'(<(?:collision|visual) name="{collision_name}">'
                         rf'(?:(?!</(?:collision|visual)>).)*?<pose>)[^<]*(</pose>)', re.DOTALL)
    new, count = pattern.subn(rf"\g<1>{pose}\g<2>", text, count=1)
    if count != 1:
        raise ValueError(f"hoverbot.sdf: no <pose> under {collision_name!r}")
    return new


def summary(m: Dict[str, float]) -> List[str]:
    return [f"CoM x {m['com_x']:.3f} z {m['com_z']:.3f} m",
            f"slopes: front {m['front']:.1f}°, rear {m['rear']:.1f}°, "
            f"sides {m['left']:.1f}°/{m['right']:.1f}°",
            f"brake {m['brake_g']:.2f} g, launch {m['launch_g']:.2f} g"]
