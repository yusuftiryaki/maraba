"""kit_model.py: the tip margins by hand and the Gazebo model it builds. ROS-free."""

import math
import os
import xml.etree.ElementTree as ET

import pytest

from robot_sim import kit_model as k

GAZEBO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gazebo")


def test_a_box_on_a_square_by_hand():
    square = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    m = k.margins([k.Mass("box", 1.0, 0.0, 0.0, 1.0)], square)
    for side in ("front", "rear", "left", "right"):
        assert m[side] == pytest.approx(45.0)
    assert m["brake_g"] == pytest.approx(1.0) and m["launch_g"] == pytest.approx(1.0)
    # clockwise input gives the same answer
    assert k.margins([k.Mass("box", 1.0, 0.0, 0.0, 1.0)], square[::-1])["front"] == \
        pytest.approx(45.0)


def test_the_mast_halves_the_robots_weak_side_and_the_plate_must_not_move_back():
    bare = k.margins(k.ROBOT)
    kit = k.margins((*k.ROBOT, k.kit_mass()))
    behind = k.margins((*k.ROBOT, k.kit_mass(mount=(-0.15, 0.0))))
    # The weak side is the REAR (the drive axle): front casters only.
    assert bare["rear"] < bare["front"] and kit["rear"] < kit["front"]
    assert kit["rear"] == pytest.approx(16.4, abs=0.2)
    assert kit["launch_g"] == pytest.approx(0.29, abs=0.01)
    assert behind["rear"] < 7.0, "the mast where the robot's own mast is: ~6°"
    assert bare["rear"] == pytest.approx(31.9, abs=0.2)


def test_the_kit_model_is_valid_sdf_with_the_urdf_casters_and_a_3d_pose():
    with open(os.path.join(GAZEBO, "hoverbot.sdf"), encoding="utf-8") as handle:
        text = k.kit_sdf(handle.read())
    root = ET.fromstring(text)
    model = root.find("model")
    names = [link.get("name") for link in model.findall("link")]
    assert "kit" in names
    kit = [link for link in model.findall("link") if link.get("name") == "kit"][0]
    assert float(kit.find("inertial/mass").text) == k.KIT_MASS_KG
    base = [link for link in model.findall("link") if link.get("name") == "base_link"][0]
    casters = {c.get("name"): c.find("pose").text.split()
               for c in base.findall("collision") if c.get("name").startswith("caster")}
    assert {tuple(map(float, p[:2])) for p in casters.values()} == {(0.22, 0.15), (0.22, -0.15)}
    topics = [p.find("odom_topic").text for p in model.findall("plugin")
              if p.find("odom_topic") is not None]
    assert "/model/hoverbot/pose_3d" in topics


def test_tilted_world_gravity():
    with open(os.path.join(GAZEBO, "empty.sdf"), encoding="utf-8") as handle:
        text = k.tilted_world(handle.read(), 30.0, 180.0)
    gx, gy, gz = map(float, ET.fromstring(text).find("world/gravity").text.split())
    assert gx == pytest.approx(-k.G / 2, abs=1e-3) and abs(gy) < 1e-3
    assert gz == pytest.approx(-k.G * math.cos(math.radians(30)), abs=1e-3)
