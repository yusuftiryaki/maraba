"""The orchard grid scan (planner.orchard_grid, mission type orchard_grid). ROS-free.

What the orchadOS scan needs from the drive: every tree passed on all four
sides — so the 360° camera sees it from each — without driving into one.
Measured here on hand-built orchards, regular, rotated, gappy and with a
stray tree; the stack test drives it (test_orchard_stack.py).
"""

import math

import pytest
from shapely.geometry import LineString, Point, Polygon

from robot_missions import planner
from robot_missions.mission import TASK_TYPES, parse_mission, plan_mission
from robot_missions.planner import PlanError
from test_mission import ROBOT, enu, feature, ll, polygon
from robot_missions.site import parse_site

SPACING = 8.0
TRUNK = 0.15
CLEARANCE = ROBOT.robot_radius + ROBOT.obstacle_margin


def grid(cols=5, rows=4, spacing=SPACING, angle_deg=0.0, skip=()):
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    out = []
    for i in range(cols):
        for j in range(rows):
            if (i, j) in skip:
                continue
            x, y = i * spacing, j * spacing
            out.append((c * x - s * y, s * x + c * y))
    return out


def region_round(trees, pad=6.0, angle_deg=0.0):
    hull = Polygon([(-pad, -pad), (4 * SPACING + pad, -pad),
                    (4 * SPACING + pad, 3 * SPACING + pad), (-pad, 3 * SPACING + pad)])
    from shapely import affinity
    area = affinity.rotate(hull, angle_deg, origin=(0, 0))
    return area, planner.free_region(area, [Point(t).buffer(TRUNK) for t in trees],
                                     ROBOT.robot_radius, CLEARANCE)


def work_lines(plan):
    wps = plan.waypoints
    return [LineString([(a.x, a.y), (b.x, b.y)]) for a, b in zip(wps, wps[1:]) if b.work]


def sides_seen(tree, lines, angle_deg, reach=SPACING / 2 + 0.5):
    """Which of the tree's four sides (in the orchard's own frame) a work line
    passes, within `reach`, alongside it."""
    a = math.radians(angle_deg)
    seen = set()
    for line in lines:
        if line.distance(Point(tree)) > reach:
            continue
        nearest = line.interpolate(line.project(Point(tree)))
        dx, dy = nearest.x - tree[0], nearest.y - tree[1]
        fx, fy = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
        if abs(fy) > abs(fx):
            seen.add("north" if fy > 0 else "south")
        else:
            seen.add("east" if fx > 0 else "west")
    return seen


@pytest.mark.parametrize("angle_deg", [0.0, 17.0, 90.0, -33.0])
def test_every_tree_seen_from_four_sides(angle_deg):
    trees = grid(angle_deg=angle_deg)
    _area, region = region_round(trees, angle_deg=angle_deg)
    plan = planner.orchard_grid(region, trees)
    lines = work_lines(plan)
    for tree in trees:
        assert sides_seen(tree, lines, angle_deg) == {"north", "south", "east", "west"}, tree
    # 4 tree rows -> 3 between + 2 outside; 5 columns -> 4 + 2
    assert plan.info["corridors"] == 11
    assert math.degrees(plan.info["angle_rad"]) % 90 == pytest.approx(angle_deg % 90, abs=0.5)


def test_corridors_run_midway_and_clear_the_trunks():
    trees = grid()
    _area, region = region_round(trees)
    plan = planner.orchard_grid(region, trees)
    for line in work_lines(plan):
        nearest = min(line.distance(Point(t)) for t in trees)
        assert nearest == pytest.approx(SPACING / 2, abs=0.01)
    along = sorted({round(w.y, 2) for w in plan.waypoints
                    if abs(math.sin(w.yaw)) < 0.1})
    assert along == [-4.0, 4.0, 12.0, 20.0, 28.0]


def test_gaps_and_scatter_do_not_lose_a_row():
    trees = grid(skip={(1, 1), (2, 1), (3, 1), (4, 3)})
    scattered = [(x + 0.3 * math.sin(7 * x + y), y + 0.3 * math.cos(3 * x - y))
                 for x, y in trees]
    _area, region = region_round(scattered)
    plan = planner.orchard_grid(region, scattered)
    assert plan.info["corridors"] == 11
    lines = work_lines(plan)
    for tree in scattered:
        assert len(sides_seen(tree, lines, 0.0)) == 4


def test_one_direction_is_half_the_corridors():
    trees = grid()
    _area, region = region_round(trees)
    assert planner.orchard_grid(region, trees, directions="one").info["corridors"] == 5


def test_without_trees_it_says_what_to_mark():
    _area, region = region_round([])
    with pytest.raises(PlanError, match="'ağaç'"):
        planner.orchard_grid(region, [])


def test_the_mission_type_needs_the_kit_and_finds_the_trees_in_the_site():
    task = TASK_TYPES["orchard_grid"]
    assert task.requires == ("gnss_rtk", "cam360")
    trees = [feature("poi", f"S{j + 1}-{i + 1}",
                     {"type": "Point", "coordinates": ll(x, y)},
                     label="ağaç", radius_m=TRUNK)
             for (x, y), (i, j) in zip(grid(), [(i, j) for i in range(5) for j in range(4)])]
    well = feature("poi", "kuyu", {"type": "Point", "coordinates": ll(16, 28)},
                   label="kuyu", radius_m=0.6)
    site = parse_site({"type": "FeatureCollection", "features": [
        feature("area", "bahce", polygon((-6, -6), (38, -6), (38, 30), (-6, 30))),
        *trees, well]})
    mission = parse_mission({"type": "orchard_grid", "area": "bahce"}, site)
    plan = plan_mission(site, mission, enu, ROBOT)
    assert plan.info["trees"] == 20, "the well is not a tree"
    assert plan.info["corridors"] == 11
    lines = work_lines(plan)
    # (the clearance circle is a polygon: chords sit a few mm inside it)
    assert min(line.distance(Point(16, 28)) for line in lines) > 0.6 + CLEARANCE - 0.01
