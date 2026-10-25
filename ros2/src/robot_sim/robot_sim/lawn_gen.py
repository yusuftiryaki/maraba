"""Random lawns for learning-mode data (docs/learning.md, 6b). ROS-free.

A lawn is a star-shaped polygon (4-7 corners, convex or not) round sim_world's
origin, where sim_node puts the robot, turned at random — so the robot starts
inside, at least `margin_m` from the edge, facing a random way relative to
it. Each lawn is ACCEPTED only if the privileged expert drives a clean lap of
it on an ideal unicycle: a lawn the expert cannot do would put bad
demonstrations in the data, silently.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon

from robot_sim.edge_expert import (EdgeExpert, ExpertConfig, drive_unicycle, edge_metrics,
                                   orbit_metrics, orbit_path)


@dataclass(frozen=True)
class LawnSpec:
    offset_m: float = 0.4
    radius_m: tuple = (2.5, 5.5)
    corners: tuple = (4, 7)
    area_m2: tuple = (15.0, 70.0)
    min_angle_deg: float = 55.0       # sharper corners than this: the expert struggles
    margin_m: float = 1.0             # robot start to the edge
    width_m: float = 0.3              # spare room inside the offset path


def interior_angles(polygon: Polygon):
    coords = list(polygon.exterior.coords)[:-1]
    n = len(coords)
    for i in range(n):
        ax, ay = coords[i - 1]
        bx, by = coords[i]
        cx, cy = coords[(i + 1) % n]
        a = math.atan2(ay - by, ax - bx)
        c = math.atan2(cy - by, cx - bx)
        yield math.degrees((a - c) % (2 * math.pi))


def _candidate(rng: np.random.Generator, spec: LawnSpec) -> Polygon:
    n = int(rng.integers(spec.corners[0], spec.corners[1] + 1))
    angles = np.sort(rng.uniform(0, 2 * math.pi, n))
    radii = rng.uniform(*spec.radius_m, n)
    polygon = Polygon([(r * math.cos(a), r * math.sin(a)) for a, r in zip(angles, radii)])
    polygon = affinity.rotate(polygon, float(rng.uniform(0, 360)), origin=(0, 0))
    shift = rng.uniform(-1.0, 1.0, 2)
    return affinity.translate(polygon, float(shift[0]), float(shift[1]))


def check(polygon: Polygon, spec: LawnSpec) -> Dict[str, float]:
    """Raises ValueError when the lawn is unusable; else the expert's metrics."""
    if not polygon.is_valid:
        raise ValueError("not a simple polygon")
    if not spec.area_m2[0] <= polygon.area <= spec.area_m2[1]:
        raise ValueError(f"area {polygon.area:.1f} m2")
    origin = Point(0, 0)
    if not polygon.contains(origin) or polygon.exterior.distance(origin) < spec.margin_m:
        raise ValueError("the robot would start too near the edge")
    convex = polygon.exterior.is_ccw
    angles = list(interior_angles(polygon if convex else Polygon(polygon.exterior.coords[::-1])))
    if min(angles) < spec.min_angle_deg:
        raise ValueError(f"a {min(angles):.0f} deg corner")
    room = polygon.buffer(-(spec.offset_m + spec.width_m))
    if room.is_empty or room.geom_type != "Polygon":
        raise ValueError("too narrow somewhere")
    expert = EdgeExpert(polygon, ExpertConfig(offset_m=spec.offset_m))
    poses = drive_unicycle(expert, (0.0, 0.0, 0.0), max_s=400.0)
    if not expert.done:
        raise ValueError("the expert did not finish a lap")
    on = next(i for i, (x, y, _) in enumerate(poses)
              if expert.path.distance(Point(x, y)) < 0.2)
    metrics = edge_metrics(polygon, spec.offset_m, [(x, y) for x, y, _ in poses[on:]])
    if metrics["error_p95"] > 0.15 or metrics["outside"] > 0.0:
        raise ValueError(f"the expert drove it badly: {metrics}")
    return metrics


def random_lawn(seed: int, spec: LawnSpec = LawnSpec(), attempts: int = 500) -> Polygon:
    rng = np.random.default_rng(seed)
    for _ in range(attempts):
        polygon = _candidate(rng, spec)
        try:
            check(polygon, spec)
        except ValueError:
            continue
        return polygon
    raise RuntimeError(f"no usable lawn in {attempts} attempts (seed {seed})")


ROBOT_RADIUS = 0.4                    # sim_node / world.py robot_radius


@dataclass(frozen=True)
class TreeSpec:
    """A tree on a lawn (docs/learning.md 6f): a trunk the robot circles."""
    # base_link from the trunk's SURFACE. ⚠️ sim_node's robot is a 0.4 m disc
    # (world.robot_radius, = Nav2's): 0.45 had it pressed against the trunk.
    offset_m: float = 0.55
    trunk_m: tuple = (0.08, 0.25)     # trunk radius
    distance_m: tuple = (1.8, 3.2)    # from the robot's start to the trunk centre
    clearance_m: float = 0.5          # the orbit stays this far inside the lawn's edge


def check_tree(lawn: Polygon, center, radius: float, spec: TreeSpec,
               direction: str = "ccw") -> Dict[str, float]:
    """Raises ValueError when the tree is unusable; else the expert's metrics."""
    orbit = Point(center).buffer(radius + spec.offset_m)
    if not lawn.buffer(-spec.clearance_m).contains(orbit):
        raise ValueError("the orbit leaves the lawn")
    if orbit.buffer(0.3).contains(Point(0, 0)):
        raise ValueError("the robot would start on the orbit")
    path = orbit_path(center, radius, spec.offset_m, direction)
    expert = EdgeExpert(None, ExpertConfig(offset_m=spec.offset_m), direction, path=path)
    poses = drive_unicycle(expert, (0.0, 0.0, 0.0), max_s=200.0)
    if not expert.done:
        raise ValueError("the expert did not finish the orbit")
    on = next(i for i, (x, y, _) in enumerate(poses)
              if expert.path.distance(Point(x, y)) < 0.2)
    metrics = orbit_metrics(center, radius, spec.offset_m, [(x, y) for x, y, _ in poses[on:]])
    # The WHOLE drive, the way in too: the robot (a ROBOT_RADIUS disc) must
    # never touch the trunk.
    nearest = min(math.hypot(x - center[0], y - center[1]) for x, y, _ in poses) - radius
    if nearest < ROBOT_RADIUS + 0.05:
        raise ValueError(f"the expert came within {nearest:.2f} m of the trunk")
    if metrics["error_p95"] > 0.15:
        raise ValueError(f"the expert drove it badly: {metrics}")
    return metrics


def random_tree(seed: int, spec: TreeSpec = TreeSpec(), attempts: int = 500):
    """(lawn, (cx, cy), trunk radius): a random lawn (random_lawn(seed)) with
    one tree the expert circles cleanly both ways from the start."""
    lawn = random_lawn(seed)
    rng = np.random.default_rng(seed + 100_000)
    for _ in range(attempts):
        a = float(rng.uniform(0, 2 * math.pi))
        d = float(rng.uniform(*spec.distance_m))
        center = (d * math.cos(a), d * math.sin(a))
        radius = float(rng.uniform(*spec.trunk_m))
        try:
            check_tree(lawn, center, radius, spec, "ccw")
            check_tree(lawn, center, radius, spec, "cw")
        except ValueError:
            continue
        return lawn, center, radius
    raise RuntimeError(f"no usable tree in {attempts} attempts (seed {seed})")
