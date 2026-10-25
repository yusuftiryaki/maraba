"""Trunk localization and the tree audit (trunk_localizer.py) against the
pretend detector (trunk_sim.py). ROS-free.

The orchard: 6 x 6 trees, 8 m apart. The camera 2.0 m up, on the plate.
"""

import math
import random

import numpy as np
import pytest

from robot_kit import trunk_sim
from robot_kit.trunk_localizer import AuditParams, Params, TreeAudit, TrunkLocalizer

SPACING = 8.0
CAMERA = (0.11, 0.0, 2.0, 0.0)          # in base_link: kit_mount + the camera's height


def orchard(n=6, skip=()):
    return [(f"S{j + 1}-{i + 1}", i * SPACING, j * SPACING, 0.15)
            for i in range(n) for j in range(n) if (i, j) not in skip]


def frame(pose, world_trunks, rng, model=trunk_sim.DetectorModel()):
    x, y, yaw = pose
    cx = x + math.cos(yaw) * CAMERA[0]
    cy = y + math.sin(yaw) * CAMERA[0]
    dets = trunk_sim.detect((cx, cy, CAMERA[2], yaw), world_trunks, rng, model)
    return [(d.bearing, d.depression) for d in dets]


def corridor_poses(steps=60):
    """Along the corridor y = 12 (between rows 1 and 2), facing east."""
    return [(4.0 + k * 0.5, 12.0, 0.0) for k in range(steps)]


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (1.8, -1.5, 4.0), (-2.2, 2.0, -5.0)])
def test_recovers_the_pose_from_a_wrong_prior(offset):
    trees = orchard()
    loc = TrunkLocalizer(trees, CAMERA)
    rng = random.Random(1)
    world = [(t[1], t[2], t[3]) for t in trees]
    errors, yaw_errors = [], []
    for pose in corridor_poses(30):
        prior = (pose[0] + offset[0], pose[1] + offset[1], pose[2] + math.radians(offset[2]))
        verdict = loc.locate(prior, frame(pose, world, rng))
        if verdict.fix is None:
            assert "trunks seen" in verdict.reason or "ambiguous" in verdict.reason, \
                verdict.reason
            continue
        errors.append(math.hypot(verdict.fix.x - pose[0], verdict.fix.y - pose[1]))
        yaw_errors.append(abs(math.degrees(verdict.fix.yaw - pose[2])))
    assert len(errors) >= 25
    assert np.percentile(errors, 95) < 0.10, f"position p95 {np.percentile(errors, 95):.3f} m"
    assert np.percentile(yaw_errors, 95) < 0.5


def test_covariance_is_honest_enough():
    trees = orchard()
    loc = TrunkLocalizer(trees, CAMERA)
    rng = random.Random(2)
    world = [(t[1], t[2], t[3]) for t in trees]
    inside = fixes = 0
    poses = corridor_poses(40)
    for pose in poses:
        fix = loc.locate(pose, frame(pose, world, rng)).fix
        if fix is None:                  # a frame with too few trunks: no fix, fine
            continue
        fixes += 1
        sx, sy = math.sqrt(fix.cov[0, 0]), math.sqrt(fix.cov[1, 1])
        inside += abs(fix.x - pose[0]) < 2 * sx and abs(fix.y - pose[1]) < 2 * sy
    assert fixes >= 0.8 * len(poses)
    assert inside >= 0.9 * fixes


def test_a_prior_a_row_away_is_refused_not_snapped():
    """Half a spacing off is the periodic ambiguity: refused or right, never a row over."""
    trees = orchard()
    loc = TrunkLocalizer(trees, CAMERA, Params(search_m=5.0))
    rng = random.Random(3)
    world = [(t[1], t[2], t[3]) for t in trees]
    wrong = 0
    for pose in corridor_poses(20):
        verdict = loc.locate((pose[0], pose[1] + 4.0, pose[2]), frame(pose, world, rng))
        if verdict.fix is not None:
            wrong += math.hypot(verdict.fix.x - pose[0], verdict.fix.y - pose[1]) > 1.0
    assert wrong == 0


def test_too_few_trunks_is_no_fix():
    loc = TrunkLocalizer(orchard(2), CAMERA)
    verdict = loc.locate((0, 0, 0), [(0.1, 0.2), (1.0, 0.3)])
    assert verdict.fix is None and "need 3" in verdict.reason


def test_a_dead_tree_and_a_new_one_do_not_move_the_pose_and_are_reported():
    mapped = orchard()
    dead = ("S2-3", 2 * SPACING, 1 * SPACING)      # i=2, j=1: in the map, not in the world
    world = [(t[1], t[2], t[3]) for t in mapped if t[0] != "S3-3"]
    gone = [t for t in mapped if t[0] == "S3-3"][0]
    world.append((20.0, 16.0, 0.15))                 # planted since the inventory
    loc = TrunkLocalizer(mapped, CAMERA)
    audit = TreeAudit(loc, AuditParams())
    rng = random.Random(4)
    errors = []
    for corridor_y in (12.0, 20.0):
        for k in range(70):
            pose = (2.0 + k * 0.5, corridor_y, 0.0)
            verdict = loc.locate(pose, frame(pose, world, rng))
            if verdict.fix is None:
                continue
            errors.append(math.hypot(verdict.fix.x - pose[0], verdict.fix.y - pose[1]))
            audit.observe(verdict.fix)
    report = audit.report()
    assert np.percentile(errors, 95) < 0.12
    assert report["missing"] == [gone[0]], report
    assert len(report["new"]) == 1 and math.hypot(report["new"][0]["x"] - 20.0,
                                                  report["new"][0]["y"] - 16.0) < 0.3, report
    del dead
