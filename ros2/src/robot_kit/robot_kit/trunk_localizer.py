"""Where the robot is, from the trunks the 360° camera sees and the tree map. ROS-free.

docs/kit-soketi.md ("Kitin kamerasıyla algı", "Ağaç haritası"). Each frame's
detections (bearing, and — when the base is visible — the depression angle
that gives the range) are matched against the orchard's trees, which come
from orchadOS's inventory through the site file. The map is a PRIOR, not the
truth: a tree that died is simply never matched, a new one is an unmatched
detection — neither may drag the pose (TreeAudit reports both instead).

    locate(prior, detections) -> Fix | None

1. Hypothesis search round the prior (the EKF's pose): a grid of offsets in
   x, y and yaw, each scored by how many detections land on a tree. The
   window stays inside half the row spacing: an orchard is periodic, and a
   window wider than that has two equally good answers (the second-best is
   checked and an ambiguous frame is REFUSED, not guessed).
2. One-to-one association at the best hypothesis (bearing and range gates).
3. Gauss-Newton on (x, y, yaw) over the matched bearings and depressions;
   the covariance is the inverse information, inflated (the noise is not
   independent frame to frame).
4. Refused unless there are `min_matches` trunks, the residuals are small and
   the answer is unambiguous. A refused frame costs nothing; a wrong fix fed
   to the EKF would put the robot a row over, next to the trunks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Tree = Tuple[str, float, float, float]          # name, x, y, radius in `map`


@dataclass(frozen=True)
class Params:
    search_m: float = 2.5          # ± around the prior, x and y
    search_step_m: float = 0.5     # coarse grid; then a fine one round its best
    search_yaw_deg: float = 6.0
    search_yaw_step_deg: float = 2.0
    max_range_m: float = 12.0
    score_bearing_deg: float = 1.5
    score_range_rel: float = 0.15
    gate_bearing_deg: float = 2.5
    gate_range_rel: float = 0.25
    sigma_bearing_deg: float = 0.5
    sigma_depression_deg: float = 0.5
    min_matches: int = 3          # 3 bearings + their depressions: 6 equations, 3 unknowns
    max_rms_deg: float = 1.0
    outlier_deg: float = 1.5       # a match whose bearing is this far off after the solve is dropped
    ambiguity_ratio: float = 0.85   # second-best / best score, beyond this = ambiguous
    ambiguity_dist_m: float = 1.0
    cov_inflation: float = 4.0


@dataclass
class Fix:
    x: float
    y: float
    yaw: float
    cov: np.ndarray                     # 3x3 over (x, y, yaw)
    matches: List[Tuple[int, int]]      # (detection index, tree index)
    rms_deg: float
    unmatched: List[Tuple[float, float]] = field(default_factory=list)   # map x, y


@dataclass
class Verdict:
    fix: Optional[Fix]
    reason: str                         # "" when accepted


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class TrunkLocalizer:
    def __init__(self, trees: Sequence[Tree], camera: Tuple[float, float, float, float],
                 params: Params = Params()) -> None:
        """camera: the 360° camera in base_link (x, y, z above the ground, yaw)."""
        self.trees = list(trees)
        self.xy = np.array([(t[1], t[2]) for t in self.trees], dtype=float).reshape(-1, 2)
        self.camera = camera
        self.p = params

    # --- the measurement model ------------------------------------------------

    def _camera_pose(self, x, y, yaw):
        cx, cy, _cz, cyaw = self.camera
        return (x + np.cos(yaw) * cx - np.sin(yaw) * cy,
                y + np.sin(yaw) * cx + np.cos(yaw) * cy, yaw + cyaw)

    def predict(self, x: float, y: float, yaw: float):
        """(bearing in the camera, range) to every tree from this pose."""
        px, py, pyaw = self._camera_pose(x, y, yaw)
        dx, dy = self.xy[:, 0] - px, self.xy[:, 1] - py
        return wrap(np.arctan2(dy, dx) - pyaw), np.hypot(dx, dy)

    # --- the fix --------------------------------------------------------------

    def locate(self, prior: Tuple[float, float, float],
               detections: Sequence[Tuple[float, float]]) -> Verdict:
        """detections: (bearing, depression) — depression nan when unknown."""
        p = self.p
        if len(self.trees) < p.min_matches:
            return Verdict(None, "fewer trees in the map than min_matches")
        det = np.array(detections, dtype=float).reshape(-1, 2)
        if len(det) < p.min_matches:
            return Verdict(None, f"{len(det)} trunks seen, need {p.min_matches}")
        cz = self.camera[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            det_range = np.where(np.isfinite(det[:, 1]) & (det[:, 1] > 0),
                                 cz / np.tan(det[:, 1]), np.nan)

        # 1. hypotheses: coarse round the prior, then fine round its best.
        # Only trees that could be in view from somewhere in the window take part.
        near = np.hypot(self.xy[:, 0] - prior[0], self.xy[:, 1] - prior[1]) \
            < p.max_range_m + p.search_m * 1.5
        if near.sum() < p.min_matches:
            return Verdict(None, "no mapped trees near the robot")
        xy = self.xy[near]
        hx, hy, hyaw = self._grid(prior, p.search_m, p.search_step_m,
                                  p.search_yaw_deg, p.search_yaw_step_deg)
        # The coarse grid's points sit up to half a step off the peak: score
        # them with wider tolerances, or the true place scores like a wrong one.
        scores = self._score(xy, hx, hy, hyaw, det[:, 0], det_range, widen=2.0)
        best = int(np.argmax(scores))
        far = np.hypot(hx - hx[best], hy - hy[best]) > p.ambiguity_dist_m
        if np.any(far) and scores[far].max() > p.ambiguity_ratio * scores[best]:
            return Verdict(None, f"ambiguous: two places score {scores[best]:.1f} / "
                                 f"{scores[far].max():.1f} (orchard periodicity?)")
        fx, fy, fyaw = self._grid((hx[best], hy[best], hyaw[best]), p.search_step_m,
                                  p.search_step_m / 4, p.search_yaw_step_deg,
                                  p.search_yaw_step_deg / 4)
        fine = self._score(xy, fx, fy, fyaw, det[:, 0], det_range)
        k = int(np.argmax(fine))
        x, y, yaw = fx[k], fy[k], fyaw[k]

        # 2. association, 3. refinement (associate again after each solve: the
        # first association is made from a 0.25 m / 1° grid point)
        matches: List[Tuple[int, int]] = []
        for _round in range(3):
            matches = self._associate(x, y, yaw, det[:, 0], det_range)
            if len(matches) < p.min_matches:
                return Verdict(None, f"{len(matches)} trunks matched, need {p.min_matches}")
            x, y, yaw, cov, rms = self._refine(x, y, yaw, det, matches)
        # Outliers, one at a time, worst first: a trunk the map does not have
        # (planted since the inventory) can pass the gates against a mapped
        # neighbour and bend the solve towards it.
        while True:
            tb, _tr = self.predict(x, y, yaw)
            res = [abs(math.degrees(wrap(det[i, 0] - tb[j]))) for i, j in matches]
            worst = int(np.argmax(res))
            if res[worst] <= p.outlier_deg:
                break
            matches = matches[:worst] + matches[worst + 1:]
            if len(matches) < p.min_matches:
                return Verdict(None, f"{len(matches)} trunks left after outliers, "
                                     f"need {p.min_matches}")
            x, y, yaw, cov, rms = self._refine(x, y, yaw, det, matches)
        if rms > p.max_rms_deg:
            return Verdict(None, f"residual {rms:.2f}° > {p.max_rms_deg}°")
        used = {d for d, _ in matches}
        unmatched = []
        px, py, pyaw = self._camera_pose(x, y, yaw)
        for i in range(len(det)):
            if i in used or not np.isfinite(det_range[i]):
                continue
            a = pyaw + det[i, 0]
            unmatched.append((float(px + det_range[i] * np.cos(a)),
                              float(py + det_range[i] * np.sin(a))))
        return Verdict(Fix(float(x), float(y), float(wrap(yaw)), cov * p.cov_inflation,
                           matches, rms, unmatched), "")

    @staticmethod
    def _grid(centre, half_m, step_m, half_deg, step_deg):
        steps = int(round(half_m / step_m))
        offsets = np.arange(-steps, steps + 1) * step_m
        ysteps = int(round(half_deg / step_deg))
        yaws = np.radians(np.arange(-ysteps, ysteps + 1) * step_deg)
        hx, hy, hyaw = np.meshgrid(centre[0] + offsets, centre[1] + offsets, centre[2] + yaws,
                                   indexing="ij")
        return hx.ravel(), hy.ravel(), hyaw.ravel()

    def _score(self, xy, hx, hy, hyaw, bearings, ranges, widen=1.0):
        p = self.p
        cx, cy, _cz, cyaw = self.camera
        px = hx + np.cos(hyaw) * cx - np.sin(hyaw) * cy
        py = hy + np.sin(hyaw) * cx + np.cos(hyaw) * cy
        dx = xy[None, :, 0] - px[:, None]                       # H x T
        dy = xy[None, :, 1] - py[:, None]
        tb = np.arctan2(dy, dx) - (hyaw + cyaw)[:, None]
        tr = np.hypot(dx, dy)
        in_range = tr < p.max_range_m
        db = wrap(tb[:, None, :] - bearings[None, :, None])     # H x D x T
        sb = math.radians(p.score_bearing_deg) * widen
        cost = (db / sb) ** 2
        rel = (tr[:, None, :] - ranges[None, :, None]) / np.maximum(tr[:, None, :], 0.5)
        known = np.isfinite(ranges)[None, :, None]
        cost = cost + np.where(known, (rel / (p.score_range_rel * widen)) ** 2, 0.0)
        cost = np.where(in_range[:, None, :], cost, np.inf)
        return np.exp(-0.5 * cost.min(axis=2)).sum(axis=1)      # H

    def _associate(self, x, y, yaw, bearings, ranges) -> List[Tuple[int, int]]:
        p = self.p
        tb, tr = self.predict(x, y, yaw)
        pairs = []
        for i, (b, r) in enumerate(zip(bearings, ranges)):
            db = np.abs(wrap(tb - b))
            ok = (db < math.radians(p.gate_bearing_deg)) & (tr < p.max_range_m)
            if np.isfinite(r):
                ok &= np.abs(tr - r) < p.gate_range_rel * np.maximum(tr, 0.5)
            for j in np.nonzero(ok)[0]:
                cost = (db[j] / math.radians(p.gate_bearing_deg)) ** 2
                if np.isfinite(r):
                    cost += ((tr[j] - r) / (p.gate_range_rel * max(tr[j], 0.5))) ** 2
                pairs.append((cost, i, int(j)))
        pairs.sort()
        used_d, used_t, out = set(), set(), []
        for _cost, i, j in pairs:
            if i in used_d or j in used_t:
                continue
            used_d.add(i)
            used_t.add(j)
            out.append((i, j))
        return out

    def _refine(self, x, y, yaw, det, matches):
        p = self.p
        sb, sd = math.radians(p.sigma_bearing_deg), math.radians(p.sigma_depression_deg)
        cz = self.camera[2]
        state = np.array([x, y, yaw], dtype=float)

        def residuals(s):
            tb, tr = self.predict(*s)
            res, weights = [], []
            for i, j in matches:
                res.append(wrap(det[i, 0] - tb[j]))
                weights.append(1.0 / sb)
                if np.isfinite(det[i, 1]):
                    res.append(det[i, 1] - math.atan2(cz, tr[j]))
                    weights.append(1.0 / sd)
            return np.array(res), np.array(weights)

        for _ in range(6):
            r, w = residuals(state)
            jac = np.zeros((len(r), 3))
            for k, h in enumerate((1e-4, 1e-4, 1e-5)):
                bumped = state.copy()
                bumped[k] += h
                jac[:, k] = (residuals(bumped)[0] - r) / h
            jw, rw = jac * w[:, None], r * w
            try:
                step = np.linalg.lstsq(jw, -rw, rcond=None)[0]
            except np.linalg.LinAlgError:
                break
            state += step
            if np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-5:
                break
        r, w = residuals(state)
        jw = np.zeros((len(r), 3))
        for k, h in enumerate((1e-4, 1e-4, 1e-5)):
            bumped = state.copy()
            bumped[k] += h
            jw[:, k] = (residuals(bumped)[0] - r) / h * w
        try:
            cov = np.linalg.inv(jw.T @ jw)
        except np.linalg.LinAlgError:
            cov = np.diag([1e3, 1e3, 1e3])
        bearing_res = [abs(math.degrees(wrap(det[i, 0] - self.predict(*state)[0][j])))
                       for i, j in matches]
        rms = math.sqrt(sum(v * v for v in bearing_res) / len(bearing_res))
        return state[0], state[1], state[2], cov, rms


# --- the tree audit -----------------------------------------------------------

@dataclass(frozen=True)
class AuditParams:
    audit_range_m: float = 8.0
    nadir_deg: float = 60.0
    min_expected: int = 15
    missing_ratio: float = 0.15
    cluster_m: float = 0.6
    min_new_views: int = 8
    min_new_baseline_m: float = 2.0
    clear_of_map_m: float = 1.0


class TreeAudit:
    """What the robot finds wrong with the map, for orchadOS — never applied
    to the map here (docs/kit-soketi.md, "Ağaç haritası": the map changes only
    after the expert and the owner approve).

    missing: a mapped tree that should have been in view (in range, base above
    the robot's own blind spot, not behind a nearer trunk) many times and was
    almost never matched. new: unmatched detections that land on one spot from
    several robot positions, away from every mapped tree."""

    def __init__(self, localizer: TrunkLocalizer, params: AuditParams = AuditParams()) -> None:
        self.loc = localizer
        self.p = params
        n = len(localizer.trees)
        self.expected = np.zeros(n, dtype=int)
        self.seen = np.zeros(n, dtype=int)
        self.clusters: List[Dict] = []           # {"x", "y", "n", "from": [(x, y)]}

    def observe(self, fix: Fix) -> None:
        tb, tr = self.loc.predict(fix.x, fix.y, fix.yaw)
        cz = self.loc.camera[2]
        radius = np.array([t[3] for t in self.loc.trees])
        order = np.argsort(tr)
        shadow: List[Tuple[float, float]] = []
        for j in order:
            if tr[j] > self.p.audit_range_m:
                break
            if math.degrees(math.atan2(cz, tr[j])) > self.p.nadir_deg:
                continue
            half = math.asin(min(1.0, radius[j] / max(tr[j], 1e-3)))
            if any(abs(math.atan2(math.sin(tb[j] - b), math.cos(tb[j] - b))) < 0.7 * (half + h)
                   for b, h in shadow):
                continue
            shadow.append((tb[j], half))
            self.expected[j] += 1
        for _i, j in fix.matches:
            self.seen[j] += 1
        for x, y in fix.unmatched:
            if np.min(np.hypot(self.loc.xy[:, 0] - x, self.loc.xy[:, 1] - y)) < self.p.clear_of_map_m:
                continue
            for c in self.clusters:
                if math.hypot(c["x"] - x, c["y"] - y) < self.p.cluster_m:
                    c["x"] = (c["x"] * c["n"] + x) / (c["n"] + 1)
                    c["y"] = (c["y"] * c["n"] + y) / (c["n"] + 1)
                    c["n"] += 1
                    c["from"].append((fix.x, fix.y))
                    break
            else:
                self.clusters.append({"x": x, "y": y, "n": 1, "from": [(fix.x, fix.y)]})

    def report(self) -> Dict:
        missing = [self.loc.trees[j][0] for j in range(len(self.loc.trees))
                   if self.expected[j] >= self.p.min_expected
                   and self.seen[j] <= self.p.missing_ratio * self.expected[j]]
        new = []
        for c in self.clusters:
            if c["n"] < self.p.min_new_views:
                continue
            xs, ys = zip(*c["from"])
            if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < self.p.min_new_baseline_m:
                continue
            new.append({"x": round(c["x"], 2), "y": round(c["y"], 2), "views": c["n"]})
        checked = int(np.sum(self.expected >= self.p.min_expected))
        return {"checked": checked, "missing": missing, "new": new}
