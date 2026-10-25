"""A pretend trunk detector on the kit's 360° camera. ROS-free.

robot_sim's sim_node publishes what this returns as kit/trunks (TrunkDetections) while a
kit with a 360° camera is plugged in. It stands in for a detector that does
not exist yet — orchadOS will train it on the office tool's clicks
(docs/kit-soketi.md, "Envanter sırası") — so it is GEOMETRY plus plausible
failure, not a model of any real network:

  - a trunk is seen if it is within `max_range_m`, its base is not hidden
    under the robot (`nadir_deg`: the chassis fills the bottom of the sphere),
    and no nearer trunk covers it (angular overlap);
  - bearing and depression get Gaussian noise (`bearing_sigma_deg`,
    `depression_sigma_deg`); a fraction of detections lose the depression
    (`base_hidden`: grass round the trunk), a fraction are missed outright
    (`miss`), and some are false (`false_rate` per frame, anywhere);
  - the numbers behind the defaults: 1920x960 preview ~0.19°/px; a trunk at
    4 m ~11 px wide (docs/kit-soketi.md).

What it is NOT: lighting, motion blur, a 30 fps preview at a few Hz of
detection on a Pi 4, foliage hanging into the view, or two trunks the real
network would merge. A localizer that works here is wired right; whether the
real camera supports it is a field question.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Trunk = Tuple[float, float, float]          # x, y, radius in the world


@dataclass(frozen=True)
class DetectorModel:
    max_range_m: float = 12.0
    nadir_deg: float = 60.0                 # bases steeper than this are under the robot
    bearing_sigma_deg: float = 0.3
    depression_sigma_deg: float = 0.3
    base_hidden: float = 0.1
    miss: float = 0.05
    false_rate: float = 0.05


@dataclass(frozen=True)
class Detection:
    bearing: float          # rad, in the camera's horizontal plane, CCW from its +x
    depression: float       # rad below horizontal to the base; nan = hidden
    confidence: float


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def detect(camera: Tuple[float, float, float, float], trunks: Sequence[Trunk],
           rng: random.Random, model: DetectorModel = DetectorModel()) -> List[Detection]:
    """camera: (x, y, height above ground, yaw) in the world."""
    cx, cy, cz, cyaw = camera
    visible = []
    for tx, ty, radius in trunks:
        rng_m = math.hypot(tx - cx, ty - cy)
        if rng_m < 1e-3 or rng_m > model.max_range_m:
            continue
        depression = math.atan2(cz, rng_m)
        if math.degrees(depression) > model.nadir_deg:
            continue
        bearing = math.atan2(ty - cy, tx - cx)
        half = math.asin(min(1.0, radius / rng_m))
        visible.append((rng_m, bearing, half, depression))
    visible.sort()
    seen: List[Tuple[float, float]] = []                 # (bearing, half-width) in front
    out = []
    for rng_m, bearing, half, depression in visible:
        if any(abs(wrap(bearing - b)) < 0.7 * (half + h) for b, h in seen):
            continue                                       # behind a nearer trunk
        seen.append((bearing, half))
        if rng.random() < model.miss:
            continue
        noisy_dep = depression + math.radians(rng.gauss(0.0, model.depression_sigma_deg))
        if rng.random() < model.base_hidden:
            noisy_dep = math.nan
        out.append(Detection(
            bearing=wrap(bearing - cyaw + math.radians(rng.gauss(0.0, model.bearing_sigma_deg))),
            depression=noisy_dep, confidence=max(0.3, min(1.0, 1.0 - rng_m / 20.0))))
    while rng.random() < model.false_rate:
        out.append(Detection(bearing=rng.uniform(-math.pi, math.pi),
                             depression=math.radians(rng.uniform(5.0, 40.0)),
                             confidence=rng.uniform(0.3, 0.6)))
    return out
