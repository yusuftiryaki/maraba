"""The policy's observation, built from live topics exactly as training built
it from the bag (docs/learning.md 6d). ROS-free (numpy + OpenCV).

⚠️ A mismatch here is SILENT: a policy fed state columns in another order, BGR
for RGB, or a differently sized frame does not fail — it drives wrongly. So
the definitions are pinned to the training side by tests:

    state   the columns the checkpoint was trained on (policy_node reads
            them from the pack's hoverbot_conversion.json). Default: the
            accelerometer only — to_lerobot.DEFAULT_STATE (velocities in the
            state taught the first policy to copy its own speed: learning.md
            6d, "copycat").
            The columns policy_node can build, from export.py's series:
            odom.v, odom.w  = /odom twist (linear.x, angular.z)
            imu.gz, imu.ax, imu.ay = /imu/data_raw (angular_velocity.z,
                                     linear_acceleration.x, .y)
            edge.dir = the task: +1 ccw (boundary on the right), -1 cw
    image   /camera/jpeg decoded, RGB, resized to the dataset's image_size
            (export.py, 320x240), CHW float32 in [0, 1] (what LeRobot's
            dataset hands the model; the checkpoint's preprocessor normalizes)
"""

from __future__ import annotations

import json
import os
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

STATE_NAMES = ("imu.ax", "imu.ay", "edge.dir")         # = to_lerobot.DEFAULT_STATE
# What can be built. edge.dir is the TASK (policy_node's `direction`: +1 ccw,
# -1 cw), not a sensor — to_lerobot.py takes it from the episode's note.
LIVE_STATE = ("odom.v", "odom.w", "imu.gz", "imu.ax", "imu.ay", "edge.dir")
LEGACY_STATE = ("odom.v", "odom.w", "imu.gz", "imu.ax", "imu.ay")   # packs without a summary
IMAGE_SIZE = (320, 240)                      # (width, height)


def state_vector(latest: Dict[str, float], names: Sequence[str] = STATE_NAMES
                 ) -> Optional[np.ndarray]:
    """None until every column has been heard at least once."""
    if any(n not in latest for n in names):
        return None
    return np.array([latest[n] for n in names], dtype=np.float32)


def checkpoint_state(checkpoint: str, input_features) -> tuple:
    """The state columns, in order, the checkpoint was trained on.

    ⚠️ A wrong guess is SILENT (observation.py): so the names come from the
    conversion summary packed next to the weights (train_edge.ipynb), and
    their count must match what the model takes.
    """
    feature = input_features.get("observation.state")
    if feature is None:
        return ()                                  # camera only
    summary = os.path.join(checkpoint, "hoverbot_conversion.json")
    if os.path.exists(summary):
        with open(summary, encoding="utf-8") as handle:
            names = tuple(json.load(handle)["state"])
    else:
        names = LEGACY_STATE
    if len(names) != feature.shape[0]:
        raise ValueError(f"checkpoint takes {feature.shape[0]} state columns, "
                         f"its summary names {len(names)}: {names}")
    unknown = [n for n in names if n not in LIVE_STATE]
    if unknown:
        raise ValueError(f"policy_node cannot build state columns {unknown}")
    return names

def image_chw(jpeg: bytes, size: Tuple[int, int] = IMAGE_SIZE) -> Optional[np.ndarray]:
    import cv2
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    if (frame.shape[1], frame.shape[0]) != size:
        frame = cv2.resize(frame, size)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1).copy()
