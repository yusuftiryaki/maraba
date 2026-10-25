"""The policy's observation, built from live topics exactly as training built
it from the bag (docs/learning.md 6d). ROS-free (numpy + OpenCV).

⚠️ A mismatch here is SILENT: a policy fed state columns in another order, BGR
for RGB, or a differently sized frame does not fail — it drives wrongly. So
the definitions are pinned to the training side by tests:

    state   learning/to_lerobot.py DEFAULT_STATE, from export.py's series:
            odom.v, odom.w  = /odom twist (linear.x, angular.z)
            imu.gz, imu.ax, imu.ay = /imu/data_raw (angular_velocity.z,
                                     linear_acceleration.x, .y)
    image   /camera/jpeg decoded, RGB, resized to the dataset's image_size
            (export.py, 320x240), CHW float32 in [0, 1] (what LeRobot's
            dataset hands the model; the checkpoint's preprocessor normalizes)
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

import numpy as np

STATE_NAMES = ("odom.v", "odom.w", "imu.gz", "imu.ax", "imu.ay")
IMAGE_SIZE = (320, 240)                      # (width, height)


def state_vector(latest: Dict[str, float], names: Sequence[str] = STATE_NAMES
                 ) -> Optional[np.ndarray]:
    """None until every column has been heard at least once."""
    if any(n not in latest for n in names):
        return None
    return np.array([latest[n] for n in names], dtype=np.float32)


def image_chw(jpeg: bytes, size: Tuple[int, int] = IMAGE_SIZE) -> Optional[np.ndarray]:
    import cv2
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    if (frame.shape[1], frame.shape[0]) != size:
        frame = cv2.resize(frame, size)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1).copy()
