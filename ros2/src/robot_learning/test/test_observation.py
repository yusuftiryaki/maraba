"""observation.py against the training side's own definitions. ROS-free."""

import importlib.util
import os

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robot_learning.observation import (IMAGE_SIZE, LEGACY_STATE, LIVE_STATE, STATE_NAMES,
                                        checkpoint_state, image_chw, state_vector)

HERE = os.path.dirname(os.path.abspath(__file__))


def test_the_state_columns_are_the_ones_the_policy_was_trained_on():
    spec = importlib.util.spec_from_file_location(
        "to_lerobot", os.path.join(HERE, "..", "..", "..", "..", "learning", "to_lerobot.py"))
    to_lerobot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(to_lerobot)
    assert STATE_NAMES == tuple(to_lerobot.DEFAULT_STATE)
    from robot_learning.dataset import STATE_NAMES as EXPORTED
    assert set(LIVE_STATE) <= set(EXPORTED)


def test_the_image_size_is_the_exports():
    import inspect
    from robot_learning import export
    assert inspect.signature(export.export).parameters["size"].default == IMAGE_SIZE


def test_state_waits_for_every_column_and_keeps_the_order():
    assert state_vector({"odom.v": 0.3}, LIVE_STATE) is None
    latest = {"imu.ay": 5.0, "odom.v": 1.0, "imu.gz": 3.0, "odom.w": 2.0, "imu.ax": 4.0}
    assert state_vector(latest, LIVE_STATE).tolist() == [1.0, 2.0, 3.0, 4.0, 5.0]


def test_the_default_state_says_nothing_about_speed():
    """learning.md 6d, "copycat": velocities in the state became a shortcut."""
    assert not set(STATE_NAMES) & {"odom.v", "odom.w", "imu.gz"}
    assert state_vector({}, ()).shape == (0,), "camera only still builds"


def test_the_image_is_rgb_chw_unit_floats_at_the_training_size():
    bgr = np.zeros((480, 640, 3), np.uint8)
    bgr[..., 2] = 255                                   # pure RED in OpenCV's BGR
    ok, jpeg = cv2.imencode(".jpg", bgr)
    chw = image_chw(jpeg.tobytes())
    assert chw.shape == (3, 240, 320) and chw.dtype == np.float32
    assert chw[0].mean() > 0.95 and chw[1].mean() < 0.05 and chw[2].mean() < 0.05, (
        "channel 0 must be red: RGB, not BGR")
    assert image_chw(b"not a jpeg") is None


class _Feature:
    def __init__(self, k):
        self.shape = (k,)


def test_the_checkpoint_says_which_state_columns_it_takes(tmp_path):
    import json
    assert checkpoint_state(str(tmp_path), {}) == (), "no state feature: camera only"
    # A pack from before the summary was packed: the old five columns.
    assert checkpoint_state(str(tmp_path), {"observation.state": _Feature(5)}) == LEGACY_STATE
    (tmp_path / "hoverbot_conversion.json").write_text(json.dumps({"state": ["imu.ax", "imu.ay"]}))
    assert checkpoint_state(str(tmp_path), {"observation.state": _Feature(2)}) == ("imu.ax", "imu.ay")
    with pytest.raises(ValueError, match="takes 3"):
        checkpoint_state(str(tmp_path), {"observation.state": _Feature(3)})
    (tmp_path / "hoverbot_conversion.json").write_text(json.dumps({"state": ["map.x"]}))
    with pytest.raises(ValueError, match="cannot build"):
        checkpoint_state(str(tmp_path), {"observation.state": _Feature(1)})
