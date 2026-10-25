"""learning/to_lerobot.py's trimming and gap filling, without LeRobot. ROS-free.

LeRobot itself is not in CI (torch, a GPU stack): the conversion, training and
offline evaluation were run end to end against lerobot 0.6.1 (learning.md,
6c). What is tested here is OUR logic: which frames a training set starts
from, and how gaps are filled.
"""

import importlib.util
import json
import os

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = importlib.util.spec_from_file_location(
    "to_lerobot", os.path.join(HERE, "..", "..", "..", "..", "learning", "to_lerobot.py"))
to_lerobot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(to_lerobot)


def make_episode(root, n=6):
    os.makedirs(root / "episodes"), os.makedirs(root / "videos")
    state = np.arange(n * 3, dtype=float).reshape(n, 3)
    state[0, 1] = np.nan                                   # not every column yet
    state[3, 2] = np.nan                                   # a gap later on
    expert = np.tile([0.3, 0.1], (n, 1))
    expert[:2] = np.nan                                    # the expert starts at frame 2
    image_age = np.array([np.nan, 0.1, 0.1, 0.1, 0.1, 0.1])
    np.savez(root / "episodes" / "e.npz", state=state, action=np.zeros((n, 2)),
             action_expert=expert, image_age=image_age)
    writer = cv2.VideoWriter(str(root / "videos" / "e.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                             10.0, (32, 24))
    for k in range(n):
        writer.write(np.full((24, 32, 3), 40 * k, np.uint8))
    writer.release()
    return {"data": "episodes/e.npz", "video": "videos/e.mp4"}


def test_frames_start_where_image_label_and_state_all_exist(tmp_path):
    item = make_episode(tmp_path)
    images, state, action = to_lerobot.episode_arrays(str(tmp_path), item, ("a", "b", "c"),
                                                      [0, 1, 2], "expert")
    assert len(images) == len(state) == len(action) == 4       # frames 2..5
    assert images.shape[1:] == (24, 32, 3) and images.dtype == np.uint8
    assert state[0].tolist() == [6.0, 7.0, 8.0]
    assert state[1, 2] == 8.0, "a later gap is filled with the last value"
    assert np.allclose(action, [[0.3, 0.1]] * 4) and action.dtype == np.float32


def test_camera_only_needs_no_state_column(tmp_path):
    item = make_episode(tmp_path)
    images, state, action = to_lerobot.episode_arrays(str(tmp_path), item, (), [], "expert")
    assert len(images) == len(action) == 4 and state.shape == (4, 0)


def test_no_expert_means_no_expert_episode(tmp_path):
    item = make_episode(tmp_path)
    data = dict(np.load(tmp_path / item["data"]))
    data["action_expert"][:] = np.nan
    np.savez(tmp_path / item["data"], **data)
    assert to_lerobot.episode_arrays(str(tmp_path), item, ("a",), [0], "expert") is None
    # The person's label still works.
    assert to_lerobot.episode_arrays(str(tmp_path), item, ("a",), [0], "label") is not None


def test_the_default_state_never_includes_a_position():
    assert not any(n.startswith(("map.", "local.", "gps.")) for n in to_lerobot.DEFAULT_STATE)
    from robot_learning.dataset import STATE_NAMES
    assert set(to_lerobot.DEFAULT_STATE) <= set(STATE_NAMES)


def test_the_notebook_is_valid_json_and_pins_lerobot():
    path = os.path.join(HERE, "..", "..", "..", "..", "learning", "train_edge.ipynb")
    nb = json.load(open(path, encoding="utf-8"))
    sources = "\n".join(c["source"] for c in nb["cells"])
    assert '"lerobot[dataset,training]==0.6.1"' in sources
    assert "--policy.chunk_size=20" in sources and "--policy.n_action_steps=5" in sources
    assert all("id" in c for c in nb["cells"])
