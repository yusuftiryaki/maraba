#!/usr/bin/env python3
"""Our exported episodes -> a LeRobot dataset (docs/learning.md, 6c).

    python to_lerobot.py ~/dataset --out ./hoverbot_edge --repo-id local/hoverbot_edge

Input: what `ros2 run robot_learning episode_export` writes (meta.json,
episodes/*.npz, videos/*.mp4). Output: a LeRobotDataset (LeRobot 0.6.1's
format) that `lerobot-train --dataset.root=...` trains on.

No ROS here — this runs in Colab/Kaggle. Needs `pip install
"lerobot[dataset]==0.6.1"` (Python >= 3.12). Pinned on purpose: LeRobot's
dataset format has changed between releases.

Choices, all flags:
  --state   which state columns become observation.state. Default: the
            accelerometer only (imu.ax, imu.ay) — nothing that says how fast
            the robot goes. NOT the map/GPS position either way — that would
            let the policy memorise one lawn's coordinates instead of looking
            at the edge.
            ⚠️ The first default was wheel odometry + IMU (odom.v, odom.w,
            imu.gz, ...). The policy learned to COPY its current speed: on the
            training frames with odom.v/w zeroed it stopped on 51 % of them
            (real state: 4 %), and in closed loop it stood still 55 % of the
            time — once slow, it stayed slow (learning.md, 6d "copycat").
            The expert's next command is ~ the robot's current speed, so a
            state with velocities in it is a shortcut around the camera.
            Why not NO state ("" works here): LeRobot 0.6.1's ACT reads
            observation.state's device in forward() even without a state
            feature — KeyError on Kaggle, 2026-10-25.
            edge.dir: +1 ccw (boundary on the right), -1 cw, from the
            episode's note (edge_batch writes it). Not a sensor — the TASK:
            the dataset holds both directions, and facing the edge at a
            corner or at the start, a frame cannot say which way to turn.
            Without it the policy dithered left-right at corners (27 sign
            flips a minute) and set off the wrong way (learning.md 6d-4).
            ACT ignores the task string, so the direction goes in the state.
  --action  "expert" (the simulator's privileged expert, 6a) or "label" (the
            person / Nav2 — real episodes). Default: expert when present.
Frames are trimmed at the start until there is a camera frame, a label and
every state column; isolated gaps after that are filled with the last value.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

DEFAULT_STATE = ("imu.ax", "imu.ay", "edge.dir")
TASK_COLUMNS = ("edge.dir",)          # from the episode's note, not from the bag
IMAGE_KEY = "observation.images.front"


def _fill_forward(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    for i in range(1, len(out)):
        bad = np.isnan(out[i])
        out[i][bad] = out[i - 1][bad]
    return out


def episode_arrays(dataset_dir: str, item: dict, state_names, state_index, action: str):
    """(images uint8 [N,H,W,3], state float32 [N,k], action float32 [N,2]) or None."""
    import cv2

    data = np.load(os.path.join(dataset_dir, item["data"]))
    state = data["state"][:, state_index]
    if action == "expert":
        if "action_expert" not in data or np.isnan(data["action_expert"]).all():
            return None
        act = data["action_expert"]
    else:
        act = data["action"]
    if not item.get("video"):
        return None
    capture = cv2.VideoCapture(os.path.join(dataset_dir, item["video"]))
    frames = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    n = min(len(frames), len(state), len(act))
    image_ok = ~np.isnan(data["image_age"][:n])
    ready = image_ok & ~np.isnan(act[:n]).any(axis=1) & ~np.isnan(state[:n]).any(axis=1)
    if not ready.any():
        return None
    start = int(np.argmax(ready))
    images = np.stack(frames[start:n]).astype(np.uint8)
    return (images, _fill_forward(state[start:n]).astype(np.float32),
            _fill_forward(act[start:n]).astype(np.float32))


def assemble_state(names, sensed, state: np.ndarray, constants: dict) -> np.ndarray:
    """The columns in `names` order: sensed ones from `state`, task ones constant."""
    columns = [state[:, sensed.index(n)] if n in sensed
               else np.full(len(state), constants[n], np.float32) for n in names]
    return np.stack(columns, axis=1).astype(np.float32) if columns else state


def task_values(item: dict, names) -> "list | None":
    """The task columns' values for one episode; None if its note lacks them."""
    try:
        note = json.loads(item.get("note") or "{}")
    except ValueError:
        return None
    out = []
    for name in names:
        if name == "edge.dir":
            direction = note.get("direction")
            if direction not in ("ccw", "cw"):
                return None
            out.append(1.0 if direction == "ccw" else -1.0)
    return out


def convert(dataset_dir: str, out: str, repo_id: str, state_names=DEFAULT_STATE,
            action: str = "auto", use_videos: bool = True) -> dict:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    with open(os.path.join(dataset_dir, "meta.json"), encoding="utf-8") as handle:
        meta = json.load(handle)
    all_names = meta["state_names"]
    sensed = [n for n in state_names if n not in TASK_COLUMNS]
    tasks = [n for n in state_names if n in TASK_COLUMNS]
    missing = [n for n in sensed if n not in all_names]
    if missing:
        raise SystemExit(f"unknown state columns {missing}; have {all_names} + {TASK_COLUMNS}")
    state_index = [all_names.index(n) for n in sensed]
    if action == "auto":
        action = "expert" if any(e.get("source") == "sim" for e in meta["episodes"]) else "label"
    width, height = meta["image_size"]
    features = {
        IMAGE_KEY: {"dtype": "video" if use_videos else "image", "shape": (height, width, 3),
                    "names": ["height", "width", "channel"]},
        "action": {"dtype": "float32", "shape": (2,), "names": list(meta["action_names"])},
    }
    if state_names:
        features["observation.state"] = {"dtype": "float32", "shape": (len(state_names),),
                                         "names": list(state_names)}
    dataset = LeRobotDataset.create(repo_id=repo_id, fps=int(round(meta["fps"])),
                                    features=features, root=out, robot_type="hoverbot",
                                    use_videos=use_videos)
    kept, skipped = [], []
    for item in meta["episodes"]:
        values = task_values(item, tasks)
        arrays = (episode_arrays(dataset_dir, item, sensed, state_index, action)
                  if values is not None else None)
        if arrays is None:
            skipped.append(item["name"])
            continue
        images, state, act = arrays
        state = assemble_state(state_names, sensed, state, dict(zip(tasks, values)))
        for k in range(len(images)):
            frame = {IMAGE_KEY: images[k], "action": act[k], "task": item["task"]}
            if state_names:
                frame["observation.state"] = state[k]
            dataset.add_frame(frame)
        dataset.save_episode()
        kept.append({"name": item["name"], "frames": len(images)})
        print(f"{item['name']}: {len(images)} frames")
    dataset.finalize()
    summary = {"root": out, "repo_id": repo_id, "action": action,
               "state": list(state_names), "episodes": kept, "skipped": skipped}
    with open(os.path.join(out, "hoverbot_conversion.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1, ensure_ascii=False)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("dataset", help="episode_export's output directory")
    parser.add_argument("--out", required=True, help="the LeRobot dataset root (must not exist)")
    parser.add_argument("--repo-id", default="local/hoverbot_edge")
    parser.add_argument("--state", default=",".join(DEFAULT_STATE),
                        help='comma-separated columns; "" = camera only (ACT 0.6.1 cannot)')
    parser.add_argument("--action", choices=("auto", "expert", "label"), default="auto")
    parser.add_argument("--images", action="store_true",
                        help="store frames as images instead of video (no encoder needed)")
    args = parser.parse_args(argv)
    summary = convert(args.dataset, args.out, args.repo_id,
                      tuple(n for n in args.state.split(",") if n),
                      args.action, use_videos=not args.images)
    print(f"{len(summary['episodes'])} episodes ({summary['action']} actions) -> {args.out}; "
          f"skipped {summary['skipped']}")
    return 0 if summary["episodes"] else 1


if __name__ == "__main__":
    sys.exit(main())
