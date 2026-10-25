#!/usr/bin/env python3
"""A trained policy against held-out episodes, offline (docs/learning.md, 6c).

    python eval_offline.py outputs/act/checkpoints/last/pretrained_model \
        --dataset ./hoverbot_edge --episodes 0,1

For every frame of the chosen episodes: the policy's action (select_action,
exactly as a robot would call it — chunks and all) next to the label. Reports
the mean absolute error per action column and the inference time per call
(mean, p95, max — the max is a chunk being computed).

This is the OPEN-loop check: "does it say what the expert said, on the
expert's own states". It cannot show compounding error — a policy that is
slightly off drifts into states the expert never visited. That needs the
closed loop in the simulator (6d) — never trust this number alone.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np


def load(checkpoint: str, device: str):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(checkpoint)
    config.device = device
    policy = get_policy_class(config.type).from_pretrained(checkpoint, config=config)
    policy.to(device).eval()
    # ⚠️ The preprocessor saves the TRAINING device ("cuda" from Kaggle).
    pre, post = make_pre_post_processors(
        config, pretrained_path=checkpoint,
        preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post


def evaluate(checkpoint: str, dataset_root: str, episodes, device: str = "cpu",
             repo_id: str = "local/hoverbot_edge") -> dict:
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    policy, pre, post = load(checkpoint, device)
    data = LeRobotDataset(repo_id, root=dataset_root, episodes=list(episodes))
    names = data.meta.features["action"]["names"]
    errors, times, per_episode = [], [], {}
    current = None
    for i in range(len(data)):
        item = data[i]
        episode = int(item["episode_index"])
        if episode != current:
            policy.reset()
            current = episode
        batch = {k: v.unsqueeze(0) for k, v in item.items()
                 if k.startswith("observation.") and hasattr(v, "unsqueeze")}
        batch["task"] = [item["task"]]
        started = time.perf_counter()
        with torch.no_grad():
            action = post(policy.select_action(pre(batch)))
        times.append(time.perf_counter() - started)
        error = np.abs(action.squeeze(0).cpu().numpy() - item["action"].numpy())
        errors.append(error)
        per_episode.setdefault(episode, []).append(error)
    errors = np.array(errors)
    times = np.array(times) * 1000.0
    return {"frames": len(errors),
            "mae": {n: round(float(errors[:, j].mean()), 4) for j, n in enumerate(names)},
            "mae_per_episode": {e: [round(float(x), 4) for x in np.mean(v, axis=0)]
                                for e, v in per_episode.items()},
            "inference_ms": {"mean": round(float(times.mean()), 2),
                             "p95": round(float(np.percentile(times, 95)), 2),
                             "max": round(float(times.max()), 2)},
            "device": device}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("checkpoint", help=".../checkpoints/<step>/pretrained_model")
    parser.add_argument("--dataset", required=True, help="the LeRobot dataset root")
    parser.add_argument("--episodes", required=True, help="held-out episode indices, e.g. 0,1")
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args(argv)
    report = evaluate(args.checkpoint, args.dataset,
                      [int(e) for e in args.episodes.split(",")], args.device)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
