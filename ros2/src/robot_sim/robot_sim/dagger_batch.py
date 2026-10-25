"""dagger_batch — the POLICY drives, the expert labels (docs/learning.md 6d-5).

    ros2 run robot_sim dagger_batch --python <venv>/bin/python \
        --checkpoint ~/.robot/policies/act_edge_18_dir --lawns 10 --seed 101 \
        --out ~/.robot/sim_episodes_dagger1
    ros2 run robot_learning episode_export ~/.robot/sim_episodes_dagger1 -o ~/dagger1

DAgger (Ross et al. 2011): a policy trained on the expert's own drives has only
seen the states the EXPERT visits. Its small errors take it elsewhere — the
lawn whose edge is closer than it learned, the corner it overshot — and there
it has no data. So: let the policy drive, and record what the expert WOULD
have commanded in every state it reaches (edge_expert with drive:=false still
publishes expert/cmd_vel — its label is a pure function of the pose,
edge_expert.label). Those labels, added to the expert's own episodes, are the
next training set.

For each lawn and direction: a fresh stack as edge_batch's (learning mode,
the episode recorder), edge_expert labelling only, policy_node through
policy_gate (armed with the lawn). The episode ends when the gate closes (it
left the area: those last metres are the most useful labels there are), the
expert counts the laps done, or --seconds runs out. mode "autonomous": the
policy drove; the note says which checkpoint.

⚠️ Episodes need edge.dir: the note carries the direction, as edge_batch's.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import tempfile
import time
from typing import Dict, List

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_missions_interfaces.srv import PolicyArm


def run_lawn(driver, seed: int, direction: str, args) -> Dict:
    from robot_sim.edge_batch import Stack
    from robot_sim.lawn_gen import random_lawn
    from robot_sim.site_world import area_feature

    lawn = random_lawn(seed)
    workdir = tempfile.mkdtemp(prefix=f"dagger_{seed}_{direction}_")
    site_file = os.path.join(workdir, "site.geojson")
    with open(site_file, "w", encoding="utf-8") as handle:
        json.dump({"type": "FeatureCollection",
                   "features": [area_feature("cim", lawn, 41.0, 29.0)]}, handle)
    stack = Stack(workdir, site_file, args.out, seed, args.gps, args.camera_hz)
    outcome: Dict = {"seed": seed, "direction": direction}
    try:
        driver.recorder = driver.expert = None
        driver.wait(lambda: driver.recorder is not None, 90.0, "the episode recorder")
        stack.spawn(["ros2", "run", "robot_sim", "edge_expert", "--ros-args",
                     "-p", f"site_file:={site_file}", "-p", f"offset_m:={args.offset}",
                     "-p", f"direction:={direction}", "-p", f"laps:={args.laps}",
                     "-p", "drive:=false"])
        driver.wait(lambda: driver.expert is not None, 30.0, "the expert")
        policy = stack.spawn([args.python, "-m", "robot_learning.policy_node", "--ros-args",
                              "-p", f"checkpoint:={args.checkpoint}",
                              "-p", f"direction:={direction}"])
        time.sleep(15.0)                           # torch + checkpoint load
        if policy.poll() is not None:
            log = os.path.join(workdir, f"proc{len(stack.procs) - 1}.log")
            raise RuntimeError("policy_node died: " + open(log, encoding="utf-8").read()[-800:])
        note = {"seed": seed, "direction": direction, "gps": args.gps, "offset_m": args.offset,
                "lawn_wkt": lawn.wkt, "expert": "robot_sim edge_expert (privileged, labels only)",
                "driver": f"policy {os.path.basename(args.checkpoint.rstrip('/'))}"}
        name = driver.episode("start", {"task": "kenar takibi (uzman)", "mode": "autonomous",
                                        "note": json.dumps(note)})
        driver.wait(lambda: (driver.recorder or {}).get("ready"), 60.0, "READY")
        gate: List[Dict] = []
        gate_sub = driver.node.create_subscription(
            String, "/policy_gate/state", lambda m: gate.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        driver.trigger("/edge_expert/start")       # labels from here on
        request = PolicyArm.Request()
        request.area = json.dumps([list(p) for p in lawn.exterior.coords[:-1]])
        request.budget_s = args.seconds + 30.0
        response = driver.call(PolicyArm, "/policy_gate/arm", request)
        if not response.success:
            raise RuntimeError(response.message)
        end, ended = time.monotonic() + args.seconds, "time"
        while time.monotonic() < end:
            driver.rclpy.spin_once(driver.node, timeout_sec=0.1)
            if (driver.expert or {}).get("state") == "done":
                ended = "laps done"
                break
            # Closed AFTER it was open: the latched "disarmed" from before
            # the arm is not an ending.
            if (any(g.get("armed") for g in gate) and gate[-1].get("armed") is False):
                ended = f"gate: {gate[-1].get('reason')}"
                break
        driver.node.destroy_subscription(gate_sub)
        driver.episode("stop", {"outcome": "success" if ended == "laps done" else "failure",
                                "note": ended})
        outcome.update(episode=name, ended=ended,
                       progress_m=(driver.expert or {}).get("progress_m"),
                       length_m=(driver.expert or {}).get("length_m"))
    finally:
        stack.close()
    return outcome


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--python", required=True, help="a Python with torch + lerobot + rclpy")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--lawns", type=int, default=10)
    parser.add_argument("--seed", type=int, default=101,
                        help="first lawn's seed: NOT the evaluation lawns (20-22)")
    parser.add_argument("--out", default=os.path.expanduser("~/.robot/sim_episodes_dagger"))
    parser.add_argument("--directions", default="ccw,cw")
    parser.add_argument("--gps", choices=("quiet", "neo6m"), default="quiet",
                        help="quiet: the gate's fence is in map, = sim_world only then")
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--offset", type=float, default=0.4)
    parser.add_argument("--laps", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=150.0)
    args = parser.parse_args(argv)
    args.checkpoint = os.path.expanduser(args.checkpoint)

    import rclpy
    from robot_sim.edge_batch import Driver
    rclpy.init()
    driver = Driver()
    results = []
    try:
        for i in range(args.lawns):
            for direction in args.directions.split(","):
                seed = args.seed + i
                print(f"lawn {i + 1}/{args.lawns} (seed {seed}) {direction}", flush=True)
                try:
                    result = run_lawn(driver, seed, direction, args)
                except (TimeoutError, RuntimeError) as exc:
                    result = {"seed": seed, "direction": direction, "error": str(exc)}
                print("  " + json.dumps(result), flush=True)
                results.append(result)
    finally:
        driver.close()
        rclpy.try_shutdown()
    made = [r for r in results if "episode" in r]
    print(f"{len(made)} episodes in {args.out}")
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
