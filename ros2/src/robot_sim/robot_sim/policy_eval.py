"""policy_eval — a trained policy in CLOSED loop, scored like the expert
(docs/learning.md 6d).

    ros2 run robot_sim policy_eval --python <venv>/bin/python \
        --checkpoint ~/.robot/policies/act_edge_18 --seeds 20,21,22

For each lawn: a fresh simulator and robot.launch.py (learning mode on, so
policy_gate and image_compressor run), policy_node in the given Python (it
needs torch), the gate armed with the lawn (quiet GPS: `map` is sim_world),
and the robot left to the policy for --seconds. Scored on /ground_truth with
edge_metrics — the same metric as the expert — and, on the same lawn, the
expert's own lap for comparison.

Open-loop MAE (eval_offline) says "it answers like the expert on the
expert's states". This says what happens when its own small errors take it
where the expert never was. Use lawns the policy never saw (edge_batch seeds
it was not trained on).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from typing import Dict, List

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from shapely.geometry import Point
from std_msgs.msg import String

from robot_missions_interfaces.srv import PolicyArm


def run_lawn(driver, seed: int, mode: str, python: str, checkpoint: str, seconds: float,
             offset: float, direction: str = "ccw", camera: str = "front") -> Dict:
    from robot_sim.edge_batch import Stack
    from robot_sim.edge_expert import EdgeExpert, ExpertConfig, edge_metrics
    from robot_sim.lawn_gen import random_lawn
    from robot_sim.site_world import area_feature

    lawn = random_lawn(seed)
    workdir = tempfile.mkdtemp(prefix=f"policy_eval_{seed}_{mode}_")
    site_file = os.path.join(workdir, "site.geojson")
    with open(site_file, "w", encoding="utf-8") as handle:
        json.dump({"type": "FeatureCollection",
                   "features": [area_feature("cim", lawn, 41.0, 29.0)]}, handle)
    stack = Stack(workdir, site_file, os.path.join(workdir, "episodes"), seed, "quiet", 10.0,
                  camera)
    poses: List = []
    speeds: Dict[str, List[float]] = {"asked": [], "passed": [], "true": []}
    turns: List[float] = []                     # the policy's asked w, beside "asked"
    outcome: Dict = {"seed": seed, "mode": mode, "direction": direction}
    try:
        driver.recorder = None
        driver.wait(lambda: driver.recorder is not None, 90.0, "the stack")
        sub = driver.node.create_subscription(
            Odometry, "/ground_truth",
            lambda m: (poses.append((m.pose.pose.position.x, m.pose.pose.position.y)),
                       speeds["true"].append(m.twist.twist.linear.x)), 50)
        # Where speed is lost: what the policy ASKED, what the gate PASSED (its
        # caps and accel limits), what the robot really did.
        speed_subs = [
            driver.node.create_subscription(
                TwistStamped, "/policy/cmd",
                lambda m: (speeds["asked"].append(m.twist.linear.x),
                           turns.append(m.twist.angular.z)), 10),
            driver.node.create_subscription(
                Twist, "/cmd_vel_policy", lambda m: speeds["passed"].append(m.linear.x), 10)]
        if mode == "expert":
            stack.spawn(["ros2", "run", "robot_sim", "edge_expert", "--ros-args",
                         "-p", f"site_file:={site_file}", "-p", f"offset_m:={offset}",
                         "-p", f"direction:={direction}"])
            driver.expert = None
            driver.wait(lambda: driver.expert is not None, 30.0, "the expert")
            driver.trigger("/edge_expert/start")
        else:
            node = stack.spawn([python, "-m", "robot_learning.policy_node", "--ros-args",
                                "-p", f"checkpoint:={checkpoint}",
                                "-p", f"direction:={direction}"])
            time.sleep(15.0)                       # torch + checkpoint load
            if node.poll() is not None:
                # ⚠️ Silent before: a policy_node that died loading left the
                # gate "armed, no command yet" for the whole run.
                log = os.path.join(workdir, f"proc{len(stack.procs) - 1}.log")
                raise RuntimeError("policy_node died: "
                                   + open(log, encoding="utf-8").read()[-800:])
            request = PolicyArm.Request()
            request.area = json.dumps([list(p) for p in lawn.exterior.coords[:-1]])
            request.budget_s = seconds + 30.0
            response = driver.call(PolicyArm, "/policy_gate/arm", request)
            if not response.success:
                raise RuntimeError(response.message)
        gate = []
        gate_sub = driver.node.create_subscription(
            String, "/policy_gate/state", lambda m: gate.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            driver.rclpy.spin_once(driver.node, timeout_sec=0.1)
            if mode == "expert" and (driver.expert or {}).get("state") == "done":
                break
        driver.node.destroy_subscription(sub)
        driver.node.destroy_subscription(gate_sub)
        for speed_sub in speed_subs:
            driver.node.destroy_subscription(speed_sub)
        # Lap progress along the edge path, and the metric from the first pose on it.
        watch = EdgeExpert(lawn, ExpertConfig(offset_m=offset), direction)
        for x, y in poses[::5]:
            watch.step(x, y, 0.0)
        on = next((i for i, p in enumerate(poses)
                   if watch.path.distance(Point(p)) < 0.2), None)
        outcome.update(lap=round(watch.progress_m / watch.length, 2), poses=len(poses),
                       gate=(gate[-1] if gate else None),
                       speed={k: _speed_stats(v) for k, v in speeds.items() if v})
        if turns:
            # Standing still is two things: turning on the spot (the expert
            # does it at corners: v 0, w max) or not moving at all.
            pairs = list(zip(speeds["asked"], turns))
            outcome["asked_stopped"] = {
                "turning": round(sum(1 for v, w in pairs if abs(v) < 0.03 and abs(w) >= 0.2)
                                 / len(pairs), 2),
                "idle": round(sum(1 for v, w in pairs if abs(v) < 0.03 and abs(w) < 0.2)
                              / len(pairs), 2),
                "w_sign_flips_per_min": round(
                    sum(1 for a, b in zip(turns, turns[1:]) if a * b < 0 and abs(a - b) > 0.4)
                    / (len(turns) / 600.0), 1)}
        if on is not None and len(poses) - on > 10:
            outcome.update({k: round(v, 3) for k, v in
                            edge_metrics(lawn, offset, poses[on:]).items()})
        else:
            outcome["note"] = "never reached the edge"
    finally:
        stack.close()
    return outcome


def _speed_stats(values: List[float]) -> Dict:
    ordered = sorted(values)
    return {"mean": round(sum(values) / len(values), 3),
            "p50": round(ordered[len(ordered) // 2], 3),
            "p90": round(ordered[int(len(ordered) * 0.9)], 3),
            "stopped": round(sum(1 for v in values if abs(v) < 0.03) / len(values), 2)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--python", required=True, help="a Python with torch + lerobot + rclpy")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--seeds", default="20,21,22", help="lawns the policy never saw")
    parser.add_argument("--seconds", type=float, default=150.0)
    parser.add_argument("--offset", type=float, default=0.4)
    parser.add_argument("--directions", default="ccw",
                        help="ccw,cw: each lawn both ways (packs with edge.dir in the state)")
    parser.add_argument("--camera", choices=("front", "edge"), default="front",
                        help="the camera the policy was trained on (edge_batch --camera)")
    parser.add_argument("--no-expert", action="store_true")
    args = parser.parse_args(argv)

    import rclpy
    from robot_sim.edge_batch import Driver
    rclpy.init()
    driver = Driver()
    results = []
    try:
        for seed in (int(s) for s in args.seeds.split(",")):
            for direction in args.directions.split(","):
                for mode in (("policy",) if args.no_expert else ("expert", "policy")):
                    print(f"lawn {seed} {direction}: {mode}", flush=True)
                    result = run_lawn(driver, seed, mode, args.python,
                                      os.path.expanduser(args.checkpoint), args.seconds,
                                      args.offset, direction, args.camera)
                    print("  " + json.dumps(result), flush=True)
                    results.append(result)
    finally:
        driver.close()
        rclpy.try_shutdown()
    print(json.dumps(results, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
