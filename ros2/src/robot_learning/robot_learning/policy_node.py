"""policy_node — a trained policy, driving through policy_gate (docs/learning.md 6d).

    camera/jpeg, /odom, imu/data_raw    in   (observation.py: as in training)
    policy_gate/state                   in   (armed -> the policy's queue resets)
    policy/cmd (TwistStamped)           out  stamp = the camera frame's stamp

Runs at the dataset's fps (10 Hz: each select_action is one training frame —
ACT's chunks are counted in those steps). It never touches /cmd_vel: the gate
decides what reaches the wheels, and while the gate is disarmed nothing does.

⚠️ Needs torch + lerobot, which ROS's Python does not have. Run it with a
venv made with --system-site-packages (so rclpy is there too):
    <venv>/bin/python -m robot_learning.policy_node --ros-args -p checkpoint:=<dir>
"""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String

from robot_learning.observation import (checkpoint_image_size, checkpoint_state, image_chw,
                                        state_vector)



class PolicyNode(Node):
    def __init__(self, **kwargs):
        super().__init__("policy_node", **kwargs)
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("task", "kenar takibi (uzman)")
        # The edge direction, for packs trained with edge.dir in the state:
        # ccw = boundary on the right (edge_expert's default), cw = on the left.
        self.declare_parameter("direction", "ccw")
        p = self.get_parameter
        self._load(p("checkpoint").value, p("device").value)
        direction = p("direction").value
        if direction not in ("ccw", "cw"):
            raise ValueError(f"direction is ccw or cw, got {direction!r}")
        self._latest = {"edge.dir": 1.0 if direction == "ccw" else -1.0}
        self._image = None                   # (stamp msg, jpeg bytes)
        self._armed = False
        self.timings = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(TwistStamped, "policy/cmd", 10)
        self.create_subscription(CompressedImage, "camera/jpeg", self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(Imu, "imu/data_raw", self._on_imu, 20)
        self.create_subscription(String, "policy_gate/state", self._on_gate, latched)
        self.create_timer(1.0 / p("rate_hz").value, self._step)

    def _load(self, checkpoint: str, device: str) -> None:
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        if not checkpoint:
            raise ValueError("checkpoint (a pretrained_model directory) is required")
        config = PreTrainedConfig.from_pretrained(checkpoint)
        config.device = device
        self._torch = torch
        self._policy = get_policy_class(config.type).from_pretrained(checkpoint, config=config)
        self._policy.to(device).eval()
        self._state_names = checkpoint_state(checkpoint, config.input_features)
        self._image_size = checkpoint_image_size(config.input_features)
        # ⚠️ The preprocessor saves the TRAINING device: a checkpoint from a
        # Kaggle GPU says "cuda" and refuses to load on a CPU (here, the Pi).
        self._pre, self._post = make_pre_post_processors(
            config, pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": device}})
        self.get_logger().info(f"loaded {config.type} from {checkpoint} on {device}, "
                               f"state {list(self._state_names) or 'none (camera only)'}")

    def _on_image(self, msg: CompressedImage) -> None:
        self._image = (msg.header.stamp, bytes(msg.data))

    def _on_odom(self, msg: Odometry) -> None:
        self._latest["odom.v"] = msg.twist.twist.linear.x
        self._latest["odom.w"] = msg.twist.twist.angular.z

    def _on_imu(self, msg: Imu) -> None:
        self._latest["imu.gz"] = msg.angular_velocity.z
        self._latest["imu.ax"] = msg.linear_acceleration.x
        self._latest["imu.ay"] = msg.linear_acceleration.y

    def _on_gate(self, msg: String) -> None:
        armed = bool(json.loads(msg.data).get("armed"))
        if armed and not self._armed:
            self._policy.reset()             # a fresh run: no chunk from the last one
        self._armed = armed

    def _step(self) -> None:
        if self._image is None:
            return
        state = state_vector(self._latest, self._state_names)
        stamp, jpeg = self._image
        image = image_chw(jpeg, self._image_size)
        if state is None or image is None:
            return
        torch = self._torch
        batch = {"observation.images.front": torch.from_numpy(image).unsqueeze(0),
                 "task": [self.get_parameter("task").value]}
        if self._state_names:
            batch["observation.state"] = torch.from_numpy(state).unsqueeze(0)
        started = time.perf_counter()
        with torch.no_grad():
            action = self._post(self._policy.select_action(self._pre(batch)))
        self.timings.append(time.perf_counter() - started)
        v, w = (float(x) for x in action.squeeze(0).cpu().numpy()[:2])
        out = TwistStamped()
        out.header.stamp = stamp
        out.twist.linear.x, out.twist.angular.z = v, w
        self._pub.publish(out)
        if len(self.timings) % 100 == 0:
            t = sorted(self.timings[-100:])
            self.get_logger().info(f"inference: median {1000 * t[50]:.0f} ms, "
                                   f"max {1000 * t[-1]:.0f} ms (last 100)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
