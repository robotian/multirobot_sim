"""Commanded (arm_0/joint_command, what moveit_sim_bridge sends the sim) vs. observed (platform/joint_states)
arm joint positions, as JSON. On a real robot nothing publishes arm_0/joint_command: "cmd" stays empty.

  arm_joints              one snapshot: {"cmd": {...}, "obs": {...}}
  arm_joints --record 12  one JSON line per sample at --rate Hz (default 20) for 12 s: {"t": .., "cmd": .., "obs": ..}
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from moveit_sim_bridge.namespace import robot_namespace


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", type=float)
    ap.add_argument("--rate", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("arm_joints", namespace=robot_namespace())
    cmd, obs = {}, {}

    def keep(store):
        return lambda m: store.update((n, round(p, 4)) for n, p in zip(m.name, m.position) if n.startswith("arm_0"))

    node.create_subscription(JointState, "arm_0/joint_command", keep(cmd), 50)
    node.create_subscription(JointState, "platform/joint_states", keep(obs), 50)

    if args.record is None:
        deadline = time.monotonic() + 2.0
        while not obs and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(json.dumps({"cmd": cmd, "obs": obs}))
    else:
        t0 = time.monotonic()
        next_sample = t0
        while (now := time.monotonic()) - t0 < args.record:
            rclpy.spin_once(node, timeout_sec=max(0.0, next_sample - now))
            if time.monotonic() >= next_sample:
                print(json.dumps({"t": round(time.monotonic() - t0, 3), "cmd": cmd, "obs": obs}), flush=True)
                next_sample += 1.0 / args.rate
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
