#!/usr/bin/env python3
"""Is each robot's ROS data reaching this machine, how often and how late? Runs in the base station container
(scripts/ is mounted there as /scripts), started by the web UI (tools/sim_ui) while its Communication card is open;
prints one JSON line per second.

  python3 /scripts/link_monitor.py a300_00036 [j100_0921 ...]

Per robot (namespace), subscribed here so the data crosses the robot's link like any base station's would:
  odom          platform/odom/filtered
  gps_<n>       every sensors/gps_<n>/fix the robot publishes (found from the ROS graph, re-checked every 10 s)
  tf chains     map -> odom, odom -> base_link and, when tf_static has arm_0_base_link, arm_0_base_link ->
                arm_0_end_effector_link (the robot computes the arm's TF from joint_states, so this also tells
                whether joint_states flow on the robot: joint_states itself is 1 kHz / 400 KB/s on a300_00036 and is
                deliberately not subscribed -- measured cost: +8 % of a robot core in its zenoh router, ~30 % of a core
                here, 3.2 Mbit/s of WiFi)
For a topic: rate (last 5 s), its usual rate (median of the last 60 one-second rates), longest gap between messages
in the last 30 s (the current silence included), age = receive time - header stamp (mean over the last second;
this machine's clock minus the robot's, measured by the UI over SSH, is not corrected here). For a TF chain: the
same from its moving edges (the chain's age is that of its stalest edge, its rate that of its slowest); "broken"
names the first frame missing on the way.

Small topics only: subscribing pulls the data over the robot's WiFi. Measured on a300_00036 for tf + odom + GPS:
robot router 3.8 % of a core (3.2 % idle), ~12 % of a core here, ~0.7 Mbit/s.
"""
import collections
import json
import re
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

CHAINS = (("map", "odom"), ("odom", "base_link"), ("arm_0_base_link", "arm_0_end_effector_link"))
ARM_FRAME = "arm_0_base_link"
GAP_WINDOW_S = 30
RATE_WINDOW_S = 5
HISTORY_S = 60


class Stream:
    """Arrivals of one topic, or of one TF edge: counts and gaps per second, header-stamp ages."""

    def __init__(self, now):
        self.last_recv = None
        self.last_stamp = None
        self.count = 0          # this second
        self.max_gap = 0.0      # this second
        self.ages = []          # this second
        self.started = now
        self.counts = collections.deque(maxlen=HISTORY_S)
        self.gaps = collections.deque(maxlen=GAP_WINDOW_S)
        self.mean_ages = collections.deque(maxlen=RATE_WINDOW_S)

    def add(self, now, stamp):
        if self.last_recv is not None:
            self.max_gap = max(self.max_gap, now - self.last_recv)
        self.last_recv, self.count = now, self.count + 1
        if stamp:  # 0: no stamp (shouldn't happen on these topics)
            self.last_stamp = stamp
            self.ages.append(now - stamp)

    def tick(self, now):
        """Close the second: push its count / gap / mean age."""
        self.counts.append(self.count)
        silent = now - (self.last_recv if self.last_recv is not None else self.started)
        self.gaps.append(max(self.max_gap, silent))
        if self.ages:
            self.mean_ages.append(sum(self.ages) / len(self.ages))
        self.count, self.max_gap, self.ages = 0, 0.0, []

    def summary(self, now):
        recent = list(self.counts)[-RATE_WINDOW_S:]
        return {
            "rate": round(sum(recent) / len(recent), 1) if recent else 0.0,
            "usual_rate": float(sorted(self.counts)[len(self.counts) // 2]) if self.counts else 0.0,
            "max_gap": round(max(self.gaps), 3) if self.gaps else None,
            "silent": round(now - self.last_recv, 3) if self.last_recv is not None else None,
            "age": round(self.mean_ages[-1], 4) if self.mean_ages else None,
            "max_age": round(max(self.mean_ages), 4) if self.mean_ages else None,
        }


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Robot:
    def __init__(self, node, ns):
        self.node, self.ns = node, ns
        now = time.time()
        self.topics = {"odom": Stream(now)}
        self.edges = {}         # child -> (parent, static)
        self.edge_streams = {}  # child -> Stream (moving edges only)
        self.subs = [
            node.create_subscription(Odometry, f"/{ns}/platform/odom/filtered",
                                     lambda m: self.topics["odom"].add(time.time(), stamp_of(m.header)),
                                     qos_profile_sensor_data),
            node.create_subscription(TFMessage, f"/{ns}/tf", lambda m: self.on_tf(m, False), 100),
            node.create_subscription(TFMessage, f"/{ns}/tf_static", lambda m: self.on_tf(m, True),
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE)),
        ]

    def on_tf(self, msg, static):
        now = time.time()
        for t in msg.transforms:
            child, parent = t.child_frame_id.strip("/"), t.header.frame_id.strip("/")
            self.edges[child] = (parent, static)
            if not static:
                stream = self.edge_streams.get(child)
                if stream is None:
                    stream = self.edge_streams[child] = Stream(now)
                stream.add(now, stamp_of(t.header))

    def add_gps(self, topic):
        name = re.search(r"(gps_\d+)", topic).group(1)
        if name in self.topics:
            return
        self.topics[name] = Stream(time.time())
        self.subs.append(self.node.create_subscription(
            NavSatFix, topic, lambda m: self.topics[name].add(time.time(), stamp_of(m.header)),
            qos_profile_sensor_data))

    def path_to_root(self, frame):
        path = [frame]
        while frame in self.edges and len(path) < 100:
            frame = self.edges[frame][0]
            path.append(frame)
        return path

    def chain(self, target, source, now):
        """target -> source through the tree: the moving edges on the way and their worst numbers."""
        up_source, up_target = self.path_to_root(source), self.path_to_root(target)
        common = next((f for f in up_source if f in up_target), None)
        if common is None:  # two separate trees: name where each one ends
            return {"broken": f"{source} (tree {up_source[-1]}) not connected to {target} (tree {up_target[-1]})"}
        frames = up_source[:up_source.index(common)] + up_target[:up_target.index(common)]
        moving = [self.edge_streams[f] for f in frames if not self.edges[f][1] and f in self.edge_streams]
        if not moving:
            return {"static": True}
        parts = [s.summary(now) for s in moving]

        def worst(key):
            values = [p[key] for p in parts if p[key] is not None]
            return max(values) if values else None

        stamps = [s.last_stamp for s in moving if s.last_stamp]
        return {"edges": len(moving), "rate": min(p["rate"] for p in parts),
                "usual_rate": min(p["usual_rate"] for p in parts), "max_gap": worst("max_gap"),
                "silent": worst("silent"), "max_age": worst("max_age"),
                # the chain is as old as its stalest edge (tf2's "latest common time")
                "age": round(now - min(stamps), 4) if stamps else None}

    def tick(self, now):
        for s in list(self.topics.values()) + list(self.edge_streams.values()):
            s.tick(now)

    def report(self, now):
        chains = {}
        for target, source in CHAINS:
            if target == ARM_FRAME and ARM_FRAME not in self.edges:
                continue  # no arm (tf_static never had its base)
            chains[f"{target}->{source}"] = self.chain(target, source, now)
        return {"topics": {k: s.summary(now) for k, s in self.topics.items()}, "tf": chains}


def main():
    namespaces = [a for a in sys.argv[1:] if re.fullmatch(r"[A-Za-z0-9_]+", a)]
    rclpy.init()
    node = rclpy.create_node("link_monitor")  # wall clock: these are real robots
    robots = {ns: Robot(node, ns) for ns in namespaces}
    gps_re = re.compile(r"^/(%s)/sensors/gps_\d+/fix$" % "|".join(map(re.escape, namespaces)))
    next_tick, next_scan = time.time() + 1.0, 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.0, min(next_tick - time.time(), 0.2)))
            now = time.time()
            if now >= next_scan:  # GPS topics a robot publishes (the graph is cheap to read)
                for topic, _ in node.get_topic_names_and_types():
                    m = gps_re.match(topic)
                    if m:
                        robots[m.group(1)].add_gps(topic)
                next_scan = now + 10.0
            if now >= next_tick:
                for r in robots.values():
                    r.tick(now)
                print(json.dumps({"t": round(now, 3), "robots": {ns: r.report(now) for ns, r in robots.items()}}),
                      flush=True)
                next_tick += 1.0
                if next_tick < now:  # fell behind (suspended?): don't burst
                    next_tick = now + 1.0
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    except BaseException:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass  # rclpy's SIGINT handler shut the context down meanwhile


if __name__ == "__main__":
    main()
