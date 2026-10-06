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
  round trip    every RTT_EVERY_S, get_parameter_types (empty request) on the robot's robot_state_publisher: a
                request and its reply through zenoh both ways (the topics only show robot -> here, ping doesn't
                involve zenoh); mean / max over the last 60 s and the share of calls unanswered in RTT_TIMEOUT_S.
                ~2 ms on a300_00036 (18 ms on the first call)
  localizer     ref_localizer/status (1 Hz JSON, a few hundred bytes): whether it publishes map -> odom at all, so a
                robot without a localization source yet (no Motive rigid body, no GPS fix) isn't mistaken for a link
                problem
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
import warnings

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameterTypes
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

CHAINS = (("map", "odom"), ("odom", "base_link"), ("arm_0_base_link", "arm_0_end_effector_link"))
ARM_FRAME = "arm_0_base_link"
GAP_WINDOW_S = 30
RATE_WINDOW_S = 5
HISTORY_S = 60
# Ctrl+C mid-spin leaves one of the executor's coroutines un-awaited: harmless, not worth a warning at exit
warnings.filterwarnings("ignore", message="coroutine .* was never awaited", category=RuntimeWarning)
RTT_NODE = "robot_state_publisher"  # in the robot's namespace: always there, light, answers from its own executor
RTT_EVERY_S = 2.0
RTT_TIMEOUT_S = 2.0


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
        self.rtt_client = None  # created once the node shows up in the graph
        self.rtt_pending = None  # (future, sent at)
        self.rtt_next = 0.0
        self.rtt_warm = True
        self.rtts = collections.deque()  # (time, ms or None: no answer)
        self.localizer = None   # (receive time, ref_localizer's status dict)
        self.subs = [
            node.create_subscription(Odometry, f"/{ns}/platform/odom/filtered",
                                     lambda m: self.topics["odom"].add(time.time(), stamp_of(m.header)),
                                     qos_profile_sensor_data),
            node.create_subscription(String, f"/{ns}/ref_localizer/status", self.on_localizer, 1),
            node.create_subscription(TFMessage, f"/{ns}/tf", lambda m: self.on_tf(m, False), 100),
            node.create_subscription(TFMessage, f"/{ns}/tf_static", lambda m: self.on_tf(m, True),
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE)),
        ]

    def on_localizer(self, msg):
        try:
            self.localizer = (time.time(), json.loads(msg.data))
        except ValueError:
            pass

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

    def rtt_step(self, now):
        """Collect a finished / timed-out round trip, start the next one when due."""
        if self.rtt_client is None:
            return
        if self.rtt_pending:
            future, sent = self.rtt_pending
            if future.done():
                if self.rtt_warm:  # the first call sets the route up (208 ms on a300_00036, then ~2 ms): not counted
                    self.rtt_warm = False
                else:
                    self.rtts.append((now, (time.monotonic() - sent) * 1000 if future.result() is not None else None))
                self.rtt_pending = None
            elif time.monotonic() - sent > RTT_TIMEOUT_S:
                self.rtt_client.remove_pending_request(future)
                self.rtts.append((now, None))
                self.rtt_pending = None
        if self.rtt_pending is None and now >= self.rtt_next and self.rtt_client.service_is_ready():
            self.rtt_pending = (self.rtt_client.call_async(GetParameterTypes.Request(names=[])), time.monotonic())
            self.rtt_next = now + RTT_EVERY_S
        while self.rtts and now - self.rtts[0][0] > HISTORY_S:
            self.rtts.popleft()

    def rtt_report(self):
        if self.rtt_client is None:
            return {"error": f"no /{self.ns}/{RTT_NODE} in the ROS graph"}
        answered = [ms for _, ms in self.rtts if ms is not None]
        return {"node": RTT_NODE, "count": len(self.rtts),
                "rtt_ms": round(sum(answered) / len(answered), 2) if answered else None,
                "max_ms": round(max(answered), 1) if answered else None,
                "timeouts_pct": round(100 * (len(self.rtts) - len(answered)) / len(self.rtts), 1) if self.rtts else None}

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
        localizer = None
        if self.localizer:
            t, s = self.localizer
            localizer = {k: s.get(k) for k in ("source", "active", "publishing", "ref_age", "gps_age")}
            localizer["age"] = round(now - t, 1)
        return {"topics": {k: s.summary(now) for k, s in self.topics.items()}, "tf": chains,
                "round_trip": self.rtt_report(), "localizer": localizer}


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
            if now >= next_scan:  # GPS topics and the round-trip node a robot has (the graph is cheap to read)
                for topic, _ in node.get_topic_names_and_types():
                    m = gps_re.match(topic)
                    if m:
                        robots[m.group(1)].add_gps(topic)
                present = {f"{n_ns.rstrip('/')}/{n}" for n, n_ns in node.get_node_names_and_namespaces()}
                for ns, r in robots.items():
                    if r.rtt_client is None and f"/{ns}/{RTT_NODE}" in present:
                        r.rtt_client = node.create_client(GetParameterTypes, f"/{ns}/{RTT_NODE}/get_parameter_types")
                next_scan = now + 10.0
            for r in robots.values():
                r.rtt_step(now)
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
        # a request still in flight at exit made rclpy's teardown fail ("Unable to convert call argument"):
        # drop it and the clients first
        for r in robots.values():
            if r.rtt_client is not None:
                if r.rtt_pending:
                    r.rtt_client.remove_pending_request(r.rtt_pending[0])
                node.destroy_client(r.rtt_client)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass  # rclpy's SIGINT handler shut the context down meanwhile


if __name__ == "__main__":
    main()
