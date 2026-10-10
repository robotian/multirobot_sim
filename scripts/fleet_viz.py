#!/usr/bin/env python3
"""Fleet view relay: every robot's TF and a few display topics, merged into one tree a single viewer can show.

Each robot publishes its own TF on /<ns>/tf and /<ns>/tf_static with the same frame names (base_link, odom, map,
arm_0_*), so one viewer can't tell the robots apart. This node renames each robot's frames to <ns>/<frame> and
publishes them all on /fleet/tf and /fleet/tf_static. The shared world frame (ref_frame: the sim's world, Motive's
frame in the lab) keeps its name, so every robot sits at its true place under it:

    ref_frame -> <ns>/map -> <ns>/odom -> <ns>/base_link -> ...

Display topics (DATA) are republished as /fleet/<ns>/<topic> with every frame_id in them renamed the same way,
throttled to the rate given there; the robot_description too (latched), for the viewer's URDF layer with frame
prefix "<ns>/". Run by scripts/fleet_viz.sh in the base station, next to a foxglove_bridge that only exposes
/fleet/*, so opening and closing viewer panels never subscribes on the robots (each new subscription pauses a
robot's data ~0.3 s, scripts/CLAUDE.md); this node keeps one long-lived subscription per robot topic.

Robots are found from the graph (a /<ns>/tf with a publisher), checked every DISCOVERY_S; one whose TF publishers
are gone for two checks is dropped (its static frames too). The current robots and frames are written to
--status (JSON) for scripts/fleet_viz_layout.py.

Base-station tool, not robot code: it lives in scripts/, not colcon_ws/src.
"""
import argparse
import json
import os
import re

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

# (topic relative to the robot's namespace, max Hz; 0 = every message, for latched topics)
DATA = [
    ("robot_description", 0),
    ("map", 0),
    ("plan", 2),
    ("optimal_trajectory", 5),                # MPPI's chosen local trajectory
    ("local_costmap/published_footprint", 5),
    ("sensors/lidar2d_0/scan_filtered", 5),
    ("sensors/lidar2d_0/scan", 5),
    ("local_costmap/costmap", 1),
]
SHARED_FRAMES = ["ref_frame"]
DISCOVERY_S = 5.0
STATIC_FLUSH_S = 0.5
TF_TOPIC = re.compile(r"^/([A-Za-z][A-Za-z0-9_]*)/tf$")
FRAME_FIELDS = ("frame_id", "child_frame_id")


class Renamer:
    """Renames frame_id / child_frame_id everywhere in a message (headers, nested and sequence fields)."""

    def __init__(self, shared):
        self.shared = set(shared)
        self.plans = {}  # message class -> [(field, kind)], kind: frame / msg / seq

    def frame(self, ns, f):
        f = f.lstrip("/")
        return f if not f or f in self.shared else f"{ns}/{f}"

    def plan(self, cls):
        p = self.plans.get(cls)
        if p is None:
            p = []
            for name, typ in cls.get_fields_and_field_types().items():
                if name in FRAME_FIELDS and typ == "string":
                    p.append((name, "frame"))
                elif "/" in typ:  # a message type: nested, or a sequence / array of messages
                    p.append((name, "seq" if typ.startswith("sequence<") or typ.endswith("]") else "msg"))
            self.plans[cls] = p
        return p

    def apply(self, ns, msg):
        for name, kind in self.plan(type(msg)):
            v = getattr(msg, name)
            if kind == "frame":
                setattr(msg, name, self.frame(ns, v))
            elif kind == "msg":
                self.apply(ns, v)
            else:
                for item in v:
                    self.apply(ns, item)
        return msg


def qos_like(node, topic):
    """A subscription QoS every publisher of the topic matches: latched only when all are, reliable only when all are."""
    pubs = node.get_publishers_info_by_topic(topic)
    if not pubs:
        return None
    latched = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
    reliable = all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs)
    return QoSProfile(depth=1 if latched else 5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)


class Robot:
    def __init__(self, ns):
        self.ns = ns
        self.subs = {}       # source topic -> subscription
        self.statics = {}    # renamed child frame -> TransformStamped
        self.frames = set()  # renamed frames seen
        self.missing = 0     # discovery checks without TF publishers


class FleetViz(Node):
    def __init__(self, a):
        super().__init__("fleet_viz", namespace="/fleet")
        self.only = set(a.robots or [])
        self.data = DATA + [(t, h) for t, h in a.topic]
        self.renamer = Renamer(SHARED_FRAMES + a.shared)
        self.status_path = a.status
        self.robots = {}
        self.pubs = {}       # output topic -> publisher
        self.last = {}       # output topic -> steady time of the last message passed
        self.static_dirty = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.tf_pub = self.create_publisher(TFMessage, "/fleet/tf", QoSProfile(depth=100))
        self.static_pub = self.create_publisher(TFMessage, "/fleet/tf_static", latched)
        # Graph checks, static flushes and display throttling run on the steady clock on purpose: none of them
        # waits on a robot, and with use_sim_time a sim that isn't publishing /clock yet would stop them all.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.steady = steady
        self.create_timer(DISCOVERY_S, self.discover, clock=steady)
        self.create_timer(STATIC_FLUSH_S, self.flush_statics, clock=steady)
        self.discover()

    # --- robots -----------------------------------------------------------------------------------------------

    def discover(self):
        topics = dict(self.get_topic_names_and_types())
        found = set()
        for t in topics:
            m = TF_TOPIC.match(t)
            if m and m[1] != "fleet" and (not self.only or m[1] in self.only) and self.count_publishers(t):
                found.add(m[1])
        for ns in found - set(self.robots):
            self.robots[ns] = Robot(ns)
            self.get_logger().info(f"robot {ns}")
        for ns, r in list(self.robots.items()):
            if ns in found:
                r.missing = 0
                self.subscribe_robot(r, topics)
            else:
                r.missing += 1
                if r.missing >= 2:
                    self.drop(r)
        self.write_status()

    def subscribe_robot(self, r, topics):
        """Subscribe to the robot's topics that exist and aren't subscribed yet."""
        wanted = [(f"/{r.ns}/tf", 0), (f"/{r.ns}/tf_static", 0)] + [(f"/{r.ns}/{rel}", hz) for rel, hz in self.data]
        for src, hz in wanted:
            if src in topics and src not in r.subs:
                self.subscribe(r, src, topics[src][0], hz)

    def subscribe(self, r, src, type_name, hz):
        ns = r.ns
        if src == f"/{ns}/tf":
            r.subs[src] = self.create_subscription(
                TFMessage, src, lambda m, r=r: self.on_tf(r, m), QoSProfile(depth=100))
            return
        if src == f"/{ns}/tf_static":
            r.subs[src] = self.create_subscription(
                TFMessage, src, lambda m, r=r: self.on_static(r, m),
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            return
        rel = src[len(f"/{ns}/"):]
        qos = qos_like(self, src)
        if qos is None:  # listed, but no publisher yet: tried again at the next check
            return
        try:
            cls = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as e:
            self.get_logger().warn(f"{src}: {type_name} not available here ({e})")
            r.subs[src] = None
            return
        out = f"/fleet/{ns}/{rel}"
        if out not in self.pubs:
            self.pubs[out] = self.create_publisher(cls, out, QoSProfile(
                depth=qos.depth, durability=qos.durability, reliability=ReliabilityPolicy.RELIABLE))
        period = 1.0 / hz if hz else 0.0
        r.subs[src] = self.create_subscription(
            cls, src, lambda m, ns=ns, out=out, period=period: self.on_data(ns, out, period, m), qos)
        self.get_logger().info(f"{src} -> {out}" + (f" (<= {hz} Hz)" if hz else ""))

    def drop(self, r):
        for s in r.subs.values():
            if s is not None:
                self.destroy_subscription(s)
        del self.robots[r.ns]
        for out in [o for o in self.pubs if o.startswith(f"/fleet/{r.ns}/")]:
            self.destroy_publisher(self.pubs.pop(out))
            self.last.pop(out, None)
        self.static_dirty = True
        self.get_logger().info(f"robot {r.ns} gone")

    # --- TF -----------------------------------------------------------------------------------------------------

    def on_tf(self, r, msg):
        self.renamer.apply(r.ns, msg)
        for t in msg.transforms:
            r.frames.add(t.child_frame_id)
            r.frames.add(t.header.frame_id)
        self.tf_pub.publish(msg)

    def on_static(self, r, msg):
        self.renamer.apply(r.ns, msg)
        for t in msg.transforms:
            # Time zero: a static transform holds at any time, also in a viewer that only treats /tf_static by
            # name as static, and after a sim reset sends the clock back.
            t.header.stamp.sec, t.header.stamp.nanosec = 0, 0
            r.statics[t.child_frame_id] = t
            r.frames.add(t.child_frame_id)
            r.frames.add(t.header.frame_id)
        self.static_dirty = True

    def flush_statics(self):
        if not self.static_dirty:
            return
        self.static_dirty = False
        msg = TFMessage(transforms=[t for r in self.robots.values() for t in r.statics.values()])
        self.static_pub.publish(msg)
        self.write_status()

    # --- display topics -------------------------------------------------------------------------------------------

    def on_data(self, ns, out, period, msg):
        if period:
            now = self.steady.now().nanoseconds * 1e-9
            if now - self.last.get(out, -1e9) < period:
                return
            self.last[out] = now
        pub = self.pubs.get(out)
        if pub is not None:
            pub.publish(self.renamer.apply(ns, msg))

    def write_status(self):
        if not self.status_path:
            return
        status = {"shared_frames": sorted(self.renamer.shared),
                  "robots": {ns: {"frames": sorted(r.frames),
                                  "topics": sorted(o for o in self.pubs if o.startswith(f"/fleet/{ns}/"))}
                             for ns, r in sorted(self.robots.items())}}
        tmp = self.status_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(status, f, indent=1)
        os.replace(tmp, self.status_path)


def topic_rate(s):
    t, _, hz = s.rpartition(":")
    if not t:
        raise argparse.ArgumentTypeError("TOPIC:HZ, e.g. global_costmap/costmap:0.2")
    return t.strip("/"), float(hz)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robots", nargs="*", help="only these namespaces (default: every robot in the graph)")
    ap.add_argument("--topic", type=topic_rate, action="append", default=[], metavar="TOPIC:HZ",
                    help="another display topic to relay, relative to the robot's namespace (0 = every message)")
    ap.add_argument("--shared", action="append", default=[], metavar="FRAME",
                    help="another frame every robot shares (kept unprefixed), e.g. a GPS datum frame")
    ap.add_argument("--status", default="/tmp/fleet_viz_status.json", help="where to write robots and frames")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FleetViz(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
