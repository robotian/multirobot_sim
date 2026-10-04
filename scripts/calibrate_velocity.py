#!/usr/bin/env python3
"""Velocity calibration sweep, run inside a robot container.

    docker exec j100_0921 bash -c 'python3 /scripts/calibrate_velocity.py'
    docker exec r100_0000 bash -c 'python3 /scripts/calibrate_velocity.py --modes lin lat ang --levels 0.1 0.5 1.0'
    docker exec r100_0000 bash -c 'python3 /scripts/calibrate_velocity.py --ns a200_0333'   # drive another robot

Ramps cmd_vel at LOW acceleration (--lin-acc/--ang-acc; some robots tip with high acceleration) to each level,
holds it, and compares the speed the robot actually achieved with the commanded one, for linear x ('lin'), sideways
y ('lat', omnidirectional Ridgeback only) and yaw rate ('ang'). The achieved speed comes from the pose in
platform/odom: distance (or yaw) over the hold window divided by SIM time. With USE_SIM_TIME=true the odom stamps are
sim time (the sim's /clock) and are used directly, and ramp/hold times are sim seconds too. Otherwise the stamps are
wall-clock and the sim usually runs slower than real time, so sim time is (number of odom messages) x (physics time
per frame), where physics time per frame is floor(PHYSICS_HZ / SIM_RATE_HZ) / PHYSICS_HZ (Kit runs a whole number of
fixed PhysX steps per frame: 2 steps = 1/30 s at 22 Hz frames and 60 Hz physics, NOT 1/22 s), see --physics-hz /
--sim-rate-hz.

A robot with an arm is first put into its SRDF 'stow' group state (/etc/clearpath/robot.srdf), which keeps the centre
of mass low (less tipping). Exit status 1 if any level is off by more than --tolerance (default 10%).
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--ns", default=os.environ.get("ROBOT_NAMESPACE"), help="robot namespace (default: this container's)")
ap.add_argument("--modes", nargs="+", default=["lin", "ang"], choices=["lin", "lat", "ang"])
ap.add_argument("--levels", nargs="+", type=float, default=[0.1, 0.2, 0.4, 0.6, 0.8, 1.0])
ap.add_argument("--lin-acc", type=float, default=0.25, help="m/s^2 (default 0.25)")
ap.add_argument("--ang-acc", type=float, default=0.5, help="rad/s^2 (default 0.5)")
ap.add_argument("--hold", type=float, default=8.0, help="seconds at each level (the last 60%% is measured)")
ap.add_argument("--tolerance", type=float, default=0.10)
ap.add_argument("--physics-hz", type=float, default=float(os.environ.get("PHYSICS_HZ", 60)))
ap.add_argument("--sim-rate-hz", type=float, default=float(os.environ.get("SIM_RATE_HZ", 22)))
ap.add_argument("--no-stow", action="store_true")
args = ap.parse_args()
ns = args.ns
frame_dt = max(1, int(args.physics_hz // args.sim_rate_hz)) / args.physics_hz

rclpy.init()
sim_time = os.environ.get("USE_SIM_TIME", "false") == "true"
node = rclpy.create_node("calibrate_velocity", parameter_overrides=[Parameter("use_sim_time", value=sim_time)])
pub = node.create_publisher(TwistStamped, f"/{ns}/cmd_vel", 10)
poses = []  # (x, y, yaw, pitch_deg, roll_deg, stamp) per odom message
joints = {}


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    poses.append((p.x, p.y, yaw, pitch, roll, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))


node.create_subscription(Odometry, f"/{ns}/platform/odom", on_odom, 100)
node.create_subscription(JointState, f"/{ns}/platform/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)


def now():
    """Seconds on the node's clock (the sim's /clock with use_sim_time)."""
    return node.get_clock().now().nanoseconds * 1e-9


def spin(seconds):
    end = now() + seconds
    while now() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def stow_arm():
    """Drive arm_0 to the SRDF 'stow' state (smooth 6 s ramp), if this robot has both."""
    try:
        srdf = ET.parse("/etc/clearpath/robot.srdf").getroot()
    except (OSError, ET.ParseError):
        return
    state = next((g for g in srdf.findall("group_state") if g.get("name") == "stow" and g.get("group") == "arm_0"), None)
    if state is None:
        return
    target = {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
    spin(1.0)
    if not all(n in joints for n in target):
        return
    cmd_pub = node.create_publisher(JointState, f"/{ns}/arm_0/joint_command", 10)
    while cmd_pub.get_subscription_count() == 0:
        spin(0.2)
    start = {n: joints[n] for n in target}
    t0 = now()
    while now() - t0 < 10.0:
        f = min(1.0, (now() - t0) / 6.0)
        f = 0.5 - 0.5 * math.cos(math.pi * f)
        m = JointState()
        m.name = list(target)
        m.position = [start[n] + (target[n] - start[n]) * f for n in target]
        cmd_pub.publish(m)
        spin(0.05)
    err = max(abs(joints[n] - target[n]) for n in target)
    print(f"[{ns}] arm stowed (max joint error {err:.3f} rad)")


def twist(kind, v):
    """Publish a cmd_vel (geometry_msgs/TwistStamped, as on the real Clearpath platform) on one axis."""
    m = TwistStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = "base_link"
    if kind == "lin":
        m.twist.linear.x = float(v)
    elif kind == "lat":
        m.twist.linear.y = float(v)
    else:
        m.twist.angular.z = float(v)
    pub.publish(m)


def run_level(kind, target):
    acc = args.ang_acc if kind == "ang" else args.lin_acc
    ramp = abs(target) / acc
    n0 = len(poses)
    t0 = now()
    while now() - t0 < ramp + args.hold:
        t = now() - t0
        twist(kind, math.copysign(min(abs(target), acc * t), target))
        spin(0.05)
    seg = poses[n0:]
    a = int(len(seg) * (ramp + 0.4 * args.hold) / (ramp + args.hold))
    window = seg[a:]
    dt = window[-1][5] - window[0][5] if sim_time else (len(window) - 1) * frame_dt
    x0, y0, yaw0 = window[0][:3]
    x1, y1 = window[-1][:2]
    if kind == "ang":
        turned = sum(math.atan2(math.sin(b[2] - p[2]), math.cos(b[2] - p[2])) for p, b in zip(window, window[1:]))
        got = turned / dt
    elif kind == "lat":
        got = ((x1 - x0) * -math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)) / dt
    else:
        got = ((x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)) / dt
    tilt = max(max(abs(p[3]), abs(p[4])) for p in seg)
    # ramp back down at the same low acceleration
    v = target
    while abs(v) > 1e-3:
        v -= math.copysign(acc * 0.05, target)
        if v * target < 0:
            v = 0.0
        twist(kind, v)
        spin(0.05)
    twist("lin", 0.0)
    spin(1.0)
    return got, tilt


while not poses or now() == 0.0:  # with use_sim_time, now() is 0 until the first /clock message
    rclpy.spin_once(node, timeout_sec=0.5)
while pub.get_subscription_count() == 0:
    spin(0.2)
if not args.no_stow:
    stow_arm()

unit = {"lin": "m/s", "lat": "m/s", "ang": "rad/s"}
bad = 0
time_base = ("sim time from the odom stamps" if sim_time else
             f"frame dt {frame_dt:.4f} s (physics {args.physics_hz:g} Hz, sim rate {args.sim_rate_hz:g} Hz)")
print(f"[{ns}] {time_base}; hold {args.hold:g} s, tolerance {args.tolerance:.0%}")
print(f"{'mode':5} {'command':>9} {'achieved':>9} {'ratio':>7} {'max tilt':>9}")
sign = 1
for kind in args.modes:
    for level in args.levels:
        target = sign * level
        got, tilt = run_level(kind, target)
        ratio = got / target
        flag = "" if abs(ratio - 1.0) <= args.tolerance else "  <-- OUT OF TOLERANCE"
        bad += bool(flag)
        print(f"{kind:5} {target:+8.2f}  {got:+8.3f} {ratio:7.3f} {tilt:8.1f}d{flag}", flush=True)
        if kind != "ang":
            sign = -sign  # alternate direction so the robot stays roughly where it started
print("PASS" if not bad else f"FAIL: {bad} level(s) out of tolerance")
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1 if bad else 0)
