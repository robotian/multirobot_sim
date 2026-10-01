#!/usr/bin/env python3
"""Smoke test, run inside a robot container: drive the robot and compare commanded vs measured motion.

    docker exec a300_0000 python3 /scripts/drive_test.py [linear_x] [angular_z] [seconds]

Publishes cmd_vel at 20 Hz for <seconds>, then a zero command, and prints the odometry change.
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

lin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
ang = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
ns = os.environ.get("ROBOT_NAMESPACE", "a300_0000")

rclpy.init()
node = rclpy.create_node("drive_test")
pub = node.create_publisher(TwistStamped, f"/{ns}/cmd_vel", 10)
last = {}
node.create_subscription(Odometry, f"/{ns}/platform/odom", lambda m: last.update(msg=m, t=time.time()), 10)


def publish_cmd(lin_x=0.0, lin_y=0.0, ang_z=0.0):
    """cmd_vel is geometry_msgs/TwistStamped (as on the real Clearpath platform)."""
    m = TwistStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = "base_link"
    m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(lin_x), float(lin_y), float(ang_z)
    pub.publish(m)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def pose():
    p = last["msg"].pose.pose
    q = p.orientation
    return p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


while "msg" not in last:
    rclpy.spin_once(node, timeout_sec=0.5)
while pub.get_subscription_count() == 0:  # wait for the sim's subscriber before publishing
    rclpy.spin_once(node, timeout_sec=0.2)
spin(0.3)
x0, y0, th0 = pose()
t0 = time.time()
while time.time() - t0 < dur:
    publish_cmd(lin, 0.0, ang)
    spin(0.05)
elapsed = time.time() - t0
v = last["msg"].twist.twist
x1, y1, th1 = pose()
publish_cmd()
spin(1.5)
xs, ys, ths = pose()
dth = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
print(f"[{ns}] cmd lin={lin} ang={ang} for {elapsed:.1f}s")
print(f"  odom velocity at end : lin={v.linear.x:.3f} m/s  ang={v.angular.z:.3f} rad/s")
print(f"  displacement         : {math.hypot(x1 - x0, y1 - y0):.2f} m  ({math.hypot(x1 - x0, y1 - y0) / elapsed:.3f} m/s avg), turned {dth:.2f} rad")
print(f"  coast after stop     : {math.hypot(xs - x1, ys - y1):.3f} m")
node.destroy_node()
rclpy.shutdown()
