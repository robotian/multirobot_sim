"""Simulated attitude SwiftNav Duro: publishes the moving-baseline Baseline from two simulated GPS fixes.

On the real robot the attitude Duro (rover) gets RTK corrections from the reference Duro (moving base) and the
sbp-to-ros driver publishes MSG_BASELINE_NED as swiftnav_ros2_driver/Baseline: the vector from the reference antenna
to the attitude antenna, in metres, North/East/Down. dual_duro_heading turns that into a heading.

The sim (sim/scripts/setup_scene.py, GPS_READ_SCRIPT) publishes a NavSatFix per antenna from its true world position
with a flat-earth projection, so the inverse of that projection gives the exact antenna-to-antenna vector. The fields
are filled the way baseline_publisher.cpp does for a fixed RTK solution (mode 4), including direction/dip and their
error estimates from the configured accuracy.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from swiftnav_ros2_driver.msg import Baseline

M_PER_DEG_LAT = 111320.0  # same constant as setup_scene.py's GPS_READ_SCRIPT
FIX_MODE_FIXED_RTK = 4


class DuroBaselineSim(Node):
    def __init__(self):
        super().__init__('att_duro_node')
        self.frame = self.declare_parameter('frame_name', 'gps_0_link').value
        rover_topic = self.declare_parameter('rover_fix_topic', 'fix').value
        base_topic = self.declare_parameter('base_fix_topic', '../gps_1/fix').value
        self.err_h = self.declare_parameter('baseline_err_h_m', 0.01).value
        self.err_v = self.declare_parameter('baseline_err_v_m', 0.02).value
        self.dir_offset = self.declare_parameter('baseline_dir_offset_deg', 0.0).value
        self.dip_offset = self.declare_parameter('baseline_dip_offset_deg', 0.0).value
        self.n_sats = self.declare_parameter('satellites_used', 20).value
        self.period = 1.0 / self.declare_parameter('rate_hz', 10.0).value
        self.max_age = self.declare_parameter('max_fix_age_s', 0.5).value

        self.base = None
        self.last_pub = None
        self.pub = self.create_publisher(Baseline, 'baseline', 10)
        self.create_subscription(NavSatFix, base_topic, self.on_base, 10)
        self.create_subscription(NavSatFix, rover_topic, self.on_rover, 10)
        self.get_logger().info(
            f"Simulated Duro baseline: {self.resolve_topic_name(base_topic)} -> "
            f"{self.resolve_topic_name(rover_topic)}, publishing {self.pub.topic_name} in frame {self.frame}")

    def on_base(self, msg):
        self.base = msg

    def on_rover(self, rover):
        now = self.get_clock().now()
        if self.last_pub is not None and (now - self.last_pub).nanoseconds * 1e-9 < self.period:
            return
        base = self.base
        if base is None or abs((Time.from_msg(rover.header.stamp) - Time.from_msg(base.header.stamp))
                               .nanoseconds) * 1e-9 > self.max_age:
            self.get_logger().warn('Waiting for the reference antenna fix...', throttle_duration_sec=3.0)
            return
        self.last_pub = now

        m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(base.latitude))
        msg = Baseline()
        msg.header.stamp = now.to_msg()  # timestamp_source_gnss: False -> platform time
        msg.header.frame_id = self.frame
        msg.mode = FIX_MODE_FIXED_RTK
        msg.satellites_used = self.n_sats
        msg.baseline_n_m = (rover.latitude - base.latitude) * M_PER_DEG_LAT
        msg.baseline_e_m = (rover.longitude - base.longitude) * m_per_deg_lon
        msg.baseline_d_m = -(rover.altitude - base.altitude)
        msg.baseline_err_h_m = self.err_h
        msg.baseline_err_v_m = self.err_v
        b2 = msg.baseline_n_m ** 2 + msg.baseline_e_m ** 2
        msg.baseline_length_m = math.sqrt(b2 + msg.baseline_d_m ** 2)
        msg.baseline_length_h_m = math.sqrt(b2)

        # as baseline_publisher.cpp, fixed-RTK branch
        dir_rad = math.atan2(msg.baseline_e_m, msg.baseline_n_m)
        if dir_rad < 0.0:
            dir_rad += 2.0 * math.pi
        msg.baseline_dir_deg = (math.degrees(dir_rad) + self.dir_offset) % 360.0
        msg.baseline_dir_err_deg = math.degrees(math.atan2(msg.baseline_err_h_m, msg.baseline_length_h_m))
        msg.baseline_dip_deg = math.degrees(math.atan2(msg.baseline_d_m, msg.baseline_length_h_m)) + self.dip_offset
        msg.baseline_dip_err_deg = math.degrees(math.atan2(msg.baseline_err_v_m, msg.baseline_length_h_m))
        msg.baseline_orientation_valid = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DuroBaselineSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
