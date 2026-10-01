# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# from swiftnav_ros2_driver.msg import Baseline
# from sensor_msgs.msg import Imu

# class DualDuroHeadingNode(Node):
#     def __init__(self):
#         super().__init__('dual_duro_heading_node')
#         self.sub = self.create_subscription(Baseline, 'baseline', self.callback, 10)
#         self.pub = self.create_publisher(Imu, 'heading_imu', 10)
#         self.get_logger().info("Dual Duro Heading Node Started. Awaiting Baseline data...")

#     def callback(self, msg):
#         # mode 4 = Fixed RTK (centimeter accurate), mode 3 = Float RTK
#         if msg.mode not in [3, 4]:
#             self.get_logger().warn("Waiting for RTK Baseline Fix...", throttle_duration_sec=3.0)
#             return

#         # Calculate Yaw in ROS ENU frame (Assumes Right Antenna -> Left Antenna alignment)
#         yaw_enu = math.atan2(msg.baseline_n_m, msg.baseline_e_m) - (math.pi / 2.0)

#         if yaw_enu < -math.pi:
#             yaw_enu += 2.0 * math.pi
#         elif yaw_enu > math.pi:
#             yaw_enu -= 2.0 * math.pi

#         # self.get_logger().info(f"Received Baseline: N={msg.baseline_n_m:.3f} m, E={msg.baseline_e_m:.3f} m, Yaw (ENU)={math.degrees(yaw_enu):.2f} deg") 
#         # Convert Euler Yaw to Quaternion
#         cy = math.cos(yaw_enu * 0.5)
#         sy = math.sin(yaw_enu * 0.5)
        
#         imu_msg = Imu()
#         imu_msg.header = msg.header
#         imu_msg.header.frame_id = "gps_2_link"
        
#         imu_msg.orientation.w = cy
#         imu_msg.orientation.x = 0.0
#         imu_msg.orientation.y = 0.0
#         imu_msg.orientation.z = sy

#         # Set a tight covariance for Fixed RTK heading (e.g., ~0.0001 rad^2)
#         imu_msg.orientation_covariance[8] = 0.0001 
        
#         self.pub.publish(imu_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DualDuroHeadingNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from swiftnav_ros2_driver.msg import Baseline
from sensor_msgs.msg import Imu

class DualDuroHeadingNode(Node):
    def __init__(self):
        super().__init__('dual_duro_heading_node')
        
        # ROS 2 Parameters
        self.declare_parameter('max_heading_err_deg', 5.0)  # Drop if error estimate > 5 degrees
        # Frame of the heading IMU message. Default is the Jackal's reference antenna; a200/a300 have gps_0/gps_1
        # (gps_2_link does not exist there, so robot_localization could not transform the heading).
        self.frame_id = self.declare_parameter('frame_id', 'gps_2_link').value
        
        self.sub = self.create_subscription(Baseline, 'baseline', self.callback, 10)
        self.pub = self.create_publisher(Imu, 'heading_imu', 10)
        self.get_logger().info("Dual Duro Heading Node Started. Awaiting Fixed RTK Baseline data...")

    def callback(self, msg):
        # -------------------------------------------------------------
        # FIX 1: Strict Mode Filtering
        # Mode 4 = Fixed RTK (Centimeter / high angular precision).
        # Mode 3 = Float RTK (DO NOT USE near buildings; causes major yaw jumps).
        # -------------------------------------------------------------
        if msg.mode != 4:
            self.get_logger().warn(
                f"Dropping baseline: Low RTK precision (mode={msg.mode}). Requires Fixed RTK (mode=4).", 
                throttle_duration_sec=3.0
            )
            return

        # -------------------------------------------------------------
        # FIX 2: Validate Hardware Orientation Flags & Error Thresholds
        # -------------------------------------------------------------
        if not getattr(msg, 'baseline_orientation_valid', True):
            self.get_logger().warn("Baseline report indicates invalid orientation.", throttle_duration_sec=3.0)
            return

        max_err = self.get_parameter('max_heading_err_deg').get_parameter_value().double_value
        # if hasattr(msg, 'baseline_dir_err_deg') and msg.baseline_dir_err_deg > max_err:
        #     self.get_logger().warn(
        #         f"Heading error too high ({msg.baseline_dir_err_deg:.2f}° > {max_err:.2f}°). Dropping measurement.",
        #         throttle_duration_sec=3.0
        #     )
        #     return

        # -------------------------------------------------------------
        # Calculate Yaw in ROS ENU frame
        # (Assumes Right Antenna -> Left Antenna lateral alignment)
        # -------------------------------------------------------------
        yaw_enu = math.atan2(msg.baseline_n_m, msg.baseline_e_m) - (math.pi / 2.0)

        # Normalize yaw to [-pi, pi]
        if yaw_enu < -math.pi:
            yaw_enu += 2.0 * math.pi
        elif yaw_enu > math.pi:
            yaw_enu -= 2.0 * math.pi

        # Euler to Quaternion (Yaw only, Roll=0, Pitch=0)
        cy = math.cos(yaw_enu * 0.5)
        sy = math.sin(yaw_enu * 0.5)
        
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = self.frame_id
        
        imu_msg.orientation.w = cy
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = sy

        # -------------------------------------------------------------
        # FIX 3: Proper Covariance Initialization
        # Set large variance for unused variables (Roll, Pitch) so EKF ignores them.
        # Set tight variance for Yaw.
        # -------------------------------------------------------------
        yaw_variance = 0.0001  # ~0.57 degrees standard deviation squared
        
        # Optional: Dynamically calculate yaw variance if error is provided by driver
        if hasattr(msg, 'baseline_dir_err_deg') and msg.baseline_dir_err_deg > 0.0:
            rad_err = math.radians(msg.baseline_dir_err_deg)
            yaw_variance = max(rad_err * rad_err, 0.0001)

        imu_msg.orientation_covariance = [
            1e6,  0.0,  0.0,
            0.0,  1e6,  0.0,
            0.0,  0.0,  yaw_variance
        ]

        # -------------------------------------------------------------
        # FIX 4: Explicitly mark Angular Velocity and Acceleration as unmeasured (-1)
        # -------------------------------------------------------------
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualDuroHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()