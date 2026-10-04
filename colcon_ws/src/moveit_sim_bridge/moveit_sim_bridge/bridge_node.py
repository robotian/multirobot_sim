#!/usr/bin/env python3
"""FollowJointTrajectory + GripperCommand action servers that drive this sim's own arm_0/joint_command topic
(an IsaacArticulationController subscribed via ROS2SubscribeJointState -- see sim/scripts/setup_scene.py's
"Arm + gripper" OmniGraph section) instead of a real ros2_control hardware interface, which this project's arm
does not have.

Exists because move_group/moveit_servo's own moveit_simple_controller_manager plugin expects a real
FollowJointTrajectory/GripperCommand action server at a fixed name to actually carry out a planned trajectory --
without this, MoveIt plans successfully but every execute() call fails immediately with CONTROL_FAILED
(confirmed live: no action server was ever listening on .../arm_0_joint_trajectory_controller/
follow_joint_trajectory, only moveit_simple_controller_manager itself as a client).

Deliberately time-based, not feedback-convergence-based: plays a trajectory back at its own real time pacing
and reports SUCCEEDED once playback finishes -- it does not check the sim's actual joint_states to confirm the
arm really reached each target, since IsaacArticulationController's own position servo (already tuned via
configure_arm_drives) is what's actually responsible for getting there. A reasonable sim-bridge simplification,
not a claim that this matches a real JointTrajectoryController's own tolerance/aborting behavior.

**Linearly interpolates between consecutive trajectory waypoints (at INTERP_PERIOD_S) rather than jumping
straight to each one's own raw position.** An earlier version published exactly one JointState per waypoint, at
that waypoint's own time_from_start -- reported live as "the arm moves too fast compared to the real robot, and
it overshoots at the goal position." Root cause: a real JointTrajectoryController continuously interpolates
between a planned trajectory's (often sparse) waypoints, so the commanded position ramps smoothly; publishing
only the waypoints themselves meant IsaacArticulationController's own PD position servo (configure_arm_drives'
own STIFFNESS/DAMPING, already documented there as untuned against a real robot) saw large instantaneous target
jumps instead, and tried to close each one as fast as its own gains allowed -- fast, and prone to overshoot on a
step input, independent of how slowly the *trajectory itself* was actually paced. Interpolating removes the
step inputs without needing to touch the PD gains themselves.

Of the gripper's 4 real finger joints (from the Kinova 2F Lite -- see robot.srdf's own arm_0_gripper group),
only arm_0_gripper_right_finger_bottom_joint is actually independently actuated; the other 3 are <mimic> joints
of it in the real URDF (confirmed by reading /etc/clearpath/robot.urdf directly, not assumed from the group
name), each with its own multiplier -- notably right_finger_tip_joint and left_finger_tip_joint both mimic at
-0.676 (an *inverted* sign relative to the driven joint). Sending GripperCommand's single position identically
to all 4 (an earlier version of this bridge did) ignored those multipliers entirely, so most of the gripper's
own joints moved with the wrong sign/magnitude relative to the one real command -- this is what a report of the
gripper's motion being "reversed" traced back to, not a single global sign flip. Fixed by treating
GripperCommand's position as the primary joint's own raw value and deriving the other 3 from their real
multipliers; overridable via the gripper_joint_multipliers parameter for a different gripper.

Also plain-passthroughs moveit_servo's own real-time output: with `command_out_type: trajectory_msgs/
JointTrajectory` (see mtu32_bringup's servo_config.yaml), servo_node doesn't use the FollowJointTrajectory
action at all -- it streams single-point JointTrajectory messages directly onto a plain topic
(command_out_topic, 100Hz) meant for a real JointTrajectoryController's own topic-based command input. Found
live: grid_cutter_action_server's coarse move_group-planned moves worked immediately once the action server
above existed, but its later fine "servo_to_pose" approach phase still timed out, stuck at a fixed distance from
target -- because nothing was listening on this separate topic at all.

**Every publish sends the full, merged joint set (arm + gripper), not just whichever joints the triggering
command actually named.** Found live, the hard way: closing the gripper genuinely worked in isolation (a raw
GripperCommand goal with nothing else running), but during a real cut_stem sequence the gripper opened and then
never closed, even though close_gripper_on_stem's own GripperCommand goal did succeed -- because
ROS2SubscribeJointState (arm_0/joint_command's subscriber) only ever remembers the *last received message's*
own name/position arrays, applying them wholesale every tick (see setup_scene.py's own "ArmCmd's outputs hold
their last-received message's values between messages" comment for the analogous arm-vs-wheel case). The very
next arm-only trajectory/servo message (moveit_servo streams continuously) would silently replace the whole
commanded set, dropping the gripper's just-closed target the instant it arrived -- not a hypothetical race, a
guaranteed one, since try_prune_once always calls another servo_to_pose immediately after closing the gripper.
Keeping this bridge's own last-known position for every joint and publishing all of them together every time
means a gripper command can never again be silently overwritten by an unrelated arm command, or vice versa.
"""
import threading
import time

import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

# Fallback gripper description (Kinova 2F Lite): real URDF joint names + their <mimic> multiplier relative to the
# one actually-driven joint (arm_0_gripper_right_finger_bottom_joint, multiplier 1.0), read from
# /etc/clearpath/robot.urdf's own <mimic joint="..." multiplier="..."/> tags. Only used when the gripper can't be
# derived from the robot's own URDF (see MoveItSimBridge._gripper_from_urdf) and no parameter overrides it --
# this package is shared by every robot model, so the gripper is normally auto-detected (e.g. a200_0284 has a
# Robotiq 2F-85: driver arm_0_gripper_robotiq_85_left_knuckle_joint, 5 mimics with multipliers -1/1/-1/-1/1).
DEFAULT_GRIPPER_JOINT_NAMES = [
    'arm_0_gripper_right_finger_bottom_joint',
    'arm_0_gripper_right_finger_tip_joint',
    'arm_0_gripper_left_finger_bottom_joint',
    'arm_0_gripper_left_finger_tip_joint',
]
DEFAULT_GRIPPER_JOINT_MULTIPLIERS = [1.0, -0.676, 1.0, -0.676]

INTERP_PERIOD_S = 0.02  # 50Hz -- roughly a real JointTrajectoryController's own control-loop cadence


class MoveItSimBridge(Node):

    def __init__(self):
        super().__init__('moveit_sim_bridge')
        # Empty (the default) = derive the gripper from the robot's URDF; set both to force a specific gripper.
        # ('' / 1.0 are placeholders: an empty list can't carry a type, and an untyped declaration logs a
        # "parameter not initialized" warning whenever a tool lists this node's parameters.)
        self.declare_parameter('gripper_joint_names', [''])
        self.declare_parameter('gripper_joint_multipliers', [1.0])
        self.declare_parameter('gripper_joint_prefix', 'arm_0_gripper')
        self.gripper_joint_names = [n for n in self.get_parameter('gripper_joint_names').value if n]
        self.gripper_joint_multipliers = (
            list(self.get_parameter('gripper_joint_multipliers').value) if self.gripper_joint_names else [])
        self.gripper_joint_offsets = [0.0] * len(self.gripper_joint_names)
        self._gripper_explicit = bool(self.gripper_joint_names)
        self._gripper_ready = threading.Event()
        if self._gripper_explicit:
            self._gripper_ready.set()
        else:
            # robot_state_publisher's latched robot_description (same namespace as this node).
            self.create_subscription(
                String, 'robot_description', self._on_robot_description,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))

        # After the last trajectory point has been played back, wait until the arm has actually arrived (like a real
        # JointTrajectoryController's goal tolerance / settle time) before reporting success. Without this, robots
        # whose joints slew slower in the sim than the URDF velocity limits MoveIt plans with (a200_0284's Gen3 7-DOF
        # wrists: ~0.5-0.8 rad/s vs. 1.22) are still ~1 rad short when the caller starts its next step.
        self.declare_parameter('settle_tolerance', 0.03)  # rad, per joint
        self.declare_parameter('settle_timeout', 10.0)    # s, after the last point; then succeed anyway with a warning
        # GripperCommand completes as soon as the driven joint is within gripper_tolerance of the target (at once if it
        # already is), or after gripper_timeout (fingers blocked by an object: stalled, still reported as success like
        # a real gripper holding something). It used to sleep a fixed 1.0 s, which MoveIt aborted as TIMED_OUT
        # whenever RViz sent a near-zero-length gripper trajectory (e.g. "close" while already closed): MoveIt only
        # allows trajectory duration x 1.2 + 0.5 s, i.e. 0.5 s for those.
        self.declare_parameter('gripper_tolerance', 0.02)  # rad
        self.declare_parameter('gripper_timeout', 3.0)     # s
        self._observed = {}  # joint name -> latest position from platform/joint_states
        self.create_subscription(JointState, 'platform/joint_states', self._on_joint_states, 50)

        self.cmd_pub = self.create_publisher(JointState, 'arm_0/joint_command', 10)
        self._positions_lock = threading.Lock()
        self._last_positions = {}  # joint name -> last commanded position, merged across all sources

        # moveit_servo's own streamed output (see module docstring) -- a plain topic, not the action below.
        self.create_subscription(
            JointTrajectory,
            'manipulators/arm_0_joint_trajectory_controller/joint_trajectory',
            self._on_servo_trajectory,
            10,
        )

        cb_group = ReentrantCallbackGroup()
        self._trajectory_server = ActionServer(
            self, FollowJointTrajectory,
            'manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )
        self._gripper_server = ActionServer(
            self, GripperCommand,
            'manipulators/arm_0_gripper_controller/gripper_cmd',
            execute_callback=self._execute_gripper,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )
        self.get_logger().info(
            'moveit_sim_bridge ready: bridging FollowJointTrajectory/GripperCommand to arm_0/joint_command')

    def _on_joint_states(self, msg):
        self._observed.update(zip(msg.name, msg.position))

    def _wait_until_settled(self, names, final_positions):
        """Block until every named joint is within settle_tolerance of its final trajectory position."""
        tol = self.get_parameter('settle_tolerance').value
        timeout = self.get_parameter('settle_timeout').value
        start = self._now()
        while True:
            worst = max((abs(self._observed[n] - p) for n, p in zip(names, final_positions) if n in self._observed),
                        default=0.0)
            if worst <= tol:
                if self._now() - start > 0.2:
                    self.get_logger().info('arm settled %.1fs after the last trajectory point (residual %.3f rad)' % (
                        self._now() - start, worst))
                return
            if self._now() - start > timeout:
                lag = {n: round(self._observed[n] - p, 2) for n, p in zip(names, final_positions)
                       if n in self._observed and abs(self._observed[n] - p) > tol}
                self.get_logger().warning('arm did not settle within %.0fs of the last trajectory point: %s' % (timeout, lag))
                return
            self._sleep(0.05)

    def _on_robot_description(self, msg):
        if self._gripper_ready.is_set():
            return
        try:
            names, mults, offsets = self._gripper_from_urdf(msg.data, self.get_parameter('gripper_joint_prefix').value)
        except Exception as e:  # malformed URDF: keep waiting for the timeout fallback, don't crash the bridge
            self.get_logger().warning(f'could not parse robot_description for the gripper: {e!r}')
            return
        if names:
            self.gripper_joint_names, self.gripper_joint_multipliers, self.gripper_joint_offsets = names, mults, offsets
            self.get_logger().info('gripper from URDF: driver %s, mimics %s' % (
                names[0], ', '.join(f'{n} x{m:g}{o:+g}' for n, m, o in zip(names[1:], mults[1:], offsets[1:]))))
            self._gripper_ready.set()

    @staticmethod
    def _gripper_from_urdf(urdf_xml, prefix):
        """Names, multipliers and offsets of the gripper joints, driver first: every non-fixed joint whose name
        starts with `prefix`; the driver is the one with no <mimic>, every other joint's value is
        multiplier * (its mimic target's value) + offset, resolved through chains of mimics down to the driver."""
        root = ET.fromstring(urdf_xml)
        joints = {}
        for j in root.findall('joint'):
            name = j.get('name')
            if not name.startswith(prefix) or j.get('type') == 'fixed':
                continue
            m = j.find('mimic')
            joints[name] = None if m is None else (
                m.get('joint'), float(m.get('multiplier', 1.0)), float(m.get('offset', 0.0)))
        drivers = [n for n, m in joints.items() if m is None]
        if not drivers:
            return [], [], []
        driver = drivers[0]

        def resolve(name, depth=0):  # -> (multiplier, offset) relative to the driver, or None if it isn't tied to it
            if name == driver:
                return 1.0, 0.0
            m = joints.get(name)
            if m is None or depth > 10:
                return None
            up = resolve(m[0], depth + 1)
            if up is None:
                return None
            return m[1] * up[0], m[1] * up[1] + m[2]

        names, mults, offsets = [driver], [1.0], [0.0]
        for n in joints:
            r = resolve(n) if n != driver else None
            if r is not None:
                names.append(n)
                mults.append(r[0])
                offsets.append(r[1])
        return names, mults, offsets

    def _await_gripper(self, timeout=5.0):
        """Block (in the action's own thread) until the gripper is known; fall back to the Kinova 2F Lite."""
        if not self._gripper_ready.wait(timeout):
            self.get_logger().warning(
                f'no usable robot_description after {timeout:g}s: assuming the Kinova 2F Lite gripper')
            self.gripper_joint_names = list(DEFAULT_GRIPPER_JOINT_NAMES)
            self.gripper_joint_multipliers = list(DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
            self.gripper_joint_offsets = [0.0] * len(self.gripper_joint_names)
            self._gripper_ready.set()

    def _publish(self, names, positions):
        # Merge into the persistent last-known set and publish all of it, not just this update's own joints --
        # see module docstring for why a partial message would silently drop whichever joints it omits.
        with self._positions_lock:
            for name, position in zip(names, positions):
                self._last_positions[name] = float(position)
            names_out, positions_out = zip(*self._last_positions.items())
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names_out)
        msg.position = list(positions_out)
        self.cmd_pub.publish(msg)

    def _on_servo_trajectory(self, msg):
        if msg.points:
            self._publish(msg.joint_names, msg.points[0].positions)

    @staticmethod
    def _seconds(duration):
        return duration.sec + duration.nanosec * 1e-9

    def _now(self):
        """Seconds on the node's clock: ROS time, i.e. the simulator's /clock with use_sim_time, else wall time.
        All pacing and timeouts here use it, so a sim running slower than real time plays a trajectory at its
        planned speed in simulated time (wall-clock pacing would move the arm too fast relative to the sim)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep_until(self, start, target_t):
        while self._now() < start + target_t and rclpy.ok():
            time.sleep(0.005)

    def _sleep(self, seconds):
        self._sleep_until(self._now(), seconds)

    def _execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = trajectory.points
        if not points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        self.get_logger().info(
            'trajectory: %d points over %.1fs, first %s, final %s' % (
                len(points), self._seconds(points[-1].time_from_start),
                [round(v, 3) for v in points[0].positions], [round(v, 3) for v in points[-1].positions]))
        start = self._now()

        # First waypoint: no prior point to interpolate from, so jump straight to it (matches MoveIt-generated
        # trajectories, whose own first point is normally at time_from_start == 0, i.e. the planning start state).
        prev_point = points[0]
        prev_t = self._seconds(prev_point.time_from_start)
        self._sleep_until(start, prev_t)
        self._publish(names, prev_point.positions)

        for point in points[1:]:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
            curr_t = self._seconds(point.time_from_start)
            segment_duration = curr_t - prev_t
            steps = max(1, round(segment_duration / INTERP_PERIOD_S))
            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
                frac = step / steps
                self._sleep_until(start, prev_t + frac * segment_duration)
                interp = [p0 + frac * (p1 - p0) for p0, p1 in zip(prev_point.positions, point.positions)]
                self._publish(names, interp)
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = names
            feedback.desired = point
            goal_handle.publish_feedback(feedback)
            prev_point = point
            prev_t = curr_t

        self._wait_until_settled(names, points[-1].positions)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _execute_gripper(self, goal_handle):
        position = goal_handle.request.command.position
        self._await_gripper()
        offsets = self.gripper_joint_offsets or [0.0] * len(self.gripper_joint_multipliers)
        positions = [position * m + o for m, o in zip(self.gripper_joint_multipliers, offsets)]
        self._publish(self.gripper_joint_names, positions)
        driver, target = self.gripper_joint_names[0], positions[0]
        tol = self.get_parameter('gripper_tolerance').value
        timeout = self.get_parameter('gripper_timeout').value
        start = self._now()
        last, last_change = self._observed.get(driver), start
        reached = stalled = False
        while True:
            current = self._observed.get(driver)
            if current is not None and abs(current - target) <= tol:
                reached = True
                break
            if current is not None and last is not None and abs(current - last) > 1e-3:
                last_change = self._now()
            last = current
            if self._now() - last_change > 0.5 and self._now() - start > 0.5:
                stalled = True  # not moving any more (e.g. closed on an object)
                break
            if self._now() - start > timeout:
                break
            self._sleep(0.02)
        self.get_logger().info('gripper -> %.3f: %s after %.2fs (driver at %s)' % (
            position, 'reached' if reached else ('stalled' if stalled else 'timed out'), self._now() - start,
            'unknown' if current is None else '%.3f' % current))
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = float(current) if current is not None else position
        result.reached_goal = reached
        result.stalled = stalled
        return result


def main():
    rclpy.init()
    node = MoveItSimBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
