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

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Real URDF joint names + their <mimic> multiplier relative to the one actually-driven joint
# (arm_0_gripper_right_finger_bottom_joint, multiplier 1.0), read from /etc/clearpath/robot.urdf's own
# <mimic joint="arm_0_gripper_right_finger_bottom_joint" multiplier="..."/> tags.
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
        self.declare_parameter('gripper_joint_names', DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter('gripper_joint_multipliers', DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.gripper_joint_names = list(self.get_parameter('gripper_joint_names').value)
        self.gripper_joint_multipliers = list(self.get_parameter('gripper_joint_multipliers').value)

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

    def _sleep_until(self, start, target_t):
        sleep_for = start + target_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    def _execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = trajectory.points
        if not points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        start = time.monotonic()

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

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _execute_gripper(self, goal_handle):
        position = goal_handle.request.command.position
        positions = [position * m for m in self.gripper_joint_multipliers]
        self._publish(self.gripper_joint_names, positions)
        time.sleep(1.0)  # crude settle time -- no force/position feedback loop, see module docstring
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = True
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
