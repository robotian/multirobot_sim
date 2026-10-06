"""Move the arm (or gripper) to a named group_state from /etc/clearpath/robot.srdf.

  arm_goto --list                       list groups and their named states as JSON
  arm_goto cut_init                     plan + execute with move_group (collision-aware, like RViz)
  arm_goto cut_init --direct            skip planning: straight joint-space trajectory via moveit_sim_bridge
  arm_goto open --group arm_0_gripper   gripper states go through the GripperCommand action
  options: --velocity-scale S (0-1, default 0.3), --duration T (direct mode only, seconds)

Needs move_group for plan mode (in the sim: sim_robot_upstart.launch.py; on a real robot: clearpath-manipulators or
bringup_main) and, in the sim, moveit_sim_bridge to execute. --direct is sim only (it goes straight to
moveit_sim_bridge's trajectory server). `ros2 run moveit_sim_bridge arm_goto ...` (robot/bin/arm_goto in the sim);
the namespace is $ROBOT_NAMESPACE, else robot.yaml's.
"""
import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET

from moveit_sim_bridge.namespace import robot_namespace

SRDF = "/etc/clearpath/robot.srdf"
MAX_JOINT_VEL = 1.6  # rad/s, the Kinova Gen3 Lite's URDF velocity limit for joints 1-5


def load_states():
    groups = {}
    for gs in ET.parse(SRDF).getroot().findall("group_state"):
        states = groups.setdefault(gs.get("group"), {})
        # The generated SRDF can list the same name twice (e.g. "zero"); keep the first.
        states.setdefault(gs.get("name"), {j.get("name"): float(j.get("value")) for j in gs.findall("joint")})
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("state", nargs="?")
    ap.add_argument("--group", default="arm_0")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--velocity-scale", type=float, default=0.3)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    groups = load_states()
    if args.list:
        print(json.dumps(groups))
        return 0
    target = groups.get(args.group, {}).get(args.state)
    if target is None:
        print(f"unknown state {args.state!r} for group {args.group!r}; have {sorted(groups.get(args.group, {}))}")
        return 2

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    rclpy.init()
    node = Node("arm_goto", namespace=robot_namespace())
    try:
        if args.group.endswith("gripper"):
            return gripper_goto(node, ActionClient, target, args)
        if args.direct:
            return direct_goto(node, ActionClient, target, args)
        return plan_goto(node, ActionClient, target, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_result(node, client, goal, timeout, label):
    import rclpy
    if not client.wait_for_server(timeout_sec=5.0):
        print(f"{label}: action server not available -- is sim_robot_upstart.launch.py running?")
        return None
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print(f"{label}: goal rejected")
        return None
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=timeout)
    if not res.done():
        print(f"{label}: no result after {timeout:.0f}s")
        return None
    return res.result().result


def plan_goto(node, ActionClient, target, args):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = args.group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = args.velocity_scale
    req.max_acceleration_scaling_factor = args.velocity_scale
    req.start_state.is_diff = True
    req.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
        for j, v in target.items()])]
    goal.planning_options.plan_only = False

    result = wait_result(node, ActionClient(node, MoveGroup, "move_action"), goal, args.timeout, "move_group")
    if result is None:
        return 1
    code = result.error_code.val
    names = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
    print(f"move_group: {names.get(code, code)}")
    return 0 if code == MoveItErrorCodes.SUCCESS else 1


def current_positions(node, joints):
    import rclpy
    from sensor_msgs.msg import JointState
    seen = {}
    sub = node.create_subscription(JointState, "platform/joint_states",
                                   lambda m: seen.update(zip(m.name, m.position)), 10)
    deadline = time.monotonic() + 5.0
    while not all(j in seen for j in joints) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return seen


def direct_goto(node, ActionClient, target, args):
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    names = list(target)
    now = current_positions(node, names)
    if not all(j in now for j in names):
        print("direct: no platform/joint_states received")
        return 1
    delta = max(abs(target[j] - now[j]) for j in names)
    duration = args.duration or max(1.0, delta / (MAX_JOINT_VEL * max(args.velocity_scale, 0.01)))

    def point(positions, t):
        return JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9)))

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    goal.trajectory.points = [point([now[j] for j in names], 0.0), point([target[j] for j in names], duration)]
    client = ActionClient(node, FollowJointTrajectory,
                          "manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory")
    result = wait_result(node, client, goal, duration + args.timeout, "trajectory")
    if result is None:
        return 1
    print(f"direct: sent {delta:.3f} rad max joint delta over {duration:.2f}s, error_code={result.error_code}")
    return 0 if result.error_code == 0 else 1


def gripper_goto(node, ActionClient, target, args):
    from control_msgs.action import GripperCommand
    # Only the right bottom finger joint is actuated; the other three are URDF mimics of it.
    position = target.get("arm_0_gripper_right_finger_bottom_joint", next(iter(target.values())))
    goal = GripperCommand.Goal()
    goal.command.position = position
    result = wait_result(node, ActionClient(node, GripperCommand, "manipulators/arm_0_gripper_controller/gripper_cmd"),
                         goal, args.timeout, "gripper")
    if result is None:
        return 1
    print(f"gripper: position {position:.3f}, reached_goal={result.reached_goal}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
