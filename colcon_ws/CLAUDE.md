# colcon_ws/ — MTU bringup, MoveIt bridge, cut_stem, Nav2

Sim-side arm drives are in `sim/CLAUDE.md`, container services (`robot_state`, `ekf`, `pruner_stub`) in `robot/CLAUDE.md`, deploying to a robot in `scripts/CLAUDE.md`, per-robot findings in skill `add-real-robot`.

## What's in `src/`

- `mtu32_husky/mtu32_bringup` (submodule): MTU's launch files and per-platform config in `config/<platform>/` (`j100`/`a200`/`a300`).
- Copied verbatim from `robot_data/j100_0921/colcon_ws/src/` (not apt): `docking_utils`, `stow_arm_cpp`, `pruner_action_server`, `kinova_game_pad`, `laser_filters`, `depthimage_to_laserscan`, `plant_cutter_msgs`, `serial_interfaces`, `dual_duro_heading`.
- `moveit_servo`: patched copy overlaying apt's 2.12.4 (`MTU_PATCH.md`). With `use_sim_time` its loop steps `publish_period` on the node clock (upstream's wall-rate loop moved the arm ~2x too far per sim second); wall time unchanged.
- `moveit_sim_bridge` (ours): arm execution in the sim, plus the `arm_goto`/`arm_joints` helpers.
- Sim stand-ins: `swiftnav_ros2_driver` holds only the real `Baseline.msg`; `duro_sim` provides `baseline_node`.

## Launch structure

`mtu32_bringup/launch/sim_robot_upstart.launch.py` is the sim's main launch. It starts:
- `SetParameter use_sim_time` (default `$USE_SIM_TIME`, so false on a real robot);
- the `moveit_sim_bridge` node;
- `bringup_main.launch.py` with `use_natnet:=false` (the sim publishes `ref_pose`) and `moveit:=true`. It forwards `ref_source`, `ref_anchor`, `moveit_delay`, `use_nav2`, `nav2_map` and `scan_topic`;
- `sim_swift_nav_dual.launch.py` when `use_gps_localization` is set (default) and robot.yaml has two or more GPS sensors (generic models have none).

No EKF is started here: the container's `ekf` service publishes `odom→base_link` and `platform/odom/filtered`.

`bringup_main.launch.py` runs on real robots too, via `clearpath-platform-extras`. It starts:
- sensing: `scan_to_scan_filter_chain`, `pcl_filter.launch.py`, `depthimage_to_laserscan` (`config/<platform>/depth2scan.yaml`), `apriltag_node`, `tf2_pose_node`;
- localization: `ref_localizer` (`use_ref_localizer`) and `natnet_ref_pose` (`use_natnet`);
- the cutter stack: `stow_arm_cpp/launch/grid_cutter.launch.py`, `pruner_server` on `/dev/ttyOpenCR` (`pruner_stub` in the sim), and `kinova_game_pad`'s `cut_stem_gamepad.launch.py` (namespace defaults to `j100_0921`);
- `moveit.launch.py` (move_group + servo_node), after `moveit_delay`;
- `bringup_nav2_map.launch.py`, unless `use_nav2:=false`.

- `moveit:=auto` starts MoveIt unless robot.yaml has `manipulators.moveit.enable: true` (Clearpath's `clearpath-manipulators` then runs move_group); see the "MoveIt: ..." log line.
- Open TODO: the cutter stack starts on every robot, but only j100_0921 has the hardware; gate it per robot.

`use_sim_time` reaches every node through `SetParameter`, but a node's own params file overrides it. That's why `bringup_nav2_map` and `sim_nav2` write the value into every `ros__parameters` of the generated Nav2 yaml: some `nav2*.yaml` hardcode false.

## Arm execution in the sim: `moveit_sim_bridge`

The sim arm has no ros2_control: Isaac reads `arm_0/joint_command` (JointState) directly. Without the bridge, move_group finds no controller and every execute returns `CONTROL_FAILED`. `bridge_node.py` provides three interfaces.

`manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory` (FollowJointTrajectory):
- Interpolates between waypoints at 50 Hz (`INTERP_PERIOD_S`); raw waypoint steps made the drive overshoot.
- Reports done only once the arm settles: `settle_tolerance` 0.03 rad, `settle_timeout` 10 s.

`manipulators/arm_0_gripper_controller/gripper_cmd` (GripperCommand):
- `position` is the driven joint's value; the mimic joints follow from the URDF multipliers and offsets.
- Joints are read from `robot_description` (prefix `gripper_joint_prefix`) unless `gripper_joint_names`/`gripper_joint_multipliers` are set.
- Finishes within `gripper_tolerance` (0.02) of the target, on a stall, or after `gripper_timeout` (3 s).

A subscriber on `.../arm_0_joint_trajectory_controller/joint_trajectory`: servo streams there (`command_out_type: trajectory_msgs/JointTrajectory`), not through the action.

Pitfalls:
- Every publish must carry every joint ever commanded. The sim keeps only the last message's arrays, so an arm-only message drops the gripper target.
- Don't use a fixed sleep for GripperCommand. MoveIt allows planned duration × 1.2 + 0.5 s, so a near-zero plan times out.
- With `use_sim_time`, `moveit.launch.py` raises move_group's `allowed_goal_duration_margin` to 12 s (a `<ns>/move_group` params file after `moveit.yaml`; a parameters dict would lose to its node-specific value): data stalls through the shared zenoh router made moves time out while the arm got there. The bridge's `settle_timeout` decides instead. Real robots keep the generated 0.5 s.
- Don't copy `position` to every gripper joint. The 2F Lite tip mimics use −0.676 with a +0.149 offset, so copying moves them backwards.

## Arm helpers (`moveit_sim_bridge`, deployed to real robots)

- `ros2 run moveit_sim_bridge arm_goto <state> [--group G] [--velocity-scale S] [--direct]`: a group_state from `/etc/clearpath/robot.srdf` via `move_action`; `--direct` bypasses MoveIt for the bridge (sim only).
- `arm_joints [--record S]` compares `arm_0/joint_command` with `platform/joint_states`. A real robot has no `joint_command`.
- `robot/bin/arm_goto`/`arm_joints` are wrappers. The web UI uses them on sim and real robots (real robots: only after a deploy). Namespace: `$ROBOT_NAMESPACE`, else robot.yaml `system.ros2.namespace`.
- Ctrl+C: `arm_goto` exits 130 but the goal keeps running (stop it with the UI's Stop motion); `arm_joints --record` exits 0. Any exception raised while `not rclpy.ok()` counts as the interrupt.
- `--timeout` (60 s) is wall time, so in a slow sim a move can still finish after "no result".
- Each `arm_goto` is a new zenoh session: finding move_action takes seconds (waits 15 s) and the goal's answer arrived 6-11 s after the request with three robots moving at once (waits 30 s, logs "goal accepted after N s" past 5 s). "no answer to the goal request" ≠ rejected: move_group may still run it.

## `cut_stem` / `grid_cutter_action_server` (`stow_arm_cpp`)

Start a cut: `ros2 action send_goal /<ns>/cut_stem plant_cutter_msgs/action/CutStem "{start_cutting: true}"`.

Configuration:
- `config/grid_cutter_params.yaml` holds the shared values, tuned for j100_0921 (Gen3 Lite + 2F Lite).
- `grid_cutter.launch.py` then loads `config/robots/<namespace>.yaml` if it exists. Per-robot differences go there; don't fork the package.
- `j100_0921.yaml` sets the IK seed, `tool_xyz` [0,0,0.05] and `moveit_vel_scale` 0.9.
- `a200_0284.yaml` (Gen3 7-DOF + Robotiq 2F-85): zone x 0.44–0.56, 7-joint seed, `drop_joint_positions`, vel/acc scale 0.4 (its sim wrists are slower than the URDF limits), `gripper_close` 0.8, tool offset with matching `grasp_orientation`, ground box lowered for its 0.411 m arm base.

Parameters:
- `preferred_joint_positions` is the IK seed: every pose takes the nearest solution to it. Pick one that keeps the whole zone, approach points included, on one branch.
- `drop_joint_positions`: empty means the SRDF `drop` pose. a200_0284 has no `drop` pose; its joint values are a guess above its `basket` box and should be replaced with a real pose.
- `drop_lower_distance` (0.1): how far the tool goes down after the drop. Not re-verified on a200_0284; if it stops at "singularity", set 0 in its robot file.

Tool frame:
- Params: `tool_link`, `tool_xyz`, `tool_rpy` (the tool pose in `arm_0_end_effector_link`; identity by default) and `publish_tool_tf` (static TF on `tf_static`).
- All patch, approach and drop positions, and `grasp_orientation`, are `tool_link` poses. IK gets the matching ee pose; servo feedback reads base→`tool_link`.
- After changing the offset or rpy, re-check the zone, seed, drop pose and `grasp_orientation`.
- Measure fingertips with `tf2_echo arm_0_end_effector_link <fingertip link>`. Robotiq open: x=±0.068, z=0.098.

Clocks:
- Node clock (`deadline_in`/`ros_sleep`): timeouts, retry pauses and TF-settle waits.
- Wall time, on purpose: `servo_timer_`, action discovery waits, `pruner_server`'s serial timeout, `tf2_pose_node`'s timer.

## Sim-side arm facts behind cut_stem

- Drive force = `ARM_EFFORT_SCALE` (3.0) × the URDF effort for each `arm_0_joint_N` (`configure_arm_drives`). At 1×, joint 2 saturates and the arm collapses. Why the sim needs more torque than the real arm is still unknown.
- `drop_mimic_constraints` (default on for every arm) removes the importer's `NewtonMimicAPI`, which fought the drives.
- Debugging: `arm_joints --record`, the effort field of `platform/joint_states` (a spike of hundreds of N·m means a collision), and `ros2 topic pub` on `arm_0/joint_command`. Only trust these on a fresh sim.

## Nav2

`bringup_main` starts `bringup_nav2_map.launch.py` at boot, the same in the sim and on real robots. It runs map_server and the navigation servers, but no AMCL: `map→odom` comes from `ref_localizer`.

Per-robot settings live in `config/nav2_robots.yaml`, merged in the order defaults → `platforms.<model>` → `robots.<namespace>`:
- `params_file`: default `[nav2_map.yaml, nav2.yaml]`, the first that exists in `config/<platform>/`.
- `scan_topic`: a300 and j100 use `sensors/camera_0/scan`; a200 uses `params`, which keeps the params file's sources.
- `map`: default `zone_end_2.yaml`.
- `param_overrides`: values merged over the params file.

From `bringup_main` or the upstart, override with `nav2_map:=` / `scan_topic:=`. Starting another Nav2 by hand (`sim_nav2`, `bringup_nav2_*`) needs `use_nav2:=false`, or two stacks run.

`sim_nav2.launch.py`, the sim's manual Nav2:
- uses `config/<platform>/nav2.yaml` with `platform/odom`, no static layer and a fixed 100 m global costmap;
- self-filters `sensors/lidar2d_0/scan` with a box. Without it, the sim lidar sees the robot body and the collision monitor won't move. Jackals have no sim lidar: pass `scan_topic:=.../camera_0/scan`;
- publishes a static `map→odom` unless `gps:=true`. Alongside `bringup_main`, that needs `ref_source:=external`.

## Outdoor dual-GPS localization

Real robot: `swift_nav_dual.launch.py`, then `bringup_nav2_mapping_j100.launch.py`. Sim: `sim_swift_nav_dual.launch.py`, then `sim_nav2.launch.py gps:=true` (uses `nav2_mapping.yaml` if the platform has one).

`sim_swift_nav_dual` stands in for the Duro drivers, which need `libsbp`, not packaged for Ubuntu:
- the reference fix comes from the sim's `sensors/gps_<ref>/fix`;
- `baseline_node`, named `att_duro_node`, computes the baseline from the two fixes;
- the real heading filter, `navsat_transform` and `ekf_global_node` then run with `config/<platform>/dual_duro_heading.yaml` (j100's if the platform has none).

GPS details:
- Antennas follow robot.yaml order: the first is attitude (left), the second reference (right). a200/a300 use `gps_0`/`gps_1`, Jackals `gps_1`/`gps_2`.
- The heading frame is `dual_duro_heading`'s `frame_id` parameter (default `gps_2_link`).
- The datum `SIM_DATUM` is the sim's GPS origin, so `map` is Isaac's world frame.
- `ekf_global_node` runs with `publish_tf: false`; use `ekf_publish_tf:=true` only without `bringup_main`.

## Global localization: `ref_localizer` (`mocap_fake_localizer`)

- Frames: `ref_frame → map → odom → base_link`. `ref_frame` is Motive's frame in the lab (`natnet_ref_pose.py`, Motive at 192.168.50.80, rigid body = namespace) or Isaac's world frame in the sim.
- Parameter files, loaded in order: `mtu32_bringup/config/ref_localization.yaml`, then `config/ref_localization/assignments.yaml` (written by the web UI), then `config/ref_localization/<namespace>.yaml`.
- `source` selects what drives `map→odom`: `auto` (the ref pose while it arrives, else GPS), `ref`, `gps` or `external`.
- `anchor` places `map` in `ref_frame`: `fixed` (`map_pose_in_ref`/`anchor_file`), `start` or `external`.
- A stale source holds its last value. Override with `ref_source:=` and `ref_anchor:=`.
- `calibrate_ref_offset.py` measures `base_link_offset` (`--apply`/`--write`). It has only been tested against `test/fake_motive.py`.

## Real robots

- robot.yaml `platform.extras.launch` starts `intel_realsense.launch.py`, `swift_nav_dual.launch.py` and `bringup_main.launch.py`. One failure aborts all of them, GPS and localization included, so each must work on every robot.
- `mtu32_bringup` must keep `<exec_depend>moveit_servo</exec_depend>`; without it, the extras launch fails with "package 'moveit_servo' not found".
- To use Clearpath's move_group: `manipulators.moveit.enable: true` plus `sudo systemctl enable --now clearpath-manipulators`, which provides the Kinova driver, joint_states and TF.
- Foxglove: each robot's robot.yaml needs these under `platform.extras.ros_parameters.foxglove_bridge`, then a restart of clearpath-robot.
  - `asset_uri_allowlist: ['^package://(?:[-A-Za-z0-9_%]+/)*[-A-Za-z0-9_%]+[.](?:dae|DAE|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|STL|tif|tiff|urdf|webp|xacro)$']`, plus the same pattern with `^file:///opt/ros/jazzy/share/` for `file://` meshes. The generator mangles `\\w`, and its default misses `.STL`.
  - `topic_whitelist: ['^/<ns>/.*', '^/rosout$']`. With `.*`, every robot's TF shares one tree.

## Expected warnings

Harmless: `arm_0_gripper` "is not a chain"; "No 3D sensor plugin(s) defined for octomap updates"; `tf2_pose_node` lookups of `jackal_charger_april` (no AprilTag dock visible).
