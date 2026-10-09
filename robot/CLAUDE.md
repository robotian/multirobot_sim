# robot/ — robot containers

One image (`Dockerfile`, base `osrf/ros:jazzy-desktop-full`) for every model; only `/etc/clearpath/robot.yaml` differs. Container main process runs as root.

## Boot order (`entrypoint.sh`)

1. Real robot id (`ROBOT_MODEL` contains `_`, e.g. `j100_0921`): `ROBOT_NAMESPACE` = that id (drops the compose slot suffix); written to `/etc/robot_ns_env.sh` so `docker exec` shells get it too.
2. Serial: real id with `_`→`-`, else namespace with `_`→`-` (clearpath_config accepts only `<model>-<unit>` or `cpr-<model>-<unit>`).
3. `robot.yaml`: `robot_data/<id>/robot.yaml` copied as is if it exists, except `domain_id` and middleware are rewritten to the fleet's `ROS_DOMAIN_ID`/`FLEET_RMW` (a200_0284's own yaml says domain 1). Otherwise rendered from `/opt/clearpath/robot.<model>.yaml.tmpl` (`__NS__`, `__SERIAL__`, `__RMW__`, `__DOMAIN__`).
4. Creates a placeholder `colcon_ws/install/setup.bash` if nothing is built yet, so sourcing never fails.
5. uid remap of `robot` (below), then `chown -R robot:robot colcon_ws`.
6. `generate_bash` (clearpath_generator_common, same as a real robot's systemd units) → `/etc/clearpath/setup.bash`: ROS, `colcon_ws/install`, `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` from robot.yaml. Sourced, then the `/opt/clearpath_robot_ws` overlay.
7. One-shot `generate_params`, then `generate_srdf`.
8. Exports `ROS_NAMESPACE` (launch files only; `ros2 run` still needs `__ns`).
9. Four background services, each `while true; do X || true; sleep 2; done`, logging to `/tmp/<name>.log`: `robot_state`, `ekf`, `foxglove`, `pruner_stub`. `|| true` hides crash loops: check the log.

`docker exec` shells skip the entrypoint: `BASH_ENV`/bashrc source `/etc/ros_env.sh` (`/etc/clearpath/setup.bash` or plain ROS, the overlay, then `/etc/robot_ns_env.sh`).

## `robot` user and colcon_ws

- Image renames Ubuntu 24.04's uid 1000 `ubuntu` account to `robot`.
- At boot `robot` moves to the uid/gid owning the host checkout (`stat /scripts`, read-only and never chowned; `HOST_UID`/`HOST_GID` override; skipped for uid 0).
- `/home/robot/colcon_ws` is one bind mount of host `./colcon_ws`, shared by all robot containers. `robot` is only for `docker exec -u robot` workspace work.
- The workspace reaches `setup.bash` through robot.yaml's `system.ros2.workspaces` (as on a real robot).

## bin/ tools

- `generate_params`: runs `RobotParamGenerator`/`RobotLaunchGenerator` from `clearpath_generator_robot` → `/etc/clearpath/{platform,manipulators,sensors}/{config,launch}`. That package is source-only (clearpath_robot repo, jazzy branch). The Dockerfile builds it plus `clearpath_sensors` (its templates, required) into `/opt/clearpath_robot_ws`. (apt's clearpath_generator_common leaves `generate_sensors()`/`LaunchGenerator` abstract; the unavailable `exec_depend`s are harmless.) Output is for reference only: the launch files expect a real ros2_control hardware interface (the sim drives wheels via OmniGraph), hardcode `use_sim_time=false` and would clash on topics. `ekf` only uses `localization.yaml`.
- `generate_srdf`: `/etc/clearpath/robot.srdf` for MoveIt (`mtu32_bringup` `moveit.launch.py` fails without it). Same steps as `generate_semantic_description` (`SemanticDescriptionGenerator` + `moveit_collision_updater`), but:
  - `--default --always --trials 10000` instead of Clearpath's `--trials 100000`, because ≥ ~30000 always crashes (stack smashing).
  - Trials = random configurations sampled; a pair that never collides in any sample is disabled for good. Too few (`--trials 1`: ~368 pairs) let MoveIt plan the a200_0284 gripper into `rail_link`. 10000 (~1 s, ~265 pairs) keeps those enabled. Stripping all "Never" pairs (~47 left) made `moveit_servo` jerky.
  - Still aborts at random (~50% at 10000), so it retries 10000×4, 5000×2, 2000×2, then 1.
  - Runs `generate_description` itself (`robot_state` starts after it) and prunes dangling joints first.
- `robot_state`: `generate_description` + xacro → `robot_state_publisher` (namespaced, `joint_states:=platform/joint_states`). Before publishing it:
  - prunes dangling joints (mtu32_description mounts `camera_1` on `arm_0_end_effector_link` even without an arm, e.g. j100_0922; the strict loader aborts);
  - turns `arm_0_joint*` continuous joints into ±3.12 rad revolute (else MoveIt plans across ±π and the start state goes invalid);
  - if either changed the URDF, overwrites `robot.urdf.xacro` with it too (moveit.launch.py reads the xacro);
  - rewrites `file://.../share/<pkg>/` meshes to `package://` in the published topic only (foxglove_bridge serves only `package://`).
- Dangling-joint pruning is copied inline from `scripts/flatten_urdf.py` (host-side, not in the image): keep the copies in sync.
- `ekf`: `robot_localization` `ekf_node` with the generated `localization.yaml`, `publish_tf:=true` → `platform/odom/filtered` + `odom→base_link`. The sim's exact pose is only `ground_truth→base_link_ground_truth`, so base_link has one parent.
- `foxglove`: port 8765 (compose maps a host port per robot), topics whitelisted to `^/<ns>/.*` + `/rosout`.
- `pruner_stub`: fake OpenCR firmware for `pruner_action_server`, stdlib only. PTY with `/dev/ttyOpenCR` symlinked to the slave. On any `<int>\n`, waits 2 s and replies `STATUS:DONE\n`; it never fails.
- `restart_ros`: `pkill -9` of everything ROS (`ros2 run|launch`, `/opt/ros/*/lib/`, `colcon_ws/install/*/lib/`, pruner_stub). After 5 s it reports whether the four looped services are back. Manually launched stacks stay stopped.
- `teleop` (TwistStamped, `stamped:=true`), `rviz` (from `robot.rviz.tmpl`), `camera_view` (`rqt_image_view` on `sensors/camera_0/<color|depth>/image`). `arm_goto`/`arm_joints` are shims to `moveit_sim_bridge` in colcon_ws.

## Sim time

`USE_SIM_TIME` (from `.env`) is in every container's environment. `robot_state`, `ekf`, `foxglove`, `teleop` and `rviz` pass `-p use_sim_time:=${USE_SIM_TIME:-false}`. In `ekf` it comes after `--params-file` because the generated `localization.yaml` says `False`.

## Upstream quirks and workarounds

- `generate_bash` (clearpath_generator_common 2.9.15) uses `workspace.strip('setup.bash')`, a character-set strip: keep the template's `/home/robot/colcon_ws/install` exactly (immune). Other spellings get mangled (`colcon_ws` → `colcon_w`).
- `intel_realsense.urdf.xacro` is patched in the Dockerfile with `use_nominal_extrinsics="true"`, because no realsense driver publishes the depth/color frames in the sim (`camera_0_depth_frame`, `camera_0_color_optical_frame`).
- Deleting `colcon_ws/install` under a running container: new `docker exec` shells print one `No such file` line and carry on; a rebuild fixes it.
