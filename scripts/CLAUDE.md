# scripts/ — robot description pipeline, real-robot deploy and backup

## URDF pipeline (`gen_urdf.sh` → `flatten_urdf.py`)

- Input per model: generic `robot/config/robot.<model>.yaml.tmpl` (placeholders filled with throwaway values), or a real robot's `robot_data/<id>/robot.yaml` used unmodified.
- Clearpath's `generate_description` + `xacro` run in the robot image; `mtu32_description` is built from the `colcon_ws/src/mtu32_husky` submodule, never a separate copy (one went stale before).
- `flatten_urdf.py` output is self-contained (`sim/assets/<model>/`), so the Isaac container needs no ROS packages:
  - meshes copied into `meshes/<pkg>/`, paths made relative; `<gazebo>`/`<ros2_control>` dropped; root name = model id (the generated serial would be mangled by USD).
  - Collada: unnamed materials get a name (they crash the importer); non-Blender `.dae` (RealSense `d435.dae`) becomes a vertex-clustered OBJ (`DECIMATE_CELL`), because the importer silently drops it (invisible camera) and its 231k triangles cost ~30% fps.
  - `prune_dangling_joints`: drops joints whose parent link is undefined (xacro doesn't validate; e.g. a camera on an arm the robot doesn't have).
  - `merge_visual_only_links`: links with only `<visual>` (Jackal fenders) are folded into their parent (`merged_links.json` tells `setup_scene.py` where a merged sensor frame went). As separate bodies their fixed joint drifts once the robot drives; a synthetic mass didn't help. Their orientation is correct: don't "fix" it.
  - `weld_empty_root_children`: an empty root link with several fixed children gets them re-parented onto the one with `<inertial>`; otherwise each becomes its own articulation fixed to the world.
  - `limit_continuous_mimic_followers`: continuous `<mimic>` joints (Robotiq 2F-85) get a finite range from their driver; PhysX rejects them otherwise (sim crash).
  - `limit_continuous_arm_joints`: continuous `arm_0_joint*` become ±3.12 rad, or MoveIt plans past ±π and then refuses every start state. Wheels stay continuous.
  - `apply_mass_deltas`: what-if mass offsets wired per model in `gen_urdf.sh` (j100_0921 `chassis_link:10`); inertia unchanged.

## Deploying to a real robot (`deploy_robot.sh <id>`)

Workspaces on the robot, listed in `/etc/clearpath/robot.yaml` `system.ros2.workspaces` (sourced in order, later override):
- `~/robot_ws` first: the robot's own packages (drivers, arm, cameras), built by hand (`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`), with a `COLCON_IGNORE` in each package this repo provides.
- any other workspaces next, `~/colcon_ws` last: exactly `colcon_ws/src` (rsync `--delete`), built by the script with the earlier workspaces as underlays.

Pitfalls:
- A package an underlay still builds is the robot's own: the script skips this repo's copy ("not built") and removes an earlier build of it. `COLCON_IGNORE` it in `robot_ws` to use this repo's.
- Same name ≠ same package: `swiftnav_ros2_driver` here is a messages-only stand-in for the sim; on the robot it's Swift Navigation's driver, and overriding it broke `clearpath-platform-extras`.
- Robot-side edits: `~/colcon_ws/src` has no `.git`; the deploy compares against `~/colcon_ws/.deployed_manifest` and stops on drift (`--pull` to bring edits back, `--force` to overwrite). Building writes nothing into `src`, so any drift is a real edit.
- Host default `cpr-<id, _→->.local` (mDNS): robot.yaml hostnames don't resolve from the lab network. Needs key login (`ssh-copy-id`). Restarts nothing.

Per-robot workspaces:
- a300_00036: `~/robot_ws`, `~/mocap4r2_ws`, `~/colcon_ws`. Keeps its own `swiftnav_ros2_driver` and `status_interfaces` (its own packages depend on it; an underlay can't depend on its overlay). `image_detection/.venv` was rewritten to `~/robot_ws` paths. Clearpath forks are `COLCON_IGNORE`d (apt packages run). `~/colcon_ws_backup` = pre-deploy workspace: keep it.

## Backing up a real robot (`backup_robot.sh <id> [--no-sudo]`)

- Snapshots what `deploy_robot.sh` doesn't own into `robot_data/<id>/backups/` (gitignored, mode 700: netplan holds the WiFi password); layout in the script header.
- Not backed up: `~/colcon_ws`, `~/colcon_ws_backup` (3.5 GB on a300_00036), rosbags (hundreds of GB).
- `sudo` needs a password (netplan files are root-only), asked once over `ssh -t`; `--no-sudo` skips those files.
- `RESTORE.md` is manual on purpose: writing robot.yaml restarts the robot's services.

## Real-robot middleware (zenoh)

- Router config: `ssh robot@<ip> python3 - [wifi_if] < scripts/zenoh_router_config.py`, then `profile: /home/robot/zenoh_config/router.json5` under `system.ros2.middleware` in robot.yaml. It rate-limits only WiFi egress, so the robot's own nodes keep full rate. The profile replaces the whole router config: test a new one with a throwaway router (`ZENOH_ROUTER_CONFIG_URI=<file> ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:17447"]' timeout 5 ros2 run rmw_zenoh_cpp rmw_zenohd`). Clearpath's `setup.bash` also exports it as `FASTRTPS_DEFAULT_PROFILES_FILE`: remove the line before switching that robot to Fast DDS. Installed on a300_00036 and a200_0284 (`wlo1`).
- Remote sessions must be clients (`ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<robot>:7447"]'`): a peer sees almost nothing, since the robot's nodes listen on localhost and Clearpath's router has `peers_failover_brokering: false`.
- Data stalls come from subscribe/unsubscribe, not bandwidth: each new subscription pauses the robot's data ~0.3 s, and dozens at once pin its `rmw_zenohd` for seconds. Use long-lived subscriptions, RViz with only the needed displays, and per-robot tools as clients of that robot's router.
- Several real robots through the base station's router need its tx queues at 16 batches per priority (set in `basestation/entrypoint.sh`, not as a runtime override, which a restart loses); with the default 2, a router restart left robots without data. Not yet stress-tested (RViz open/close through the base station, a robot's router restarting); per-robot tools still connect straight to each robot's router.
- Fleet view (`fleet_viz.sh`: relay `fleet_viz.py` + a read-only `foxglove_bridge` in the base station): both are plain base station sessions (its own router). As two clients straight of the sim's router (`tcp/127.0.0.1:7448`), the relay's `/fleet/tf` reached the other client in one run of two, 0 msg/s in the other (2026-10-10); through the base station's router, every run. Frames are renamed `<ns>/<frame>` except `ref_frame` (`--shared` adds more); static transforms are re-stamped to time zero. The base station mounts the main checkout's `scripts/`, so `fleet_viz.sh` copies the relay in from its own checkout at each start. Its start and its stop each paused every sim robot's TF ~2.5-3.5 s, then ~3-5 s at a quarter rate (delayed, not lost); subscribing one topic per 0.25 s instead of ~40 at once changed nothing, so it's the two sessions joining / leaving, not the subscriptions.
- Cyclone DDS was tried and reverted: its default participant index allows ~10 ROS processes per host (the robot runs ~30), and the robot then can't talk to the zenoh sim and robots.
