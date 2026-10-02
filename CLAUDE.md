# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated fleet of Clearpath robots in Isaac Sim 6.0, driven over ROS 2 Jazzy. Isaac Sim runs in one container (WebRTC streaming or a headed window); each robot is its own ROS 2 container standing in for the robot's onboard computer. Robots are either a generic model (`a300`/`a200`/`j100`/`r100`, with a RealSense D435i) or one of MTU's real robots from `robot_data/<id>/robot.yaml` (`j100_0921`, `j100_0922`, `a200_0284`, `a200_0333`, `a300_00036`, ...), chosen per slot by `ROBOT_MODEL_<i>` in `.env`.

There is no build system, test suite or linter: Docker Compose plus scripts. Git: `main`, remote `origin` = github.com/robotian/multirobot_sim. `colcon_ws/src/` holds git submodules; `mtu32_husky` and `mocap_fake_localizer` track a `sim` branch of their own repos, so commit/push there first, then commit the submodule pointer here. `sim/assets/`, `sim/generated/`, `robot_data/` and `sim/colcon_ws/` are gitignored. `.env` is tracked (no secrets, but holds this machine's LAN IP in `ISAACSIM_HOST`).

## Commands

```bash
scripts/x11_auth.sh                       # once per login: /tmp/.docker.xauth (rqt_image_view, rviz, headed mode)
docker compose build                      # clearpath-robot:jazzy (robots + zenoh router) and a300-isaac-sim:6.0.0
scripts/gen_urdf.sh                       # regenerate sim/assets/<model>/ for the 4 generic models + every robot_data/<id>/ with a robot.yaml
scripts/fleet.sh scene                    # start the sim, scene only (no robots)
scripts/fleet.sh spawn [N] [--poses JSON] # spawn N robots (models ROBOT_MODEL_<i>), replacing existing ones; (re)create their containers
scripts/fleet.sh [N|down]                 # scene + spawn N / stop everything (same as scripts/stop_sim.sh)
scripts/fleet_ctl.py state|wait-scene|spawn|clear|reset   # host side of the spawn protocol (fleet.sh and the web UI use it)
scripts/colcon_build.sh [colcon args...]  # colcon build ~/colcon_ws (as `robot`) in every running robot container
python3 tools/sim_ui/server.py            # web UI on http://127.0.0.1:8090 (start/stop/reset, spawn at poses, arm moves, Cut stem)
SIM_MODE=headed scripts/fleet.sh          # Isaac's desktop window instead of WebRTC (needs x11_auth.sh; ~3 min to start)
docker compose logs -f isaac-sim          # sim's own lines are prefixed [fleet]
docker exec -it a300_0000 bash            # robot shell (ROS env sourced); add `-u robot` for ~/colcon_ws work
docker exec -it a300_0000 teleop          # keyboard control -> /<ns>/cmd_vel
docker exec -it a300_0000 rviz            # RViz (fixed frame odom)
docker exec a300_0000 camera_view [depth] # rqt_image_view of the camera
docker exec a300_0000 restart_ros         # kill every ROS 2 node; robot_state/ekf/foxglove/pruner_stub self-heal, manual launches don't
docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py [lin_x] [ang_z] [seconds]'   # smoke test: commanded vs. odometry
docker exec j100_0921 bash -c 'python3 /scripts/calibrate_velocity.py [--modes lin lat ang] [--levels ...]'   # velocity sweep, exit 1 if >10% off
docker exec j100_0921 bash -c 'arm_goto cut_init [--direct] [--velocity-scale 0.3]'   # arm to an SRDF group_state (`arm_goto --list`)
docker exec j100_0921 bash -c 'arm_joints [--record 12]'  # commanded vs observed arm joints as JSON
scripts/foxglove_layout.sh [ns...]        # foxglove/<ns>.json layout with the robot's URDF in a 3D panel
```

## Conventions

- Plain `docker exec <c> python3 ...` has no ROS environment; wrap in `bash -c` (`BASH_ENV=/etc/ros_env.sh` sources `/etc/clearpath/setup.bash`) or use `-it ... bash`.
- `ros2 run` ignores `ROS_NAMESPACE`; namespaced tools pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE` (as `robot/bin/teleop`/`camera_view` do). Topics are relative and land under the robot's namespace.
- Namespace / container name: generic models are `<model>_%04d` by slot (`a300_0000`); real robots (model id contains `_`) use their id with no suffix (`j100_0921`). Compose can't compute this, so `scripts/fleet.sh` writes `ROBOT_SUFFIX_<i>`/`ROBOT_HOSTNAME_<i>` into `.env` — use `fleet.sh`, not `docker compose up -d`, after changing a slot's model. `robot_namespace()` in `setup_scene.py` and `robot/entrypoint.sh` apply the same rule.
- Compose: eight service keys `robot0`…`robot7`; slot *i* has profiles `n<k>` for k > i and `.env` sets `COMPOSE_PROFILES=n${NUM_ROBOTS}`. Compose doesn't stop surplus slots, `fleet.sh` does (`docker compose rm -sf robot<i>`). More than 8 means copying a service block and raising `MAX` in `fleet.sh`.
- Sim tuning knobs are `.env` vars (`FLEET_RMW`, `SIM_RATE_HZ`, `PHYSICS_HZ`, `CAMERA_*`, `SCENE_LANES`, `SIM_SCENE` (empty = ground plane + lights, `lavender`, or a USD file in `sim/scene/`), `ROBOT_LOOKS`, `FORCE_REIMPORT`, `FLEET_DEBUG=1`, `FLEET_SETTINGS="/carb/key=val;..."`). They are passed explicitly in the `isaac-sim` service's `environment:` block, so a new variable must be added there too.
- Ports: Foxglove `ws://<host>:8765+slot` (foxglove_bridge 3.x speaks `foxglove.sdk.v1`); WebRTC client to `ISAACSIM_HOST` on 49100/tcp + 47998/udp.
- Middleware: `FLEET_RMW` (not `RMW_IMPLEMENTATION`, which a ROS host shell would override) = `rmw_fastrtps_cpp` (current `.env`) or `rmw_zenoh_cpp`. Zenoh sessions run in client mode against the `zenoh-router` service (peer mode fails across containers); it costs the sim ~3-4 fps. FastDDS is UDP only (`docker/fastdds_udp.xml`; containers don't share `/dev/shm`). All containers use the same `ROS_DOMAIN_ID` — `entrypoint.sh` rewrites a real robot.yaml's `domain_id` to it.
- TF: `odom→base_link` comes from each robot's EKF (`robot/bin/ekf`) on the sim's `platform/odom`; the sim's exact pose is a separate `ground_truth→base_link_ground_truth` branch. URDF link/joint names must match the USD's.

## Architecture

- **URDF pipeline** (host, `scripts/`): `robot/config/robot.<model>.yaml.tmpl` or a real `robot_data/<id>/robot.yaml` → Clearpath `generate_description` + `xacro` (inside the robot image) → `scripts/flatten_urdf.py` → self-contained `sim/assets/<model>/` (meshes copied, Collada→OBJ, visual-only links merged, dangling joints pruned). See `scripts/CLAUDE.md`.
- **Sim** (`sim/scripts/setup_scene.py`, run in Isaac with `./sim` mounted at `/sim`): imports every model (cached in `sim/generated/`), builds the scene chosen by `SIM_SCENE` (ground plane + lights, the lavender farm, or a USD file from `sim/scene/`) with the timeline stopped, then spawns robots from `spawn_request.json` and plays. Per robot an OmniGraph handles `cmd_vel` → closed-loop velocity control → wheel drives, odometry, joint states, TF, camera, IMU/GPS/lidar script nodes and the arm. Drive constants per model in `MODEL_PARAMS`. See `sim/CLAUDE.md`.
- **Robot containers** (`robot/`): `entrypoint.sh` writes `/etc/clearpath/robot.yaml`, runs Clearpath's `generate_bash`/`generate_params` and `generate_srdf`, then supervises `robot_state`, `ekf`, `foxglove` and `pruner_stub`. `./colcon_ws` is bind-mounted into every robot as `/home/robot/colcon_ws`. See `robot/CLAUDE.md`.
- **ROS workspace** (`colcon_ws/`): MTU's `sim_robot_upstart.launch.py` (MoveIt, `cut_stem`, perception), `moveit_sim_bridge` (the arm's execution path, since there is no ros2_control), Nav2 and dual-GPS localization. See `colcon_ws/CLAUDE.md`.
- **Web UI** (`tools/sim_ui/`): stdlib-only, binds 127.0.0.1, POSTs require `Content-Type: application/json`. See `tools/sim_ui/CLAUDE.md`.
- **Real robots**: adding or debugging one of MTU's robots → skill `add-real-robot` (`.claude/skills/add-real-robot/SKILL.md`).

## Gotchas

- `MODEL_PARAMS[model]["chassis_link"]` must be the prim the importer made the articulation root, not whatever looks like the chassis; a wrong one fails only at runtime ("Articulation controller failed"). Re-check with `drive_test.py` after *any* URDF change.
- `scripts/fleet.sh` does not restart a running sim: after `gen_urdf.sh`, `docker restart a300-isaac-sim`. `gen_urdf.sh` runs inside the robot image (build it first) and skips a real robot whose `robot_data/<id>/` is missing.
- `robot/entrypoint.sh` and `robot/bin/*` are baked into the image: `docker compose build robot0`, then recreate (`scripts/fleet.sh N`). After editing a colcon package: `scripts/colcon_build.sh --packages-select <pkg>` and restart its launch.
- After a run goes chaotic (joints far past limits) everything measured afterwards is meaningless: reset first (`scripts/fleet_ctl.py reset` / the UI's *Reset scene*, ~3.5 s, or restart the sim).
- Sim start is flaky (~1 in 10: Kit hangs at ~35 s with no `[fleet]` line, or exit 139 right after "simulation running"); another `docker restart a300-isaac-sim` clears it. `docker logs` is lost when compose recreates the sim, so save it right after a crash.
- Connecting the WebRTC client stalls the sim a few seconds — enough to fail a MoveIt move; don't connect mid-run.
- 3 robots with `ROBOT_LOOKS=full` crashed the sim (NVIDIA ray-tracing compiler) or froze the PC when robots were added to an already-playing scene; not seen since the scene loads stopped and plays only after spawning. Check the kernel log before debugging sim start failures.
- All 4 generic models in one sim reproduced a `PhysX Internal CUDA error` crash (not root-caused); any ≤3 distinct models work.
