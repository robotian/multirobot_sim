# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated fleet of Clearpath robots in Isaac Sim 6.0, driven over ROS 2 Jazzy. Isaac Sim runs in one container (WebRTC streaming or a headed window); each robot is its own ROS 2 container standing in for the robot's onboard computer. Robots are either a generic model (`a300`/`a200`/`j100`/`r100`, with a RealSense D435i) or one of MTU's real robots from `robot_data/<id>/robot.yaml` (`j100_0921`, `j100_0922`, `a200_0284`, `a200_0333`, `a300_00036`, ...), chosen per slot by `ROBOT_MODEL_<i>` in `.env`.

There is no build system, test suite or linter: Docker Compose plus scripts. Git: `main`, remote `origin` = github.com/robotian/multirobot_sim. `colcon_ws/src/` holds git submodules; `mtu32_husky` and `mocap_fake_localizer` track a `sim` branch of their own repos, so commit/push there first, then commit the submodule pointer here. `sim/generated/`, `robot_data/` and the colcon `build/`/`install/`/`log/` dirs are gitignored; `sim/assets/` is tracked, its binaries (usd, png, jpg, hdr, dae, obj, stl) through Git LFS (`.gitattributes`, run `git lfs install` once per machine). `.env` is tracked (no secrets, but holds this machine's LAN IP in `ISAACSIM_HOST`).

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
scripts/make_farm_scene.py                # sim/scene/lavender_farm.usd from the farm DB's object_data (use: SIM_SCENE=lavender_farm.usd)
docker compose -f basestation.compose.yml up -d --build   # base station: ROS 2 + the farm PostgreSQL (port 5433); `docker exec -it basestation psql`
python3 tools/sim_ui/server.py [--mode sim|real|both]   # web UI on http://127.0.0.1:8090: sim (start/stop/reset, spawn at poses), real robots (tools/sim_ui/real_robots.json, over SSH: services, deploy), both (arm moves, Cut stem, Stop motion, RViz, localization); base station card in every mode (state, database, zenoh links, start/stop/recreate)
SIM_MODE=headed scripts/fleet.sh          # Isaac's desktop window instead of WebRTC (needs x11_auth.sh; ~3 min to start)
docker compose logs -f isaac-sim          # sim's own lines are prefixed [fleet]
docker exec -it a300_0000 bash            # robot shell (ROS env sourced); add `-u robot` for ~/colcon_ws work
docker exec -it a300_0000 teleop          # keyboard control -> /<ns>/cmd_vel
docker exec -it a300_0000 rviz            # RViz (fixed frame odom)
docker exec a300_0000 camera_view [depth] # rqt_image_view of the camera
docker exec a300_0000 restart_ros         # kill every ROS 2 node; robot_state/ekf/foxglove/pruner_stub self-heal, manual launches don't
docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py [lin_x] [ang_z] [seconds]'   # smoke test: commanded vs. odometry
docker exec j100_0921 bash -c 'python3 /scripts/calibrate_velocity.py [--modes lin lat ang] [--levels ...]'   # velocity sweep, exit 1 if >10% off
docker exec j100_0921 bash -c 'arm_goto cut_init [--direct] [--velocity-scale 0.3]'   # arm to an SRDF group_state (`arm_goto --list`); on a real robot: ros2 run moveit_sim_bridge arm_goto (no --direct)
docker exec j100_0921 bash -c 'arm_joints [--record 12]'  # commanded vs observed arm joints as JSON (moveit_sim_bridge's; real robot: observed only)
scripts/foxglove_layout.sh [ns...]        # foxglove/<ns>.json layout with the robot's URDF in a 3D panel
scripts/deploy_robot.sh <id> [--dry-run]  # rsync colcon_ws/src to a real robot's ~/colcon_ws and build it there (over its ~/robot_ws); see scripts/CLAUDE.md
scripts/deploy_robot.sh <id> --pull       # copy files edited on the robot since the last deploy back into colcon_ws/src (deploys nothing)
```

## Conventions

- Plain `docker exec <c> python3 ...` has no ROS environment; wrap in `bash -c` (`BASH_ENV=/etc/ros_env.sh` sources `/etc/clearpath/setup.bash`) or use `-it ... bash`.
- `ros2 run` ignores `ROS_NAMESPACE`; namespaced tools pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE` (as `robot/bin/teleop`/`camera_view` do). Topics are relative and land under the robot's namespace.
- Namespace / container name: generic models are `<model>_%04d` by slot (`a300_0000`); real robots (model id contains `_`) use their id with no suffix (`j100_0921`). Compose can't compute this, so `scripts/fleet.sh` writes `ROBOT_SUFFIX_<i>`/`ROBOT_HOSTNAME_<i>` into `.env` — use `fleet.sh`, not `docker compose up -d`, after changing a slot's model. `robot_namespace()` in `setup_scene.py` and `robot/entrypoint.sh` apply the same rule.
- Compose: eight service keys `robot0`…`robot7`; slot *i* has profiles `n<k>` for k > i and `.env` sets `COMPOSE_PROFILES=n${NUM_ROBOTS}`. Compose doesn't stop surplus slots, `fleet.sh` does (`docker compose rm -sf robot<i>`). More than 8 means copying a service block and raising `MAX` in `fleet.sh`.
- Sim tuning knobs are `.env` vars (`FLEET_RMW`, `SIM_RATE_HZ`, `PHYSICS_HZ`, `CAMERA_*`, `SCENE_LANES`, `SIM_SCENE` (empty = ground plane + lights, `lavender`, or a USD file in `sim/scene/`), `USE_SIM_TIME`, `SIM_REF_POSE` (default 1: per robot `ref_pose` + TF `ref_frame→base_link_ref`, the exact world pose, the sim's stand-in for motion capture), `ROBOT_LOOKS`, `FORCE_REIMPORT`, `FLEET_DEBUG=1`, `FLEET_SETTINGS="/carb/key=val;..."`). They are passed explicitly in the `isaac-sim` service's `environment:` block, so a new variable must be added there too.
- Ports: Foxglove `ws://<host>:8765+slot` (foxglove_bridge 3.x speaks `foxglove.sdk.v1`); WebRTC client to `ISAACSIM_HOST` on 49100/tcp + 47998/udp.
- Middleware: `FLEET_RMW` (not `RMW_IMPLEMENTATION`, which a ROS host shell would override) = `rmw_zenoh_cpp` (current `.env`, as on the real robots) or `rmw_fastrtps_cpp`. Zenoh sessions run in client mode against the `zenoh-router` service (peer mode fails across containers); it costs the sim ~3-4 fps. FastDDS is UDP only (`docker/fastdds_udp.xml`; containers don't share `/dev/shm`). All containers use the same `ROS_DOMAIN_ID` — `entrypoint.sh` rewrites a real robot.yaml's `domain_id` and `middleware.implementation` to the fleet's.
- Time: `USE_SIM_TIME` (`.env`, default true, passed to the sim and every robot) = the sim publishes `/clock` (physics time, monotonic across Stop/Play) and stamps everything with it; `robot/bin/*` pass `use_sim_time:=$USE_SIM_TIME`, `sim_robot_upstart`/`sim_nav2`/`sim_swift_nav_dual` default to it (unset = false on a real robot) and `SetParameter` it for every node. New ROS code: time with the node clock, never `time.time()`/`steady_clock`/wall timers for anything that waits on the robot; a node of your own needs `use_sim_time:=true`.
- TF: `odom→base_link` comes from each robot's EKF (`robot/bin/ekf`) on the sim's `platform/odom`; the sim's exact pose is a separate `ground_truth→base_link_ground_truth` branch (zero at spawn) and, in world coordinates, `ref_frame→base_link_ref` (`SIM_REF_POSE`). `map→odom` has one publisher, `mocap_fake_localizer`'s `ref_localizer.py` (started by `bringup_main`), from `ref_pose` (the sim, or OptiTrack via `natnet_ref_pose.py` on a real robot) or GPS (`ekf_global_node` with `publish_tf: false`); `ref_frame→map` is its anchor. Anything else that publishes `map→odom` (AMCL, slam_toolbox, `sim_nav2.launch.py`'s static identity) needs `ref_source:=external`. URDF link/joint names must match the USD's.

## Architecture

- **URDF pipeline** (host, `scripts/`): `robot/config/robot.<model>.yaml.tmpl` or a real `robot_data/<id>/robot.yaml` → Clearpath `generate_description` + `xacro` (inside the robot image) → `scripts/flatten_urdf.py` → self-contained `sim/assets/<model>/`. `mtu32_description` (the real robots' `platform.extras` xacro) is built from `colcon_ws/src/mtu32_husky`, the same source the robots use; after changing it, rerun `gen_urdf.sh` and restart the sim (meshes copied, Collada→OBJ, visual-only links merged, dangling joints pruned). See `scripts/CLAUDE.md`.
- **Sim** (`sim/scripts/setup_scene.py`, run in Isaac with `./sim` mounted at `/sim`): imports every model (cached in `sim/generated/`), builds the scene chosen by `SIM_SCENE` (ground plane + lights, the lavender farm, or a USD file from `sim/scene/`) with the timeline stopped, then spawns robots from `spawn_request.json` and plays. Per robot an OmniGraph handles `cmd_vel` → closed-loop velocity control → wheel drives, odometry, joint states, TF, camera, IMU/GPS/lidar script nodes and the arm. Per-model parameters (drive, sensors, arm, chassis_link) are derived at sim start from each model's robot.yaml + flattened URDF; hand tuning only in `sim/config/model_params.yaml`, never in `setup_scene.py`. See `sim/CLAUDE.md`.
- **Robot containers** (`robot/`): `entrypoint.sh` writes `/etc/clearpath/robot.yaml`, runs Clearpath's `generate_bash`/`generate_params` and `generate_srdf`, then supervises `robot_state`, `ekf`, `foxglove` and `pruner_stub`. `./colcon_ws` is bind-mounted into every robot as `/home/robot/colcon_ws`. See `robot/CLAUDE.md`.
- **ROS workspace** (`colcon_ws/`): MTU's `sim_robot_upstart.launch.py` (MoveIt, `cut_stem`, perception), `moveit_sim_bridge` (the arm's execution path, since there is no ros2_control), Nav2 and dual-GPS localization. See `colcon_ws/CLAUDE.md`.
- **Base station** (`basestation.compose.yml`, `basestation/`): separate compose project (`fleet.sh down` leaves it running), image `FROM clearpath-robot:jazzy` + PostgreSQL 18. `network_mode: host`, so it is on the LAN like a real base station: real robots' FastDDS discovery works, sim robots reach it via the fleet bridge `br-fleet` (fixed name in `docker-compose.yml`) and, for zenoh, it runs its own router on host port 7447 (sessions are its clients) that dials `BASESTATION_ZENOH_CONNECT` (space-separated; default the fleet's `zenoh-router`, published on `127.0.0.1:7448`; add real robots' `tcp/<ip>:7447`). The database (port 5433, `admin`, `test_lavender_farming`, `PGPASSWORD` from `db.env`) is what `status_server`'s `config.yaml` (`host.docker.internal:5433`) points at; `basestation/initdb/` (gitignored dumps) loads only on the first start (empty `basestation_pgdata` volume).
- **Web UI** (`tools/sim_ui/`): stdlib-only, binds 127.0.0.1, POSTs require `Content-Type: application/json`. Sim robots through `docker exec`, real robots over SSH; the page's mode (sim / real / both) picks which. See `tools/sim_ui/CLAUDE.md`.
- **Real robots**: adding or debugging one of MTU's robots → skill `add-real-robot` (`.claude/skills/add-real-robot/SKILL.md`).

##Important
- The contents in colcon_ws/src folder should be identical across the robot models. 
- The colcon_ws/src will be deployed to real robots. It should work in real robots too.




## Gotchas

- This host runs ufw (default drop), which drops container→host traffic, so the sim robots can't reach host-network services (base station DDS and PostgreSQL) until `sudo ufw allow in on br-fleet`. Published ports, like the old `robotian_database` container's, bypass it.

- The drive graph's chassis is the importer's articulation root, read at spawn (`chassis_prim`); a robot with several roots is refused at spawn. Still run `drive_test.py` after *any* URDF change, and check the `[fleet] params <model>:` log line for a new robot.
- `scripts/fleet.sh` does not restart a running sim: after `gen_urdf.sh`, `docker restart a300-isaac-sim`. `gen_urdf.sh` runs inside the robot image (build it first) and skips a real robot whose `robot_data/<id>/` is missing.
- `robot/entrypoint.sh` and `robot/bin/*` are baked into the image: `docker compose build robot0`, then recreate (`scripts/fleet.sh N`). After editing a colcon package: `scripts/colcon_build.sh --packages-select <pkg>` and restart its launch.
- After a run goes chaotic (joints far past limits) everything measured afterwards is meaningless: reset first (`scripts/fleet_ctl.py reset` / the UI's *Reset scene*, ~3.5 s, or restart the sim).
- Sim start is flaky (~1 in 10: Kit hangs at ~35 s with no `[fleet]` line, or exit 139 right after "simulation running"); another `docker restart a300-isaac-sim` clears it. `docker logs` is lost when compose recreates the sim, so save it right after a crash.
- Connecting the WebRTC client stalls the sim a few seconds — enough to fail a MoveIt move; don't connect mid-run.
- 3 robots with `ROBOT_LOOKS=full` crashed the sim (NVIDIA ray-tracing compiler) or froze the PC when robots were added to an already-playing scene; not seen since the scene loads stopped and plays only after spawning. Check the kernel log before debugging sim start failures.
- All 4 generic models in one sim reproduced a `PhysX Internal CUDA error` crash (not root-caused); any ≤3 distinct models work.
