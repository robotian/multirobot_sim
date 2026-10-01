# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated fleet of Clearpath robots (each with a RealSense D435i), one of A300/A200/Jackal(j100)/Ridgeback(r100) per slot (`ROBOT_MODEL_<i>` in `.env`, default a300), in Isaac Sim 6.0, driven over ROS 2 Jazzy. Isaac Sim runs in one container (WebRTC streaming, no local GUI); each robot is its own ROS 2 container standing in for the robot's onboard computer. There is no build system, test suite or linter — everything is Docker Compose plus a few scripts. It is a git repo (`main`, remote `origin` = github.com/robotian/multirobot_sim, public; clone with `--recurse-submodules` — `colcon_ws/src/` holds 7 git submodules (`mtu32_husky` and `mocap_fake_localizer` track a `sim` branch of their own repos, so commit/push sim edits there first, then commit the updated submodule pointer here) plus plain-file packages); `sim/assets/` and `sim/generated/` are gitignored because `scripts/gen_urdf.sh` and the sim's first start rebuild them, and `.env` is tracked (no secrets, but it holds this machine's LAN IP in `ISAACSIM_HOST`).

## Commands

```bash
scripts/x11_auth.sh                       # once per login: builds /tmp/.docker.xauth (needed for rqt_image_view in robot containers)
docker compose build                      # clearpath-robot:jazzy (robots + zenoh router) and a300-isaac-sim:6.0.0 (Isaac + system ROS 2)
scripts/gen_urdf.sh                       # regenerate sim/assets/<model>/{<model>.urdf,meshes,robot.yaml}: the 4 generic models (a300 a200 j100 r100, from robot/config/robot.<model>.yaml.tmpl) plus one per robot_data/<id>/ folder that has a robot.yaml
scripts/fleet.sh scene                    # start the sim with the scene only (no robots); waits for it. On a stopped sim it also deletes the old spawn request and robot containers
scripts/fleet.sh spawn [N] [--poses JSON] # spawn N robots (NUM_ROBOTS; models ROBOT_MODEL_<i>) into the running scene, replacing its robots, then (re)create their containers (--no-deps --force-recreate); removes surplus slots
scripts/fleet.sh [N|down]                 # scene + spawn N / stop everything
scripts/fleet_ctl.py state|wait-scene|spawn|clear   # host side of the spawn protocol (fleet.sh and the web UI use it)
scripts/stop_sim.sh                       # stop and remove every container + the spawn request (docker compose down misses profiles outside NUM_ROBOTS; same as `fleet.sh down`)
scripts/colcon_build.sh [colcon args...]  # colcon build ~/colcon_ws (as `robot`) in every running robot container
docker compose logs -f isaac-sim          # the sim's own lines are prefixed [fleet] (plain `docker compose up -d` spawns no robots unless a spawn request is left over)
docker exec -it a300_0000 bash            # shell in a robot (sources /etc/clearpath/setup.bash via /etc/bash.bashrc; ROBOT_NAMESPACE=a300_0000)
docker exec -it -u robot a300_0000 bash   # shell as the `robot` user, for ~/colcon_ws (shared across all robot containers)
docker exec -it a300_0000 teleop          # keyboard control (i/j/l/,/k), publishes /a300_0000/cmd_vel
docker exec -it a300_0001 rviz            # RViz with fixed frame odom, RobotModel, TF, camera (robot/config/robot.rviz.tmpl, model-agnostic)
scripts/foxglove_layout.sh [ns...]        # foxglove/<ns>.json: Foxglove layout with a 3D panel showing the robot's URDF (from /<ns>/robot_description; the panel only auto-loads /robot_description)
docker exec a300_0000 camera_view [depth] # rqt_image_view of the D435i on the host display (needs x11_auth.sh)
docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py [lin_x] [ang_z] [seconds]'   # smoke test: commanded vs. odometry
docker exec j100_0921 bash -c 'python3 /scripts/calibrate_velocity.py [--modes lin lat ang] [--levels 0.1 .. 1.0] [--ns <other robot>]'   # velocity calibration sweep (stows the arm first, low acceleration, exit 1 if any level is >10% off)
docker exec a300_0000 restart_ros         # kill every ROS 2 node in the container; robot_state/foxglove/pruner_stub self-heal, anything manually launched does not (see CLAUDE.md)
python3 tools/sim_ui/server.py            # web UI on http://127.0.0.1:8090: start (scene only)/stop/reset sim, spawn robots at chosen poses, start/stop sim_robot_upstart, move arm to SRDF states (+ cmd-vs-obs joint plots)
SIM_MODE=headed scripts/fleet.sh          # Isaac Sim's own desktop window on this machine's X display instead of WebRTC streaming (or the web UI's View selector; needs scripts/x11_auth.sh; the desktop app takes ~3 min to reach setup_scene.py vs ~30 s)
docker exec j100_0921 bash -c 'arm_goto cut_init [--direct] [--velocity-scale 0.3]'   # arm to a named SRDF group_state (MoveIt plan, or --direct trajectory); `arm_goto --list`
docker exec j100_0921 bash -c 'arm_joints [--record 12]'  # commanded (arm_0/joint_command) vs observed (platform/joint_states) arm joints as JSON
```

- **`tools/sim_ui/`**: stdlib-only local web UI that runs the commands above (start/stop/reset sim, spawn at poses, arm moves with cmd-vs-obs plots, Cut stem). Binds 127.0.0.1; POSTs require `Content-Type: application/json`. Details: `tools/sim_ui/CLAUDE.md`.

- `gen_urdf.sh` runs inside the robot image, so build it first. It always (re)generates every model unconditionally -- the 4 generic ones plus every `robot_data/<id>/` folder containing a `robot.yaml` (the folder name is the model id; nothing to register in the script) -- so it doesn't need to know which are in use. A real robot whose `robot_data` folder is missing (e.g. `j100_0936`, `a200_0333` at the time of writing) is simply not regenerated; its old `sim/assets/<id>/` stays until deleted. Re-run it after changing any `robot.<model>.yaml.tmpl` or `flatten_urdf.py`.

- Plain `docker exec <c> python3 ...` has no ROS environment (the entrypoint only covers PID 1); wrap in `bash -c` (the image sets `BASH_ENV=/etc/ros_env.sh`, which sources `/etc/clearpath/setup.bash` once it exists) or use `-it ... bash`.

- `ros2 run` ignores `ROS_NAMESPACE` (only launch files honour it). Namespaced tools must pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`, which is what `robot/bin/teleop` and `camera_view` do.

- Foxglove: each robot runs `foxglove_bridge` (`robot/bin/foxglove`, whitelisted to its own namespace) on host port `8765 + slot index` (slot *i* is that robot, whatever model it runs) — connect with `ws://<host>:<port>`. foxglove_bridge 3.x speaks the `foxglove.sdk.v1` subprotocol (not `foxglove.websocket.v1`), so custom clients must request that.

- Streaming client connects to `ISAACSIM_HOST` (from `.env`) on ports 49100/tcp and 47998/udp.

- Sim tuning knobs are env vars in `.env` (`FLEET_RMW`, `ZENOH_ROUTER`, `SIM_RATE_HZ`, `PHYSICS_HZ`, `CAMERA_*`, `CAMERA_STREAMS`, `FORCE_REIMPORT`, `FLEET_DEBUG=1` for rtf/fps/pose logging, `FLEET_SETTINGS="/carb/key=val;..."` for arbitrary Kit settings). They are passed through explicitly in the `isaac-sim` service `environment:` block, so a new variable must be added there too.

## Architecture

Detailed notes live next to the code and load when you work there — read the relevant one before changing that area:
- `scripts/CLAUDE.md` — URDF pipeline (`gen_urdf.sh` → `generate_description` → `flatten_urdf.py`: mesh copying, Collada→OBJ, `merge_visual_only_links`, why visual-only links must be merged).
- `sim/CLAUDE.md` — `setup_scene.py`: import cache, scene-then-spawn protocol, time/rate caveat, `MODEL_PARAMS`/`chassis_link`, Ridgeback omni drive, velocity calibration loop, wheel brake, robot looks, 2D/3D raycast lidar.
- `robot/CLAUDE.md` — container boot (`entrypoint.sh`, `generate_bash`/`generate_params`/`generate_srdf`, background services, `pruner_stub`, `restart_ros`, upstream quirks).
- `colcon_ws/CLAUDE.md` — `sim_robot_upstart.launch.py`, `moveit_sim_bridge`, arm torque limits, `cut_stem` per-robot overrides and tool frame, Nav2 + dual-GPS localization.
- Skill `add-real-robot` (`.claude/skills/add-real-robot/SKILL.md`) — the real MTU robots from `robot_data/` and everything found wiring each one up.

Cross-cutting gotchas (each one has cost a debugging session):
- `MODEL_PARAMS[model]["chassis_link"]` must be the prim the importer made the articulation root, not whatever looks like the chassis; a wrong one fails only at runtime ("Articulation controller failed"), with no import warning. Re-check with `drive_test.py` after *any* URDF change.
- `scripts/fleet.sh` does not restart a running sim: after `gen_urdf.sh` run `docker restart a300-isaac-sim`. Restarting the sim is also the only way to reset robots' physical state; after a run goes chaotic (joints far past limits) everything measured afterwards is meaningless, so reset first.
- Sim start is flaky (~1 in 10: Kit hangs at ~35 s with no `[fleet]` line, or exit 139 right after "simulation running"); another `docker restart a300-isaac-sim` clears it.
- `robot/entrypoint.sh` and `robot/bin/*` are baked into the image: `docker compose build robot0` then recreate (`scripts/fleet.sh N`). After editing a colcon package: `scripts/colcon_build.sh --packages-select <pkg>` and restart its launch.
- Real robots (model id contains `_`) use their id as namespace and container name with no slot suffix; `fleet.sh` computes `ROBOT_SUFFIX_<i>`/`ROBOT_HOSTNAME_<i>` in `.env`, so use it, not `docker compose up -d`, after changing a slot's model.
- Connecting the WebRTC client stalls the sim a few seconds — enough to fail a MoveIt move; don't connect mid-run.

**TF ownership:** `odom→base_link` comes from the robot's platform EKF (`robot/bin/ekf`, below), fusing the sim's `platform/odom` (frame `odom`, an exact pose relative to the spawn point, standing in for wheel odometry) and the IMU where `localization.yaml` lists one. The sim publishes its exact pose only as a separate branch, `ground_truth→base_link_ground_truth` (a frame can have one parent, so it can't be `ground_truth→base_link`; compare `base_link` with `base_link_ground_truth` in RViz), plus `camera_0_link→camera_0_color_optical_frame`, so URDF link/joint names must match the USD's; wheel joints come from `platform/joint_states`. The `teleop`, `camera_view` and `rviz` wrappers in `robot/bin/` are installed to `/usr/local/bin` and are model-agnostic (they only read `$ROBOT_NAMESPACE`).

Robot services in compose share the `x-robot` anchor; compose defines eight robot service **keys** `robot0`…`robot7` (stable, model-agnostic — never typed by a user), each with `container_name`/`hostname`/`ROBOT_NAMESPACE`/`ROS_NAMESPACE` interpolated as `${ROBOT_MODEL_<i>:-a300}_%04d` for its fixed index *i* (e.g. slot 1 with `ROBOT_MODEL_1=j100` becomes `j100_0001`) and `ROBOT_MODEL: ${ROBOT_MODEL_<i>:-a300}` in its environment. Slot *i* has the profiles `n<k>` for every k > i, and `.env` sets `COMPOSE_PROFILES=n${NUM_ROBOTS}` (compose interpolates inside `.env`, and a shell `NUM_ROBOTS` overrides it), so `NUM_ROBOTS=N` starts exactly the first N. The `isaac-sim` service gets nothing robot-related (no `NUM_ROBOTS`/`ROBOT_MODELS` any more, only `SCENE_LANES`): robots come from spawn requests, so changing them can never make compose recreate the sim, and `fleet.sh spawn` starts the robot services with `--no-deps --force-recreate` (explicitly naming a service enables its profile). The sim derives each slot's namespace with `robot_namespace()`, the same rule compose uses. Ports/hostnames are fixed per slot index (Foxglove `8765+i`) regardless of model. Compose does not stop services of inactive profiles when the count is lowered (`--remove-orphans` doesn't either), hence `scripts/fleet.sh`, which removes surplus slots via `docker compose rm -sf robot<i>` (the stable service key, not a guessed container name — correct regardless of which model that slot last ran) and reads real slot names back one at a time via `docker compose ps --format '{{.Name}}' robot<i>` for its status line (batch-querying several service names at once returns them alphabetically, **not** in argument/slot order — found by testing, not assumed). Going beyond 8 means copying a service block and raising `MAX` in `fleet.sh`.

**Networking / middleware:** sim and robots share the private `ros` bridge network. `FLEET_RMW` in `.env` (deliberately not `RMW_IMPLEMENTATION`, which a ROS-configured host shell would export over `.env`) picks `rmw_zenoh_cpp` (compose fallback if `.env` omits it) or `rmw_fastrtps_cpp` for every container via the `x-rmw-env` anchor. The tracked `.env` currently pins `rmw_fastrtps_cpp` — a runtime choice made while investigating streaming performance, not a change of the intended default. The image tag is `clearpath-robot:jazzy` (renamed from `a300-robot:jazzy` once it started serving 4 models) and the compose project is `clearpath-fleet` (was `a300-fleet`); `docker compose down` under the old project name/file won't find containers created under the new one and vice versa — clean up stragglers with plain `docker rm -f`/`docker network rm` if switching between very old and current checkouts of this repo.

- zenoh: all sessions run in **client** mode (`ZENOH_CONFIG_OVERRIDE`) against the single `zenoh-router` service. The default `peer` mode fails across containers because peers only listen on loopback. `ZENOH_ROUTER` can point at an external router.

- Isaac Sim's bundled ROS libs only have Fast DDS/Cyclone, so `docker/isaac-sim.Dockerfile` adds system ROS 2 Jazzy + rmw_zenoh_cpp; `docker/isaac-entrypoint.sh` sources it only when `FLEET_RMW=rmw_zenoh_cpp` (Isaac then loads the system ROS instead of its internal one). Zenoh costs ~3–4 fps in the sim relative to FastDDS, so lower `SIM_RATE_HZ` when using it (~15 gave rtf ≈ 0.95 for 3 robots).

- FastDDS: UDP only (`docker/fastdds_udp.xml`, mounted into every container); shared-memory transport must stay disabled because containers don't share `/dev/shm`.
All containers must have the same `ROS_DOMAIN_ID`. `volume-init` chowns the named volumes to uid 1234 (Isaac Sim's user) before the sim starts.

