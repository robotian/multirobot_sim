# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

A fleet of Clearpath robots simulated in Isaac Sim 6.0 and driven over ROS 2 Jazzy: one Isaac Sim container, plus one ROS 2 container per robot standing in for its onboard computer. Docker Compose plus scripts; no build system, test suite or linter. Architecture, setup and the full command reference are in `README.md`; each area has its own `CLAUDE.md` (`sim/`, `robot/`, `colcon_ws/`, `scripts/`, `tools/sim_ui/`). Adding or debugging a real MTU robot → skill `add-real-robot`.

## How it fits together

`scripts/gen_urdf.sh` turns each model's Clearpath config (`robot/config/robot.<model>.yaml.tmpl`, or a real robot's `robot_data/<id>/robot.yaml`) into a self-contained URDF under `sim/assets/<model>/`. The sim (`sim/scripts/setup_scene.py`, `./sim` mounted) imports every model at start, builds the scene, then spawns robots when `scripts/fleet_ctl.py` writes `sim/generated/fleet/spawn_request.json` (progress in `state.json`), so changing robots never restarts the sim. Each robot container boots like a real robot (`robot/entrypoint.sh` renders `/etc/clearpath/robot.yaml` and runs Clearpath's generators) and shares one bind-mounted `colcon_ws`. Under zenoh (the default) every session is a client of the `zenoh-router` service. The base station (`basestation.compose.yml`: ROS 2 + the farm's PostgreSQL on port 5433) is a separate compose project on the host network, untouched by `fleet.sh down`. Its server also holds `fleet_config`, the fleet's settings (profiles, robot slots, real robots, change log, runs): `.env` is generated from it by `scripts/fleetcfg.py` (catalog: `scripts/fleet_settings.py`), and edited in the web UI's `/config` page.

## Commands

```bash
scripts/fleet.sh [N]                 # start the sim (scene) and spawn N robots (renders .env from the settings first)
scripts/fleet.sh scene | spawn [N] [--poses JSON] | down
scripts/fleetcfg.py show | set KEY=VALUE | slot I MODEL [--pose X,Y,YAW] | robots N | profile ... | history | runs
scripts/fleet_ctl.py reset           # robots back to their spawn state (~3.5 s); also state | snapshot | clear
docker compose logs -f isaac-sim     # the sim's own lines start with [fleet]
scripts/colcon_build.sh [--packages-select <pkg>]   # builds colcon_ws in every running robot container
scripts/gen_urdf.sh                  # regenerate URDFs (build the robot image first)
scripts/x11_auth.sh                  # X auth (.x11/xauth) for the headed sim and RViz windows; once per login
docker compose -f basestation.compose.yml up -d --build
python3 tools/sim_ui/server.py       # web UI on 127.0.0.1:8090 (/config: settings; --mode real: real robots over SSH)
```

No test suite: checks run against the live sim (`/scripts/drive_test.py`, `scripts/calibrate_velocity.py`, `arm_joints`).

## Rules

- IMPORTANT: `colcon_ws/src` is deployed unchanged to the real robots (`scripts/deploy_robot.sh`) and must stay identical across robot models. Code there must work on a real robot too: no sim-only paths, container names or assumptions without a fallback.
- `mtu32_husky` and `mocap_fake_localizer` (submodules) track a `sim` branch: commit and push inside the submodule first, then commit the pointer here.
- Hand tuning of sim parameters goes in `sim/config/model_params.yaml`, never in `setup_scene.py` (the rest is derived from robot.yaml + URDF at sim start).
- Settings live in the `fleet_config` database; `.env` is generated (untracked): never edit it, change settings with `scripts/fleetcfg.py set` or the `/config` page. A new setting goes into `scripts/fleet_settings.py` (default = compose's; `scripts/fleetcfg.py check`) and, for the sim, the `isaac-sim` service's `environment:` block in `docker-compose.yml`. Tables change only through a new `basestation/config_migrations/NNN_*.sql`.
- Middleware is the setting `FLEET_RMW`, not `RMW_IMPLEMENTATION`.
- Start or change robots with `scripts/fleet.sh`, never `docker compose up -d` (it renders `.env`, with the per-slot `ROBOT_SUFFIX_<i>`/`ROBOT_HOSTNAME_<i>`, from the settings first). Stop with `scripts/fleet.sh down`. A setting reaches a container only when it is recreated (`docker restart` keeps the old environment).
- Time: use the node clock in ROS code, never `time.time()`, `steady_clock` or wall timers for anything that waits on the robot; new nodes need `use_sim_time:=true` (`USE_SIM_TIME`).
- `map→odom` has exactly one publisher (`mocap_fake_localizer`'s `ref_localizer.py`). Anything else that publishes it (AMCL, slam_toolbox, a static identity) needs `ref_source:=external`.
- URDF link/joint names must match the USD's.
- `sim/assets/` binaries are Git LFS. Testing settings code from a worktree: `FLEET_CONFIG_DB=<scratch db>` keeps it off the real `fleet_config` (`fleetcfg.py` creates it on first use; drop it afterwards).

## Running ROS commands

- Plain `docker exec <c> python3 ...` has no ROS environment: use `docker exec <c> bash -c '...'` (or `-it <c> bash`; `-u robot` for `~/colcon_ws` work).
- `ros2 run` ignores `ROS_NAMESPACE`: pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`. Topics are relative to the robot's namespace.
- Container name = namespace: generic models `<model>_<slot %04d>` (`a300_0000`), real robots their id with no suffix (`j100_0921`).

## What to rerun after a change

- `robot/entrypoint.sh`, `robot/bin/*`, `robot/config/*.tmpl`: baked into the image → `docker compose build robot0`, then `scripts/fleet.sh N` (a `.tmpl` also needs `scripts/gen_urdf.sh` and a sim restart).
- A colcon package → `scripts/colcon_build.sh --packages-select <pkg>`, then restart its launch.
- `sim/scripts/*`, or URDFs regenerated with `scripts/gen_urdf.sh` → `docker restart a300-isaac-sim` (`fleet.sh` doesn't restart a running sim).
- Any URDF or drive change → verify with `docker exec <robot> bash -c 'python3 /scripts/drive_test.py'`; for a new robot also check the `[fleet] params <model>:` line in `docker compose logs isaac-sim`.

## Gotchas

- After a run goes chaotic (joints far past their limits), every later measurement is meaningless: reset first (`scripts/fleet_ctl.py reset`) or restart the sim.
- Sim start fails ~1 in 10 (Kit hangs ~35 s with no `[fleet]` line, or exits 139 right after "simulation running"): `docker restart a300-isaac-sim` again. Save `docker logs` right after a crash; compose recreating the sim loses them. Check the kernel log (`journalctl -k`, GPU Xid errors) before debugging start failures.
- All 4 generic models in one sim crashed PhysX (CUDA error); any ≤3 distinct models work.
- Connecting the WebRTC client stalls the sim for a few seconds, enough to fail a MoveIt move: don't connect mid-run.
- ufw (default drop) blocks container→host traffic: sim robots reach the base station's DDS/PostgreSQL only after `sudo ufw allow in on br-fleet`.
- `docker exec <c> restart_ros` kills every ROS node; only `robot_state`/`ekf`/`foxglove`/`pruner_stub` come back on their own.
