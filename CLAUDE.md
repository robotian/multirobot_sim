# CLAUDE.md

A fleet of Clearpath robots simulated in Isaac Sim 6.0 and driven over ROS 2 Jazzy: one Isaac Sim container, plus one ROS 2 container per robot standing in for its onboard computer. Docker Compose plus scripts; no build system, test suite or linter. Architecture, setup and the full command reference are in `README.md`; each area has its own `CLAUDE.md` (`sim/`, `robot/`, `colcon_ws/`, `scripts/`, `tools/sim_ui/`). Adding or debugging a real MTU robot → skill `add-real-robot`.

## Rules

- IMPORTANT: `colcon_ws/src` is deployed unchanged to the real robots (`scripts/deploy_robot.sh`) and must stay identical across robot models. Code there must work on a real robot too: no sim-only paths, container names or assumptions without a fallback.
- `mtu32_husky` and `mocap_fake_localizer` (submodules) track a `sim` branch: commit and push inside the submodule first, then commit the pointer here.
- Hand tuning of sim parameters goes in `sim/config/model_params.yaml`, never in `setup_scene.py` (the rest is derived from robot.yaml + URDF at sim start).
- A new `.env` variable for the sim must also be added to the `isaac-sim` service's `environment:` block in `docker-compose.yml`.
- Middleware is `FLEET_RMW` in `.env`, not `RMW_IMPLEMENTATION`.
- Start or change robots with `scripts/fleet.sh`, never `docker compose up -d` (it writes the per-slot `ROBOT_SUFFIX_<i>`/`ROBOT_HOSTNAME_<i>` into `.env`). Stop with `scripts/fleet.sh down`.
- Time: use the node clock in ROS code, never `time.time()`, `steady_clock` or wall timers for anything that waits on the robot; new nodes need `use_sim_time:=true` (`USE_SIM_TIME`).
- `map→odom` has exactly one publisher (`mocap_fake_localizer`'s `ref_localizer.py`). Anything else that publishes it (AMCL, slam_toolbox, a static identity) needs `ref_source:=external`.
- URDF link/joint names must match the USD's.

## Running ROS commands

- Plain `docker exec <c> python3 ...` has no ROS environment: use `docker exec <c> bash -c '...'` (or `-it <c> bash`; `-u robot` for `~/colcon_ws` work).
- `ros2 run` ignores `ROS_NAMESPACE`: pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`. Topics are relative to the robot's namespace.
- Container name = namespace: generic models `<model>_<slot %04d>` (`a300_0000`), real robots their id with no suffix (`j100_0921`).

## What to rerun after a change

- `robot/entrypoint.sh`, `robot/bin/*`: baked into the image → `docker compose build robot0`, then `scripts/fleet.sh N`.
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
