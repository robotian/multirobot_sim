# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated fleet of Clearpath A300 skid-steer robots (each with a RealSense D435i) in Isaac Sim 6.0, driven over ROS 2 Jazzy. Isaac Sim runs in one container (WebRTC streaming, no local GUI); each robot is its own ROS 2 container standing in for the robot's onboard computer. There is no build system, test suite or linter — everything is Docker Compose plus a few scripts. The directory is not a git repo.

## Commands

```bash
scripts/x11_auth.sh                       # once per login: builds /tmp/.docker.xauth (needed for rqt_image_view in robot containers)
docker compose build                      # a300-robot:jazzy (robots + zenoh router) and a300-isaac-sim:6.0.0 (Isaac + system ROS 2)
scripts/gen_urdf.sh                       # regenerate sim/assets/a300/{a300.urdf,meshes,robot.yaml} from robot/config/robot.yaml.tmpl
docker compose up -d                      # sim + 3 robots; watch with `docker compose logs -f isaac-sim` (lines prefixed [fleet])
docker exec -it a300_0000 bash            # shell in a robot (ROS sourced via /etc/bash.bashrc; ROBOT_NAMESPACE=a300_0000)
docker exec -it a300_0000 teleop          # keyboard control (i/j/l/,/k), publishes /a300_0000/cmd_vel
docker exec -it a300_0001 rviz             # RViz with fixed frame odom, RobotModel, TF, camera (robot/config/a300.rviz.tmpl)
docker exec a300_0000 camera_view [depth] # rqt_image_view of the D435i on the host display (needs x11_auth.sh)
docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py [lin_x] [ang_z] [seconds]'   # smoke test: commanded vs. odometry
```

- `gen_urdf.sh` runs inside the robot image, so build it first. Re-run it after changing `robot.yaml.tmpl` or `flatten_urdf.py`.
- Plain `docker exec <c> python3 ...` has no ROS environment (the entrypoint only covers PID 1); wrap in `bash -c` (the image sets `BASH_ENV`) or use `-it ... bash`.
- `ros2 run` ignores `ROS_NAMESPACE` (only launch files honour it). Namespaced tools must pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`, which is what `robot/bin/teleop` and `camera_view` do.
- Foxglove: each robot runs `foxglove_bridge` (`robot/bin/foxglove`, whitelisted to its own namespace) on host ports 8765/8766/8767 for a300_0000/1/2 — connect with `ws://<host>:<port>`. foxglove_bridge 3.x speaks the `foxglove.sdk.v1` subprotocol (not `foxglove.websocket.v1`), so custom clients must request that.
- Streaming client connects to `ISAACSIM_HOST` (from `.env`) on ports 49100/tcp and 47998/udp.
- Sim tuning knobs are env vars in `.env` (`FLEET_RMW`, `ZENOH_ROUTER`, `SIM_RATE_HZ`, `PHYSICS_HZ`, `CAMERA_*`, `CAMERA_STREAMS`, `FORCE_REIMPORT`, `FLEET_DEBUG=1` for rtf/fps/pose logging, `FLEET_SETTINGS="/carb/key=val;..."` for arbitrary Kit settings). They are passed through explicitly in the `isaac-sim` service `environment:` block, so a new variable must be added there too.

## Architecture

**Robot description pipeline (host-side, one-time):** `robot/config/robot.yaml.tmpl` (Clearpath config with `__NS__`/`__SERIAL__` placeholders) → Clearpath's `generate_description` + `xacro` inside the robot image → `scripts/flatten_urdf.py` copies every `package://`/`file://` mesh into `sim/assets/a300/meshes/`, rewrites paths to relative, drops `<gazebo>`/`<ros2_control>`, patches unnamed Collada materials that crash the importer, and converts non-Blender Collada (the RealSense `d435.dae`) to a vertex-clustered OBJ (`DECIMATE_CELL`; the original has 231k triangles and cost ~30% fps) because the importer silently drops the .dae and leaves an empty Xform (invisible camera). Result: a self-contained URDF so the Isaac container needs no ROS packages. One URDF/USD serves all robots.

**Sim (`sim/scripts/setup_scene.py`, run via `--exec` in the Isaac streaming app; `./sim` is mounted at `/sim`):**
1. URDF → USD import, cached in `sim/generated/a300/` and invalidated by a stamp file (`.import_stamp`: import settings + URDF mtime/size) or `FORCE_REIMPORT=1`. The importer output path is fragile — the script deletes `USD_DIR` first so it doesn't write `a300_1/`, `a300_2/`, ….
2. Builds the world, spawns one robot per name in `ROBOT_NAMESPACES` (spaced along Y), adds a D435i USD camera, and builds one OmniGraph per robot using `isaacsim.ros2.bridge` nodes: `cmd_vel` → differential drive → wheel velocity drives; publishes `platform/odom`, `platform/joint_states`, TF, and camera color/depth + camera_info. Topics are relative and land under each robot's namespace.
3. Time: the stage runs at `SIM_RATE_HZ` (default 15) with PhysX substepping at `PHYSICS_HZ` (60), because camera rendering per robot is too costly for 60 Hz frames. Keep real-time factor ≥ 1.0 (check with `FLEET_DEBUG=1`).
4. Drive constants (wheel radius/separation, `SEPARATION_MULTIPLIER` slip fudge, speed limits) are copied from Clearpath's `diff_4wd.yaml` for the A300 — keep them in sync with the real config.

**Robots (`robot/`):** `entrypoint.sh` renders `/etc/clearpath/robot.yaml` from the template using `ROBOT_NAMESPACE` and `RMW_IMPLEMENTATION` (i.e. `FLEET_RMW`, written to `system.ros2.middleware.implementation`) (serial = namespace with `_` → `-`, since Clearpath hostnames can't contain underscores), sources ROS, and sets `ROS_NAMESPACE`, and starts `robot_state` and `foxglove` in the background (regenerates the URDF from `robot.yaml` and runs `robot_state_publisher` under the namespace, giving the full TF tree plus `/<ns>/robot_description`; the sim itself only publishes `odom→base_link` and `camera_0_link→camera_0_color_optical_frame`, so URDF link/joint names must match the USD's — wheel joints come from `platform/joint_states`). The `teleop`/`camera_view` wrappers in `robot/bin/` are installed to `/usr/local/bin`. Robot services in compose share the `x-robot` anchor; adding a robot means adding a service there **and** its name to `ROBOT_NAMESPACES` in `.env` (they must match).

**Networking / middleware:** sim and robots share the private `ros` bridge network. `FLEET_RMW` in `.env` (deliberately not `RMW_IMPLEMENTATION`, which a ROS-configured host shell would export over `.env`) picks `rmw_zenoh_cpp` (default) or `rmw_fastrtps_cpp` for every container via the `x-rmw-env` anchor.
- zenoh: all sessions run in **client** mode (`ZENOH_CONFIG_OVERRIDE`) against the single `zenoh-router` service. The default `peer` mode fails across containers because peers only listen on loopback. `ZENOH_ROUTER` can point at an external router.
- Isaac Sim's bundled ROS libs only have Fast DDS/Cyclone, so `docker/isaac-sim.Dockerfile` adds system ROS 2 Jazzy + rmw_zenoh_cpp; `docker/isaac-entrypoint.sh` sources it only when `FLEET_RMW=rmw_zenoh_cpp` (Isaac then loads the system ROS instead of its internal one). Zenoh costs ~3–4 fps in the sim, hence `SIM_RATE_HZ=15` (rtf ≈ 0.95).
- FastDDS: UDP only (`docker/fastdds_udp.xml`, mounted into every container); shared-memory transport must stay disabled because containers don't share `/dev/shm`.
All containers must have the same `ROS_DOMAIN_ID`. `volume-init` chowns the named volumes to uid 1234 (Isaac Sim's user) before the sim starts.
