# Clearpath fleet in Isaac Sim

A configurable number (0–8, three if `.env` doesn't say otherwise) of Clearpath robots, each with a RealSense D435i facing forward, simulated in NVIDIA Isaac Sim 6.0 and driven over ROS 2 Jazzy. Each robot slot independently runs one of four real Clearpath models — A300, A200, Jackal (`j100`) or Ridgeback (`r100`) — so the fleet can be a single model or a mix; see *Number of robots*.

- **Isaac Sim** runs in one container and is streamed to you over WebRTC (no local GUI).
- **Each robot** has its own ROS 2 container, standing in for the robot's onboard computer. It talks to the sim over a private Docker network, as a real robot would over a LAN. The middleware is `rmw_zenoh_cpp` (as on the real robots, through a `zenoh-router` container) or Fast DDS; see *Middleware*.
- Inside a robot container you can view the camera, drive with the keyboard, open RViz and run a Foxglove bridge.

```
                 ┌──────────────────── docker network "ros" (rmw_zenoh_cpp via zenoh-router, or FastDDS) ────┐
 WebRTC client ──┤ isaac-sim  (N robots, each A300/A200/Jackal/Ridgeback + D435i, ROS 2 bridge)                │
 49100/tcp       │    ▲ cmd_vel        │ odom, joint_states, tf, camera images                                │
 47998/udp       │    │                ▼                                                                      │
                 │ a300_0000   j100_0001   ...        ← robot_state_publisher, teleop, RViz, foxglove_bridge  │
                 └────────────────────────────────────────────────────────────────────────────────────────────┘
                                8765         8766       (N = NUM_ROBOTS, Foxglove WebSocket on 8765 + slot index)
```

## Requirements

- Linux with an NVIDIA GPU, a recent driver and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) (tested on an RTX 4080 SUPER)
- Docker with the Compose plugin
- Access to the Isaac Sim image `nvcr.io/nvidia/isaac-sim:6.0.0` (`docker login nvcr.io` with an NGC API key)
- An X11 desktop (for `camera_view` and RViz windows) and `xauth`
- The Isaac Sim WebRTC Streaming Client (NVIDIA's desktop app, see the Isaac Sim livestream docs) to see the simulation

## Quick start

```bash
# 0. Get the code, including the ROS packages in colcon_ws/src (several are git submodules)
git clone --recurse-submodules https://github.com/robotian/multirobot_sim.git
cd multirobot_sim          # existing clone instead: git submodule update --init --recursive

# 1. Log in to NVIDIA's registry once (NGC API key); the Isaac Sim base image is pulled during the build
docker login nvcr.io

# 2. Build both images: clearpath-robot:jazzy (every robot + the zenoh router) and a300-isaac-sim:6.0.0
docker compose build

# 3. Generate the robot descriptions (URDF + meshes -> sim/assets/); needs the robot image from step 2
scripts/gen_urdf.sh

# 4. Allow the containers to open windows on your display (once per login)
scripts/x11_auth.sh

# 5. Choose the robots in .env (NUM_ROBOTS, ROBOT_MODEL_<i>; see "Number of robots"), then start everything
scripts/fleet.sh            # or: scripts/fleet.sh 2  to also set NUM_ROBOTS
docker compose logs -f isaac-sim     # wait for "[fleet] simulation running with N robots"

# 6. Build the shared ROS workspace colcon_ws/src in the running robot containers (first time, and after editing packages)
scripts/colcon_build.sh
```

Notes:

- **Use `scripts/fleet.sh`, not a bare `docker compose up -d`, to start and to change the robot count or models:** it keeps the per-slot container names and hostnames in `.env` in sync and removes robot containers you no longer want. A bare `docker compose up -d` is fine for restarting an unchanged setup.
- `robot_data/` (the real MTU robots' own `robot.yaml` files) and `sim/colcon_ws/` are not in git. Without them `gen_urdf.sh` still generates the four generic models (`a300`, `a200`, `j100`, `r100`); real-robot ids such as `j100_0921` only work once their `robot_data/<id>/robot.yaml` is present (see *Adding a real robot configuration file*).
- The first start is slow: Isaac Sim compiles shaders and imports the URDF to USD. Later starts reuse the caches. `sim/assets/` and `sim/generated/` are not in git; `scripts/gen_urdf.sh` and the first start create them, so run the script after every fresh clone.
- Then open the WebRTC Streaming Client and connect to `ISAACSIM_HOST` (from `.env`; use `127.0.0.1` when it runs on the same machine).
- Optional: `python3 tools/sim_ui/server.py` serves a local web UI on <http://127.0.0.1:8090> to start/stop/reset the sim, spawn robots, run `sim_robot_upstart`, move the arm and start/stop `cut_stem`.

Stop everything with `scripts/stop_sim.sh` (plain `docker compose down` misses robot services outside the active `NUM_ROBOTS` profile).

### Rebuilding after a change

Which command you need depends on what you edited. Nothing else is baked into an image.

| You changed | Do this |
|---|---|
| `robot/Dockerfile`, `robot/entrypoint.sh`, `robot/bin/*` (`generate_srdf`, `robot_state`, `teleop`, ...), `robot/config/*.tmpl` | `docker compose build robot0`, then recreate the robots: `scripts/fleet.sh` (or `docker compose up -d --force-recreate <robot service>`). These files are copied into the image at build time. One build serves every slot: `robot0`..`robot7` share the tag `clearpath-robot:jazzy`. |
| `docker/isaac-sim.Dockerfile`, `docker/isaac-entrypoint.sh` | `docker compose build isaac-sim`, then `scripts/fleet.sh` to recreate the sim |
| `sim/scripts/setup_scene.py` (the scene, drive and sensor graphs) | no build: `./sim` is mounted. `docker restart a300-isaac-sim` |
| a robot template (`robot.<model>.yaml.tmpl`), `scripts/flatten_urdf.py`, or a `robot_data/<id>/robot.yaml` | rebuild the robot image if it is a `.tmpl`, then `scripts/gen_urdf.sh`, then `docker restart a300-isaac-sim` (the sim re-imports the URDF when it changed; `FORCE_REIMPORT=1` forces it) |
| ROS packages in `colcon_ws/src` | no image build: `scripts/colcon_build.sh [--packages-select <pkg>]`, then restart whatever node uses it (a running node keeps its old binary; for the arm stack relaunch `sim_robot_upstart`) |
| `scripts/*.sh`, `scripts/drive_test.py` | nothing: `./scripts` is mounted into the robots (`gen_urdf.sh` and `fleet.sh` run on the host) |
| `.env` | `scripts/fleet.sh` (a change to `NUM_ROBOTS` or a slot's model needs it; sim tuning variables only need the sim restarted) |

Rebuild from scratch (new base image, or to pick up the latest apt packages and the `clearpath_robot` source that the Dockerfile clones at build time):

```bash
docker compose build --no-cache
scripts/gen_urdf.sh
scripts/fleet.sh
```

A running container keeps the image it was created from, so after `docker compose build` the robots only change once they are recreated. If a build fails or a robot seems to run old code, check `docker images clearpath-robot:jazzy` and that the container was recreated (`docker inspect <robot> --format '{{.Image}}'`). Dangling old layers can be cleaned with `docker image prune`.

## Number of robots

`NUM_ROBOTS` in `.env` (0–8, default 3) sets how many robots are simulated: the sim spawns robots for slots `0 … N-1` and compose starts the matching robot containers. To change it use the helper, which also restarts the sim (it has to spawn a different number of robots) and removes robot containers that are no longer wanted:

```bash
scripts/fleet.sh 5        # set NUM_ROBOTS=5 in .env and (re)start the sim and 5 robots
scripts/fleet.sh          # (re)start with the current NUM_ROBOTS
scripts/fleet.sh down     # stop and remove everything (same as scripts/stop_sim.sh)
```

Each of the 8 possible slots gets a fixed Foxglove port (`8765 + slot index`), whatever the total. A slot's container name, hostname and ROS namespace are its **real Clearpath model name** plus its slot index — see below — not a generic label.

You can also edit `NUM_ROBOTS` by hand and run `docker compose up -d`, which works for increasing the count. After lowering it, `docker compose up` leaves the surplus robot containers running (compose does not stop services of inactive profiles), so use `scripts/fleet.sh`.

More robots cost frame rate, roughly linearly (RTX 4080 SUPER, FastDDS, async rendering): about 40 fps with 1 robot, 23 with 2, 11 with 5. Set `SIM_RATE_HZ` to about the frame rate you get (`FLEET_DEBUG=1` prints it), or the sim does not run in real time; see *Faster streaming*. How a count above 8 could be added is described under *Changing the robots*.

### Robot models

Each slot's model comes from `ROBOT_MODEL_<i>` in `.env` (`i` = 0–7, matching the slot), one of:

| Code | Robot | Drivetrain in this sim |
|---|---|---|
| `a300` (default) | Clearpath A300 | skid-steer, native |
| `a200` | Clearpath A200 | skid-steer, native |
| `j100` | Clearpath Jackal | skid-steer, native |
| `r100` | Clearpath Ridgeback | omnidirectional — see below |
| `j100_0921` / `j100_0936` / `j100_0922` | MTU's own real Jackals, from their actual `robot_data/<serial>/robot.yaml` | skid-steer, native |
| `a200_0333` | MTU's own real A200, from `robot_data/a200_0333/robot.yaml` | skid-steer, native |
| `a300_00036` | MTU's own real A300, from `robot_data/a300_00036/robot.yaml` | skid-steer, native |

Leaving `ROBOT_MODEL_<i>` unset defaults that slot to `a300` (matches every earlier version of this project). To mix models:

```bash
# .env
NUM_ROBOTS=3
ROBOT_MODEL_1=j100   # slot 1 becomes a Jackal, container/hostname/namespace j100_0001
ROBOT_MODEL_2=r100   # slot 2 becomes a Ridgeback, r100_0002
# slot 0 stays a300 (unset)
```

then `scripts/fleet.sh` (or `docker compose up -d --force-recreate`) to apply it. `scripts/gen_urdf.sh` generates every model's URDF unconditionally (the four generic ones plus one per `robot_data/<id>/` folder that has a `robot.yaml`), so nothing needs regenerating when you change `ROBOT_MODEL_<i>`.

Two things worth knowing:
- **A slot's numeric suffix is its slot index, not a per-model count.** Two Jackals in slots 1 and 4 show up as `j100_0001` and `j100_0004`, not `j100_0000`/`j100_0001`.
- **Ridgeback is Clearpath's holonomic mecanum-wheel platform and drives omnidirectionally here** — it can strafe sideways and combine translation with rotation, unlike the other three models. Forward/back and rotation use the same real `diff_4wd.yaml` OmniGraph as the others; sideways motion is patched in separately (a script node sets the chassis's lateral velocity directly each tick), because Ridgeback's own URDF gives every wheel a plain cylinder collision shape — the angled-roller detail is mesh-only — which can't physically produce sideways thrust no matter how it's driven kinematically. See `MODEL_PARAMS`/`BODY_DRIVE_SCRIPT` in `sim/scripts/setup_scene.py` for the full reasoning. One known limitation: a small *pure* in-place rotation command from a standstill (e.g. 0.5 rad/s alone) is mostly absorbed by static friction between the wheels and ground and barely turns the robot; a larger command, or any rotation combined with translation, comes through close to correctly.
- **Running all four distinct models at once (4 robots, no repeats) crashed the sim with a `PhysX Internal CUDA error`** on the machine this was built on. Every individual model, and every combination of up to 3 distinct models tried, worked fine; only the specific 4-distinct-model combination reproduced it. Not root-caused — if you hit it, try fewer distinct models simultaneously.
- Robots also keep a fixed spacing regardless of model size (fine for A300/A200/Jackal; Ridgeback is larger and might feel tight next to another robot).
- **A `ROBOT_MODEL_<i>` for a slot `NUM_ROBOTS` doesn't reach is silently ignored** — that slot just never starts, so e.g. `NUM_ROBOTS=2` with `ROBOT_MODEL_2` set gives you slots 0/1 (defaulting to a300 if unset) and no slot 2 at all, not the model you configured. `scripts/fleet.sh` now warns about this (`ROBOT_MODEL_<i> ... is not running`) instead of leaving it to be found by getting the wrong robot.
- **Jackal's fenders looked attached at spawn but drifted away once it drove or turned.** They're purely decorative (no collision, no mass) in Clearpath's own mesh, and Isaac's importer still makes them a separate physics body with a fixed-joint constraint to the chassis — one too light relative to the rest of the robot to stay perfectly rigid under motion. Fixed by folding them directly into the chassis at URDF-generation time (`merge_visual_only_links` in `scripts/flatten_urdf.py`) instead of relying on that constraint; see *Changing the robots* if you add a model with similar decorative parts.
- **`j100_0921`/`j100_0936` are MTU's own physical robots**, spawned from their real `robot_data/<serial>/robot.yaml` files, not a generic Clearpath sample (`j100_0921`'s is used completely unmodified, including its `platform.extras` — MTU's own `mtu32_description` package is colcon-built and included; `j100_0936`'s own `robot_data` folder isn't currently available, so it still goes through a stripped-down template with `platform.extras` dropped), and — unlike every other model, which is `<model>_%04d` per slot — both their ROS namespace *and* their docker container name are their own id directly (`j100_0921`, not `j100_0921_0000`): they're one specific real robot each, not a generic model needing a slot index to stay unique. Run `scripts/fleet.sh`, not `docker compose up -d` directly, after changing a slot's model for this to take effect (it keeps `ROBOT_SUFFIX_<i>` in `.env` in sync with `ROBOT_MODEL_<i>`, working around `docker-compose.yml`'s own inability to compute this conditionally). See CLAUDE.md's *Real robots* section for exactly what's kept/dropped/fixed versus the real config. Their full real sensor/arm loadout is simulated: a Stereolabs ZED2i camera, a Microstrain IMU, dual SwiftNav Duro GPS (a flat-earth projection around Michigan Tech's Houghton campus — not real satellite geometry), and a Kinova Gen3 Lite arm + 2F Lite gripper (drive a pose with `ros2 topic pub .../arm_0/joint_command sensor_msgs/msg/JointState "{name: [...], position: [...]}"`). `j100_0936` additionally carries a real SICK LMS1xx 2D lidar — 2D lidar itself is now simulated (see `a200_0333` below and *2D lidar* below), but `j100_0936` isn't wired up to it yet since its own `robot_data` folder isn't currently available.
- **`a200_0333`/`a300_00036` are two more MTU real robots**, same "real `robot_data/<serial>/robot.yaml` used directly" pipeline as `j100_0921` — neither references any private package, so `generate_description` succeeded first try with no workarounds needed. `a200_0333` carries a real D435 camera, a Hokuyo UST 2D lidar, and a Velodyne VLP16 3D lidar — all three simulated; see *2D lidar* and *3D lidar* below. `a300_00036` carries only a real Phidgets Spatial IMU (simulated) and no camera at all.

### 2D lidar

`isaacsim.sensors.experimental.physics.Raycast`/`RaycastSensor` — a real, per-physics-step PhysX raycast sensor, the same module the IMU already uses, entirely independent of the broken RTX Lidar pipeline — casts a ray fan matching the real sensor's FOV and publishes `sensor_msgs/LaserScan` on `sensors/lidar2d_0/scan`. Live on `a200_0333`'s Hokuyo UST; `j100_0936`'s SICK LMS1xx would use the same mechanism once its own `robot_data` is available.

A real bug in this "experimental" API was found and worked around while wiring this up: its own `get_data()['depths']` field doesn't report genuine per-ray distances (isolated live: a 3-ray down/forward/up probe returned `min_range` for all three regardless of what each ray actually hit). `hit_positions` from the same reading is correct per ray, so the range published is computed as the Euclidean distance to each ray's own `hit_positions` entry instead. See `LIDAR2D_READ_SCRIPT` in `sim/scripts/setup_scene.py` for the full explanation.

### 3D lidar

Same `RaycastSensor` mechanism as 2D lidar, extended to `a200_0333`'s real Velodyne VLP16 (16 channels × 360 horizontal steps, real ±15° vertical FOV) — publishes `sensor_msgs/PointCloud2` on `sensors/lidar3d_0/points`, reusing the same `hit_positions`-based fix for the same `depths` bug.

**The first attempt segfaulted the whole simulation container.** Root cause: `a200_0333`'s sensor-arch mount (the VLP16's real ancestor chain) is a USD-instanceable subtree, and authoring a new prim under it is rejected outright by USD — the sensor's own retry-on-not-ready logic then kept re-attempting that same, permanently-failing call every tick, which is what actually crashed the process, not the ray count. Fixed by parenting the raycast sensor under the chassis link instead (never instanced) with an explicit computed offset, rather than nesting it directly under the real sensor link. Retested at the full original ray count afterward with no issue — ray count itself was never the problem. See `LIDAR3D_READ_SCRIPT` and the `Lidar3dRead` wiring in `sim/scripts/setup_scene.py` for the full explanation.
- **`j100_0922` is `j100_0921`'s twin minus the arm** — same camera/IMU/GPS loadout, but its `robot.yaml` has no `manipulators:` section at all. **It visibly wheelies (pitches up on its rear wheels) under even a gentle drive command** — reproducible from a fresh, level spawn every time, not random settling noise. Likely cause: genuinely lighter than the arm-equipped Jackals, so the same wheel-drive torque that's gentle on every other real robot here produces a much larger pitching moment on this one. Not fixed yet — documented as a known characteristic; see *Known limitations*.

## Adding a real robot configuration file

A real Clearpath robot's own `robot.yaml` can be simulated directly, unmodified — no template, no placeholder substitution — as long as any private/custom packages it depends on are made available. `j100_0921`, `a200_0333` and `a300_00036` already work this way; use one of them as a worked example.

1. **Drop the robot's own config into `robot_data/<id>/robot.yaml`**, where `<id>` is the model code you'll use everywhere else (e.g. `robot_data/j100_0955/robot.yaml`, giving the code `j100_0955`). This directory is gitignored — it holds real, potentially private robot data, not project source.
   - Check `serial_number:` uses a **hyphen** (`j100-0955`), not an underscore — `clearpath_config`'s schema requires the hyphenated form, and it's an easy typo to copy in from elsewhere. This would also break the real robot booting with the same file, so it's worth fixing at the source.
   - If `platform.extras.urdf` points at a private package (check the file — Jackals in this fleet do, A200/A300 usually don't), that package needs to be colcon-built and on the include path in **two** places: `sim/colcon_ws/src/` (for host-side URDF generation — `scripts/gen_urdf.sh` colcon-builds this automatically on every run) and `colcon_ws/src/` (the runtime workspace shared by every robot container — build it with `scripts/colcon_build.sh` once the fleet is up, or `docker exec -u robot <container> colcon build`).

2. **Generate its URDF** (`scripts/gen_urdf.sh` picks up every `robot_data/<id>/robot.yaml` automatically -- no list to edit; a generic catalog model would instead be added to the script's default `MODELS`):
   ```bash
   scripts/gen_urdf.sh
   ```
   This runs the same generator (`clearpath_generator_common generate_description` + `xacro`) a real robot's own boot process uses, against the file exactly as given. Watch for errors — a robot with a sensor/mount combination not seen before can turn up a broken upstream mesh export or a dangling link reference (a custom xacro assuming hardware, like an arm, that this particular robot doesn't have); `scripts/flatten_urdf.py` already patches a few known cases of each, but a genuinely new one will need the same treatment (see its own docstrings for examples).

3. **Wire it into the sim**: add an entry to `MODEL_PARAMS` in `sim/scripts/setup_scene.py`, copying an existing real robot's entry as a starting point (`MODEL_ASSETS` needs nothing: it lists every `sim/assets/<id>/<id>.urdf` that `gen_urdf.sh` produced). The robot's own `domain_id` is ignored: the robot container is put on the fleet's `ROS_DOMAIN_ID` (see `robot/entrypoint.sh`).
   - Drivetrain (`wheel_radius`, `wheel_separation`, `separation_multiplier`, `max_linear`, `max_angular`): use the matching generic model's own values, unless the robot's `robot.yaml` has a `platform.extras.ros_parameters.platform_velocity_controller` override (real calibrated numbers, common on the Jackals) — use those instead.
   - `chassis_link` (the link the drive/odometry graph targets) **must be confirmed against the actual imported USD, not guessed** — it has to be a prim the importer gave `UsdPhysics.ArticulationRootAPI`, and it isn't always the obviously chassis-looking link (A200's own case is the standing example). Guessing wrong fails silently at import time; it only shows up the first time you try to drive it, as `OmniGraph Error: Articulation controller failed`.
   - Sensors, as present: `camera_optical_link` (leave unset if the camera produces a plain `camera_0_link`, like every generic model — `add_camera`'s default path handles that; set it only if the camera's own xacro already emits a correctly-oriented optical frame under a different name, like the ZED2i's), `imu_link`, `gps_links`, `has_arm`, `has_camera=False` (only if there's no camera at all), `lidar2d_link`/`lidar3d_link` (both simulated via a real per-physics-step raycast sensor, not RTX; see *2D lidar*/*3D lidar*). **Check every sensor's link name against the flattened URDF, not the raw `robot.yaml`** — a purely-visual sensor mount can get folded into a different, higher-up link during flattening (`merge_visual_only_links`); an existing real robot's own comment in `MODEL_PARAMS` shows a worked example.

4. **Start it** like any other model:
   ```bash
   # .env
   ROBOT_MODEL_0=j100_0955
   ```
   ```bash
   scripts/fleet.sh 1     # not `docker compose up -d` directly -- see below
   docker compose logs -f isaac-sim     # wait for "[fleet] simulation running with N robots"
   ```
   Use `scripts/fleet.sh`, not a plain `docker compose up -d`: a real robot's container name and hostname are `<model>-<serial>`-style values computed in bash from `ROBOT_MODEL_<i>`'s content, since `docker-compose.yml`'s own interpolation can't inspect a variable to decide this — only `fleet.sh` keeps them in sync when you change a slot's model.

5. **Verify it the same way every model here is verified — live, not just "it imported cleanly"**: a gentle drive test first (`docker exec j100_0955 bash -c 'python3 /scripts/drive_test.py 0.15 0 1.5'` — small commands especially for anything lighter than a fully-loaded robot, which can wheelie under too aggressive a command), then check each sensor's actual topic delivers real data, not just that it's listed (`ros2 topic echo`, not only `ros2 topic list`).

## Using the robots

Run these on the host. `a300_0000` can be replaced by any other robot, whatever model it is (`j100_0001`, `a200_0002`, `r100_0003`, …).

| Task | Command |
|---|---|
| Drive with the keyboard (`i` forward, `,` back, `j`/`l` turn, `k` stop, `q`/`z` speed) | `docker exec -it a300_0000 teleop` |
| Camera view (colour, or `depth`) | `docker exec a300_0000 camera_view [depth]` |
| RViz (RobotModel, TF, camera) | `docker exec -it a300_0000 rviz` |
| Shell in the robot | `docker exec -it a300_0000 bash` |
| Shell as the `robot` user (for the ROS workspace) | `docker exec -it -u robot a300_0000 bash` |
| Drive test (commanded vs. measured motion) | `docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py 0.5 0 4'` |

Plain `docker exec <container> <ros command>` has no ROS environment. Use one of the commands above, `bash -c '…'`, or open a shell first.

### ROS workspace

Every robot container has a `robot` user (uid/gid 1000, matching the typical host user, so files are writable from both sides without extra chown steps) whose home directory has a `colcon_ws/src`. That directory is bind-mounted from `./colcon_ws` on the host into **every** robot container at `/home/robot/colcon_ws` — the same host folder, not a copy per robot — so a change made from inside one robot's container (or straight on the host) is immediately visible in all of them.

```bash
docker exec -it -u robot a300_0000 bash    # ROS + colcon_ws are already sourced, see below
cd ~/colcon_ws
# put/clone packages under src/, then:
colcon build
source install/setup.bash    # only needed to pick up a *new* package without opening a fresh shell
```

Or build without opening a shell, in every running robot container at once:

```bash
scripts/colcon_build.sh                    # colcon build
scripts/colcon_build.sh --symlink-install  # extra arguments are passed straight to colcon build
```

Because `colcon_ws` is the same host folder in every container, the first container's build already produces the `build/`/`install/` that all the others see; the script still runs `colcon build` in each running robot container in turn (cheap and incremental after the first) so none of them can be left stale, and reports which ones (if any) failed.

The container's main process still runs as root (the entrypoint needs it to write `/etc/clearpath/robot.yaml` and to run `robot_state`/`foxglove`); `robot` is only for workspace work via `docker exec -u robot`. `colcon_ws/build/`, `install/` and `log/` are gitignored; put packages under `colcon_ws/src/`.

`colcon_ws` is sourced the same way it would be on a real Clearpath robot: `robot.yaml`'s `system.ros2.workspaces` names it, and the entrypoint runs Clearpath's own `generate_bash` generator against that `robot.yaml` to produce `/etc/clearpath/setup.bash`, which sources ROS, then `colcon_ws/install/setup.bash`, then applies `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`. Every shell in the container — the main process and any `docker exec` — sources that same generated file, so a package built once is available everywhere without hand-rolled sourcing logic. On a workspace nobody has built yet, the entrypoint creates a one-line placeholder `install/setup.bash` so this never fails on a fresh clone; `colcon build` overwrites it with the real thing.

### Foxglove

Every robot runs a `foxglove_bridge` that exposes only its own namespace. In Foxglove choose *Open connection → Foxglove WebSocket*:

| Robot | URL |
|---|---|
| slot 0 (e.g. `a300_0000`) | `ws://<host>:8765` |
| slot 1 (e.g. `j100_0001`) | `ws://<host>:8766` |
| slot 2 (e.g. `a200_0002`) | `ws://<host>:8767` |
| slot *i* | `ws://<host>:<8765 + i>` |

The transforms are on `/<robot>/tf` and `/<robot>/tf_static`, not on `/tf`. If the 3D panel shows no frames, enable those topics in its settings.

**Robot model in the 3D panel.** The 3D panel loads a URDF on its own only from `/robot_description`; ours is `/<robot>/robot_description`, so the panel needs a URDF layer pointing at it. Either generate a ready-made layout:

```bash
scripts/foxglove_layout.sh            # foxglove/<robot>.json for every running robot (or pass namespaces)
```

and in Foxglove use *Layouts → Import from file* with `foxglove/<robot>.json` (a 3D panel following `base_link`, a grid on `odom`, and the robot's URDF), or add it by hand: 3D panel settings → *Custom layers* → **+** → *URDF*, *Source* = *Topic*, *Topic* = `/<robot>/robot_description`, and turn on *Scene → Ignore COLLADA <up_axis>* (like RViz; `mtu32_description`'s top plate is `Y_UP` and would otherwise be rotated). The `package://` meshes are served by the robot's bridge. The Jackals' `top_assy_rev1.dae` is 41 MB, so the model takes a few seconds to appear, longer over Wi-Fi.

### ROS interface

All topics live under the robot's namespace (`a300_0000`, `j100_0001`, …, whatever model each slot runs).

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/TwistStamped` | input (as on the real Clearpath Jazzy platform; plain `Twist` is ignored); skid-steer, limits depend on the model (`MODEL_PARAMS` in `sim/scripts/setup_scene.py`) |
| `platform/odom` | `nav_msgs/Odometry` | |
| `platform/joint_states` | `sensor_msgs/JointState` | the four wheel joints |
| `tf`, `tf_static` | `tf2_msgs/TFMessage` | `odom → base_link` from the sim, the rest from `robot_state_publisher` |
| `robot_description` | `std_msgs/String` | latched (transient local) |
| `sensors/camera_0/color/image`, `…/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `rgb8`, frame `camera_0_color_optical_frame` |
| `sensors/camera_0/depth/image`, `…/depth/camera_info` | `sensor_msgs/Image`, `CameraInfo` | |

`ros2 run` ignores `ROS_NAMESPACE`, so custom tools must set the namespace with `--ros-args -r __ns:=/$ROBOT_NAMESPACE`, as the `teleop` and `camera_view` wrappers do.

## Middleware

`FLEET_RMW` in `.env` selects the ROS 2 middleware for every container:

| Value | Setup |
|---|---|
| `rmw_zenoh_cpp` (default if `.env` doesn't say otherwise; this repo's checked-in `.env` currently has `rmw_fastrtps_cpp`) | Every session, including Isaac Sim's, runs in zenoh *client* mode and connects to the `zenoh-router` service (`tcp/zenoh-router:7447`). Set `ZENOH_ROUTER=tcp/<host>:7447` to use another router, e.g. a real robot's. |
| `rmw_fastrtps_cpp` | FastDDS over UDP only (`docker/fastdds_udp.xml`); the router container just idles. |

### Switching the middleware

Change it in `.env`, not in `docker-compose.yml`:

```
FLEET_RMW=rmw_fastrtps_cpp
```

Then recreate the containers (restarts the sim and the robots, about a minute):

```bash
docker compose up -d --force-recreate
```

For a one-off run without editing `.env`, prefix the command: `FLEET_RMW=rmw_fastrtps_cpp docker compose up -d --force-recreate`. A value set in the shell overrides `.env`, so use the same prefix on every `docker compose` command or the next plain `docker compose up` goes back to the `.env` value.

Check that it took effect with `docker exec a300_0000 bash -c 'echo $RMW_IMPLEMENTATION'`.

What follows from the switch:
- Nothing else needs changing: Isaac Sim only loads its zenoh libraries when `FLEET_RMW=rmw_zenoh_cpp`, and each robot's `/etc/clearpath/robot.yaml` picks up the new value at container start. `scripts/gen_urdf.sh` does not need to be re-run, since the URDF does not depend on the middleware.
- `zenoh-router` keeps running but idles under FastDDS, and `ZENOH_ROUTER` has no effect.
- FastDDS is about 3–4 fps cheaper in the sim, so `SIM_RATE_HZ` can be raised accordingly in `.env`.

### Notes

- The variable is deliberately not called `RMW_IMPLEMENTATION`: a host shell that exports it (for example from `~/.bashrc`) would silently override `.env`.
- The bundled ROS 2 libraries of Isaac Sim have no zenoh, so the sim runs from `docker/isaac-sim.Dockerfile`, which adds a system ROS 2 Jazzy. Its entrypoint only sources it when `FLEET_RMW=rmw_zenoh_cpp`.
- `peer` mode, the rmw_zenoh_cpp default, does not work between containers: sessions listen on loopback only, so peers in different containers never see each other. That is why the sessions are clients.
- Zenoh costs about 3–4 frames per second in the sim compared with FastDDS, so lower `SIM_RATE_HZ` (about 15 for 3 robots on zenoh).
- `ros2` CLI tools in a container may print `Unable to connect to any locator of scouted peer` warnings from zenoh; they are harmless. `ros2 topic list --no-daemon --spin-time 4` gives the most reliable listing.

## Configuration

Edit `.env` and restart with `docker compose up -d`.

"Default" below is the fallback in `docker-compose.yml`, used only if a variable is missing from `.env` entirely. This repo's checked-in `.env` sets most of them explicitly, currently: `NUM_ROBOTS=2`, `FLEET_RMW=rmw_fastrtps_cpp`, `SIM_RATE_HZ=22`, `FLEET_SETTINGS` = async rendering (see below) — a state tuned in earlier testing on this machine, not a recommendation for yours.

| Variable | Default | Meaning |
|---|---|---|
| `FLEET_RMW` | `rmw_zenoh_cpp` | ROS 2 middleware, see *Middleware* |
| `ZENOH_ROUTER` | `tcp/zenoh-router:7447` | router the zenoh sessions connect to |
| `ISAACSIM_HOST` | `127.0.0.1` | address the WebRTC client uses to reach the sim (the machine's LAN IP for remote clients; the `.env` in this repo holds this machine's LAN IP, change it for yours) |
| `NUM_ROBOTS` | `3` | number of robots (0–8), see *Number of robots*; `.env` also derives `COMPOSE_PROFILES=n${NUM_ROBOTS}` from it, which selects the robot containers |
| `ROS_DOMAIN_ID` | `0` | written into each robot's `robot.yaml` (`system.ros2.domain_id`) and from there into the generated `/etc/clearpath/setup.bash` |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `360` | D435i image size |
| `CAMERA_FRAME_SKIP` | `0` | publish every (N+1)th sim frame |
| `CAMERA_STREAMS` | `color,depth` | streams to publish; `none` turns the cameras off completely |
| `SIM_RATE_HZ` | `20` | frames per second of simulated time; keep it close to the frame rate the sim reaches (see *Faster streaming*) |
| `PHYSICS_HZ` | `60` | physics steps per second of simulated time |
| `FLEET_DEBUG` | `0` | `1` logs real-time factor, render fps and robot pose every few seconds |
| `FORCE_REIMPORT` | `0` | `1` re-imports the URDF into USD |
| `FLEET_SETTINGS` | (none) | extra Kit settings, `"/path/a=1;/path/b=text"`; this repo's `.env` sets `/app/asyncRendering=true` and `/app/asyncRenderingLowLatency=true`, see *Faster streaming* |
| `FLEET_VIEWPORT_RES` | (client window size) | e.g. `1280x720`: render the streamed viewport at a fixed size |
| `ROBOT_LOOKS` | `full` | robot materials, see *Robot materials*: `full` (textured, dusty, worn), `basic` (realistic materials without textures), `0` (the importer's flat colours) |
| `FLEET_SNAPSHOT` | (none) | e.g. `/sim/generated/snapshots`: once the sim runs, save viewport PNGs of every robot from three angles (`<ns>_<view>_<ROBOT_LOOKS>.png`) |

Rendering is the bottleneck: each robot with cameras costs a fixed 15–20 ms per frame. The sim frame rate (`render_fps` in the `FLEET_DEBUG=1` output) is also the frame rate of the WebRTC stream. If the real-time factor falls under 1.0, lower `SIM_RATE_HZ` or the number of robots.

### Faster streaming

The WebRTC client shows at most the frame rate of the sim, because the sim renders the streamed viewport once per frame. Measured on an RTX 4080 SUPER with 2 robots and no client connected (the GPU stayed at 20–45% busy, the limit is CPU-side synchronisation of the camera render products):

| Change | Frame rate |
|---|---|
| baseline | 19 fps |
| `/app/asyncRendering=true` (+ low latency), now the default in `.env` | **23 fps** (camera images lag one frame) |
| cameras off (`CAMERA_STREAMS=none`) | about 31–36 fps |
| fewer robots | 1 robot about 40 fps, 5 robots 11 fps |
| lower camera resolution, colour only, no depth, `RaytracedLighting`, hiding the Kit UI, camera `CAMERA_FRAME_SKIP`, async replicator | no change |
| viewport at 3440×1440 instead of 1280×720 | 18 vs 19 fps, GPU load 46% vs 30% |

So the levers that matter are the number of robots with cameras and async rendering. If you only need to watch the scene, `CAMERA_STREAMS=none` roughly doubles the frame rate (the robots then publish no images). `FLEET_VIEWPORT_RES=1280x720` keeps the GPU load down on a large client window at almost no cost in frame rate. Not measured: the encoder and network path with a client connected.

## Scene

Besides the robots, the sim spawns some static scene dressing in `build_world()` (`sim/scripts/setup_scene.py`):

- a **lavender farm** look, modelled on real photos of the field:
  - **Ground:** the `/World/ground` box (80 x 80 m, physics-material friction for traction) has a dark-brown soil material (`GROUND_SOIL_COLOR`), so soil rather than a bright box shows through the grass. `add_ground_cover()` references one unscaled patch of `sim/assets/Ground_cover/ground_cover.usd` on top of it: a 100 x 100 m grass field of ~73k PointInstancer blades, 9–11 cm tall (the layer says `metersPerUnit=0.01` but its geometry is really in metres; tiling many patches exceeds the renderer's instance limit and nothing draws). Visual only, no collider.
  - **Lavender rows:** `add_lavender()` plants (`SM_Lavender_Nanite_01.usd` under `sim/assets/lavender/`, instanceable, scaled by 0.006) form one overlapping hedge row on each side of the robots' driving lanes (`LAVENDER_ROW_*`, `LAVENDER_PLANT_PITCH`; 10 plants per row along +X). No collider. Each plant is ~1.26M triangles, so row length is what costs fps.
  - **Horizon:** `add_horizon_vegetation()` scatters NVIDIA Omniverse library assets (`Assets/Vegetation/Trees|Shrub|Rocks` from the public `omniverse-content-production` S3 bucket, downloaded into `sim/assets/trees|shrubs|rocks/` together with their `materials/` or `textures/` folders, which the assets need or the foliage renders red) on an arc 19–29 m ahead: 36 trees (Douglas fir, black oak), 70 shrubs, 30 boulders (`TREE_*`/`SHRUB_*`/`ROCK_*`). The cameras clip at 30 m, so everything has to sit inside that. Each asset is sized from its own bbox to a random target height, and referenced under its own child prim because the asset roots carry xform ops. Some shrub assets are unusable (`Cedar_Shrub` has an empty bbox).
  - **Sky and light:** the dome light uses `sim/assets/sky/farm_field_puresky_2k.hdr` (a cloud panorama, also the camera background) at `SKY_INTENSITY = 400`; the distant sun is intensity 10000, rotation (-60, 33, -30). The old target boxes, wall and pillars, and the lavender fill light, were removed.

### Robot materials

The URDF importer gives every robot part one flat colour with the same plastic-like shine, so the robots looked smooth and factory-clean. `sim/scripts/robot_looks.py` replaces those materials at sim start, modelled on photos of the real robots (`robot_data/pictures/`, untracked like the rest of `robot_data/`):

- **Looks:** each visual part is matched by name, material name and colour (rules `RULES`, per-robot overrides `MODEL_RULES`) to one of a dozen NVIDIA OmniPBR materials: glossy Clearpath-yellow paint (clear coat, orange peel), black powder coat with scuffs, semi-gloss black bumpers, knobby rubber tyres with mud, brushed aluminium (the A300's arch posts, the Jackal's top assembly), anodised sensor housings, glossy white Kinova links, matte white GNSS domes, black plastic, red e-stops, glass. Unmatched coloured parts keep their colour with a plastic surface; emissive status lights are left alone.
- **Detail:** colour, roughness and normal textures are generated procedurally (numpy, seamless) on the first start into `sim/generated/looks/` (~25 s, cached afterwards; delete the folder or bump `VERSION` to regenerate). Parts have no UVs, so OmniPBR projects the textures in object space; a texture tile is 0.5 m on every part. Parts low on the robot (< 0.18 m, plus tyres and bumpers) get the heavier dust variant. Edges get OmniPBR's shading-only rounded edges (3 mm).
- **Visual only:** only the visual material bindings change, in memory (the import cache in `sim/generated/<model>/` stays as imported). Colliders, physics materials and masses are untouched, so driving and the lidars behave the same.
- **Cost:** measured with 3 camera-equipped robots (RTX 4080 SUPER, 22 Hz, async rendering): render fps 8.8-9.3 with the flat materials, 9.2-9.7 with `full`, 9.6-9.8 with `basic`, i.e. no measurable difference; `full` adds ~25 s to the first start only. If it costs too much on another GPU, set `ROBOT_LOOKS=basic` (no textures) or `ROBOT_LOOKS=0` (original materials) in `.env` and restart the sim (`docker compose up -d isaac-sim`).

| Before (`ROBOT_LOOKS=0`) | After (`ROBOT_LOOKS=full`) |
|---|---|
| ![robots with the importer's flat materials](docs/images/robot_looks_off.jpg) | ![robots with realistic, dusty materials](docs/images/robot_looks_full.jpg) |

To tune a look, edit its entry in `LOOKS` (colour, roughness, dust, scratches) and restart the sim; `FLEET_SNAPSHOT=/sim/generated/snapshots` saves comparison images.

### Lavender material

The plant is a Nanite mesh exported from Unreal, shipped with its own real MDL shaders and albedo/normal textures (`sim/assets/lavender/Materials/`) — but as exported, none of its 4 materials (`MI_Stem_01`, `MI_Leaf_high_01`, `MI_Lavender_Flower_Branch_01`, `MI_Leaf_01`) were actually wired to a shader implementation: each material's `outputs:surface` had no connection at all, so Isaac silently fell back to the mesh's own baked `displayColor` primvar — a flat grayscale AO/lightmap channel, not real color, which is why it rendered as a dark, nearly colorless silhouette. Three fixes, applied directly to `sim/assets/lavender/SM_Lavender_Nanite_01.usd` (`.usd.orig` alongside it is the untouched backup from before any of this):

1. **Real shaders.** Each material's existing shader prim (which already carried the correct `TintColor`/`ColorFresnel`/... values from the Unreal export) now points at its real `.mdl` module in `Materials/`, instead of nothing.
2. **`doubleSided`.** All 5 mesh sections had `doubleSided=False`; for blade-thin foliage geometry like this, that silently culls/darkens roughly half of all viewing angles. Set to `True`.
3. **Subsurface translucency.** The shaders are built on NVIDIA's `OmniUe4Subsurface` module, which has a real `diffuse_transmission_bsdf` lobe for light passing through thin geometry — but every material hard-coded `subsurface_color = 0` (black) and `opacity = 1.0` in its `.mdl` source, which disabled transmission entirely *and* wasted half its shading budget on a black reflection lobe. `MI_Stem_01.mdl` and `MI_Lavender_Flower_Branch_01.mdl` now expose this as two real, tunable parameters — **Subsurface Color** and **Subsurface Opacity** — under a new "07 - Subsurface" parameter group. `MI_Leaf_01`/`MI_Leaf_high_01` still have it hard-coded off; the same fix would apply there too if it matters.

| Close-up | In the scene |
|---|---|
| ![lavender close-up, stems and flower spikes visible](docs/images/lavender_closeup.png) | ![lavender row alongside a target box](docs/images/lavender_in_scene.png) |

*(Both grabbed from `j100_0921`'s own camera feed over ROS — `sensor_msgs/Image` on `sensors/camera_0/color/image`, rotated 180° for display — not the Isaac Sim client's own viewport, which renders noticeably cleaner: its RTX-Real-Time mode converges better than this path-traced ROS camera stream, especially on thin geometry like the stems. The rotation isn't a sim quirk: `j100_0921`'s (and `a200_0333`'s) real camera is physically mounted rolled 180° — see `robot_data/j100_0921/robot.yaml`'s `zed_mount` link (`rpy: [3.14159, 0, 0]`) — so the raw topic is genuinely upside-down on both the real robot and here, matching hardware faithfully.)*

**Tuning it live, in the Isaac Sim client, no file editing required:**

1. **File → Open Stage**, and open `/sim/assets/lavender/SM_Lavender_Nanite_01.usd` directly — standalone, not through the fleet scene. The three plants in the fleet are `instanceable=True` copies of this file, which makes their materials read-only when selected there.
2. In the **Stage** panel, expand `Root → Looks` and click the material to edit (`MI_Stem_01`, `MI_Leaf_high_01`, `MI_Lavender_Flower_Branch_01`, or `MI_Leaf_01`).
3. In the **Property** panel, its inputs show up grouped and labelled — straight from the `.mdl` file's own `anno::display_name`/`anno::in_group` annotations, e.g. "01 - Albedo" → **Tint Color**, "07 - Subsurface" → **Subsurface Color**/**Subsurface Opacity**. Edit them and the viewport updates live, no restart needed.
4. **Ctrl+S** to save back to the file, then restart the sim (`docker restart a300-isaac-sim`, or `docker compose up -d`) to see it in the full fleet scene.

To edit live *inside* the running fleet scene instead of the standalone file: select a plant (`/World/lavender/plant_0` in the Stage tree), uncheck **Instanceable** in the Property panel for that session, then drill into `Looks` the same way. That only edits the fleet's own in-memory session layer, though — to make it stick, set your edit target (Window → Layers) to the `SM_Lavender_Nanite_01.usd` sublayer first, or just use the standalone-file route above, which is simpler for anything you want to keep.

Current tuned values, as a reference starting point:

| Material | Parameter | Value |
|---|---|---|
| `MI_Lavender_Flower_Branch_01` | Brightness dots | `9.0` (was `3.0`) |
| `MI_Lavender_Flower_Branch_01` | Tint Color | `(0.50, 0.35, 0.75)` (was `(0.29, 0.20, 0.46)`) |
| `MI_Lavender_Flower_Branch_01` | Subsurface Color | `(0.55, 0.35, 0.72)` |
| `MI_Lavender_Flower_Branch_01` | Subsurface Opacity | `0.6` |
| `MI_Stem_01` | Subsurface Color | `(0.55, 0.80, 0.35)` |
| `MI_Stem_01` | Subsurface Opacity | `0.6` |

## Known limitations

- **Frame rate:** the sim renders the robots' cameras and the scene (including the lavender plants) through a path tracer, so the real-time factor is the limit. Zenoh adds about 3–4 fps of cost over FastDDS; the three lavender plants cost a similar amount.
- **One router:** all zenoh sessions share a single `zenoh-router`. A real fleet would have a router per robot; that topology is not simulated.
- **Raw images:** colour and depth are published uncompressed (about 30 MB/s per robot at 20 Hz), which is fine on the local machine but heavy for Wi-Fi Foxglove clients.
- **Not tested against real robots:** interoperability with the real robots' zenoh router (`ZENOH_ROUTER`) has not been tried.
- **Ridgeback's and the real MTU robots' in-place rotation is weak from a standstill** for small commands (static friction absorbs most of it; a larger command or one combined with translation works fine) — see *Robot models*. Running all four distinct catalog models at once can also crash the sim — see *Robot models*.
- **Robot spacing** is a fixed constant regardless of model size — see *Robot models*.
- **`j100_0936`'s 2D lidar isn't wired up yet** (its own `robot_data` folder isn't currently available) — 2D lidar itself is simulated (see *2D lidar*), this is just a missing wiring step, not a blocker.
- **GPS on the real MTU robots is a flat-earth projection, not real satellite geometry** — same simplification Gazebo's own GPS plugins make; see *Robot models*.
- **`j100_0922` wheelies under even a gentle drive command** — see *Robot models*. Not caused by a config error; a real (if likely exaggerated) mass/inertia effect of it having no arm.

## Changing the robots

- **Generic model configuration:** each generic model has its own template, `robot/config/robot.<model>.yaml.tmpl` (`a300`/`a200`/`j100`/`r100`, plus `j100_0936`, whose own `robot_data` isn't available). Edit the one you want to change, rebuild the robot image (`docker compose build robot0`), run `scripts/gen_urdf.sh`, then `docker restart a300-isaac-sim`; the sim re-imports that model's USD when its URDF changes. Each template's `system.ros2.middleware.implementation` follows `FLEET_RMW`, so `/etc/clearpath/robot.yaml` in each robot names the same middleware the container runs. See *Rebuilding after a change*.
- **Real robots** (`j100_0921`, `j100_0922`, `a200_0284`, `a300_00036`, ...): no template. Put the robot's own file in `robot_data/<id>/robot.yaml`; `scripts/gen_urdf.sh` and the robot container pick it up by folder name, with nothing to register in either script. The sim still needs a `MODEL_PARAMS` entry for it in `sim/scripts/setup_scene.py` (drivetrain, `chassis_link`, sensor links, arm settings). Step by step: *Adding a real robot configuration file*. Per-robot behaviour of the arm cutter (zone, IK seed, gripper range, drop pose, planning speed) goes in `colcon_ws/src/stow_arm_cpp/config/robots/<id>.yaml`, which overrides the shared `grid_cutter_params.yaml` (the stow node reads the same files).
- **Adding another generic model:** it needs (a) a new `robot.<code>.yaml.tmpl` in `robot/config/`, following the existing four as examples, and a matching `COPY` line in `robot/Dockerfile`, (b) an entry in `MODEL_PARAMS` in `sim/scripts/setup_scene.py` (`MODEL_ASSETS` is derived from `sim/assets/`) — including its correct `chassis_link` name, the one URDF link the drive/odometry graph targets; check this against the imported USD (`UsdPhysics.ArticulationRootAPI`), it isn't always literally called `chassis_link`, see A200's case there — and re-check it again after any URDF-shape change, even one that looks unrelated (adding the Jackal fender fix changed *which* link the importer chose as the root), (c) adding it to the `MODELS` list in `scripts/gen_urdf.sh`. If the new model has decorative parts with no collision/mass (check each `<link>` in its generated URDF), `scripts/flatten_urdf.py`'s `merge_visual_only_links` already folds those into their parent automatically — no per-model work needed unless the part also needs a mesh-orientation fix like the Jackal fender's.
- **MoveIt collision matrix (robots with an arm):** `robot/bin/generate_srdf` writes `/etc/clearpath/robot.srdf` at every container start. It uses `moveit_collision_updater` with `--trials 10000` and retries, because Clearpath's own default (100000 trials) crashes in this container, and too few trials wrongly mark arm-vs-body link pairs as never colliding, so MoveIt plans through the robot. Details in `CLAUDE.md`. Editing that script needs a robot image rebuild.
- **More than 8 robots:** `docker-compose.yml` defines eight robot services (`robot0` … `robot7`, container-named from `ROBOT_MODEL_<i>` + the slot index), each active for the profiles `n<k>` with `k` above its index. Copy the last block for `robot8`, give it the next Foxglove port and a profile list extended by `n9`, raise `MAX` in `scripts/fleet.sh`, and use `NUM_ROBOTS=9`. The sim needs no change.

## Layout

| Path | Purpose |
|---|---|
| `docker-compose.yml`, `.env` | the whole stack (`.env` holds this machine's LAN IP in `ISAACSIM_HOST`) |
| `robot/` | robot container image: `Dockerfile`, `entrypoint.sh`, per-model config templates `config/robot.a300/a200/j100/r100/j100_0936.yaml.tmpl` and the generic `config/robot.rviz.tmpl`. Real robots in `robot_data/` use their own `robot.yaml` directly, no template. Everything here is baked into the image: rebuild after editing (*Rebuilding after a change*) |
| `robot/bin/` | commands installed in every robot: `teleop`, `camera_view`, `rviz`, `foxglove`, `restart_ros` and the boot-time/background helpers `robot_state`, `generate_params`, `generate_srdf` (MoveIt collision matrix), `pruner_stub` (fake pruner serial device); arm tools `arm_goto` and `arm_joints` |
| `robot_data/<id>/robot.yaml` | the real MTU robots' own Clearpath configs (`j100_0921`, `j100_0922`, `a200_0284`, `a300_00036`, ...), used unmodified. Not in git (private lab material); gitignored |
| `sim/scripts/setup_scene.py` | builds the Isaac Sim scene and ROS 2 graphs (`./sim` is mounted into the sim container: edit, then `docker restart a300-isaac-sim`) |
| `sim/assets/<model>/`, `sim/generated/<model>/` | generated URDF and meshes, cached USD, one set per model. Not in git; `gen_urdf.sh` and the first start create them |
| `sim/colcon_ws/` | private ROS packages that `gen_urdf.sh` builds for URDF generation on the host (e.g. `mtu32_description`). Not in git |
| `scripts/` | `fleet.sh` (choose robots, restart), `stop_sim.sh`, `colcon_build.sh`, `gen_urdf.sh` + `flatten_urdf.py` (URDF generation), `x11_auth.sh`, `drive_test.py`. Mounted read-only into the robots, so edits need no rebuild |
| `colcon_ws/src/` | ROS workspace shared by every robot container (see *ROS workspace*): 7 git submodules plus plain packages. Arm cutting stack: `stow_arm_cpp` (`cut_stem` action server `grid_cutter_action_server`, stow node, per-robot config in `config/robots/`), `moveit_sim_bridge` (executes MoveIt trajectories and gripper commands in the sim), `pruner_action_server`, `plant_cutter_msgs`, `serial_interfaces`; launched by `mtu32_husky/mtu32_bringup`'s `sim_robot_upstart.launch.py` |
| `tools/sim_ui/` | `server.py` + `index.html`: local web UI (port 8090) that runs the same scripts as this README: start/stop/reset the sim, spawn robots, `sim_robot_upstart`, arm moves with commanded-vs-observed plots, Cut stem |
| `sim/assets/Ground_cover/`, `sky/`, `trees/`, `shrubs/`, `rocks/` | Grass field USD, cloud HDR, and Omniverse-library vegetation used by `build_world()` (gitignored, see *Scene*) |
| `sim/assets/lavender/` | `SM_Lavender_Nanite_01.usd` and its real `Materials/` (MDL shaders + textures, see *Lavender material*), referenced as the lavender hedge rows |
| `docker/fastdds_udp.xml` | FastDDS profile (UDP only, since containers don't share `/dev/shm`) |
| `docker/isaac-sim.Dockerfile`, `docker/isaac-entrypoint.sh` | Isaac Sim image with a system ROS 2 Jazzy (needed for zenoh) |
| `docs/images/` | images used by this README |
| `CLAUDE.md`, `last_session.md` | detailed architecture notes and the latest session log (read these before changing the sim, arm or SRDF pipeline) |

See `CLAUDE.md` for more detail on how the pieces fit together.

## Troubleshooting

- **RViz / camera window doesn't open:** run `scripts/x11_auth.sh` and check `DISPLAY`.
- **`executable file not found` from `docker exec`:** the command needs the ROS environment; see *Using the robots*.
- **RobotModel is empty in RViz:** Description Topic must be `/<robot>/robot_description` with Durability *Transient Local*. `docker exec -it <robot> rviz` sets this up.
- **RViz window opens and renders but the 3D view doesn't respond to the mouse (no orbit/pan/zoom):** the RViz config needs a `Tools:` section listing `rviz_default_plugins/Interact`/`MoveCamera` — without one, RViz loads no interaction tools at all (not "use the defaults"), unlike a bare `rviz2` session with no `-d` config file. Already fixed in `robot/config/robot.rviz.tmpl`; if this recurs after editing that template, check `Tools:` is still present.
- **Camera is invisible in the sim:** the importer drops hand-exported Collada meshes, so `scripts/flatten_urdf.py` converts them to OBJ. Re-run `scripts/gen_urdf.sh` and restart the sim.
- **A robot container is still running after lowering `NUM_ROBOTS`:** use `scripts/fleet.sh <N>` (or `docker compose rm -sf robotN`).
- **`PhysX Internal CUDA error` and the sim container dies:** seen when running 4 distinct models at once (see *Robot models*); try fewer distinct models running simultaneously.
- **Sim runs slower than real time:** check `FLEET_DEBUG=1`; lower `SIM_RATE_HZ` or the number of robots.
- **The WebRTC client is choppy:** the stream cannot be faster than the sim frame rate; see *Faster streaming*.
- **Wrong middleware in a container:** a host shell exporting `RMW_IMPLEMENTATION` does not affect the stack (use `FLEET_RMW` in `.env`); check with `docker exec a300_0000 bash -c 'echo $RMW_IMPLEMENTATION'`.
- **One-off `No such file or directory: .../colcon_ws/install/setup.bash`** on a `docker exec`: harmless — `colcon_ws/install/` was deleted after the container last (re)started `/etc/clearpath/setup.bash`; rebuilding (`scripts/colcon_build.sh`) or recreating the container (`docker compose up -d --force-recreate`) fixes it.
- **Robots don't see each other's topics:** every container needs the same `ROS_DOMAIN_ID` and the same middleware. With zenoh the `zenoh-router` must be healthy; with FastDDS the shared profile is required.
