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
# 1. Build the images (robot + Isaac Sim with ROS 2 Jazzy) and generate the robot description (URDF + meshes)
docker compose build
scripts/gen_urdf.sh

# 2. Allow the containers to open windows on your display (once per login)
scripts/x11_auth.sh

# 3. Start the router, the simulation and the robots (NUM_ROBOTS in .env, default 3; see "Number of robots")
docker compose up -d
docker compose logs -f isaac-sim     # wait for "[fleet] simulation running with N robots"
```

The first start is slow: Isaac Sim compiles shaders and imports the URDF to USD. Later starts reuse the caches. `sim/assets/` and `sim/generated/` are not in git; `scripts/gen_urdf.sh` and the first start create them, so run the script after every fresh clone.

Then open the WebRTC Streaming Client and connect to `ISAACSIM_HOST` (from `.env`; use `127.0.0.1` when it runs on the same machine).

Stop everything with `scripts/stop_sim.sh` (plain `docker compose down` misses robot services outside the active `NUM_ROBOTS` profile).

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
| `j100_0921` / `j100_0936` | MTU's own real Jackals, from their actual `robot_data/<serial>/robot.yaml` | skid-steer, native |

Leaving `ROBOT_MODEL_<i>` unset defaults that slot to `a300` (matches every earlier version of this project). To mix models:

```bash
# .env
NUM_ROBOTS=3
ROBOT_MODEL_1=j100   # slot 1 becomes a Jackal, container/hostname/namespace j100_0001
ROBOT_MODEL_2=r100   # slot 2 becomes a Ridgeback, r100_0002
# slot 0 stays a300 (unset)
```

then `scripts/fleet.sh` (or `docker compose up -d --force-recreate`) to apply it. `scripts/gen_urdf.sh` generates all four models' URDFs unconditionally, so nothing needs regenerating when you change `ROBOT_MODEL_<i>`.

Two things worth knowing:
- **A slot's numeric suffix is its slot index, not a per-model count.** Two Jackals in slots 1 and 4 show up as `j100_0001` and `j100_0004`, not `j100_0000`/`j100_0001`.
- **Ridgeback is Clearpath's holonomic mecanum-wheel platform and drives omnidirectionally here** — it can strafe sideways and combine translation with rotation, unlike the other three models. Forward/back and rotation use the same real `diff_4wd.yaml` OmniGraph as the others; sideways motion is patched in separately (a script node sets the chassis's lateral velocity directly each tick), because Ridgeback's own URDF gives every wheel a plain cylinder collision shape — the angled-roller detail is mesh-only — which can't physically produce sideways thrust no matter how it's driven kinematically. See `MODEL_PARAMS`/`BODY_DRIVE_SCRIPT` in `sim/scripts/setup_scene.py` for the full reasoning. One known limitation: a small *pure* in-place rotation command from a standstill (e.g. 0.5 rad/s alone) is mostly absorbed by static friction between the wheels and ground and barely turns the robot; a larger command, or any rotation combined with translation, comes through close to correctly.
- **Running all four distinct models at once (4 robots, no repeats) crashed the sim with a `PhysX Internal CUDA error`** on the machine this was built on. Every individual model, and every combination of up to 3 distinct models tried, worked fine; only the specific 4-distinct-model combination reproduced it. Not root-caused — if you hit it, try fewer distinct models simultaneously.
- Robots also keep a fixed spacing regardless of model size (fine for A300/A200/Jackal; Ridgeback is larger and might feel tight next to another robot).
- **A `ROBOT_MODEL_<i>` for a slot `NUM_ROBOTS` doesn't reach is silently ignored** — that slot just never starts, so e.g. `NUM_ROBOTS=2` with `ROBOT_MODEL_2` set gives you slots 0/1 (defaulting to a300 if unset) and no slot 2 at all, not the model you configured. `scripts/fleet.sh` now warns about this (`ROBOT_MODEL_<i> ... is not running`) instead of leaving it to be found by getting the wrong robot.
- **Jackal's fenders looked attached at spawn but drifted away once it drove or turned.** They're purely decorative (no collision, no mass) in Clearpath's own mesh, and Isaac's importer still makes them a separate physics body with a fixed-joint constraint to the chassis — one too light relative to the rest of the robot to stay perfectly rigid under motion. Fixed by folding them directly into the chassis at URDF-generation time (`merge_visual_only_links` in `scripts/flatten_urdf.py`) instead of relying on that constraint; see *Changing the robots* if you add a model with similar decorative parts.
- **`j100_0921`/`j100_0936` are MTU's own physical robots**, spawned from their real `robot_data/<serial>/robot.yaml` files, not a generic Clearpath sample, and — unlike every other model, which is `<model>_%04d` per slot — both their ROS namespace *and* their docker container name are their own id directly (`j100_0921`, not `j100_0921_0000`): they're one specific real robot each, not a generic model needing a slot index to stay unique. Run `scripts/fleet.sh`, not `docker compose up -d` directly, after changing a slot's model for this to take effect (it keeps `ROBOT_SUFFIX_<i>` in `.env` in sync with `ROBOT_MODEL_<i>`, working around `docker-compose.yml`'s own inability to compute this conditionally). See CLAUDE.md's *Real robots* section for exactly what's kept/dropped/fixed versus the real config. Their full real sensor/arm loadout is simulated: a Stereolabs ZED2i camera, a Microstrain IMU, dual SwiftNav Duro GPS (a flat-earth projection around Michigan Tech's Houghton campus — not real satellite geometry), and a Kinova Gen3 Lite arm + 2F Lite gripper (drive a pose with `ros2 topic pub .../arm_0/joint_command sensor_msgs/msg/JointState "{name: [...], position: [...]}"`). `j100_0936` additionally carries a real SICK LMS1xx 2D lidar, but it isn't simulated here — the only 2D-lidar pipeline this Isaac Sim version has (RTX Lidar) fails to import during Kit's own native startup in this specific install, an environment defect unrelated to this project's own code.

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

### ROS interface

All topics live under the robot's namespace (`a300_0000`, `j100_0001`, …, whatever model each slot runs).

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | input; skid-steer, limits depend on the model (`MODEL_PARAMS` in `sim/scripts/setup_scene.py`) |
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

- coloured target boxes (one per robot, with a physics collider — robots can drive into them), a wall and pillars, all for the cameras to look at;
- three lavender plants (`SM_Lavender_Nanite_01.usd` under `sim/assets/lavender/`, `add_lavender()`), referenced with `instanceable=True` so the ~1.26M-triangle mesh is shared rather than tripled, and scaled by 0.01 to convert the asset's centimetre units into this stage's metres. They sit in a row off to the side of the robots' lane, with no collider (decoration only). They cost about 3–4 fps at 2 robots on this GPU — see *Faster streaming* if that's a problem, or edit/remove the `add_lavender(...)` calls.
- They render fairly dark under the current lighting; the plant's material (converted from Unreal) needs more light than the boxes/wall to read clearly. Raising `dome`/`sun` light intensity in `build_world()` fixes it but overexposes the rest of the scene, so it hasn't been changed — worth a supplemental local light near the plants if this matters.

## Known limitations

- **Frame rate:** the sim renders the robots' cameras and the scene (including the lavender plants) through a path tracer, so the real-time factor is the limit. Zenoh adds about 3–4 fps of cost over FastDDS; the three lavender plants cost a similar amount.
- **One router:** all zenoh sessions share a single `zenoh-router`. A real fleet would have a router per robot; that topology is not simulated.
- **Raw images:** colour and depth are published uncompressed (about 30 MB/s per robot at 20 Hz), which is fine on the local machine but heavy for Wi-Fi Foxglove clients.
- **Not tested against real robots:** interoperability with the real robots' zenoh router (`ZENOH_ROUTER`) has not been tried.
- **Ridgeback's and the real MTU robots' in-place rotation is weak from a standstill** for small commands (static friction absorbs most of it; a larger command or one combined with translation works fine) — see *Robot models*. Running all four distinct catalog models at once can also crash the sim — see *Robot models*.
- **Robot spacing** is a fixed constant regardless of model size — see *Robot models*.
- **`j100_0936`'s 2D lidar isn't simulated** — an Isaac Sim 6.0 extension defect, not something this project's code can work around; see *Robot models*.
- **GPS on the real MTU robots is a flat-earth projection, not real satellite geometry** — same simplification Gazebo's own GPS plugins make; see *Robot models*.

## Changing the robots

- **Robot configuration:** each model has its own template, `robot/config/robot.<model>.yaml.tmpl` (`a300`/`a200`/`j100`/`r100`); edit the one you want to change, rebuild the image and run `scripts/gen_urdf.sh`. Each template's `system.ros2.middleware.implementation` follows `FLEET_RMW`, so `/etc/clearpath/robot.yaml` in each robot names the same middleware the container runs. The sim re-imports that model's USD when its URDF changes.
- **Adding another model:** it needs (a) a new `robot.<code>.yaml.tmpl`, following the existing four as examples, (b) an entry in `MODEL_PARAMS` (and `MODEL_ASSETS`) in `sim/scripts/setup_scene.py` — including its correct `chassis_link` name, the one URDF link the drive/odometry graph targets; check this against the imported USD (`UsdPhysics.ArticulationRootAPI`), it isn't always literally called `chassis_link`, see A200's case there — and re-check it again after any URDF-shape change, even one that looks unrelated (adding the Jackal fender fix below changed *which* link the importer chose as the root), (c) adding it to the `MODELS` list in `scripts/gen_urdf.sh`. If the new model has decorative parts with no collision/mass (check each `<link>` in its generated URDF), `scripts/flatten_urdf.py`'s `merge_visual_only_links` already folds those into their parent automatically — no per-model work needed unless the part also needs a mesh-orientation fix like the Jackal fender's.
- **More than 8 robots:** `docker-compose.yml` defines eight robot services (`robot0` … `robot7`, container-named from `ROBOT_MODEL_<i>` + the slot index), each active for the profiles `n<k>` with `k` above its index. Copy the last block for `robot8`, give it the next Foxglove port and a profile list extended by `n9`, raise `MAX` in `scripts/fleet.sh`, and use `NUM_ROBOTS=9`. The sim needs no change.

## Layout

| Path | Purpose |
|---|---|
| `docker-compose.yml`, `.env` | the whole stack |
| `robot/` | robot container image: `Dockerfile`, `entrypoint.sh`, helper commands in `bin/` (`teleop`, `camera_view`, `rviz`, `robot_state`, `foxglove`), per-model config templates `robot.a300/a200/j100/r100/j100_0921/j100_0936.yaml.tmpl` and the generic `robot.rviz.tmpl` |
| `robot_data/<serial>/robot.yaml` | the real MTU robots' own actual Clearpath configs (source for the two templates above) |
| `sim/scripts/setup_scene.py` | builds the Isaac Sim scene and ROS 2 graphs |
| `sim/assets/<model>/`, `sim/generated/<model>/` | generated URDF and meshes, cached USD, one set per model |
| `scripts/` | `fleet.sh` (choose the number of robots), `stop_sim.sh`, `colcon_build.sh`, URDF generation, X11 setup and the drive test |
| `colcon_ws/src/` | ROS workspace shared by every robot container, see *ROS workspace* |
| `docker/fastdds_udp.xml` | FastDDS profile (UDP only, since containers don't share `/dev/shm`) |
| `docker/isaac-sim.Dockerfile`, `docker/isaac-entrypoint.sh` | Isaac Sim image with a system ROS 2 Jazzy (needed for zenoh) |
| `sim/assets/lavender/` | `SM_Lavender_Nanite_01.usd` and its Materials, referenced three times as scene decoration |

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
