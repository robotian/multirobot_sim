# A300 fleet in Isaac Sim

Three identical Clearpath A300 robots, each with a RealSense D435i facing forward, simulated in NVIDIA Isaac Sim 6.0 and driven over ROS 2 Jazzy.

- **Isaac Sim** runs in one container and is streamed to you over WebRTC (no local GUI).
- **Each robot** has its own ROS 2 container, standing in for the robot's onboard computer. It talks to the sim over a private Docker network with FastDDS, as a real robot would over a LAN.
- Inside a robot container you can view the camera, drive with the keyboard, open RViz and run a Foxglove bridge.

```
                 ┌───────────────────────── docker network "ros" (FastDDS, UDP only) ─────────────────────────┐
 WebRTC client ──┤ isaac-sim  (3 × A300 + D435i, ROS 2 bridge)                                                │
 49100/tcp       │    ▲ cmd_vel        │ odom, joint_states, tf, camera images                                │
 47998/udp       │    │                ▼                                                                      │
                 │ a300_0000   a300_0001   a300_0002   ← robot_state_publisher, teleop, RViz, foxglove_bridge │
                 └────────────────────────────────────────────────────────────────────────────────────────────┘
                                8765         8766         8767   (Foxglove WebSocket, host ports)
```

## Requirements

- Linux with an NVIDIA GPU, a recent driver and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) (tested on an RTX 4080 SUPER)
- Docker with the Compose plugin
- Access to the Isaac Sim image `nvcr.io/nvidia/isaac-sim:6.0.0` (`docker login nvcr.io` with an NGC API key)
- An X11 desktop (for `camera_view` and RViz windows) and `xauth`
- The Isaac Sim WebRTC Streaming Client (NVIDIA's desktop app, see the Isaac Sim livestream docs) to see the simulation

## Quick start

```bash
# 1. Build the robot image and generate the robot description (URDF + meshes) for the sim
docker compose build a300_0000
scripts/gen_urdf.sh

# 2. Allow the containers to open windows on your display (once per login)
scripts/x11_auth.sh

# 3. Start the simulation and the three robots
docker compose up -d
docker compose logs -f isaac-sim     # wait for "[fleet] simulation running with 3 robots"
```

The first start is slow: Isaac Sim compiles shaders and imports the URDF to USD. Later starts reuse the caches.

Then open the WebRTC Streaming Client and connect to `ISAACSIM_HOST` (from `.env`; use `127.0.0.1` when it runs on the same machine).

Stop everything with `docker compose down`.

## Using the robots

Run these on the host. `a300_0000` can be replaced by `a300_0001` or `a300_0002`.

| Task | Command |
|---|---|
| Drive with the keyboard (`i` forward, `,` back, `j`/`l` turn, `k` stop, `q`/`z` speed) | `docker exec -it a300_0000 teleop` |
| Camera view (colour, or `depth`) | `docker exec a300_0000 camera_view [depth]` |
| RViz (RobotModel, TF, camera) | `docker exec -it a300_0000 rviz` |
| Shell in the robot | `docker exec -it a300_0000 bash` |
| Drive test (commanded vs. measured motion) | `docker exec a300_0000 bash -c 'python3 /scripts/drive_test.py 0.5 0 4'` |

Plain `docker exec <container> <ros command>` has no ROS environment. Use one of the commands above, `bash -c '…'`, or open a shell first.

### Foxglove

Every robot runs a `foxglove_bridge` that exposes only its own namespace. In Foxglove choose *Open connection → Foxglove WebSocket*:

| Robot | URL |
|---|---|
| `a300_0000` | `ws://<host>:8765` |
| `a300_0001` | `ws://<host>:8766` |
| `a300_0002` | `ws://<host>:8767` |

The transforms are on `/<robot>/tf` and `/<robot>/tf_static`, not on `/tf`. If the 3D panel shows no frames, enable those topics in its settings.

### ROS interface

All topics live under the robot's namespace (`a300_0000`, `a300_0001`, `a300_0002`).

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | input; skid-steer, limited to 2 m/s and 2 rad/s |
| `platform/odom` | `nav_msgs/Odometry` | |
| `platform/joint_states` | `sensor_msgs/JointState` | the four wheel joints |
| `tf`, `tf_static` | `tf2_msgs/TFMessage` | `odom → base_link` from the sim, the rest from `robot_state_publisher` |
| `robot_description` | `std_msgs/String` | latched (transient local) |
| `sensors/camera_0/color/image`, `…/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `rgb8`, frame `camera_0_color_optical_frame` |
| `sensors/camera_0/depth/image`, `…/depth/camera_info` | `sensor_msgs/Image`, `CameraInfo` | |

`ros2 run` ignores `ROS_NAMESPACE`, so custom tools must set the namespace with `--ros-args -r __ns:=/$ROBOT_NAMESPACE`, as the `teleop` and `camera_view` wrappers do.

## Configuration

Edit `.env` and restart with `docker compose up -d`.

| Variable | Default | Meaning |
|---|---|---|
| `ISAACSIM_HOST` | `127.0.0.1` | address the WebRTC client uses to reach the sim (the machine's LAN IP for remote clients) |
| `ROBOT_NAMESPACES` | `a300_0000,a300_0001,a300_0002` | robots spawned in the sim; must match the robot services in `docker-compose.yml` |
| `ROS_DOMAIN_ID` | `0` | |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `360` | D435i image size |
| `CAMERA_FRAME_SKIP` | `0` | publish every (N+1)th sim frame |
| `CAMERA_STREAMS` | `color,depth` | drop one to save frame time |
| `SIM_RATE_HZ` | `20` | rendered frames per second of simulated time |
| `PHYSICS_HZ` | `60` | physics steps per second of simulated time |
| `FLEET_DEBUG` | `0` | `1` logs real-time factor, render fps and robot pose every few seconds |
| `FORCE_REIMPORT` | `0` | `1` re-imports the URDF into USD |
| `FLEET_SETTINGS` | | extra Kit settings, `"/path/a=1;/path/b=text"` |

Rendering is the bottleneck: each robot with cameras costs about 12 ms per frame. With three robots the sim runs at 20 Hz and about real time (`FLEET_DEBUG=1` shows the real-time factor). Lower `SIM_RATE_HZ`, the camera size or `CAMERA_STREAMS` if it falls under 1.0.

## Changing the robots

- **Robot configuration:** edit `robot/config/robot.yaml.tmpl` (all robots share it), rebuild the image and run `scripts/gen_urdf.sh`. The sim re-imports the USD when the URDF changes.
- **Adding a robot:** add a service to `docker-compose.yml` (copy an existing one, new name and Foxglove port) and add its name to `ROBOT_NAMESPACES` in `.env`.

## Layout

| Path | Purpose |
|---|---|
| `docker-compose.yml`, `.env` | the whole stack |
| `robot/` | robot container image: `Dockerfile`, `entrypoint.sh`, helper commands in `bin/`, config templates |
| `sim/scripts/setup_scene.py` | builds the Isaac Sim scene and ROS 2 graphs |
| `sim/assets/`, `sim/generated/` | generated URDF and meshes, cached USD |
| `scripts/` | URDF generation, X11 setup and the drive test |
| `docker/fastdds_udp.xml` | FastDDS profile (UDP only, since containers don't share `/dev/shm`) |

See `CLAUDE.md` for more detail on how the pieces fit together.

## Troubleshooting

- **RViz / camera window doesn't open:** run `scripts/x11_auth.sh` and check `DISPLAY`.
- **`executable file not found` from `docker exec`:** the command needs the ROS environment; see *Using the robots*.
- **RobotModel is empty in RViz:** Description Topic must be `/<robot>/robot_description` with Durability *Transient Local*. `docker exec -it <robot> rviz` sets this up.
- **Camera is invisible in the sim:** the importer drops hand-exported Collada meshes, so `scripts/flatten_urdf.py` converts them to OBJ. Re-run `scripts/gen_urdf.sh` and restart the sim.
- **Robots don't see each other's topics:** every container needs the same `ROS_DOMAIN_ID` and the shared FastDDS profile.
