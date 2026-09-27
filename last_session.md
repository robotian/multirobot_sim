# Last session summary

Written 2026-09-26 at the end of the first working session. Read this first, then `CLAUDE.md` (architecture and commands) and `README.md` (user guide).

## The original request

> Simulate multiple Clearpath robots in NVIDIA Isaac Sim, with the latest Isaac Sim and ROS Jazzy. Isaac Sim runs and streams in a Docker container and loads three Clearpath A300s, all with identical configuration. Each robot has a RealSense D435i facing forward. Mimicking a real robot, each robot has its own Docker container; in it, the camera stream from Isaac Sim should be visible and the robot controllable with the keyboard.

Later requests: RViz and Foxglove in the robot containers, `rmw_zenoh_cpp` (the real robots' middleware) with the setting also in the Clearpath `robot.yaml`, a README, and git/GitHub.

## State at the end of the session

- **Everything works and is pushed.** Repo `/home/robotian/isaac_sim_project`, branch `main`, remote `origin` = https://github.com/robotian/multirobot_sim.git. Commits: `88483eb` (initial project), `6a3bb59` (zenoh, D435 fix). This file and the CLAUDE.md/README.md refresh may still be uncommitted; run `git status`.
- **Running stack** (may or may not still be up): `zenoh-router`, `a300-isaac-sim`, `a300_0000`, `a300_0001`, `a300_0002`, on rmw_zenoh_cpp, `SIM_RATE_HZ=15`.
- The URDF was regenerated (only its mtime changed), so the sim re-imports the USD once at its next start.
- Host: Ubuntu, RTX 4080 SUPER, 32 cores, X11 on `DISPLAY=:1`. `~/.bashrc` exports `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (see the pitfalls below). `gh` is not installed; pushing works over HTTPS.

## What was built

- **Isaac Sim 6.0 container** (`a300-isaac-sim`): WebRTC streaming on 49100/tcp and 47998/udp. `sim/scripts/setup_scene.py` (run by `--exec`) imports the URDF to USD, spawns 3 robots 1.6 m apart on Y, adds D435i cameras and builds one OmniGraph per robot: `cmd_vel` → differential drive → wheel drives; publishes odom, joint states, TF, colour/depth images and camera info.
- **Robot containers** (`a300_0000/1/2`, image `a300-robot:jazzy`, ROS 2 Jazzy): the entrypoint renders `/etc/clearpath/robot.yaml`, then runs `robot_state` (URDF + `robot_state_publisher`) and `foxglove` in the background. Helper commands in `robot/bin/`: `teleop`, `camera_view`, `rviz`.
- **Robot description pipeline:** `robot/config/robot.yaml.tmpl` → Clearpath generator + xacro → `scripts/flatten_urdf.py` → `sim/assets/a300/`.
- **Middleware switch:** `FLEET_RMW` in `.env` (zenoh default, or fastrtps) via the `x-rmw-env` anchor in `docker-compose.yml`.
- **Foxglove bridge** per robot, host ports 8765/8766/8767.
- **Docs and repo:** README.md, CLAUDE.md, .gitignore (ignores `sim/assets/`, `sim/generated/`, `__pycache__/`).

## Problems found and how they were solved (the non-obvious ones)

1. **`docker exec` shells had no ROS** and **`ros2 run` ignores `ROS_NAMESPACE`**, so teleop published on `/cmd_vel`. Fix: `/etc/ros_env.sh` via `BASH_ENV` and `/etc/bash.bashrc`, and wrapper scripts that pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`. A bare `docker exec c ros2 …` or `docker exec c rviz2 …` still fails ("executable not found"); commands must go through `bash -c` or the wrappers.
2. **Only `odom`/`base_link` frames in RViz:** the sim publishes just two transforms. Fix: `robot_state_publisher` per robot from the URDF that the Clearpath generator produces inside the robot container (params file, because the URDF text breaks `-p`). URDF link and joint names must match the USD (they do: the wheel joint names come from the sim's `platform/joint_states`).
3. **RobotModel empty in RViz:** description topic must be `/<ns>/robot_description` with Durability *Transient Local*. Solved with `robot/config/a300.rviz.tmpl` and the `rviz` wrapper.
4. **D435 invisible in the sim:** Isaac's URDF importer silently drops the RealSense `d435.dae` (non-Blender Collada), leaving an empty Xform. Fix in `flatten_urdf.py`: convert it to OBJ. The original has 231k triangles and cost about 30% fps for 3 robots, so it is decimated by vertex clustering (`DECIMATE_CELL = 1 mm`, ~17k triangles).
5. **foxglove_bridge 3.x uses the `foxglove.sdk.v1` WebSocket subprotocol**, not `foxglove.websocket.v1` (custom clients get HTTP 400).
6. **Host `RMW_IMPLEMENTATION` beat `.env`** (shell variables override `.env` in compose): the first zenoh run silently used Cyclone. Fix: the project variable is called `FLEET_RMW`.
7. **Isaac Sim has no zenoh RMW** in its bundled ROS libs (Fast DDS and Cyclone only). Fix: `docker/isaac-sim.Dockerfile` (FROM the NVIDIA image + ROS Jazzy apt packages incl. `ros-jazzy-rmw-zenoh-cpp`) and `docker/isaac-entrypoint.sh`, which sources `/opt/ros/jazzy/setup.bash` only when `FLEET_RMW=rmw_zenoh_cpp`; Isaac then logs "Attempting to load system rclpy".
8. **zenoh peer mode does not work across containers** (sessions listen on `tcp/localhost:0`; a publisher in one container was invisible in another although both were connected to the router). Fix: every session is a client: `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/zenoh-router:7447"]'`. The router works with its default config.
9. **Performance:** zenoh costs about 3–4 fps in the sim compared with FastDDS (14.7 vs about 18 fps at 20 Hz), so `SIM_RATE_HZ` defaults to 15, giving a real-time factor of about 0.95. Path tracing (RealTimePathTracing) is the main cost; render mode/AA experiments did not help much.
10. Smaller: the generated URDF must not be regenerated needlessly (its mtime triggers a USD re-import); `.import_stamp` in `sim/generated/a300/` decides re-import; the sim script wipes `sim/generated/a300/` before importing because the importer otherwise writes `a300_1/`, `a300_2/`.

## Verified (live, this session)

- 3 robots spawned; colour/depth/odom topics at the sim frame rate; each robot only moves when its own `cmd_vel` is used.
- Drive test on all robots; teleop via a pseudo-terminal (`pty`) sending `i`, `j`, `k`; camera frame grabbed and inspected (`rgb8`, 640×360); the D435 visible on the top plate when two robots face each other.
- `robot_state_publisher`: 31 static transforms + 4 wheel frames; `tf2_echo odom camera_0_color_optical_frame` resolves.
- RViz with `rviz` wrapper: RobotModel, TF and Camera panel showed up (screenshot via PIL `ImageGrab.grab(xdisplay=':1')`).
- Foxglove: connected to all three bridges with a `websockets` client (subprotocol `foxglove.sdk.v1`), saw the whitelisted topics and live messages.
- zenoh and FastDDS both run from the same compose file (A/B measured); Clearpath's `ClearpathConfig` parses the rendered `robot.yaml`.

Handy headless test tricks: `docker exec … bash -c 'timeout -s INT 10 ros2 topic hz …'`; `ros2 topic list --no-daemon --spin-time 4` (most reliable over zenoh); driving via `python3 /scripts/drive_test.py`.

## Not verified / open items

- **The WebRTC view itself:** ports and container health were checked, but nobody looked at the stream from a client in this session (the user did see the sim and the missing D435, which is now fixed, but did not re-confirm).
- **Foxglove app:** the 3D panel with namespaced `/<ns>/tf` topics and the URDF panel (mesh assets via the bridge) were not tried in the app.
- **Real robots:** interoperability with the real robots' zenoh router (`ZENOH_ROUTER=tcp/<robot>:7447`) was not tried; the real robots probably use a router each (the Clearpath default), the sim uses a single shared router.
- **Ideas mentioned but not done:** compressed image republishing for Foxglove over Wi-Fi; per-robot zenoh routers; the optional `profile:` key in the Clearpath middleware config; a RobotModel that renders in colour (it looks dark in RViz); host firewall for ports 8765–8767 (could not check, `sudo` needs a password); untracking `.env` in favour of `.env.example` (the user chose to keep it tracked).
- `sim/generated/` is only rebuilt by the sim; if the USD looks stale set `FORCE_REIMPORT=1`.

## Working preferences observed

- The user wants changes applied and verified against the live stack, not just written; results should be reported plainly including regressions (e.g. the fps drop).
- Commits and pushes only when asked (they asked explicitly each time); commit messages end with the `Co-Authored-By` line from the harness.
- The Claude Code memory directory (`~/.claude/projects/-home-robotian-isaac-sim-project/memory/`) is unused so far.
