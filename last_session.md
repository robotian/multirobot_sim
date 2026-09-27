# Last session summary

Written 2026-09-27, consolidating the sessions so far. Read this first, then `CLAUDE.md` (architecture and commands) and `README.md` (user guide) for anything you need in depth.

## The original request

> Simulate multiple Clearpath robots in NVIDIA Isaac Sim, with the latest Isaac Sim and ROS Jazzy. Isaac Sim runs and streams in a Docker container and loads three Clearpath A300s, all with identical configuration. Each robot has a RealSense D435i facing forward. Mimicking a real robot, each robot has its own Docker container; in it, the camera stream from Isaac Sim should be visible and the robot controllable with the keyboard.

Later requests, in order: RViz and Foxglove in the robot containers; `rmw_zenoh_cpp` (the real robots' middleware), with the setting also written into the Clearpath `robot.yaml`; a README; git + GitHub; a configurable robot count; and investigating a slow WebRTC client.

## Repo and current runtime state

- `/home/robotian/isaac_sim_project`, branch `main`, remote `origin` = https://github.com/robotian/multirobot_sim.git. Pushed commits: `88483eb` (initial project), `6a3bb59` (zenoh + D435 fix), `a4798da` (docs). **Uncommitted at last check:** `scripts/fleet.sh` (new), and edits to `.env`, `CLAUDE.md`, `README.md`, `docker-compose.yml`, `sim/scripts/setup_scene.py` — the `NUM_ROBOTS`/`fleet.sh` feature and the streaming-performance changes. Run `git status` before assuming this is still current.
- **The tracked `.env` currently has:** `NUM_ROBOTS=2`, `FLEET_RMW=rmw_fastrtps_cpp`, `SIM_RATE_HZ=22`, `FLEET_SETTINGS` set to async rendering. These came from live tuning/experiments in this machine's session, not a considered choice of what a fresh clone should default to — `docker-compose.yml`'s own fallbacks (used only if `.env` omits a variable) are `NUM_ROBOTS=3`, `rmw_zenoh_cpp`, `SIM_RATE_HZ=20`. **Open question for the user:** which pair should the checked-in `.env` actually ship, zenoh/3-robots (matches the real robots, matches the docs' "default") or the current fastrtps/2-robots/22Hz (matches what was last measured to work well on this GPU)? Asked once already, unanswered.
- Host: Ubuntu, RTX 4080 SUPER, 32 cores, X11 on `DISPLAY=:1`. `~/.bashrc` exports `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` — this shadows `RMW_IMPLEMENTATION` if anything sets it directly, which is why the project uses `FLEET_RMW` instead (see below). `gh` is not installed; pushing works over HTTPS.
- The stack may or may not be running; last known: `zenoh-router`, `a300-isaac-sim`, `a300_0000`, `a300_0001` (2 robots).

## What was built

- **Isaac Sim 6.0 container** (`a300-isaac-sim`, built from `docker/isaac-sim.Dockerfile`): WebRTC streaming on 49100/tcp and 47998/udp. `sim/scripts/setup_scene.py` (run by `--exec`) imports the URDF to USD, spawns `NUM_ROBOTS` robots spaced along Y, adds D435i cameras and builds one OmniGraph per robot: `cmd_vel` → differential drive → wheel drives; publishes odom, joint states, TF, colour/depth images and camera info.
- **Robot containers** (`a300_0000` … up to `a300_0007`, image `a300-robot:jazzy`, ROS 2 Jazzy): the entrypoint renders `/etc/clearpath/robot.yaml`, then runs `robot_state` (URDF + `robot_state_publisher`) and `foxglove` in the background. Helper commands in `robot/bin/`: `teleop`, `camera_view`, `rviz`.
- **Robot description pipeline:** `robot/config/robot.yaml.tmpl` → Clearpath generator + xacro → `scripts/flatten_urdf.py` → `sim/assets/a300/`.
- **Configurable fleet size:** `NUM_ROBOTS` (0–8) in `.env`, `scripts/fleet.sh [N|down]` to change it live.
- **Middleware switch:** `FLEET_RMW` in `.env` (zenoh or fastrtps) via the `x-rmw-env` anchor in `docker-compose.yml`.
- **Foxglove bridge** per robot, host port `8765 + index`.
- **Docs and repo:** README.md, CLAUDE.md, this file, `.gitignore` (ignores `sim/assets/`, `sim/generated/`, `__pycache__/`).

## Problems found and how they were solved (the non-obvious ones)

1. **`docker exec` shells had no ROS**, and **`ros2 run` ignores `ROS_NAMESPACE`**, so teleop published on `/cmd_vel`. Fix: `/etc/ros_env.sh` via `BASH_ENV` and `/etc/bash.bashrc`, and wrapper scripts that pass `--ros-args -r __ns:=/$ROBOT_NAMESPACE`. A bare `docker exec c ros2 …` or `docker exec c rviz2 …` still fails ("executable not found"); commands must go through `bash -c` or the wrappers.
2. **Only `odom`/`base_link` frames existed:** the sim publishes just two transforms. Fix: `robot_state_publisher` per robot from the URDF the Clearpath generator produces inside the robot container (a params file, since the URDF text breaks `-p`). URDF link/joint names must match the USD's (they do — wheel joint names come from the sim's `platform/joint_states`).
3. **RobotModel empty in RViz:** description topic must be `/<ns>/robot_description` with Durability *Transient Local*. Solved with `robot/config/a300.rviz.tmpl` and the `rviz` wrapper.
4. **D435 invisible in the sim:** Isaac's URDF importer silently drops the RealSense `d435.dae` (non-Blender Collada), leaving an empty Xform. Fix in `flatten_urdf.py`: convert it to OBJ. The original has 231k triangles and cost about 30% fps for 3 robots, so it's decimated by vertex clustering (`DECIMATE_CELL`, currently 1 mm → ~17k triangles). A much coarser cell (1 cm → 606 triangles) made no measurable fps difference, so the mesh detail isn't actually the bottleneck (see item 9).
5. **foxglove_bridge 3.x uses the `foxglove.sdk.v1` WebSocket subprotocol**, not `foxglove.websocket.v1` (custom clients get HTTP 400 otherwise).
6. **Host `RMW_IMPLEMENTATION` beat `.env`** (shell variables override `.env` in compose): the first zenoh run silently used the host's Cyclone setting. Fix: the project variable is called `FLEET_RMW`, deliberately not `RMW_IMPLEMENTATION`.
7. **Isaac Sim has no zenoh RMW** in its bundled ROS libs (Fast DDS and Cyclone only). Fix: `docker/isaac-sim.Dockerfile` (FROM the NVIDIA image + ROS Jazzy apt packages incl. `ros-jazzy-rmw-zenoh-cpp`) and `docker/isaac-entrypoint.sh`, which sources `/opt/ros/jazzy/setup.bash` only when `FLEET_RMW=rmw_zenoh_cpp`.
8. **zenoh peer mode does not work across containers** (sessions listen on `tcp/localhost:0`; a publisher in one container was invisible in another although both were connected to the router). Fix: every session is a client: `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/zenoh-router:7447"]'`. The router itself works with its default config.
9. **WebRTC stream frame rate = sim frame rate**, and it was low (about 19 fps with 2 robots). Investigated with the GPU only 20–45% busy, so the limit is CPU-side, roughly a fixed 15–20 ms per camera render product, not GPU rendering cost or mesh complexity. What helped: `/app/asyncRendering=true` + `asyncRenderingLowLatency=true` (19 → 23 fps, images lag one frame; now in `.env`'s `FLEET_SETTINGS`), fewer robots, and turning cameras off entirely (`CAMERA_STREAMS=none`, new option, → 31–36 fps). What did **not** help at all: camera resolution, colour-only vs colour+depth, `RaytracedLighting` vs path tracing, hiding the Kit UI, `CAMERA_FRAME_SKIP` (skips publishing, not rendering), async replicator setting, streamed-viewport resolution (`FLEET_VIEWPORT_RES`, new), or a coarser D435 mesh. Dead end: `FLEET_MERGE_FIXED=1` breaks the scene build (it merges away `camera_0_link`, which `find_prim` then can't find). **Unexplained:** an earlier ad-hoc test in a prior session apparently saw 60 fps with no cameras (3 robots); this session's no-camera runs only reach 31–36 fps, and the difference wasn't tracked down. Not measured at all: the actual WebRTC client (encoder/network path) with a client connected — every measurement above used `FLEET_DEBUG=1` server-side logging only.
10. **Compose profiles don't shrink automatically:** `docker compose up -d` never stops a service whose profile becomes inactive (lowering `NUM_ROBOTS`), even with `--remove-orphans`. `scripts/fleet.sh` removes the surplus containers explicitly.
11. Smaller: the generated URDF shouldn't be regenerated needlessly (its mtime triggers a USD re-import, tracked via `.import_stamp` in `sim/generated/a300/`); the sim script wipes `sim/generated/a300/` before importing because the importer otherwise writes `a300_1/`, `a300_2/`, ….

## Verified (live, across sessions)

- 1–5 robots spawned via `NUM_ROBOTS`/`scripts/fleet.sh`, including scaling down (surplus containers removed) and back up; each robot only sees/moves on its own `cmd_vel`.
- Drive test on all robots at various counts; teleop via a pseudo-terminal (`pty`) sending `i`, `j`, `k`; camera frame grabbed and inspected (`rgb8`, 640×360); the D435 mesh visible on the top plate when two robots face each other.
- `robot_state_publisher`: 31 static transforms + 4 wheel frames; `tf2_echo odom camera_0_color_optical_frame` resolves.
- RViz via the `rviz` wrapper: RobotModel, TF and Camera panel all showed up (screenshot via PIL `ImageGrab.grab(xdisplay=':1')`).
- Foxglove: connected to bridges with a `websockets` client (subprotocol `foxglove.sdk.v1`), saw the whitelisted topics and live messages, including on a 5th robot at port 8769.
- Both `rmw_zenoh_cpp` and `rmw_fastrtps_cpp` run from the same compose file (A/B measured); Clearpath's `ClearpathConfig` parses the rendered `robot.yaml` for both.
- Async rendering measured to raise fps ~20% without breaking drive/odom/camera topics (checked after enabling).

Handy headless test tricks: `docker exec … bash -c 'timeout -s INT 10 ros2 topic hz …'`; `ros2 topic list --no-daemon --spin-time 4` (most reliable over zenoh); driving via `python3 /scripts/drive_test.py`; grabbing a camera frame or a screenshot via a small inline Python script.

## Not verified / open items

- **The WebRTC client itself, with a client connected:** all frame-rate numbers above are server-side (`FLEET_DEBUG=1`); nobody has measured what a connected client actually experiences (its own decode, network path, resolution).
- **Foxglove app:** the 3D panel with namespaced `/<ns>/tf` topics, and the URDF panel (mesh assets served via the bridge), were not tried in the actual Foxglove app.
- **Real robots:** interoperability with the real robots' zenoh router (`ZENOH_ROUTER=tcp/<robot>:7447`) was not tried; real robots likely each run their own router, whereas this sim shares one `zenoh-router` for the whole fleet.
- **The unexplained fps gap** in item 9 above (60 fps vs 31–36 fps with no cameras) — worth another look if more speed is needed.
- **Default values decision** (see "Repo and current runtime state" above): zenoh/3-robots vs the currently-tracked fastrtps/2-robots/22Hz — needs the user's answer, then `.env` and the docs' wording should be reconciled one more time.
- **Ideas mentioned but not done:** compressed image republishing for Foxglove over Wi-Fi; per-robot zenoh routers; the optional `profile:` key in the Clearpath middleware config; host firewall check for ports 8765+ (`sudo` needs a password, couldn't check); untracking `.env` in favour of `.env.example` (user chose to keep it tracked, once).
- `sim/generated/` is only rebuilt by the sim itself; if the USD looks stale, set `FORCE_REIMPORT=1`.

## Working preferences observed

- Apply changes and verify them against the live stack, not just write them; report results plainly, including regressions (e.g. fps drops) and things that didn't work.
- Ask before committing/pushing; commit messages end with the `Co-Authored-By` line the harness supplies.
- Keep docs (`README.md`, `CLAUDE.md`, this file) in sync with the actual repo state after each notable change, including when a "default" documented in prose diverges from what's actually tracked in `.env`.
- The Claude Code memory directory (`~/.claude/projects/-home-robotian-isaac-sim-project/memory/`) is unused so far.

## Addendum: three lavender plants added to the scene

- User supplied `sim/assets/lavender/SM_Lavender_Nanite_01.usd` (+ Materials/) in the project. The `.usd` itself had been copied as a **Git LFS pointer file** (134 bytes, `version https://git-lfs...`), not the real 162 MB USD crate — copied it from `~/Desktop/assets/Lavender/SM_Lavender_Nanite_01.usd` (the real binary) instead; the Materials/Textures alongside it were already real files.
- Asset facts (inspected with a throwaway `usd-core` pip venv, no `pxr` on the host otherwise): default prim `/Root`, upAxis Z, **metersPerUnit 0.01** (the stage is 1.0/metres) — reference needs an explicit ×0.01 scale, USD does not convert this automatically across a reference boundary. Raw bbox z min ≈ -17.065 (→ -0.1707 m scaled): `add_lavender()` offsets by `+LAVENDER_BASE_Z` so the plant's base sits on the ground instead of poking through it. ~952k points / 1.26M triangles total (5 mesh sections) — a baked/flattened "Nanite" export, not runtime Nanite.
- Added `add_lavender(stage, path, pos, rot_z)` next to `add_box()` in `sim/scripts/setup_scene.py`, called 3× in `build_world()` in a row alongside the robots (`lavender_y = ((n-1)/2)*ROBOT_SPACING + 2.0`, x = 4.0/6.5/9.0), `instanceable=True` on each reference so the heavy mesh is shared. No physics collider (decorative only, and a 1.26M-tri collider would be expensive).
- Measured cost: 2 robots, FastDDS, async rendering: ~23 fps before → ~20 fps after (real-time factor ~0.9 at `SIM_RATE_HZ=22`). Verified: scene builds with no FATAL/material errors (MDL compiled with only harmless "unused let temporary" warnings); drive test still passes; grabbed a robot's camera image (turned it ~27° with a small turn-to-yaw script) and visually confirmed the plants render as lavender-shaped bushes (thin flower-spike geometry, purple/green material) at the correct place and scale.
- **Found and reverted, not fixed:** the plants render quite dark under the scene's default lighting (`dome=350`, `sun=1500`). Bumped to `dome=1500`/`sun=4000` as a one-off test — that made the lavender colour (purple flowers, green foliage) clearly visible, but blew out the rest of the scene (walls/boxes overexposed), so it was reverted rather than left as the new default. Left as an open item for the user: raise the global lights (trade-off above), add a local supplemental light near the plants, or leave as is.
- Files touched: `sim/scripts/setup_scene.py`, `sim/assets/lavender/SM_Lavender_Nanite_01.usd` (replaced pointer with real content — this file is gitignored along with the rest of `sim/assets/`, so it does **not** need committing/pushing, only `setup_scene.py` does), README.md, CLAUDE.md.

## Addendum: shared ROS workspace (`robot` user + colcon_ws)

- Every robot image now has a `robot` user: renamed the base image's existing uid/gid 1000 `ubuntu` account (`usermod -l robot -d /home/robot -m ubuntu && groupmod -n robot ubuntu`) rather than adding a second uid 1000 user. 1000 also happens to be the host user's (`robotian`) uid, so the bind mount below needs no chown tricks.
- `/home/robot/colcon_ws` is bind-mounted from a single host directory `./colcon_ws` into **every** robot container via the shared `x-robot` volumes anchor — one host folder, not a copy per robot, so an edit from any one container (or the host) is immediately visible everywhere. `colcon_ws/src/` is tracked in git; `build/`, `install/`, `log/` are gitignored.
- The container's main process is still root (entrypoint needs it for `/etc/clearpath/robot.yaml` and the `robot_state`/`foxglove` background services); `robot` is only for interactive workspace/colcon work via `docker exec -u robot -it <robot> bash`. ROS is already sourced for it too (the existing system-wide `/etc/bash.bashrc` + `BASH_ENV` fix from an earlier session covers every user, not just root).
- Verified live: `id robot` → uid/gid 1000; wrote a file as `robot` in one container's `colcon_ws/src`, read it back from another robot's container and from the host; wrote from the host, read it from a container; confirmed `robot_state`/`foxglove` background services and `drive_test.py` still work after rebuilding the image.
- Minor doc slip caught and fixed while editing README.md: a `str.replace` accidentally ate a `#` from an existing `### Foxglove` heading, demoting it to `##` — fixed; worth double-checking heading levels after any text-substitution edit to a doc rather than assuming the replace was clean.

## Addendum: colcon_build.sh

- `scripts/colcon_build.sh [colcon args...]`: runs `colcon build "$@"` as the `robot` user in every currently *running* robot container (`docker ps` filtered to `^a300_[0-9]+$`), not against `NUM_ROBOTS`/compose config — a stopped robot can't be `docker exec`'d into anyway. Exits 1 with a message if none are running; if a container's build fails it still tries the rest and exits 1 at the end.
- Argument forwarding: uses `bash -c '...colcon build "$@"' bash "$@"` (not `"$*"`), so multi-word/quoted colcon arguments survive the `docker exec` boundary correctly.
- Verified live: built a throwaway ament_cmake test package (`hello_pkg`) via the script across 2 running robots — first container did the real build (~0.6s), second was a no-op/incremental (~0.1s) since `colcon_ws` is the same bind-mounted host folder for both, confirming the shared-workspace design from the previous addendum actually behaves as intended under a real build tool, not just plain file I/O. Also verified the "no robots running" and "some robots stopped" paths. Test package removed afterward, `colcon_ws/src/` back to just `.gitkeep`.
- Side note (not a bug in the script, just worth remembering): that particular bare test package — `ament_package()` with no `install()`/target/dependency — never registered an `AMENT_PREFIX_PATH` environment hook (`share/colcon-core/packages/hello_pkg` marker came out empty), so `ros2 pkg list` didn't see it after sourcing `colcon_ws/install/setup.bash` even though the build genuinely succeeded and all the expected files existed. A package with actual installed content (a node, a launch file, etc.) is not expected to hit this; didn't chase it further since it's outside what was asked.
