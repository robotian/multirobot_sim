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

## Addendum: real Clearpath boot sequence (generate_bash, /etc/clearpath/setup.bash)

User described how a real Clearpath robot boots: `clearpath_generator_common` reads `robot.yaml` and generates param/launch/bash files under `/etc/clearpath`, including `setup.bash`, which gets sourced/loaded. Investigated what's actually installed and available (only `ros-jazzy-clearpath-generator-common`, no separate launch/param generator package exists in the public Jazzy apt repo) and implemented the part that is: `generate_bash`.

- `robot.yaml.tmpl` gained `system.ros2.domain_id: __DOMAIN__` and `system.ros2.workspaces: [/home/robot/colcon_ws/install]` (Clearpath's own native field for "also source this workspace" — ties directly into the shared `colcon_ws` from the previous addendum).
- `entrypoint.sh`: after rendering `robot.yaml`, ensures `colcon_ws/install/setup.bash` exists (placeholder if nothing's built yet, chowned to `robot`), runs `ros2 run clearpath_generator_common generate_bash -s /etc/clearpath` (needs ROS sourced first, chicken-and-egg — sources plain `/opt/ros/jazzy/setup.bash` just for this), then sources the **generated** `/etc/clearpath/setup.bash` instead of the previous hand-rolled `source .../setup.bash` + no domain handling.
- `robot/Dockerfile`'s `/etc/ros_env.sh` (used by `BASH_ENV`/interactive shells) now prefers `/etc/clearpath/setup.bash` if it exists, falling back to plain ROS for the brief window before the entrypoint has generated it. Every shell in the container — PID1, `docker exec`, `docker exec -u robot` — now goes through the same generated file.
- `scripts/gen_urdf.sh` needed a matching fix: it renders `robot.yaml.tmpl` too (for the one-time host-side URDF generation) and `clearpath_config`'s schema loader rejects a literal `"__DOMAIN__"` string for `domain_id` (must be an int) — added `-e "s/__DOMAIN__/0/g"` there; the value is irrelevant to URDF geometry.
- **Real bug found in the installed `clearpath_generator_common` (2.9.15)**: its bash generator builds the workspace source line with `workspace.strip('setup.bash')` — Python's character-set `str.strip`, not a substring removal — so `/home/robot/colcon_ws` → `source .../colcon_w/setup.bash` (silently wrong, no error). Worked around by pointing `workspaces` at `/home/robot/colcon_ws/install` specifically: that string has no leading/trailing characters in the set `{s,e,t,u,p,.,b,a,h}`, so the buggy strip is a no-op and the generated line comes out correct regardless of whether upstream ever fixes it. Confirmed by reading `clearpath_generator_common/bash/generator.py` directly, not just by observing output.
- **Scope limit, not a gap I could close:** `clearpath_generator_common` in this apt package only has `generate_description` (already used), `generate_bash` (now used), and `generate_discovery_server`/`generate_vcan`/`generate_zenoh_router`/`moveit_collision_updater` (`generate_semantic_description`) — none of the latter four apply here (no CAN bus, one shared `zenoh-router` rather than per-robot, and `moveit_collision_updater` crashes with an Eigen assertion on this URDF) and weren't wired in. There is no separate "generate the platform launch/params files" package available for Jazzy in the public apt repo, so the project's own `robot_state` script + Isaac's OmniGraph bridge still stand in for that part; only the bash/env half of what the user described is now the *actual* Clearpath-generated output.
- Verified live: full recreate on a genuinely empty `colcon_ws` (no `install/` at all) boots with zero errors in `docker logs`; built a real `ament_python` package (with a proper `resource_index` marker — a bare `ament_package()` with no installed content still won't register, same pre-existing quirk noted in the colcon_build.sh addendum) via `scripts/colcon_build.sh` and confirmed `ros2 pkg prefix` finds it from *both* robots and via both `docker exec` (root) and `docker exec -u robot`; switched the whole stack to `rmw_zenoh_cpp` and back via `FLEET_RMW`, redrove `drive_test.py` each time — all clean. One harmless, self-healing wrinkle documented in CLAUDE.md/README: deleting `colcon_ws/install/` on the host while a container keeps running and then opening any new shell in it prints one `No such file or directory` line (the container's already-generated `/etc/clearpath/setup.bash` unconditionally references it) but everything after that line in the file still runs fine; resolves itself once something rebuilds the workspace.
- Test packages removed afterward; `colcon_ws/src/` back to just `.gitkeep`. Files touched: `robot/entrypoint.sh`, `robot/Dockerfile`, `robot/config/robot.yaml.tmpl`, `scripts/gen_urdf.sh`, README.md, CLAUDE.md.

## Addendum: added A200, Jackal (j100), Ridgeback (r100) with real Clearpath names

User asked to add A200/Jackal/Ridgeback to the fleet, then explicitly chose "real Clearpath names" over generic slot labels when asked (via `AskUserQuestion`, in a message that initially said "wrong option" but then restated wanting real names — treated the plain-English restatement as authoritative). This was planned in plan mode first (`/home/robotian/.claude/plans/cosmic-yawning-kettle.md`) given the size (touches nearly every file) before implementing.

- **Real model codes** (confirmed via `dpkg -L`/Clearpath's own sample yamls under `/opt/ros/jazzy/share/clearpath_config/sample/`, already installed, no new apt packages needed): `a300`, `a200`, `j100` (Jackal), `r100` (Ridgeback).
- **Naming**: `ROBOT_MODEL_0`..`ROBOT_MODEL_7` in `.env` (default `a300`) assign each of the 8 compose slots a model; container/hostname/ROS namespace become `<model>_%04d` for that slot's **fixed index** (not a per-model sequential counter — disclosed as a simplification in the plan, unchallenged). Compose service *keys* were renamed from `a300_0000..a300_0007` to stable `robot0..robot7` so `scripts/fleet.sh`/`colcon_build.sh` never need to guess a container name from a model that may have changed.
- **Per-model templates**: `robot/config/robot.a300/a200/j100/r100.yaml.tmpl` (attachments differ too much to parameter-substitute into one file — a300/a200 have bumpers+top_plate, j100 has fenders, r100 has neither). `robot.rviz.tmpl` (renamed from `a300.rviz.tmpl`, was already fully generic).
- **Sim** (`sim/scripts/setup_scene.py`): `ROBOTS = [(namespace, model), ...]` replaces the old flat `NAMESPACES` list; `MODEL_ASSETS` (per-model URDF/USD paths) and `MODEL_PARAMS` (wheel_radius/separation/multiplier/max_linear/max_angular/chassis_link, from each model's real `diff_4wd.yaml`) parametrize what used to be A300-only module constants. `import_urdf_if_needed` now runs once per **distinct** model present, not once globally.
- **Two real bugs found and fixed while implementing** (not hypothetical — hit them live):
  1. `scripts/flatten_urdf.py` hardcoded the USD root prim name to `"a300"` for every model (harmless functionally, since prim lookup is by simple name not full path, but wrong/misleading) — fixed to derive it from `urdf_name`.
  2. **A200's articulation root is `base_link` itself, not a `chassis_link`-style child.** A200's URDF has *several* direct fixed-jointed children of `base_link` (`top_chassis_link` — massless, visual-only; `inertial_link` — actually carries mass; bumper mounts; wheels attach straight to `base_link` too), unlike a300/j100/r100 which each have exactly one such child (litearlly named `chassis_link`) that becomes the importer's `UsdPhysics.ArticulationRootAPI` target. Guessed `inertial_link` first (it has the mass, by analogy) — that produced a live `OmniGraph Error: Articulation controller failed for prim '.../inertial_link'` with **no error at import time**, only when the drive graph actually tried to use it. Fixed by checking `UsdPhysics.ArticulationRootAPI` on the imported USD directly (`base_link` has it) rather than reasoning from URDF structure alone. `sim/scripts/setup_scene.py`'s `MODEL_PARAMS` comment now says to verify this the same way for any future model, not to assume.
- **`scripts/fleet.sh` bug caught by testing, not assumed**: `docker compose ps --format '{{.Name}}' svc1 svc2 svc3` returns names **alphabetically**, not in the order the service names were given (verified both ways experimentally) — with only 2 robots this coincidentally matched slot order and looked fine; with 4 mixed-model robots it silently mislabeled every robot's printed Foxglove port. Fixed by querying each slot's name individually (`docker compose ps --format '{{.Name}}' robot$i` in a loop) rather than batching.
- **Real, unresolved issue**: running all **4 distinct models at once** (4 robots, no repeats — a300+j100+a200+r100 together) crashed the sim with `PhysX Internal CUDA error... Error code 700` (illegal GPU memory access) a few seconds after `timeline.play()`. Isolated by elimination: every single model alone (including Ridgeback specifically) works fine; a300+j100 (2 distinct) works fine; a300+j100+a200 (3 distinct) works fine. Only the specific 4-distinct-model combination reproduced it, and it reproduced consistently on retry. Not root-caused (would need GPU memory profiling, PhysX buffer sizing, or a driver-level investigation well beyond this task's scope) — documented as a known issue in the README/CLAUDE.md rather than chased further. If revisited: check GPU VRAM headroom at the crash moment, try lowering camera resolution/count first, and consider whether Ridgeback's more complex rocker-bogie suspension (extra joints per side) is a contributing factor given it's the common element in the one failing combination.
- Also renamed, low-risk/mechanical: image tag `a300-robot:jazzy` → `clearpath-robot:jazzy`, compose project `a300-fleet` → `clearpath-fleet`. Had to manually clean up (`docker rm -f`, `docker network rm`) containers/network left over from before these renames, since `docker compose down` under the new file can't find things created under the old project name.
- **Verified live**: all 4 models spawn together (no FATAL) once the a200 fix landed; drive tests (forward + turn) passed on a300, j100, a200 (isolated from the crash) and r100 (alone); camera frames grabbed from j100 and r100 and visually checked (D435i renders, not obviously clipped through the chassis — mount xyz values are explicitly documented as approximate/tunable, not hardware-accurate); `colcon_build.sh`'s broadened container regex confirmed to match real robot names and correctly exclude `a300-isaac-sim`/`zenoh-router`; `fleet.sh` status line confirmed correct after the ordering fix, including in the crash-triggering 4-model case (the status line itself is unaffected by the later sim crash, since it just reflects `docker compose ps`).
- `.env` restored to its prior tracked state (`NUM_ROBOTS=2`, both slots `a300` — the `ROBOT_MODEL_*` lines were only ever a live test, never committed) at the end, per established practice in this project.

## Addendum: Jackal fender fix ("default_fender is not properly attached")

User report, in a fresh message: "jackal's default_fender is not properly attached. fix it." Investigated and fixed properly rather than guessing — this took several iterations, each disproven empirically rather than assumed correct.

- **Symptom, visually confirmed** (turned a300 to face j100 and grabbed its camera image, several times at increasing zoom/context): the yellow fender mesh rendered as a large disconnected arch, sometimes floating well above the chassis roof, sometimes off to the side on the ground — clearly not attached, in a way that got *worse* the more the Jackal had been driven/turned since spawn.
- **First hypothesis, disproven**: "purely visual link isn't tracked by physics, so it stays at its pre-simulation import pose while the chassis moves." Gave the fender a small synthetic mass/inertia (`add_missing_inertial`) so Isaac's importer would include it in the simulated rigid-body chain. This did **not** fix it — confirmed by explicitly driving+spinning the Jackal and re-checking: the fender still drifted away. The real mechanism: Isaac's importer *always* makes a fixed-jointed child link a separate simulated body regardless of mass, connected by a PhysX fixed-joint constraint; that constraint just isn't perfectly rigid for a body this light relative to the rest of the robot, so it still visibly lags under fast motion. A small mass didn't remove the constraint, just made it slightly less obviously wrong.
- **Second, separate bug found in the same investigation**: even the *static* (pre-drive) position was wrong — reading Clearpath's own upstream `fender.urdf.xacro` directly (not guessing) showed its `<visual>` has no `<origin>` at all, unlike every other mesh in the same URDF (chassis/wheels each rotate 90° about X to align their mesh export with the link frame). The fender mesh was rendering in its raw, un-rotated export orientation — standing up like a fin instead of wrapping over the wheel.
- **Actual fix, in `scripts/flatten_urdf.py`**:
  1. `fix_fender_orientation`: apply the same 90°-about-X visual origin the wheel meshes use, to any link referencing `default_fender.stl`/`sensor_fender.stl` specifically (a targeted patch for this one upstream gap, not a generic transform).
  2. `merge_visual_only_links`: fold any link that is *only* a `<visual>` (no `<collision>`, no `<inertial>`) directly into its parent link at URDF-generation time, composing the joint's origin with the visual's origin via a small hand-rolled (no numpy) rotation-matrix composer (`_origin_matrix`/`_matmul`/`_matrix_to_origin`, unit-tested standalone before running the full pipeline — identity∘identity, a hand-derived rear-fender case, and a round-trip test all checked out). This removes the fixed-joint constraint entirely, so there's nothing left to drift.
  3. **Bug caught by testing, not anticipated**: merging a link that was itself the *parent* of some other joint left that other joint's `<parent link="...">` pointing at a now-deleted link name — an outright import failure (`ValueError: Parent link '...' not found`), not a cosmetic issue. Fixed by re-pointing any such joint's parent to the merged-into link and re-composing *its* origin the same way.
  4. **A200/j100 chassis-link cross-effect, again caught by testing, not predicted**: merging j100's fenders into `base_link` gave `base_link` real visual content for the first time, which alone was enough to make Isaac's importer root j100's articulation at `base_link` instead of `chassis_link` — exactly the same situation as A200 (documented in an earlier addendum), for an unrelated-looking reason. Caught because the drive test on j100 started failing (`OmniGraph Error: Articulation controller failed`, no import-time warning) after the merge landed; fixed by updating `MODEL_PARAMS["j100"]["chassis_link"]` to `"base_link"` and re-verifying with `UsdPhysics.ArticulationRootAPI` on the actual imported USD (not assumed).
- The same merge function also folds several previously-unnoticed visual-only links in **a200** (`top_chassis_link`, `front_bumper_link`, `rear_bumper_link`, `top_plate_user_rail_link`) and **r100** (`left/right_side_cover_link`, `front/rear_cover_link`, `front/rear_lights_link`, `axle_link`) into their parents — a200's a300-analogous parts had collision so weren't touched by an earlier version of this reasoning, but the bumper *links* turned out to also lack collision/inertial once actually checked; r100's `axle_link` was already showing "Invalid PhysX transform detected" warnings in an earlier session's log (from before this fix), suggesting this same class of bug was silently affecting Ridgeback's whole suspension/cover assembly too — a plausible bonus fix, not separately re-verified beyond "still drives correctly and no new errors."
- **Verified live, final state**: spun j100 in place (angular.z=2.0 for 3s) then drove it forward (linear.x=1.0 for 2s) — about as aggressive a motion test as this project's tooling allows — then viewed it externally; the fenders now sit correctly at the top of both wheel pairs, front and rear, matching a real Jackal's look, with no drift. Re-ran drive tests on a300/a200/r100 too (no regressions; r100 and a300's `chassis_link` targets were independently confirmed unaffected by checking `UsdPhysics.ArticulationRootAPI` directly rather than assuming only j100/a200 needed rechecking).
- `.env` restored to its tracked state (`NUM_ROBOTS=2`, both slots default `a300`) at the end, per established practice.

## Correction to the addendum above: the orientation "fix" was wrong, reverted

User, in the very next message: "the original jackal's fender orientation was correct... the fenders are not moving as the robot moved. Now, the fenders are moving as the robot moves, but the orientation is wrong. Use the original orientation."

I had misdiagnosed the visual symptom: the fender's *drifting away while detached* looked, in isolated screenshots, like a wrong/rotated mesh (a floating arch/fin shape), so I added `fix_fender_orientation` (a 90°-about-X visual-origin rotation, matching the wheel meshes) on top of the real fix (`merge_visual_only_links`). The orientation was never actually wrong — Clearpath's own `fender.urdf.xacro` identity/no-origin visual was correct all along; what looked like a rotation problem was just the detached body rendering at whatever odd angle it happened to drift to. Removed `fix_fender_orientation` entirely (function and its call site); `merge_visual_only_links` alone (composing with the fender's original, un-rotated visual origin) is the complete fix.

Re-verified after removing it: fresh spawn shows the fender as a flat trim skirt around the chassis base (matches the user's description of the original, correct look), and it stays in exactly that position after spinning (angular.z=2.0, 3s) and driving (linear.x=1.0, 2s) — attached and correctly oriented simultaneously. Updated CLAUDE.md's description of `merge_visual_only_links` to state plainly that the orientation is correct as-is and to not re-add a rotation fix. `.env` restored to its tracked state again afterward.

## Note: reported TF issue was a stale Foxglove view, not a bug

Follow-up user report after the fender fix: "when the Jackal moves the base_link frame does not move accordingly relative to the odom frame... base_link stays in the same place." Investigated directly rather than assuming: `ros2 topic echo /j100_0001/tf` before/after driving showed `odom -> base_link`'s translation changing correctly (e.g. x: 0.408 -> 1.37 after a 3s 0.5 m/s drive), and the fender (part of base_link's own merged geometry) had already been visually confirmed tracking correctly in the previous addendum. Opened RViz on j100 to cross-check by the same means the user might be seeing it, and in parallel asked where they were observing this (Isaac's own Stage panel vs RViz vs something else) before changing anything, given the last two reports in this thread had each needed a real fix — didn't want to guess a third time without evidence.

User clarified: they were watching in **Foxglove**, and after I'd been working on the fender fix (which involved several `docker compose up -d --force-recreate` cycles), it "fixed itself" — i.e. almost certainly a stale/cached view in the Foxglove client from before or during one of those recreations, not a bug in the TF pipeline. No code change needed or made. `.env` re-confirmed clean/restored to its tracked `NUM_ROBOTS=2` state (a leftover `ROBOT_MODEL_1/2` pair briefly reappeared after an interrupted `fleet.sh 2` run mid-investigation — a `docker compose rm -sf` targeting a container already gone via a race with `docker compose up`'s own recreation, harmless but worth knowing `fleet.sh` isn't perfectly race-proof under rapid repeated invocation; re-running it cleanly resolved it).

## Addendum: "got a300 instead of r100" (slot/NUM_ROBOTS confusion)

User: using the then-current `.env` (`NUM_ROBOTS=2`, `ROBOT_MODEL_1=a200`, `ROBOT_MODEL_2=r100` — leftovers from earlier testing in this session, with a stale mismatched comment on the first line), tried to load one a200 + one r100 and got a300 instead of r100.

- **Root cause, not a bug**: `NUM_ROBOTS=2` only starts slots 0 and 1; `ROBOT_MODEL_2` needs `NUM_ROBOTS>=3` to ever take effect. Their r100 was configured on an inactive slot, so slot 0 (unset, defaults to a300) and slot 1 (a200) were what actually ran, with r100 silently never starting — no error, no warning, just silence, which is exactly why it was confusing.
- **Fix applied**: corrected `.env` to `ROBOT_MODEL_0=a200`, `ROBOT_MODEL_1=r100` (matching `NUM_ROBOTS=2`) — this is the user's actual intended standing configuration, not a diagnostic scratch state, so **left it in `.env` rather than reverting** (a deliberate departure from this session's usual "always restore to tracked 2xa300 afterward" habit — that habit was for undoing *my own* test scaffolding, not for undoing an explicit user request).
- **Added a safety net**: `scripts/fleet.sh` now warns (`ROBOT_MODEL_<i> ... is not running`) when a configured slot's index is >= the active `NUM_ROBOTS`, so this specific confusion can't recur silently. Tested by temporarily adding an out-of-range `ROBOT_MODEL_3` and confirming the warning fires, then removing it again.
- **False alarm, corrected in-session, worth remembering**: while re-verifying the fix, a200 appeared to have "stuck" odometry (velocity read correctly and instantaneously, e.g. 0.49 m/s, but position barely advanced) when paired with r100. Jumped to "intermittent GPU/PhysX startup race, same class as the 4-distinct-model crash" and started writing that up as a known issue — but before finalizing it, checked the actually-simplest explanation: the robot's odometry position (queried directly) had drifted well away from its spawn point (from many *earlier, cumulative* manual `cmd_vel` pokes and drive_test.py calls across unrelated tests in the same session, never reset in between), consistent with it having driven into a scene prop or the *other* robot and gotten physically wedged, not a simulation bug. Confirmed by doing one clean thing: `--force-recreate` (resets every robot to its spawn pose) followed by exactly *one* fresh `drive_test.py` call before touching anything else — displacement came out correct (1.40 m for a 3 s/0.5 m/s command) on the very first try, and again for r100 (1.39 m). Removed the incorrect "known issue" bullet from README.md before it was ever seen by the user. Lesson for next time: when a drive/odometry test looks wrong *after a long sequence of other manual pokes*, check current position/rule out "it just hit something" (cheap: one `ros2 topic echo` for position, or just a clean recreate + single test) **before** reaching for a fancier root cause — the boring explanation was right both times this pattern came up in this session (this one, and indirectly the a200 "articulation root" one earlier which *was* a real bug, so the lesson is specifically about *ruling out the mundane cause first*, not "assume it's never a real bug").
- Ended this session with the stack running the user's intended fleet (a200_0000 + r100_0001), both freshly re-verified driving and turning correctly from a clean spawn.

## Addendum: Ridgeback omnidirectional drive ("make the ridgeback to move omnidirectionally using omni_4wd.yaml")

User's explicit request: give Ridgeback genuine holonomic (mecanum) control instead of the diff_4wd.yaml skid-steer approximation it had used since the multi-model addendum above. This took three implementation attempts, each one disproven live before the next, not assumed correct from the math alone.

- **Geometry derivation, verified against real Clearpath numbers**: read `sim/assets/r100/r100.urdf` directly for wheel joint positions/axes (`chassis_link` → `{front,rear}_rocker` → `*_wheel_joint`, all `rpy="0 0 0"`, axis `(0,1,0)` for every wheel) — front/rear rocker at x=±0.319, wheel joints at y=±0.2755 off that, giving `wheel_positions` (`MODEL_PARAMS["r100"]`). Sanity-checked: 0.319+0.2755=0.5945 ≈ Clearpath's own `omni_4wd.yaml` `kinematics.sum_of_robot_center_projection_on_X_Y_axis: 0.59` — confirms the URDF-derived numbers against the real control config, not just the mesh. `mecanum_angles` (front_left/rear_right share one diagonal roller angle, front_right/rear_left the other, standard "X" mecanum arrangement) aren't in the URDF at all (Clearpath's ROS description carries no roller-angle metadata) so were derived from the standard mecanum kinematics equations for `isaacsim.robot.wheeled_robots.HolonomicController`'s specific convention — including catching that its OGN schema *says* `mecanumAngles` is in radians but `isaacsim.robot.experimental.wheeled_robots.controllers.HolonomicController._build_base` actually applies it with `degrees=True`, found by reading that source directly, not trusting the schema doc.
- **Attempt 1, HolonomicController driving the wheel joints directly — computed correctly, didn't work**: wired `ROS2SubscribeTwist` → `BreakVector3`×2 → (`omni.graph.nodes.MakeVector3`, not `ConstructArray` — see below) → `HolonomicController` → `IsaacArticulationController` targeting all 4 wheel joints. First hit `omni.graph.nodes.ConstructArray`'s dynamic `inputs:input1`/`input2` needing explicit `CREATE_ATTRIBUTES` before they exist (`OmniGraphError: Parsed destination '...CombineVel.inputs:input1' as a path attribute`), then that its output type (`double[]`, a variable array) is fundamentally incompatible with `HolonomicController.inputs:inputVelocity`'s `double3` (a fixed tuple) — a silent warning in the sim log, not a hard error, so nothing else would have caught picking the wrong node. Fixed by using `omni.graph.nodes.MakeVector3` instead (`inputs:x/y/z` → `outputs:tuple`, a genuine `double[3]` tuple, no dynamic attributes needed) — found by searching `omni.graph.nodes_core`'s docs for `BreakVector3`'s counterpart. Once wired, tested live against `r100_0001` (pure Vx, pure Vy, joint_states echoed during each): forward worked; **pure strafe barely moved the robot (dy≈0) despite the wheels visibly/measurably spinning with the mathematically-correct pattern** (`ros2 topic echo .../platform/joint_states --field velocity` showed real, non-trivial per-wheel speeds). Root cause, confirmed by reading the URDF, not assumed: Ridgeback's own `<collision>` for every wheel is a plain `<cylinder>` (the angled-roller detail is mesh-only, in `<visual>`) — a spinning cylinder is physically incapable of producing sideways thrust regardless of what any kinematic layer computes. Checked whether this PhysX version could fake a roller's passive sideways slip via anisotropic/directional friction (`frictionType` scene attribute) — it's fixed to `"patch"` only in this version (checked the PhysX USD schema directly), so no material-level workaround exists either.
- **User consulted before proceeding** (`AskUserQuestion`, since this changes the fidelity/nature of the fix, not just an implementation detail): given three options — direct chassis-velocity override (recommended), revert to diff-drive, or model real roller collision geometry — chose the override.
- **Attempt 2, full (Vx,Vy,Wz) chassis-velocity override, replacing HolonomicController entirely — fixed strafe, broke rotation**: added a `omni.graph.scriptnode.ScriptNode` (`BodyDrive`) that every tick rotates the commanded body-frame (Vx,Vy) into world frame using the chassis' current yaw (`UsdGeom.Xformable(...).ComputeLocalToWorldTransform(0).ExtractRotation()`, transforming a local +X probe vector rather than trusting a specific quaternion-component convention) and calls `isaacsim.core.experimental.prims.Articulation(chassis_path).set_velocities(linear_velocities=..., angular_velocities=...)` — confirmed this is the API that sets a *floating-base articulation's root* velocity (not a generic rigid body), matching what a URDF import with `fix_base=False` gives every robot here. Tested live: pure forward and pure strafe both worked correctly (positive commanded → positive measured displacement, ~65-90% of the naive ideal, no cross-coupling) — **but pure rotation alone (0.5 rad/s, 3s) produced ~0° measured turn**, while a combined Vx+Vy+Wz command rotated close to correctly. Kept the wheel-joint HolonomicController driving active alongside this for cosmetic spin at first, and that turned out to be actively harmful, not just uselessly decorative: `platform/joint_states` during the rotation test showed large, real wheel velocities that didn't match a symmetric expected pattern, and removing that wheel-driving entirely didn't fix the rotation problem — ruling it out as the cause and pointing at something more fundamental about pure-rotation-from-rest itself.
- **Attempt 3 (final), hybrid — real diff-drive wheels for Vx/Wz, BodyDrive patches only Vy**: restored `wheel_separation=0.551`/`separation_multiplier=1.0` (r100's original real `diff_4wd.yaml` numbers) and made the `Diff`/`DriveFront`/`DriveRear` nodes build unconditionally for every model (previously only the non-omni branch built them). `BodyDrive` now only touches the lateral component: reads the chassis' *current actual* velocity (`Articulation.get_velocities()`), decomposes it into body frame, replaces just the lateral part with the commanded `vy` (leaving whatever forward speed the real diff-drive wheels produced untouched), recomposes to world frame, and calls `set_velocities()` with **only** `linear_velocities` — `angular_velocities` is never passed, so rotation is 100% governed by real wheel-ground physics, identical mechanism to the other 3 models. Tested live: forward (0.4 m/s, 3s → 1.09 m, ~91% of ideal — better than the full-override version, since real wheel rolling is more efficient than a kinematic override for this), strafe (0.4 m/s, 3s → 0.61 m lateral, ~0 forward drift), and combined (0.3/0.3/0.3 m/s/rad/s → proportionally correct on all 3 axes) all worked well.
- **Remaining, root-caused but not fully fixed limitation**: pure in-place rotation from a standstill is still weak for small commands even with real wheel-driven rotation — re-tested 0.5 rad/s alone against the *final* hybrid design and still got ~0° turned, then tried 2.0 rad/s alone and got 63-79° (real, substantial, correct-sign rotation, though well under the ~344° naive ideal). This rules out "BodyDrive/HolonomicController fighting the wheels" as the cause (attempt 3 has neither active during rotation) and points at PhysX's contact solver itself: turning a stationary 4-wheeled vehicle in place fundamentally requires each wheel's contact patch to break static friction and scrub sideways (the normal skid-steer mechanism), and that breakaway is resisted far more strongly than continuing a motion that's already sliding (kinetic friction) — consistent with small commands being almost fully absorbed while larger ones or ones combined with translation get through. Not chased further into physics/solver tuning; documented plainly as a known characteristic in README.md/CLAUDE.md/`robot.r100.yaml.tmpl` rather than presented as fully solved.
- `wheel_positions`/`wheel_axis`/`mecanum_angles` (the correctly-derived mecanum geometry from attempt 1) are kept in `MODEL_PARAMS["r100"]` as verified reference/documentation even though nothing currently reads them, in case a future fix (real roller collision geometry, or a way to drive the wheels without fighting BodyDrive) finds a use for them.
- Files touched: `sim/scripts/setup_scene.py` (`MODEL_PARAMS`, `build_ros_graph`, new `BODY_DRIVE_SCRIPT`), `robot/config/robot.r100.yaml.tmpl`, README.md, CLAUDE.md. `.env` untouched (still the user's standing a200+r100 fleet from the previous addendum).

## Addendum: real MTU robots (j100_0921, j100_0936) from their actual robot.yaml files, full sensors + arm

User dropped `robot_data/<serial>/robot.yaml` (two folders, MTU's real, currently-deployed Jackal configs) and asked to spawn these as real sim robots. Planned first (size/scope comparable to the earlier multi-model addition), then, on the plan's one open question ("visual/drive/camera only" vs "full sensor + arm simulation"), the user explicitly chose the larger scope: real ROS2 data / real command handling for every sensor and the arm, not just accurate geometry.

- **Real config → generated URDF, verified by actually running the generator, not by reading docs**: `clearpath_generator_common generate_description` + `xacro` expand cleanly (50 links/53 joints, zero errors) from the real yaml once `platform.extras` is stripped — its custom `mtu32_description`/`mtu32_bringup` xacro/launch are MTU's own private lab packages, not installed here, and turned out not to be load-bearing for the base URDF *except* for one thing (see the `top_mount_link` bug below). Every other real sensor/manipulator (`stereolabs_zed`, `microstrain_imu`, `sick_lms1xx`, `swiftnav_duro`, `kinova_gen3_lite`, `kinova_2f_lite`) resolves to an already-installed Clearpath package xacro — needed one new apt package, `ros-jazzy-kortex-description` (the Kinova arm's real mesh package, a separate dependency of `clearpath_manipulators_description`'s own xacro, not bundled with it).
- **Real bug #1, found by inspecting the generated URDF, not assumed**: the real yaml's `sensors.imu[0]` and `links.frame.top_shelf` both reference `parent: top_mount_link`, but nothing in the *real* config (Clearpath-native or otherwise) ever defines that link — it's referenced as a joint `<parent>` twice and never appears as a `<link>` anywhere, an invalid URDF that `xacro` itself doesn't catch (it's a pure text/macro processor, no semantic validation) but would break on actual import. Root cause: the stripped `mtu32_description` custom xacro was what defined it on the real hardware. Fixed by adding a substitute `links.frame` entry to both templates, aliasing `top_mount` to `default_mount` (Jackal's standard top-of-chassis mount point, `chassis_link + 0.184m up`) at zero additional offset — an approximation (the real shelf mount's exact height is unknown without the missing package), disclosed in the templates' own comments.
- **Real bug #2, found live at container boot, not anticipated**: `clearpath_config`'s `SerialNumber.parse()` requires exactly `<model>-<decimal unit>` (2 fields) or `cpr-<model>-<decimal unit>` (3, first field literally `cpr`) — a real robot id like `j100_0921` (needed as `ROBOT_MODEL` so it can reuse the whole existing per-model templating mechanism) breaks this two *different* ways: (a) `scripts/gen_urdf.sh`'s old `${m}-0000` serial derivation and `robot/entrypoint.sh`'s old `${ROBOT_NAMESPACE//_/-}` derivation both produce an invalid 3-field, non-`cpr` serial for any model id containing its own underscore — fixed by special-casing "does the model id itself contain `_`" in both scripts, using the model id's own (single) underscore→hyphen conversion directly instead of the slot-indexed namespace; (b) *even with a valid `serial_number`*, `clearpath_config`'s `SystemConfig` validates a completely separate field, `system.localhost`, whose *default* is the container's own real OS hostname (`socket.gethostname()`) — checked unconditionally, before our `robot.yaml` is even read — and `docker-compose.yml`'s per-slot `hostname: cpr-${ROBOT_MODEL_i}-%04d` is *also* model-derived, so it hit the exact same underscore problem regardless of (a)'s fix. Fixed by making `hostname:` a fixed `cpr-slot-<index>` for all 8 slots (docker compose has no string find/replace in its interpolation syntax, confirmed by testing a `${VAR//a/b}` config directly — "invalid interpolation format" — so a per-slot conditional wasn't an option) and having each real-robot template set `system.localhost` explicitly to its own correct serial, which is what actually matters for Clearpath tooling anyway.
- **ZED camera**: unlike the D435i models (which build a ROS-optical-convention frame by hand in `add_camera`, since the D435i xacro doesn't produce one), the ZED2i's own xacro already emits `camera_0_left_camera_frame_optical` with the correct rotation baked into its joint (`rpy = (-pi/2, 0, -pi/2)`, verified from the generated URDF before writing any code) — `add_camera` gained an `optical_link` parameter to mount the USD camera directly under an existing correctly-oriented link instead, used only by these two robots.
- **IMU**: no ready-made OGN "read" node exists for `isaacsim.sensors.experimental.physics`' IMU sensor (only `ROS2PublishImu`, which needs the data fed to it) — a `ScriptNode` (`ImuRead`) authors the sensor prim lazily (needs the physics tensor view, same reasoning as `BodyDrive`'s `Articulation`) and feeds it every tick. Works correctly first try, verified live by checking `linear_acceleration.z ≈ 9.8` (gravity) on a stationary robot — genuine physics, not a placeholder. Physically attached to `chassis_link`, not the URDF's own `imu_1_link` name: `imu_1_link` is visual-only, and so is *every* link on its way up to the chassis (`imu_1_base_link`, `top_mount_link`, `default_mount` itself) — `merge_visual_only_links`' cascading merge (already built, see its own docstring) folds the whole chain, one level at a time, all the way into `chassis_link`, confirmed by checking the actual flattened URDF, not assumed from a single per-link check (the first per-link check was misleading: `imu_1_base_link` alone looked like it should survive, but only *after* `imu_1_link`'s merge gives it a visual it didn't originally have, which is exactly what triggers *its own* merge into the next link up, and so on).
- **2D lidar (j100_0936 only) — genuine environment defect, not fixed**: the only 2D-lidar-capable pipeline in this Isaac Sim version is RTX Lidar (`isaacsim.sensors.experimental.rtx.Lidar`/`LidarSensor`, a newer, higher-level Python API than the raw OGN-node approach originally assumed — found via Isaac's own `standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py`, which uses `Lidar.create()` + `LidarSensor.attach_writer("RtxLidarROS2PublishLaserScan", ...)`, a Replicator-writer pattern, not `og.Controller.edit`). First hit `ImportError: cannot import name 'Lidar' from 'isaacsim.sensors.experimental.rtx' (unknown location)` when importing it from `add_lidar2d`; several iterations chasing this as a code-ordering problem (enabling the extension again after `new_stage_async()`, adding app-update delays before first import) didn't fix it and one attempt (enabling it twice) made it measurably worse (a duplicated, still-broken `__path__`). The real cause, found by checking the sim's *own* boot log rather than continuing to guess: `isaacsim.sensors.rtx.nodes` (the extension that provides the RTX sensor OGN nodes, including the ROS2 lidar publishers) fails to import *during Kit's own native startup*, well before any of this project's code runs — `ImportError: cannot import name 'register_writer_spec' from 'isaacsim.sensors.experimental.rtx' (unknown location)`, the identical broken-package symptom, hit by Isaac Sim's own built-in extension loading, not by anything this script does or how it orders imports. `add_lidar2d` now returns immediately with a docstring explaining this; `lidar2d_link` stays in `MODEL_PARAMS` as documentation of which real sensor this would be.
- **GPS (x2) — real bug in the generic ROS2Publisher OmniGraph node, found and worked around**: no per-type `NavSatFix` publish node exists in this Isaac Sim version (checked directly), so first tried `isaacsim.ros2.bridge.ROS2Publisher`, the generic any-message publisher, whose message-specific fields become dynamically-created attributes named after the message's own field path (confirmed from `isaacsim.ros2.nodes`' own generic-publisher test suite). Built it, topics advertised correctly, `status`/`service`/`position_covariance` (set via literal `SET_VALUES`) came through exactly as set — but `latitude`/`longitude` (fed via a `ScriptNode` connection, since GPS position is inherently computed per-tick, not a literal) stayed at exactly `0.0`, not even the fixed origin constant, ruling out a units/sign bug in the math itself. Isolated with a one-field A/B test (temporarily replacing just `altitude`'s connection with a literal `326.0`, leaving `latitude`/`longitude` still connected): the literal altitude came through correctly while the connected fields stayed `0.0` — confirming *connections into this node's dynamically-created inputs silently never propagate a value*, while literal values on the same kind of attribute work fine. Given the value is inherently per-tick-computed, not literal, worked around by bypassing `ROS2Publisher` entirely: the `ScriptNode` (`GpsRead`) now publishes `sensor_msgs/NavSatFix` directly via a plain `rclpy` publisher created inside the script (Isaac's ROS2 bridge already loads an internal `rclpy` into the same process, confirmed in its own boot log, and every container shares one `RMW_IMPLEMENTATION`/`ROS_DOMAIN_ID`, so a bare `rclpy.init()`/`Node()` here joins the same ROS graph with no special context wiring) — this also fixed a second, smaller gap (the generic publisher's `header.frame_id`/`stamp` were left empty/zero with no input found to set them; the raw `rclpy` message has real, correct ones). Origin for the fake lat/lon: Michigan Tech's Houghton, MI campus (~47.1211, -88.5455, ~326m) — ties it to the real institution these robots belong to (`mtu32_description`, `*.sabu.mtu.edu` hostnames in the real config) rather than an arbitrary placeholder; verified live both GPS units read close to that origin and to each other, tracking the chassis' real simulated position.
- **Arm + gripper — real bug in this project's own `IMPORT_SETTINGS`, found live, fixed**: `ROS2SubscribeJointState` → a second `IsaacArticulationController` (position-mode, dynamic `jointNames`/`positionCommand` passthrough from the subscriber rather than a hardcoded list, so it can't silently command the wrong joint if a client publishes a different subset/order than guessed) built and wired cleanly, but a real position command (`arm_0_joint_1: 0.5`) barely moved the joint at all (`platform/joint_states` showed ~0.03 rad drift over 2s). Root cause: this project's global `IMPORT_SETTINGS['override_joint_stiffness'] = 0.0` (correct for the wheels — a pure velocity drive needs zero position-holding spring so it can spin continuously) applies to *every* joint at import time, arm included, and a USD `DriveAPI` with zero stiffness is pure velocity damping — a `positionCommand` write has no effect regardless of what `IsaacArticulationController` sends, contrary to what its own OGN docstring implies ("position/velocity/effort commands" reads as if all three always work). Fixed with a new `configure_arm_drives()` pass, run once per spawned real-robot, that finds every joint prim with `"arm_0"` in its own name (covers both `arm_0_joint_N` and `arm_0_gripper_*_joint` uniformly, no hardcoded joint list) and overrides just its `DriveAPI` stiffness/damping to real position-servo values (`1e5`/`1e4`, conventional Isaac Sim position-control numbers, not the Kinova's own real servo gains — a disclosed approximation of the joints' response, not a torque-accurate model). Re-tested after the fix: the same command moved the joint to within 0.005 rad of the exact commanded value.
- **In-place rotation, same phenomenon as Ridgeback's, worse here**: first appeared as a hard *zero* — 0.5, 1.5, then 3.0 rad/s pure-rotation commands all measured ~0.00 rad turned, and even a combined 0.4 m/s + 0.8 rad/s command measured *zero displacement too*, despite `platform/joint_states` showing the wheels genuinely, correctly, differentially spinning (±2.36 rad/s, proper L/R split) during the rotation attempts — ruling out a `Diff`/kinematics bug (a wrong wheel-pairing bug would show as *wrong-direction* rotation, not exactly zero, and the wheels were provably spinning correctly). Before chasing a fancier explanation, checked the mundane one this project has already learned to check first: `docker compose up -d --force-recreate` (clean spawn) and immediately re-tested the *same* 0.5 rad/s command alone — genuinely non-zero this time (0.11 rad), confirming the accumulated sequence of prior failed-rotation attempts (and possibly the earlier successful drive/combined tests) had left the robot in a bad state, not a worsening bug. From a clean spawn, rotation is real but still weak for small commands (0.11 rad for 0.5 rad/s x 3s, ~7% of the naive ideal; 0.50 rad for 2.0 rad/s, similarly low) — consistent with the *same* static-friction-from-standstill phenomenon already documented for Ridgeback this session, compounded here by (a) the arm/sensors' added rotational inertia and (b) these two robots' own real, deliberately-honoured `max_angular: 1.0` cap (from the real robot.yaml, far below Ridgeback's 4.0 default), which limits how much a "just command more" test can even demonstrate. Documented as a known characteristic in README.md/CLAUDE.md, same as Ridgeback's, not chased into solver tuning.
- **Final verified state**: both robots' forward drive, camera (ZED, ~17-18 fps), IMU (real gravity-consistent data), dual GPS (real, moving lat/lon near the MTU origin, correct frame_id/stamp), and arm+gripper position control all confirmed working live, post-fix, on a clean spawn. 2D lidar (0936 only) is the one disclosed gap, blocked by the environment defect above.
- Files touched: two new templates `robot/config/robot.j100_0921/0936.yaml.tmpl`; `sim/scripts/setup_scene.py` (`MODEL_ASSETS`/`MODEL_PARAMS`, `add_camera`'s `optical_link` param, new `add_lidar2d`/`configure_arm_drives`, `IMU_READ_SCRIPT`/`GPS_READ_SCRIPT`, arm/IMU/GPS additions to `build_ros_graph`); `robot/entrypoint.sh` (serial derivation fix); `scripts/gen_urdf.sh` (same fix, plus the two new models); `robot/Dockerfile` (new templates, `ros-jazzy-kortex-description`, explicit `clearpath-manipulators(-description)`); `docker-compose.yml` (fixed per-slot `hostname:`); README.md, CLAUDE.md. `.env` set to `ROBOT_MODEL_0=j100_0921`, `ROBOT_MODEL_1=j100_0936` and **left running** (not restored to the prior a200+r100 fleet) — this is the feature the user just asked for, not diagnostic scratch state.

## Addendum: real robots' ROS namespace is their own serial, not slot-suffixed

Follow-up user request: "for the real robot configuration, use the robot serial number as the namespace. it should be j100_0921, not j100_0921_0000." Every other model's namespace is `<model>_%04d` (needed so multiple robots can share one generic model without colliding); a real robot is one specific physical robot with one fixed identity, so the slot suffix is redundant/wrong for it.

- `sim/scripts/setup_scene.py`'s `ROBOTS` list and `robot/entrypoint.sh`'s `ROBOT_NAMESPACE` both gained the same conditional already used for `ROBOT_SERIAL` (does the model id contain `_`? use it directly; otherwise `<model>_%04d` as before) — `docker-compose.yml` can't compute this conditionally itself (re-confirmed: `${VAR//a/b}`-style substitution in a compose file gives "invalid interpolation format", not silently ignored).
- **Real subtlety, found by reasoning about it before testing, not after**: `docker exec` shells (`bin/teleop`/`camera_view`/`bin/rviz`, which just read `$ROBOT_NAMESPACE`) don't inherit whatever `entrypoint.sh`'s own process exports at runtime — they get the container's own base environment, i.e. whatever `docker-compose.yml`'s `environment:` block set (`ROBOT_NAMESPACE: ${ROBOT_MODEL_i}_%04d`, still slot-suffixed, unchanged). Fixed by having `entrypoint.sh` write the corrected value to a new `/etc/robot_ns_env.sh` (overwritten fresh on every entrypoint run, including a plain `docker restart`, so repeated restarts never accumulate duplicate export lines) and having `/etc/ros_env.sh` (already sourced by every shell via `BASH_ENV`/`bashrc`, the existing mechanism for sourcing `/etc/clearpath/setup.bash`) source it too, last, so it wins.
- Verified live end-to-end after rebuilding the image and restarting the fleet: `/etc/robot_ns_env.sh` and `/etc/clearpath/robot.yaml`'s own `namespace:` field both show `j100_0921` (not `j100_0921_0000`); a *fresh* `docker exec` shell's `$ROBOT_NAMESPACE`/`$ROS_NAMESPACE` show the corrected value (not just entrypoint's own process); the sim's own log shows `spawned j100_0921 (j100_0921)` / `simulation running with 2 robots: j100_0921, j100_0936`; `ros2 topic list` shows every topic under `/j100_0921/`/`/j100_0936/`; and `drive_test.py` (reads `$ROBOT_NAMESPACE`) reports `[j100_0921]` and drives correctly.
- **Left as-is at the time, disclosed, then also fixed on request** (see next two addenda): the *docker container name* itself (`j100_0921_0000`, `docker ps`/`docker exec <name>`) still carried the slot suffix — `container_name:` in `docker-compose.yml` is a separate static field with the same "can't compute conditionally" limitation, and fixing it would need an additional per-slot override variable the user would have to keep in sync; flagged as a lower-stakes, separate identifier rather than silently left unmentioned.
- Files touched: `sim/scripts/setup_scene.py` (`ROBOTS`), `robot/entrypoint.sh` (`ROBOT_NAMESPACE` override + `/etc/robot_ns_env.sh`), `robot/Dockerfile` (source the new file), `docker-compose.yml`/README.md/CLAUDE.md (comments only).

## Addendum: GPS rclpy nodes weren't namespaced (found by the user, from a real `ros2 node list`)

User pointed out (pasting real `ros2 node list` output) that the GPS `rclpy` nodes `GPS_READ_SCRIPT` creates showed up as bare top-level nodes (`/gps_read_j100_0921_sensors_gps_1_fix`) instead of under the robot's namespace (`/j100_0921/...`, matching `/j100_0921/robot_state_publisher` etc.) — the *topic* was already correctly namespaced (an absolute `/j100_0921/sensors/gps_1/fix` path, unaffected), but the node's own identity wasn't, since `rclpy.node.Node(name)` was called with no `namespace=` argument. Fixed by adding a `namespace` input to the `GpsN` script node (passed `ns`, the robot's own namespace, same value used to build `topicName`) and `Node(name, namespace=str(db.inputs.namespace))`. Verified live: `ros2 node list` now shows `/j100_0921/gps_read_...`/`/j100_0936/gps_read_...`, and the topic data (lat/lon near the MTU origin, correct frame_id/stamp) is unaffected, as expected — an absolute topic path doesn't depend on the publishing node's own namespace.
- Files touched: `sim/scripts/setup_scene.py` (`GPS_READ_SCRIPT`, the `Gps{i}` node's `create_attributes`/`values` in `build_ros_graph`).

## Addendum: docker container names fixed too, on request

Direct follow-up: "fix the docker container names too." Same underlying constraint as the ROS namespace fix (`docker-compose.yml`'s interpolation can't inspect `ROBOT_MODEL_<i>`'s content to decide whether to append a slot suffix), but this field (`container_name:`) is static compose config, not something `entrypoint.sh`/`setup_scene.py` can override at runtime the way `ROBOT_NAMESPACE` was — needed an actual mechanism inside compose's own (limited) interpolation syntax.

- **Verified the key trick directly before building on it**: Docker Compose's `${VAR-default}` (no colon) genuinely distinguishes "VAR is unset" (uses default) from "VAR is set to the empty string" (uses the empty value) — confirmed with a disposable two-line test compose file, both ways, before touching the real one. (`${VAR:-default}`, the colon form used everywhere else in this project's compose file, treats empty and unset the same, so it wouldn't have worked here.)
- `docker-compose.yml`'s 8 `container_name:` lines became `${ROBOT_MODEL_<i>:-a300}${ROBOT_SUFFIX_<i>-_%04d}` (that slot's own literal 4-digit index in the default, as before). `scripts/fleet.sh` now keeps `ROBOT_SUFFIX_<i>` in `.env` in sync with `ROBOT_MODEL_<i>` on every run — writes `ROBOT_SUFFIX_<i>=` (empty) when that slot's model contains `_` (a real robot id), removes the line entirely if it later changes back to a generic model — so the user only ever manages `ROBOT_MODEL_<i>` directly, never `ROBOT_SUFFIX_<i>` by hand (documented as such in both files' comments).
- Verified live: `scripts/fleet.sh 2` recreated the containers as plain `j100_0921`/`j100_0936` (`docker ps` confirms), `.env` shows the auto-written `ROBOT_SUFFIX_0=`/`ROBOT_SUFFIX_1=`, `drive_test.py` inside the renamed container still works and reports `[j100_0921]`, and `scripts/colcon_build.sh` (matches container names via a `^[a-z0-9]+_[0-9]{4}$` regex, written with the old `<model>_<slot>` shape in mind) still finds and builds against both — turns out to work unmodified because Clearpath serial numbers are always exactly 4 digits (`SerialNumber.parse` requires decimal, not a fixed length, so this is a coincidence of these two robots' actual serials, not a guarantee for every possible future one, but not worth generalizing for a hypothetical robot that doesn't exist yet).
- Files touched: `docker-compose.yml` (8 `container_name:` lines + updated comment), `scripts/fleet.sh` (auto-sync loop + updated comment), README.md, CLAUDE.md.

## Addendum: container hostname is now `cpr-<model>-<serial>`, not `cpr-slot-<index>`

User: the fixed `cpr-slot-0000`/`cpr-slot-0001` hostnames (from the earlier real-MTU-robots addendum's `system.localhost` validation fix) should instead read `cpr-<robot model>-<robot serial number>`, e.g. `cpr-j100-0921` for `j100_0921`.

- Realized the transformation needed is exactly the one `robot/entrypoint.sh` already computes for `ROBOT_SERIAL` (used in `system.localhost` inside `robot.yaml` itself): take the robot's own namespace (`j100_0921` for a real robot, `a300_0000` for a generic model/slot) and turn `_` into `-`. Since that value is itself a valid hostname (only `[a-z0-9-]`), reusing it for the container's actual OS hostname satisfies the same `clearpath_config` validation constraint that forced the `cpr-slot-<i>` fallback in the first place, while also being far more informative.
- `docker-compose.yml` still can't compute this itself (no interpolation find/replace, re-confirmed against the existing comment on this point) — added a new per-slot override `ROBOT_HOSTNAME_<i>`, computed in bash by `scripts/fleet.sh` (merged into the existing `ROBOT_SUFFIX_<i>` sync loop, same per-slot model lookup) and written into `.env` on every run, exactly like `ROBOT_SUFFIX_<i>`. `hostname:` for all 8 slots changed from the literal `cpr-slot-000N` to `${ROBOT_HOSTNAME_N:-cpr-slot-000N}` (fallback only matters if `docker compose up` is run directly before `fleet.sh` has ever populated `.env`).
- Verified live: `scripts/fleet.sh` (fleet was `j100_0921`+`j100_0936` at the time) wrote `ROBOT_HOSTNAME_0=cpr-j100-0921`, `ROBOT_HOSTNAME_1=cpr-j100-0936`, and `ROBOT_HOSTNAME_2..7=cpr-a300-000N`; recreated containers' `hostname` command confirms `cpr-j100-0921`/`cpr-j100-0936`; both containers booted with no Clearpath config errors.
- **Unrelated false alarm during this same verification pass, worth remembering**: right after the recreate, `drive_test.py` reported exactly zero displacement/velocity on both robots for several tries in a row, including differential-turn and full-speed-reverse commands, even though `platform/joint_states` showed the wheel joints correctly, immediately flipping sign to match each new command (ruling out a stale-DDS-connection/discovery theory after the container recreate) — real, live cmd_vel delivery, just no chassis motion at all in *either* direction. User clarified live: the robots had been physically **flipped over, wheels off the ground**, from the accumulated aggressive test commands earlier in this same debugging session (large reverse+turn combos, rapid direction changes) — not caused by the hostname change itself. Fixed by restarting just the sim (`docker compose up -d --force-recreate isaac-sim`, not the robot containers — the simulated pose lives in the sim process, recreating the robot containers alone does nothing to it), which respawns every robot at its default upright pose. Re-verified with small, slow, single-direction commands (`0.15 m/s`, 1.5s) on both robots afterward — correct odometry, no regression. **Lesson for future testing in this project**: keep drive-test commands slow and single-direction (this project's own tooling can physically flip a light 4-wheeled robot with aggressive combined/reversing commands); a "robot stopped responding to cmd_vel" symptom where wheel joints are visibly, correctly responding to commands but the chassis doesn't move is a physical/pose issue (flipped or wedged), not a ROS/config bug — check pose/orientation directly (or just restart the sim to respawn) before assuming a regression in whatever was just changed.
- Files touched: `docker-compose.yml` (8 `hostname:` lines + comment), `scripts/fleet.sh` (extended the existing sync loop + comment).

## Addendum: RViz mouse control (orbit/pan/zoom) not working via the `rviz` wrapper

User report: `docker exec -it j100_0921 rviz` opens the window and renders the robot model, but the 3D view doesn't respond to mouse drag/scroll (no orbit/pan/zoom); plain `docker exec -it <c> bash` then typing `rviz` interactively had the same symptom, so it wasn't specific to how the shell was invoked. Two earlier hypotheses were tried and both ruled out live before finding the real cause:

- **X11 auth**: `/tmp/.docker.xauth` on the host had become a root-owned empty *directory* (Docker auto-creates the bind-mount source as a directory if it doesn't exist when a container first starts), silently blocking `x11_auth.sh` (runs as a normal user, no sudo) from ever writing the real xauth file there. Fixed the stray directory (`sudo rm -rf`, user ran it), re-ran `x11_auth.sh`, but the *already-running* containers still had the stale (directory) bind mount — `docker compose up -d` doesn't re-evaluate a bind-mounted source's file-type change on already-running containers. Fixed with `docker compose up -d --force-recreate robot0 robot1`. This fixed the window/auth error but not the mouse-control symptom.
- **GPU passthrough**: theorized indirect GLX/software-rasterizer fallback (no `/dev/dri` inside robot containers, confirmed via `ls -la /dev/dri` → "No such file or directory") could explain flaky OGRE mouse-interaction. Added `devices: - /dev/dri:/dev/dri` to the `x-robot` anchor in `docker-compose.yml`, force-recreated, confirmed `/dev/dri/card1`/`renderD128` now present inside the container. **Did not fix it** — kept as a legitimate improvement regardless (real GPU rendering vs. software fallback is worth having either way), but not the actual root cause.
- **Real root cause, found from the user's own working reference command**: asked what exactly they run for the case that works; answer was `rviz2 --ros-args -r __ns:=/j100_0921 -r /tf:=tf -r /tf_static:=tf_static` — critically, **no `-d` flag**, i.e. no RViz config file loaded at all (a bare default session). The project's `robot/bin/rviz` wrapper always passes `-d /tmp/robot.rviz` (generated from `robot/config/robot.rviz.tmpl`). Read the template directly: it defines `Panels` (just `Displays`) and `Visualization Manager` (`Displays`/`Global Options`/`Views`) but had **no top-level `Tools:` section at all**. RViz2's default startup auto-loads its usual toolbar (Interact, MoveCamera, Select, ...); a config file that doesn't list `Tools:` loads *no* tools, not "fall back to defaults" — so the 3D view still renders (rendering is governed by `Displays`, unaffected) but nothing handles mouse drag/scroll for camera control, since that's the `Interact`/`MoveCamera` tools' job specifically.
- **Fix**: added a `Tools:` section to `robot/config/robot.rviz.tmpl` (sibling to `Panels`/`Visualization Manager`, not nested under either) with `rviz_default_plugins/Interact` (the actual camera-control tool: left-drag orbit, shift+left-drag/middle-drag pan, scroll zoom) and `rviz_default_plugins/MoveCamera` (the classic dedicated camera-only tool, matching rviz2's own standard default toolbar rather than a cut-down one). No container restart needed — `robot/bin/rviz` regenerates `/tmp/robot.rviz` from the template fresh via `sed` on every invocation, so re-running `docker exec -it j100_0921 rviz` alone picked up the fix.
- **User confirmed live, after also separately adding the MoveCamera tool**: "after i added 'move camera' tool. it works." Mouse-driven orbit/pan/zoom now functions through the `rviz` wrapper.
- Files touched: `robot/config/robot.rviz.tmpl` (`Tools:` section added), `docker-compose.yml` (`/dev/dri` passthrough, kept as a real improvement though not the fix). GPU passthrough and xauth fixes are real, standing improvements even though the mouse-control bug itself was the RViz config's missing `Tools:` section.

## Addendum: j100_0921 now uses its real robot.yaml directly, `mtu32_description` built and included

User: "let's make big change. instead of using the config generation from robot.xxx.yaml.tmpl, use the real
robot.yaml directly for the model generation and the robot container... start with a single robot, j100_0921."
They had colcon-built `mtu32_husky` (a small repo with just `mtu32_description`+`mtu32_bringup`, MTU's private
lab packages `platform.extras` needs) into the top-level `colcon_ws` already, and copied its source into a new
`sim/colcon_ws/src/` for the host-side generation pipeline to build separately. Planned first (touches the
generation pipeline in three places), then implemented and verified against the live stack.

- **Inspected the real `robot_data/j100_0921/robot.yaml` before writing any code**: its baked-in
  `namespace: j100_0921`/`domain_id: 0`/`middleware.implementation: rmw_fastrtps_cpp`/`workspaces:
  [/home/robot/colcon_ws/install/setup.bash]` already match this project's own conventions exactly — meaning
  **no placeholder substitution is needed at all** for this file, unlike every other model. It also doesn't set
  `system.localhost`, so `clearpath_config`'s own default (this container's real OS hostname) applies — and
  that's already `cpr-j100-0921` from the `ROBOT_HOSTNAME_<i>` fix two addenda ago, which is not only valid but
  *exactly* this robot's own `system.hosts[0].hostname`. Zero extra hostname work needed for this change.
- **`docker-compose.yml`**: added `./robot_data:/robot_data:ro` to the `x-robot` volumes anchor (the Dockerfile's
  build context is `./robot`, which can't reach `robot_data/` at the repo root, so this has to be a runtime bind
  mount, not a build-time `COPY`).
- **`robot/entrypoint.sh`** and **`scripts/gen_urdf.sh`** each gained the identical branch: if a model id
  contains `_` and `/robot_data/<id>/robot.yaml` exists, `cp` it straight to the destination (no `sed`);
  otherwise the existing `.tmpl` path is unchanged (still covers `j100_0936`, whose own `robot_data` folder
  isn't currently present, and the 4 generic models). `gen_urdf.sh` also colcon-builds `sim/colcon_ws` (its own
  separate copy, not the runtime one) and sources its `install/setup.bash` before `generate_description` —
  **first real bug hit**: `colcon build`'s own `build`/`install`/`log` dirs are relative to *cwd*, not
  `--base-paths`, so the first attempt failed with `PermissionError` trying to create `log/...` in the
  container's default working directory; fixed with `(cd /colcon_ws && colcon build)`.
- **Deleted `robot/config/robot.j100_0921.yaml.tmpl`** (superseded) and dropped it from `robot/Dockerfile`'s
  `COPY config/robot.*.yaml.tmpl` line. `robot.j100_0936.yaml.tmpl` untouched.
- **Second real bug, found running `gen_urdf.sh` for the first time with `platform.extras` actually included**:
  `scripts/flatten_urdf.py`'s `dae_to_obj` (the manual Collada merger for non-Blender-exported meshes, previously
  only ever exercised by the D435i's `d435.dae`) crashed with `IndexError` converting `top_mount_link`'s real
  mesh (`top_assy_rev1.dae`, mtu32_description's own asset, 535k triangles). Root cause, found by inspecting the
  file's actual `<triangles>` structure rather than guessing: the code remapped *every* value in a `<triangles>`
  element's interleaved `<p>` index list (which carries VERTEX *and* NORMAL indices together) through the
  position-only `remap` table, then sliced out just the VERTEX-offset ones *afterward* — so any NORMAL index
  numerically larger than the vertex count threw `IndexError`. `d435.dae` happened to index normals 1:1 with
  positions (same count, same range), which is why this was never hit before. Fixed by slicing to the VERTEX
  input's own declared `offset` *before* remapping, not after.
- **Verified generate_description + xacro succeed** with `platform.extras` genuinely included (no more stripped
  config) — confirmed the flattened URDF gained `top_mount_link` (real mesh + collision this time, parented on
  `default_mount`, no longer needing the old fake `top_mount`→`default_mount` alias link) and `camera_1` (a
  RealSense D405 the xacro adds unconditionally on `arm_0_end_effector_link` — inert geometry only, since
  `sensors.camera`'s `intel_realsense` entry is still commented out in the real robot.yaml).
- **Re-verified `MODEL_PARAMS["j100_0921"]` against the new URDF shape, not assumed** (this project's own
  established practice, since a URDF shape change has silently flipped `chassis_link`/sensor mount points
  before): `chassis_link="base_link"` unaffected — confirmed via a successful drive test with no "Articulation
  controller failed" error (same live check as every other model, not `UsdPhysics.ArticulationRootAPI`
  inspection this time, but equally decisive). `imu_link` **did** change: since `top_mount_link` now has real
  collision, `merge_visual_only_links`' cascading merge for `imu_1_link`/`imu_1_base_link` (still visual-only)
  now stops there instead of continuing all the way to `chassis_link` like before — confirmed in the flattened
  URDF (no `imu_1` string survives anywhere; `top_shelf_link`'s own joint parent is `top_mount_link` directly)
  and changed `imu_link` from `"chassis_link"` to `"top_mount_link"`. `camera_optical_link`/`gps_links`/`has_arm`
  unaffected (come from `clearpath_sensors_description`'s own macros, unrelated to `mtu32_description`).
- **Full functional re-verification on a clean spawn** (`FORCE_REIMPORT=1` for the first restart, since the USD
  cache needed invalidating for the URDF shape change; a plain restart after the `imu_link` code fix, no
  re-import needed since only Python changed): no FATAL, fresh "simulation running"; `docker exec j100_0921 diff
  /robot_data/j100_0921/robot.yaml /etc/clearpath/robot.yaml` identical (confirms the real file really is used
  unmodified, not a lightly-templated copy); slow single-direction `drive_test.py` (0.15 m/s, per two addenda
  ago's lesson) gave correct non-zero displacement; IMU `linear_acceleration.z ≈ 9.8` (real gravity, not
  zero/garbage) from the new `top_mount_link` mount point; both GPS units near the MTU origin (~47.1211); camera
  publishing 640-wide frames; `platform/joint_states` shows all 4 wheel + 6 arm + 4 gripper joints.
- **`.gitignore`**: added `sim/colcon_ws/build|install|log` (mirroring the existing `colcon_ws` pattern) and, for
  visibility rather than a silent decision, `robot_data/` and `sim/colcon_ws/` in full — both hold large, private
  lab material (`mtu32_husky` ~157MB, `robot_data` ~5.5GB) that had been manually excluded from commits rather
  than actually gitignored until now.
- **Deliberately out of scope, not touched**: `j100_0936` (its own `robot_data` folder isn't currently present)
  stays on the old stripped-template path; `a200_0333`/`a300_00036` (two new real-robot `robot_data` folders the
  user also dropped in) aren't wired into `MODEL_ASSETS`/`MODEL_PARAMS`/`gen_urdf.sh`'s `MODELS` list at all yet
  — per the user's own explicit scope ("start with a single robot, j100_0921").
- **Known follow-up, not chased in this pass**: `top_assy_rev1.obj`'s 535k triangles barely reduce under the
  existing `DECIMATE_CELL` (0.001 m) — much heavier than anything else in this project's models (the D435i's own
  tuned budget is 17k). Worth an fps check and possibly a coarser cell for this specific mesh if it turns out to
  cost noticeably more.
- `.env` was already at `NUM_ROBOTS=1`/`ROBOT_MODEL_0=j100_0921` (no `ROBOT_MODEL_1`) when this session started —
  apparently the user's own doing, matching their "start with a single robot" scope exactly — left as-is, not
  restored to the prior 2-robot fleet.
- Files touched: `docker-compose.yml` (`robot_data` volume mount), `robot/entrypoint.sh` (direct-copy branch),
  `scripts/gen_urdf.sh` (colcon build + direct-copy branch), `robot/Dockerfile` (dropped the retired template's
  `COPY`), `scripts/flatten_urdf.py` (`dae_to_obj` VERTEX-offset fix), `sim/scripts/setup_scene.py`
  (`MODEL_PARAMS["j100_0921"]["imu_link"]` + comment split from `j100_0936`'s), `.gitignore`, CLAUDE.md. Deleted
  `robot/config/robot.j100_0921.yaml.tmpl`.

## Addendum: slow forward creep at rest, investigated -- not caused by today's work, damping ruled out as the fix

User: "the jackal is slowly moving forward without any cmd_vel input." Investigated live rather than guessing.

- **Confirmed real, not a stray publisher**: `FLEET_DEBUG=1`'s own existing debug loop (already reads
  `CmdVel.outputs:linearVelocity/angularVelocity`/`Diff.outputs:velocityCommand` live) showed all three
  genuinely `[0, 0(, 0)]` while the robot was visibly creeping -- the drive graph really is commanding zero, so
  this is a physics-level issue, not a leftover command.
- **Confirmed not caused by this session's j100_0921 pipeline change**: `j100_0936` (untouched, still on the old
  stripped-template path, no `top_mount_link`/`mtu32_description`) drifts at essentially the same rate
  (~0.02m/6s) when spawned and left idle -- ruling out today's work as the cause before touching any code.
- **Tried raising `IMPORT_SETTINGS["override_joint_damping"]`** (the force-drive/velocity-target/zero-stiffness
  wheel joints' only real "holding torque" parameter: damping × (targetVelocity − currentVelocity), which at
  targetVelocity=0 is also what should resist an external disturbance at rest) from 1000 → 10000 → 100000 (two
  full re-imports, since this setting is part of the USD import stamp and triggers automatic re-conversion) --
  **zero measurable effect at either step** (~0.011-0.012m/8s at all three values), decisively ruling out
  "insufficient holding torque" as the mechanism, not just an unlucky choice of value. Reverted to the original
  1000 (no evidence a higher value helps, and no reason to carry an untested 100x change with unknown side
  effects elsewhere).
- **Checked whether this is universal, not just these two heavier robots**: plain generic `j100` (no arm, much
  lighter) also creeps at rest, just slower (~0.005m/8s vs ~0.011-0.012m/8s) -- present even on the lightest,
  simplest model in this sim, scaling with mass/complexity rather than being specific to the real MTU robots'
  arm/sensor loadout. Points at a small, universal contact/substep-convergence characteristic of this sim's
  physics setup rather than anything in this project's own OmniGraph wheel-drive wiring.
- **Left unsolved, documented as a known low-priority characteristic** (`IMPORT_SETTINGS`'s own comment in
  `setup_scene.py` now records this investigation and its negative result) -- next concrete levers to try if
  revisited: `PHYSICS_HZ`/substep count, or PhysX solver iteration counts, neither of which were touched this
  session since damping was the more obviously-relevant parameter and testing it first was cheap and decisive.
- **Process note, own mistake caught and fixed within this same investigation**: reverting `ROBOT_MODEL_0` back
  to `j100_0921` in `.env` via a direct `docker compose up -d --force-recreate isaac-sim robot0` (not
  `scripts/fleet.sh`) left the container named `j100_0921_0000` -- `ROBOT_SUFFIX_0`/`ROBOT_HOSTNAME_0` only get
  resynced by `fleet.sh`'s own loop, exactly the failure mode its comments already warn about. Caught
  immediately by checking `docker ps` rather than assuming, fixed by actually running `scripts/fleet.sh 1`.
- Ended back at the user's exact prior state: `NUM_ROBOTS=1`, `ROBOT_MODEL_0=j100_0921`, container/hostname
  correctly `j100_0921`/`cpr-j100-0921`, drive re-verified working.
- Files touched: `sim/scripts/setup_scene.py` (`IMPORT_SETTINGS` comment only, value unchanged from before this
  addendum).

## Addendum: two more real MTU robots wired up (a200_0333, a300_00036)

User: "make the other robots in the robot_data folder work" — `robot_data/` had gained two new folders
(`a200_0333`, `a300_00036`; `j100_0936`'s own folder is still gone) alongside `j100_0921`. Since the generic
"real `robot.yaml` used directly" pipeline built for `j100_0921` was already fully model-agnostic (branches on
"does `/robot_data/<id>/robot.yaml` exist", no `j100_0921`-specific code), this was mostly wiring + verification,
not new infrastructure — done one robot at a time, matching this project's established practice.

- **Neither references a private package** (`a200_0333`'s `platform.extras.urdf` is an empty `{}`;
  `a300_00036` has no `extras` key at all) — `generate_description` succeeded first try for both, no
  missing-package workaround needed, unlike `j100_0921`'s `mtu32_description`.
- **Two real bugs found and fixed in `scripts/gen_urdf.sh`'s output path, both upstream/generic, not
  robot-specific**:
  1. `scripts/flatten_urdf.py`'s `dae_to_obj`: `a200_0333`'s Velodyne VLP16 3D lidar meshes
     (`velodyne_description`'s own shipped `.dae` files) failed to even XML-parse — `ET.fromstring` raised
     `not well-formed (invalid token)`. Root cause, found by testing each referenced mesh individually rather
     than guessing: the file uses the literal, unescaped string `<STL_BINARY>` as a Collada node id/name (a
     genuine upstream authoring bug in the apt package, confirmed identical across all three VLP16 mesh files).
     Fixed with a narrow text substitution (`<STL_BINARY>` → `STL_BINARY`) before XML parsing.
  2. Once parseable, the same three meshes converted to **0 triangles** — `dae_to_obj` only ever handled
     `<triangles>` elements, and these STL-derived meshes use `<polylist>` instead (confirmed: every `<vcount>`
     entry is 3, i.e. already all-triangle, just encoded differently). Rewrote the face-extraction loop to
     handle both `<triangles>` and `<polylist>` uniformly (fan-triangulating using `<vcount>`, correct even for
     a real n-gon, not just this all-triangle case) — meshes now convert to real triangle counts (1009/616/104).
- **`sim/scripts/setup_scene.py` changes**: `MODEL_ASSETS`/`MODEL_PARAMS` entries for both, reusing the generic
  a200/a300 drivetrain constants unchanged (neither real robot.yaml has a `platform_velocity_controller`
  override the way the real Jackals do). New pattern needed for the first time: **`a300_00036` has no camera at
  all** (only an IMU) — `add_camera`/`build_ros_graph`'s camera wiring were previously called unconditionally
  for every robot, which would have crashed (`find_prim` raising on a nonexistent `camera_0_link`); added a
  `has_camera` `MODEL_PARAMS` flag (default `True`, so every existing model is unaffected) guarding the call in
  `main()`, passing `cam_path=None` when absent (a path `build_ros_graph` already handled). Added a symmetric
  `add_lidar3d` no-op (same broken `isaacsim.sensors.rtx` extension as the existing `add_lidar2d`, just a
  different lidar profile — 3D lidar is the same RTX Lidar pipeline, not a separate one) for `a200_0333`'s
  Velodyne, and wired its call site into `main()`'s spawn loop.
- **`a200_0333`'s camera is a plain "d435" (not "d435i" like every other model in this project)**, mounted via
  `sensors.camera` + a `mounts.fath_pivot` adapter — a Clearpath mount type not seen elsewhere here. Verified in
  the flattened URDF that it still produces the same `camera_0_link` name `add_camera`'s existing default
  (hand-built optical frame) path already expects, so no code change or `camera_optical_link` override was
  needed — same code path as the 4 generic models, not the ZED2i's special-cased one.
- **Real bug, found live, same class already documented twice in this project (a200's `inertial_link`, j100's
  fender-merge)**: `a300_00036` first tried with generic a300's own `chassis_link="chassis_link"` — hit
  `Articulation controller failed for prim '.../base_link/chassis_link'` at runtime, no import-time warning.
  Root cause: the real robot.yaml's `phidgets_spatial` IMU (parent: `base_link`) is visual-only and merges
  straight into `base_link` (confirmed in the flattened URDF: `base_link`'s own `<visual>` is the merged
  sensor's box geometry) — giving `base_link` real visual content it didn't have in the generic model was
  enough to flip the importer's articulation root there too, exactly the same mechanism as the two earlier
  cases. Fixed by setting `chassis_link="base_link"`; re-verified via a clean drive test with no error.
- **Verified live, both robots, one at a time** (`NUM_ROBOTS=1`, swapping `ROBOT_MODEL_0` between them): clean
  boot, no FATAL; `docker exec <robot> diff /robot_data/<id>/robot.yaml /etc/clearpath/robot.yaml` identical for
  both; `a200_0333`: drive test correct (0.58m/2s @ 0.3m/s), camera topic delivers a real non-blank 640×360
  `rgb8` frame, no lidar topics (correctly inert); `a300_00036`: drive test correct (post chassis_link fix),
  `domain_id`/`middleware` correctly defaulted to `0`/`rmw_fastrtps_cpp` (both omitted in its own real
  robot.yaml) matching the rest of the fleet, IMU reads real gravity (`linear_acceleration.z = 9.81` exactly),
  no camera topics (correctly absent, not crashed). Container hostnames `cpr-a200-0333`/`cpr-a300-00036` both
  matched each robot's own `system.hosts[0].hostname`, same convenient convergence as `j100_0921`.
- Ended back at the user's prior standing state (`NUM_ROBOTS=1`, `ROBOT_MODEL_0=j100_0921`) — this was
  incremental verification, not a request to leave a different robot running; `a200_0333`/`a300_00036` are now
  available the same way any other `ROBOT_MODEL_<i>` value is.
- Files touched: `scripts/gen_urdf.sh` (`MODELS` list), `scripts/flatten_urdf.py` (`<STL_BINARY>` fix,
  `<polylist>` support), `sim/scripts/setup_scene.py` (`MODEL_ASSETS`/`MODEL_PARAMS` entries, `has_camera` guard,
  `add_lidar3d`), README.md.

## Addendum: j100_0922 added by the user directly, one typo + one real dangling-joint bug + a real wheelie

User added a fourth `robot_data` folder (`j100_0922`) and edited `scripts/gen_urdf.sh`'s `MODELS` list themselves
(had the file open in the IDE), then hit two problems in sequence, each investigated and fixed properly.

- **First**: `./gen_urdf.sh` failed with `ValueError: Serial number model entry j100_0922 must be one of [...]`.
  Root cause: `robot_data/j100_0922/robot.yaml`'s own `serial_number:` field used an underscore
  (`j100_0922`) instead of a hyphen (`j100-0922`) — a genuine typo in the robot's own data, not a sim-specific
  issue (`clearpath_config`'s `SerialNumber.parse()` requires the hyphenated form; the real physical robot
  would hit the same error booting with this file as-is). Every other real `robot.yaml` in `robot_data/`
  already used the correct hyphenated form. Offered to fix it in the file directly; user chose to fix it
  themselves rather than have their real robot data edited.
- **Second**: after adding `ROBOT_MODEL_3=j100_0922` (a 4-robot fleet: `j100_0921`/`a300_00036`/`a200_0333`/
  `j100_0922`), "the simulation does not start" — `docker logs a300-isaac-sim` showed a genuine `[fleet] FATAL
  error`: `KeyError: 'j100_0922'` in `MODEL_ASSETS[model]`. Root cause: `j100_0922` had never been wired into
  `sim/scripts/setup_scene.py` at all (URDF generation and the container's own `robot.yaml` pipeline are both
  fully generic already, but the sim's own `MODEL_ASSETS`/`MODEL_PARAMS` are not) — an unhandled exception in
  `main()` aborts the *entire* scene build, which is why all 4 robots failed to start, not just this one.
- **Diffed `j100_0922`'s real robot.yaml against `j100_0921`'s before wiring anything**: identical in every
  section except `manipulators:` — `j100_0922`'s is entirely commented out, i.e. this robot has no arm/gripper
  at all, unlike `j100_0921`/`j100_0936`.
- **Real upstream bug, found and fixed generically (not just for this one robot)**: with no arm, `xacro`+
  `generate_description` still succeeded (xacro has no semantic validation), but the flattened URDF had a
  genuinely broken reference — `camera_1_joint`'s `<parent>` is `arm_0_end_effector_link`, which
  `mtu32_description`'s own `robot_description_j100.urdf.xacro` unconditionally assumes exists (it always
  mounts a second RealSense D405 there) but which `clearpath_config` never defines when there's no arm
  configured. Fixed with a new `scripts/flatten_urdf.py` pass, `prune_dangling_joints` (called before
  `merge_visual_only_links`): drops any joint whose `<parent>` link was never actually defined anywhere,
  cascading to catch a joint that only became dangling because *its own* parent was just removed (confirmed
  live: it correctly dropped both `camera_1_joint` and, on the next pass, the now-orphaned
  `camera_1_link_joint`). General/defensive, not `j100_0922`-specific, in case a future real robot.yaml hits
  the same kind of upstream assumption elsewhere.
- **Wired `MODEL_ASSETS`/`MODEL_PARAMS["j100_0922"]`**: `chassis_link`/`imu_link`/`camera_optical_link`/
  `gps_links` identical to `j100_0921` (re-verified live via drive test + a clean `imu_0_link` inspection — this
  robot still has the platform's own separate default IMU, unaffected by the arm's absence, distinct from the
  explicit microstrain sensor that still merges into `top_mount_link` the same way), `has_arm` omitted.
- **Real, reproducible physics finding, not a config bug — flagged to the user, not chased further without
  their go-ahead**: `j100_0922` visibly wheelies under even a gentle drive command. Verified this is real, not
  leftover test-session state (this project's own "check the mundane explanation first" habit): a completely
  fresh, isolated single-robot spawn showed a perfectly level orientation quaternion (`w≈1`) before any command,
  then a single gentle `drive_test.py 0.15 0 1.5` reproducibly pitched it to the *exact same* ~77° orientation
  (`y≈0.622, w≈0.783` both times) — deterministic, not random settling noise. Likely explanation, not fully
  confirmed: `j100_0922` is genuinely lighter than `j100_0921`/`j100_0936` (missing the Kinova arm's own real,
  well-defined mass/inertia at the rear), so the same wheel-drive damping/torque that produces normal, gentle
  acceleration on every other real robot in this project produces a much larger pitching moment on this
  specific one. Not investigated further (would mean picking a lever — reducing wheel damping specifically for
  this robot, rate-limiting/ramping commanded velocity, or accepting it as a known characteristic — without a
  clear steer from the user first, matching how the earlier idle-creep investigation was also handed back
  rather than continued speculatively).
- Hit a container-name conflict restoring the intended 4-robot fleet afterward, worth remembering: a real
  robot's container name doesn't depend on slot index (`ROBOT_HOSTNAME_<i>`/`ROBOT_SUFFIX_<i>` are keyed off
  the *model*, not the slot), so moving `j100_0922` from an isolated test at slot 0 back to slot 3 hit `Error
  response from daemon: Conflict. The container name "/j100_0922" is already in use` — compose can't recreate a
  container under a name still held by a *different* service's (slot's) prior container. Fixed with a plain
  `docker rm -f j100_0922` before re-running `scripts/fleet.sh`.
- Ended with all 4 robots running (`j100_0921`, `a300_00036`, `a200_0333`, `j100_0922`), clean boot confirmed,
  matching what the user was originally trying to run before the `KeyError` blocked it.
- Files touched: `robot_data/j100_0922/robot.yaml` (fixed by the user themselves, not by me), `scripts/
  flatten_urdf.py` (`prune_dangling_joints`), `sim/scripts/setup_scene.py` (`MODEL_ASSETS`/`MODEL_PARAMS`
  entry for `j100_0922`).

## Addendum: sensor topic naming audit — a real IMU index bug found and fixed, lidar gap re-confirmed structural

User: "each robot has different sensors. but not all of them are publishing. publish the sensor data and the
topic names should following Clearpath's naming convention. sensors/camera_0, sensors/camera_1, ... etc."
Audited live (`ros2 topic list` across all 4 running real robots) against each one's own real `robot.yaml`
sensor list before changing anything, rather than assuming.

- **Real bug found: `a300_00036`'s IMU published as `sensors/imu_1/data`, but its correct Clearpath index is
  `imu_0`.** `build_ros_graph`'s IMU wiring hardcoded the literal `"1"` everywhere (topic name, TF `frameId`,
  even the sensor prim path suffix) — a leftover from when only the two Jackals had a simulated IMU, both of
  which really do belong on `imu_1` (Jackal's platform always has a separate built-in default IMU occupying
  slot 0, pushing the explicit configured sensor to slot 1). A300 has no such platform default, so its own
  explicit sensor genuinely *is* slot 0. Verified this directly, not from memory: ran `generate_description`+
  `xacro` on each real robot's raw (pre-flatten, pre-merge) `robot.yaml` and grepped for `imu_<N>_link` --
  `a300_00036` → only `imu_0_link`; `j100_0921`/`j100_0922` → both `imu_0_link` (default) and `imu_1_link`
  (explicit sensor). (First attempt at this check gave a misleading "j100_0921 also only has imu_0" result --
  the throwaway container hadn't sourced `sim/colcon_ws`'s `mtu32_description`, so `generate_description` for
  Jackal silently failed and the check was actually re-reading a300's own leftover output from the previous
  loop iteration, since nothing cleared `/tmp/setup` between iterations. Fixed the diagnostic itself
  (`set -e`, source the workspace, `rm -rf /tmp/setup` per iteration) before trusting its result.)
- **Fix**: added `imu_index` to `MODEL_PARAMS` (`1` for the three Jackals, `0` for `a300_00036`), and
  `build_ros_graph`'s IMU section now builds `sensors/imu_{imu_index}/data`, `imu_{imu_index}_link` and the
  sensor prim path from that instead of the old hardcoded `"1"`. Verified live: `a300_00036` now publishes
  `sensors/imu_0/data` with real data (`linear_acceleration.z ≈ 9.81`); the three Jackals unaffected, still
  `sensors/imu_1/data`.
- **GPS's own index derivation was reusing list position (`enumerate(gps_links, start=1)`), not the link name
  itself** — currently always correct in practice (every real robot's `gps_links` happens to already be listed
  in `gps_1_link, gps_2_link` order), but fragile in the same way the IMU code just turned out to be broken.
  Made it parse the real index straight out of the link name (`re.search(r"gps_(\d+)_link", gps_link)`)
  instead, so it can't silently drift from the real numbering the way IMU's hardcoded literal did. Re-verified
  live afterward: GPS still publishes correctly (`sensors/gps_1/fix`, real latitude near the MTU origin).
- **Camera indexing (`sensors/camera_0`) was already correct** — every real robot in this fleet has at most one
  simulated camera, always at index 0, no platform-default camera competing the way IMU's does; nothing to fix.
- **Lidar (SICK on `j100_0936`, Hokuyo UST + Velodyne VLP16 on `a200_0333`) still isn't published** — re-
  confirmed this is a genuine environment limit, not something newly broken or previously under-investigated:
  checked the running `a300-isaac-sim` container's own extension cache directly for any non-RTX lidar
  extension (an older `range_sensor`-style API) that might exist in this specific Isaac Sim 6.0 install and
  hadn't been tried — found only `omni.sensors.nv.lidar` (the low-level RTX plugin itself), no alternative.
  The actual blocker remains what was already documented: `isaacsim.sensors.rtx.nodes` (the Python/ROS2 bridge
  side of RTX Lidar, the only lidar pipeline this install has at all) fails to import during Kit's own native
  startup, confirmed in the sim's boot log well before this project's own code runs. Nothing in this session's
  scope could change that; flagged clearly to the user rather than left silent.
- Files touched: `sim/scripts/setup_scene.py` (`imu_index` in `MODEL_PARAMS`, IMU wiring in `build_ros_graph`,
  GPS index derivation, `import re`).

## Addendum: 2D lidar unblocked — a working, RTX-independent sensor the earlier investigation had missed

User: "it seems like 'isaacsim.sensors.experimental.physics.RaycastSensor' is replacing old lidar sensor
library. did you check this?" -- a direct, correct challenge to the earlier "2D/3D lidar is blocked, no
alternative pipeline exists in this install" conclusion. It hadn't been checked: the earlier investigation only
looked for extension-level lidar *packages* (extscache directory names), not classes living inside an
already-enabled, already-proven-working extension (`isaacsim.sensors.experimental.physics`, the same module
`IMU`/`IMUSensor` already use successfully). Investigated properly this time, and it was a real miss worth
correcting.

- **Confirmed `Raycast`/`RaycastSensor` are real, independent of the broken RTX pipeline**: read the extension's
  own source (`raycast.py`, `raycast_sensor.py`, `extension.py`) -- it acquires its own `IRaycastSensor`
  Carbonite interface via a separate native binding (`_physics_sensors.acquire_raycast_sensor_interface()`),
  nothing to do with `isaacsim.sensors.rtx`. Found and read this install's own `benchmark_physx_lidar.py`
  standalone example and the extension's own test suite (`test_raycast_sensor.py`) to confirm exact usage
  semantics (ray_origins/ray_directions are local-frame per-ray vectors; a prim nested as a plain child of the
  real mount link inherits its pose automatically, same pattern `add_camera` already uses) before writing any
  code, not guessing from the API surface alone.
- **Real, confirmed bug found in this "experimental"-namespace API itself, not this project's code**: initial
  implementation (mirroring `IMU_READ_SCRIPT`'s proven lazy-creation pattern, reading `get_data()['depths']`)
  produced a scan where every single ray -- regardless of direction, including ones pointed at open air -- came
  back reporting exactly `min_range`, or in some test configurations exactly `max_range`, but never anything in
  between and never varying per-ray. Root-caused through a sequence of isolating tests (single down-ray alone;
  a 3-ray down/forward/up probe; moving the "interesting" ray to different array indices) rather than guessing:
  a 3-ray test (down/forward/up) showed `depths=[0.1, 0.1, 0.1]` for all three, but the *same reading's*
  `hit_positions` were `[[0,0,-0.1], [3.77,0,0], [0,0,0]]` -- correctly different per ray (a real 0.1m hit
  straight down, a real ~3.77m hit forward on scene geometry, a genuine no-hit straight up). This proves
  `depths` itself is broken in this build (always echoing something like `min_range` rather than a genuine
  per-ray distance) while `hit_positions` is computed correctly.
- **Fix**: `LIDAR2D_READ_SCRIPT` now computes each ray's range as the Euclidean norm of its own
  `hit_positions` entry (`output_frame="SENSOR"`, the default, keeps this in the same local frame
  `ray_origins`/`ray_directions` already use, so the norm is directly the range in metres, no extra transform
  needed) instead of trusting `depths` at all. A `hit_positions` entry of exactly `[0,0,0]` means no hit
  (confirmed live: the "up" probe ray, a genuine miss, reported exactly that), remapped to `+Inf` per REP-117
  rather than `0.0`.
- **Verified live on `a200_0333`'s real Hokuyo UST** (541-ray fan, -135°..+135°, 0.1-10m range, matching the
  real robot.yaml's own declared FOV): the full scan now shows genuine, varied finite ranges (209/541 rays hit
  something, 1.36m-9.99m), matching the scene's own known geometry (its target box at ~3.2-3.3m, the far wall
  approaching ~9.99m near the range limit) -- not a uniform placeholder value. No FATAL, no articulation errors,
  the other 3 robots (no lidar2d_link) unaffected, all still drive correctly. `FLEET_DEBUG=1` showed
  `rtf=0.55, render_fps=12.1` with the full 4-robot fleet including this lidar active -- not isolated from
  camera rendering's own already-documented dominant cost, so not a clean read on the lidar's own marginal fps
  price specifically; worth a dedicated measurement if performance becomes a concern.
- **Not yet done, natural next steps, not attempted this session**: wiring this same (now proven) mechanism into
  `j100_0936` once its own `robot_data` folder is available (its SICK LMS1xx would use the exact same
  `LIDAR2D_READ_SCRIPT`/`lidar2d_link` machinery already built for `a200_0333`'s Hokuyo, just a different
  `lidar2d_link` name to wire into `MODEL_PARAMS`); 3D lidar (`a200_0333`'s Velodyne VLP16, `add_lidar3d` still
  a documented no-op) -- likely fixable the same way (same `RaycastSensor` API, same `hit_positions`-based
  workaround should apply), but a much bigger jump in ray count (thousands vs. hundreds) and a different message
  type (`sensor_msgs/PointCloud2`, not `LaserScan`), so treated as separately-sized work, not started here.
- Files touched: `sim/scripts/setup_scene.py` (`LIDAR2D_READ_SCRIPT` rewritten to use `hit_positions` instead of
  the broken `depths`, `LIDAR2D_*` constants, `Lidar2dRead` wiring in `build_ros_graph`, removed the old
  documented-no-op `add_lidar2d` function and its call site).

## Addendum: 3D lidar (Velodyne VLP16) — a real crash, root-caused and fixed, not a ray-count problem

User: "fix the VLP16 too" — extending the just-proven `RaycastSensor` mechanism from 2D lidar to `a200_0333`'s
real Velodyne. Reused the same `hit_positions`-based range computation (already proven correct; the `depths`
field bug applies here too) and the same lazy-creation ScriptNode pattern, publishing `sensor_msgs/PointCloud2`
instead of `LaserScan` (16 channels x 360 horizontal steps = 5760 rays, real VLP16 ±15° vertical FOV).

- **The very first attempt segfaulted the entire `a300-isaac-sim` container** (`docker ps -a` showed `Exited
  (139)`, `docker logs` showed a genuine `Segmentation fault (core dumped)` with a crash-reporter minidump) —
  a serious regression, not a benign error. Immediately disabled the new code (`if params.get("lidar3d_link")
  and False:`) and restarted to confirm the rest of the 4-robot fleet still came up cleanly before doing
  anything else, per this project's own practice of restoring a known-good baseline before investigating
  further.
- **Root-caused rather than assumed to be a ray-count problem**: the actual log line just before the crash
  (only visible on a closer read, not the first thing grepped for) was `Error in ... UsdStage::
  _ValidateEditPrimAtPath ... 'Cannot create prim at path .../lidar3d_0_laser/lidar3d_0_laser_raycast;
  authoring to an instance proxy is not allowed.'` — `a200_0333`'s `sensor_arch` mount subtree (the VLP16's
  own real ancestor chain) is USD-instanceable, unlike `lidar2d_0_laser`'s bracket mount, which is why 2D lidar
  never hit this. The retry-on-exception pattern (proven safe for IMU/2D lidar, where "not ready yet" is a
  genuinely transient condition) kept re-attempting the *same, permanently-failing* authoring call every single
  tick against an unrecoverable error — almost certainly what actually crashed the process, not the 5760-ray
  count itself. Confirmed this directly rather than just theorizing: once the instance-proxy issue was fixed
  (below), the exact same 5760-ray configuration was retested and is completely safe — ruling out ray count as
  a contributing factor at all, not just "a smaller count happens to work."
- **Fix**: the raycast sensor prim is no longer parented directly under the real `lidar3d_link` (the instanced
  one); it's parented under `chassis` instead (the articulation root -- never instanced, already the reference
  frame every drive/odometry node in this project targets). `build_ros_graph` computes `lidar3d_link`'s real
  pose *relative to chassis* once, live, via `UsdGeom.Xformable(...).ComputeLocalToWorldTransform()` on both
  prims (both are static, real rigid links, so this relative offset never changes as the robot drives) and
  passes it to `Raycast.create()` as explicit `translations`/`orientations`, instead of relying on parent-child
  nesting to place the sensor the way 2D lidar (and the IMU/camera before it) could.
- **Verified live, incrementally, after the fix** (all with the real 4-robot fleet running, watching for both
  crashes and correct data at each step, not just jumping straight back to 5760): 576 -> 1152 -> 3200 -> 4800 ->
  5760 rays, each one a clean boot with no FATAL and genuine point data. At 576 rays: real varied (x,y,z)
  values decoded from the raw `PointCloud2` buffer, tracing an actual ground-hit ring pattern from the lowest
  (-15°) channel — correct 3D lidar geometry, not degenerate output. At the full 5760: ~3000/5760 rays hit
  something (a realistic ratio, given open sky/long empty directions), all 4 robots still drive correctly, no
  articulation or authoring errors. `FLEET_DEBUG=1`: `render_fps` 12.1 -> 10.4 (2D lidar alone -> +3D lidar at
  full resolution) -- a real but modest ~14% additional cost on top of the scene's already camera-bound
  baseline, not measured in isolation from that dominant cost.
- Files touched: `sim/scripts/setup_scene.py` (`LIDAR3D_*` constants, `LIDAR3D_READ_SCRIPT`, `Lidar3dRead`
  wiring in `build_ros_graph` including the chassis-relative-offset computation, removed the old
  documented-no-op `add_lidar3d` function and its call site).

## Addendum: physical properties questions, then a deliberate +10kg chassis mass override for j100_0921

User asked for `j100_0921`'s component masses, then for its overall centre of gravity in the `base_link` frame,
then to increase the chassis mass by 10kg. All three answered/implemented directly from the real generated data
rather than estimated.

- **Component masses**: extracted directly from every `<link><inertial><mass>` in the flattened
  `sim/assets/j100_0921/j100_0921.urdf` (real Clearpath/Kinova values from the actual generator output, not
  guessed) — chassis 16.523kg, each wheel 0.477kg, the Kinova Gen3 Lite arm+2F Lite gripper ~5.19kg total across
  its 11 links, D405 camera 0.072kg; total declared mass 23.691kg. Flagged which links have **no** declared
  mass at all (`top_mount_link`, `gps_1/2_link`, camera sub-frames, decorative `bar_*`/`unloader_wall_*`/
  antenna links) — Isaac's importer falls back to a tiny "small sphere approximated" synthetic mass for these
  at import time (the same "possibly invalid inertia tensor... negative mass" warnings already documented
  elsewhere), not a real physical value, so they were excluded from any real mass total.
- **Centre of gravity**: computed properly as a real mass-weighted average using live TF (`base_link -> <link>`
  for every massed link, accounting for the arm's actual current joint angles) combined with each link's real
  local CoM offset from the URDF, not a rough estimate. Hit a real, non-obvious gotcha getting there: a plain
  `tf2_ros.TransformListener` subscribes to the **standard, unnamespaced** `/tf`/`/tf_static` topics by default,
  but this project's own convention (documented in README/CLAUDE.md, needed for multiple robots to coexist) is
  namespaced `/<robot>/tf`; the listener silently received nothing until the node was constructed with explicit
  `cli_args=['--ros-args', '-r', '/tf:=/j100_0921/tf', '-r', '/tf_static:=/j100_0921/tf_static']`, the same
  remap `robot/bin/rviz`'s own wrapper script already uses for exactly this reason. Reported both the full-
  system CoG at the arm's current pose (confirmed live via `platform/joint_states` to be sitting at its
  kinematic zero, not an arbitrarily extended test pose) and a pose-independent platform-only figure (chassis +
  wheels + IMU, excluding the arm) as the more generally useful reference, since the arm's ~22% mass share
  shifts the full-system figure substantially with joint angle.
- **+10kg chassis mass override**: the chassis mass comes from Clearpath's own installed
  `clearpath_platform_description` xacro (`clearpath_generator_common`'s own generated output), not from
  anything in the robot's `robot.yaml` -- so there was no config field to just edit. Implemented as a new,
  general (not `j100_0921`-specific) `apply_mass_deltas` pass in `scripts/flatten_urdf.py` (`link_name:delta_kg`
  spec, e.g. `"chassis_link:10"`), wired up per-model in `scripts/gen_urdf.sh` (only `j100_0921` gets it,
  clearly commented as a deliberate what-if experiment, not a hardware fact). Inertia tensor is deliberately
  left unchanged -- recomputing it correctly would need to know the added mass's own shape/distribution, which
  isn't modeled here; disclosed as a simplification, not presented as a physically exact reballast. Verified
  live: `chassis_link: mass 16.523 -> 26.523 kg` printed during generation, confirmed in the flattened URDF,
  total declared mass now 33.691kg (exactly +10 from the earlier 23.691kg baseline), fresh USD re-import
  (`FORCE_REIMPORT=1`) picked it up, drive test still passes, and `j100_0922`'s own chassis mass confirmed
  unaffected (16.523kg, unchanged) -- the override is genuinely scoped to just this one robot.
- Files touched: `scripts/flatten_urdf.py` (`apply_mass_deltas`, `main()`'s new `mass_overrides` parameter),
  `scripts/gen_urdf.sh` (per-model override wiring for `j100_0921`).

## Addendum: real Clearpath platform/manipulator param generation, found by re-checking rather than trusting an earlier "no equivalent exists" conclusion

User: "In the actual Clearpath robot, it generates configuration files and launch files based on the robot.yaml
file using clearpath_generator_common package. In each robot container, generate[] those files so that I can
run them later." An earlier session's own documented finding (CLAUDE.md: "There is no generate_launch/
generate_params-equivalent package in the public Jazzy apt repo") was re-checked live rather than trusted at
face value, matching the lesson from the RaycastSensor investigation two addenda ago -- and this time the
earlier conclusion was half right, not fully right.

- **Re-verified `clearpath_generator_common`'s own installed console_scripts first** (`ros2 pkg executables`):
  still only `generate_bash`/`generate_description`/`generate_discovery_server`/`generate_semantic_description`/
  `generate_vcan`/`generate_zenoh_router`/`moveit_collision_updater` -- confirmed no `generate_launch`/
  `generate_params` executable exists, and a broader sweep of *every* installed ROS2 package's own executables
  for anything matching found nothing either. The earlier conclusion about console_scripts was correct.
- **What the earlier investigation missed**: it only looked for wrapped console_scripts, not the underlying
  Python package's own modules. `clearpath_generator_common` also ships `param/generator.py`
  (`ParamGenerator`) and `launch/generator.py` (`LaunchGenerator`) as plain importable classes, never wrapped in
  a `ros2 run` entry point in this apt package at all -- found by listing every `.py` file under the installed
  package and reading the ones that looked relevant, not by searching for a name that had already failed once.
- **`ParamGenerator` is genuinely usable, confirmed live**: `generate_platform()` and `generate_manipulators()`
  are fully implemented (only `generate_sensors()` raises `NotImplementedError` in this installed version --
  confirmed by reading the class, not assumed) and produce real, correct output: ran it directly inside
  `j100_0921`'s own container and got 11 real files under `/etc/clearpath/{platform,manipulators}/config/`
  (`control.yaml`, `diagnostic_aggregator.yaml`, `diagnostic_updater.yaml`, `foxglove_bridge.yaml`,
  `imu_filter.yaml`, `localization.yaml`, `teleop_interactive_markers.yaml`, `teleop_joy.yaml`,
  `twist_mux.yaml`, plus `manipulators/config/{moveit,control}.yaml`). `platform/config/control.yaml`'s own
  `diff_drive_controller` parameters independently confirm the exact same real calibrated values this project's
  own `MODEL_PARAMS["j100_0921"]` already carries by hand (`wheel_separation_multiplier: 1.17`,
  `left/right_wheel_radius_multiplier: 0.95`, `linear.x.max_velocity: 1.0`) -- a nice cross-check that this
  project's own hand-derived numbers were right all along. Also confirmed live on a robot with **no** arm
  (`a300_00036`): `generate_manipulators()` doesn't error out, it just writes a harmless, empty
  `controller_manager` skeleton with no actual joints/controllers listed.
- **`LaunchGenerator` is a dead end, confirmed by reading it, not assumed from its name**: every one of its
  three generation methods (`generate_sensors`/`generate_platform`/`generate_manipulators`) is a bare
  `raise NotImplementedError()` -- it's an abstract base meant to be subclassed by a downstream/private
  bringup package this public apt repo doesn't ship, exactly matching the earlier session's original finding
  for the *launch* side specifically (that part of the old conclusion holds up). This doesn't block much in
  practice, though: the real launch files it would have generated thin wrapper scripts for
  (`clearpath_control/launch/control.launch.py`, `clearpath_platform_description/launch/description.launch.py`,
  `clearpath_manipulators/launch/{manipulators,control,moveit}.launch.py`) are themselves static, generic,
  already-installed files that take `setup_path`/`namespace`/etc. as plain launch arguments -- they were
  already directly runnable via `ros2 launch <pkg> <file>.launch.py setup_path:=/etc/clearpath/ namespace:=<ns>
  ...` once their param dependencies exist, which is exactly what `ParamGenerator` now provides.
- **Implemented as `robot/bin/generate_params`**, a small one-shot script (not a persistent background service,
  unlike `robot_state`/`foxglove` -- this only needs to run once, like `generate_bash`) calling
  `ParamGenerator(setup_path='/etc/clearpath/').generate_platform()` + `.generate_manipulators()` directly
  (never `.generate()`, which would also call the unimplemented `generate_sensors()` and crash). Wired into
  `robot/entrypoint.sh` right after `generate_bash`'s own output is sourced, so it runs unconditionally for
  every model (real or generic, arm or not) as part of the real boot sequence, not as a separate manual step.
  Verified live end-to-end through an actual image rebuild + full 4-robot `--force-recreate` (not just a manual
  one-off exec): all 4 containers show the real "Generated config: ..." lines in their own boot logs, no
  tracebacks (one harmless `ros-jazzy-clearpath-firmware package not found` warning, expected -- that's a
  real-hardware-only package, not installed or needed here), and `drive_test.py` still passes afterward --
  confirming this addition doesn't interfere with anything already working.
- **Important caveat, disclosed rather than left for the user to discover by trying it**: these generated files
  let the user run Clearpath's own real launch files, but actually doing so would **not** replace or integrate
  with this project's own drive/sensor pipeline -- `control.launch.py` starts a real `ros2_control_node` +
  `diff_drive_controller` expecting a real (or simulated) `ros2_control` hardware interface plugin, which this
  project doesn't have at all (this sim drives wheels via its own OmniGraph nodes writing straight to Isaac's
  PhysX joint drives, bypassing ros2_control entirely) -- so running it wouldn't make the robot move, and would
  likely fight over shared topic names (`platform/odom`, `platform/joint_states`, `cmd_vel`) with what already
  works. Framed to the user as "for inspection/reference, not a drop-in replacement," not silently left implicit.
- Files touched: `robot/bin/generate_params` (new), `robot/entrypoint.sh` (call site), `robot/Dockerfile`
  (`chmod +x` for the new script).

## Addendum: the sensors/launch gap above was real but not permanent -- `clearpath_generator_robot` (from the `clearpath_robot` GitHub repo) closes it

User, immediately after the above: "the sensor parameter will be generated by clearpath_robot package. check it"
-- pointing at a specific package name I hadn't looked for. Checked rather than deferred, matching this
project's own established practice of re-verifying a named claim instead of assuming the prior addendum's
"genuine, confirmed gap" conclusion was the end of the story.

- **Confirmed via a scratch clone** (`git clone --depth 1 -b jazzy https://github.com/clearpathrobotics/clearpath_robot.git`,
  inspected then discarded): `clearpath_robot` is a real, public, source-only repo (not in the apt index --
  only `clearpath_generator_common` is) with six packages. `clearpath_generator_robot` subclasses both of
  `clearpath_generator_common`'s abstract bases: `RobotParamGenerator(ParamGenerator)` has a genuinely
  implemented `generate_sensors()` (reads `sensors.get_all_sensors()`, filters to `get_launch_enabled()`, and
  for each one instantiates `SensorParam` -- which needs a *second* package, `clearpath_sensors`, for its
  default per-sensor-type param templates via `get_package_share_directory('clearpath_sensors')`), and
  `RobotLaunchGenerator(LaunchGenerator)` likewise has all three methods genuinely implemented (not the
  bare-abstract dead end the base class is). Both base classes also have a `generate()` aggregate
  (`generate_sensors()`+`generate_platform()`+`generate_manipulators()` in one call) that only the *subclasses*
  can safely use, since the base versions still raise on sensors.
- **First attempt failed, second one revealed the right class name**: importing
  `from clearpath_generator_robot.param.generator import ParamGenerator` resolved to the *base* class (an
  inherited/re-exported name collision), not the real subclass -- reading the actual source file directly (not
  guessing from the module path) showed the real name is `RobotParamGenerator`.
- **`package.xml` for both new packages lists many genuinely-unavailable exec_depends** (`clearpath_hardware_interfaces`,
  `micro_ros_agent`, `canopen_inventus_bringup`, `realsense2_camera`, `velodyne_driver`, `zed_wrapper`,
  `phidgets_spatial`, ...) -- verified harmless before relying on it, not assumed: `colcon build` only checks
  `build_depend`s, neither package.xml declares any, and the actual code (`param/generator.py`,
  `param/sensors.py`, `launch/generator.py`, `launch/sensors.py`) only imports `clearpath_config`/
  `clearpath_generator_common`/its own sibling modules -- confirmed by a real successful build+run in a
  throwaway container before touching the actual project.
- **Integrated into the real image** (`robot/Dockerfile`): clones `clearpath_robot` (jazzy branch, shallow) at
  build time, copies out only `clearpath_generator_robot`+`clearpath_sensors` (the other four packages are
  real-motor-controller/CAN-hardware-only, not needed), and `colcon build`s them into a separate
  `/opt/clearpath_robot_ws` overlay (kept apart from the runtime `colcon_ws` bind mount, since this is baked
  into the image, not user workspace content). Needed `bash -c "source ... && colcon build"` rather than a bare
  `RUN ... && . setup.bash && ...` -- Docker's default `RUN` shell is `/bin/sh` (dash), which can't parse
  `setup.bash`'s bash-only syntax (`Bad substitution`), caught immediately by the build failing, not assumed
  away. `entrypoint.sh` now sources this overlay (after `/etc/clearpath/setup.bash`, before calling
  `generate_params`); `/etc/ros_env.sh` (the file every `docker exec`/bashrc shell already sources) got the same
  line added, so an interactive shell can also `python3 -c "from clearpath_generator_robot..."` directly.
- **`robot/bin/generate_params` rewritten** to import `RobotParamGenerator`/`RobotLaunchGenerator` from
  `clearpath_generator_robot` instead of the base classes from `clearpath_generator_common`, and call each
  one's `.generate()` (now safe, since sensors is genuinely implemented in these subclasses) instead of calling
  `generate_platform()`/`generate_manipulators()` individually.
- **Verified live across all 4 currently-configured real robots** (image rebuild + `--force-recreate` on all
  four, not just one): `j100_0921`/`j100_0922` each produced real `sensors/config/{camera_0,imu_1}.yaml` +
  matching `launch/*.launch.py` (ZED2i + Microstrain IMU; GPS correctly absent, both `swiftnav_duro` entries
  have `launch_enabled: false`), `a300_00036` produced `imu_0.yaml` only (Phidgets Spatial, no camera --
  correctly reflects it having none), `a200_0333` produced `camera_0.yaml`+`lidar2d_0.yaml`+`lidar3d_0.yaml`
  (D435, Hokuyo UST, Velodyne VLP16). No tracebacks in any of the 4 boot logs; `drive_test.py` on `j100_0921`
  still passes afterward, confirming no regression to the working drive pipeline.
- **Same caveat as the platform/manipulator params still applies, now also to sensors**: these are real,
  runnable Clearpath launch files, not something this project's own pipeline consumes -- they default
  `use_sim_time=false` (hardcoded in the base `LaunchGenerator.__init__`) and would drive real sensor driver
  nodes (`realsense2_camera`, `microstrain_inertial_driver`, ...) this image doesn't have installed and that
  would conflict with this project's own OmniGraph-published topics if it did. For inspection/reference only.
- Scratch clone (`/tmp/clearpath_robot`) was on the host filesystem, not the session scratchpad, and has been
  deleted now that its contents are copied into the real Dockerfile step -- nothing outside the project
  references it any more.
- Files touched: `robot/Dockerfile` (clone+build step, `/etc/ros_env.sh` overlay source line),
  `robot/entrypoint.sh` (source the new overlay before `generate_params`), `robot/bin/generate_params`
  (rewritten to use `RobotParamGenerator`/`RobotLaunchGenerator`), `CLAUDE.md` (superseded the "sensors gap"/
  "LaunchGenerator dead end" paragraph with this finding).

## Addendum: real EKF localization, then a full real MTU bringup launch file, made to actually run against the sim

User asked to actually run some of the generated real launch tree "like the real robot does," flagging that
some of it directly touches hardware or topics with no sim source (e.g. battery status). Scoped down via two
direct questions: hardware-only telemetry with no sim source (battery, wifi, diagnostics) gets excluded cleanly,
no fake stubs; real `ros2_control` (wheel/arm) also excluded for now, since OmniGraph is already the working
substitute and a real sim hardware-interface bridge is a separate, larger, unscoped follow-up. The one thing
wanted for real: EKF, since it only reads `platform/odom`, which the sim genuinely publishes.

- Before I could finish proposing my own wrapper launch file for this, **the user had already written their
  own** (`colcon_ws/src/mtu32_husky/mtu32_bringup/launch/sim_robot_upstart.launch.py`, MTU's own private
  package): a single `robot_localization` `ekf_node`, reading the real generated `localization.yaml`, with a
  `{'publish_tf': False}` override -- independently arriving at the same design (including the same TF-conflict
  reasoning) my own not-yet-approved plan had. Treated as the canonical version going forward; my own draft
  plan file was abandoned.
- **Launching it hit a real error**: `ModuleNotFoundError: No module named 'nav2_common'` (imported for
  `RewrittenYaml`, unused in this file but genuinely used across most of `mtu32_bringup`'s *other* launch files
  -- confirmed by grepping the whole package, not just patched around in this one file). Fixed by adding
  `ros-jazzy-nav2-common` to `robot/Dockerfile`.
- **A second, silent bug found while verifying the fix actually did something** (checked `ros2 param get` live
  rather than trusting a clean launch log): the `ekf_node`'s `Node()` action never set `name='ekf_node'`, so it
  ran under `robot_localization`'s own default name `ekf_filter_node` -- silently mismatched against the
  generated `localization.yaml`'s own node-name-keyed structure (`j100_0921: ekf_node: ros__parameters: ...`),
  so none of the real per-robot values (`frequency: 50.0`, `odom0: platform/odom`) ever actually loaded; it ran,
  publishing nothing useful, with the stock `robot_localization` defaults instead (`frequency` reads back as
  `30.0`). Fixed with a one-line `name="ekf_node"` addition to the user's file. Re-verified live after rebuilding
  `mtu32_bringup`: `odom0`/`frequency`/`publish_tf` all read back correctly, `platform/odom/filtered` actually
  publishes at ~28Hz, and `/tf` publisher count is unaffected (three publishers before and after: the sim's own
  OmniGraph one, `robot_state_publisher`, and `ekf_node`'s own inert-but-declared handle) -- confirming
  `publish_tf: False` genuinely suppressed the broadcast rather than just being accepted as a parameter.
  (Along the way, cleaned up several of my own leftover test processes from repeated `docker exec` launches that
  briefly caused real duplicate-node-name warnings -- a self-inflicted testing artifact, not a bug in anything
  real, resolved with a plain container restart before the final clean verification run.)

Immediately after, user asked to make `sim_robot_upstart.launch.py` "the main launch file for the simulation
robot," add an `IncludeLaunchDescription` for `bringup_main.launch.py` (a much bigger MTU launch file: laser
scan filtering, depth-to-laserscan, AprilTag detection, dock pose TF, point-cloud filtering, a "grid cutter"
MoveIt action server, a pruner action server, gamepad-driven stem cutting, and MoveIt `move_group`+`servo_node`
after a delay) into it, and pointed at `robot_data/j100_0921/colcon_ws/src` (that real robot's own actual,
working colcon workspace snapshot) as the place to find whatever packages were missing, to be copied into the
shared, bind-mounted `colcon_ws/src/`.

- **10 packages copied verbatim from `robot_data/j100_0921/colcon_ws/src/`** into `colcon_ws/src/`:
  `mocap_fake_localizer`, `docking_utils`, `stow_arm_cpp`, `pruner_action_server`, `kinova_game_pad` (MTU-custom
  code), plus `laser_filters`, `depthimage_to_laserscan`, `apriltag_ros`, `plant_cutter_msgs`,
  `serial_interfaces` (forked/vendored public packages) -- copied from source rather than substituted with apt
  equivalents deliberately, to stay version/config-consistent with the launch/config files actually written
  against them (the user's own instruction: copy from that specific folder, not "find any equivalent"). Found by
  reading `bringup_main.launch.py` and its own further includes (`moveit.launch.py`, `pcl_filter.launch.py`) for
  every `package=`/`get_package_share_directory()` reference, then cross-checking each new package's own
  `package.xml` `depend`s against what else `robot_data`'s src tree had, catching `plant_cutter_msgs`/
  `serial_interfaces` as transitive needs that weren't in the first obvious pass.
- **7 more packages added to `robot/Dockerfile`'s apt list**, found only by letting `colcon build` fail and
  reading its real errors rather than trying to pre-derive the full dependency tree by hand: `pcl_ros`,
  `moveit_ros_move_group`, `moveit_servo` (generic MoveIt2/PCL framework packages `bringup_main`/`moveit`/
  `pcl_filter`.launch.py need, absent from `robot_data`'s own src tree -- confirming the real robot installs
  these from apt too, not vendored), then `apriltag`/`apriltag-msgs`/`camera-ros` (apriltag_ros's own missing
  CMake deps, caught by an actual `CMake Error` during the colcon build), then `python3-serial` (a plain,
  non-ROS Python dependency -- `pruner_action_server`'s own node does `import serial` -- invisible to colcon
  build entirely, only surfaced as a runtime `ModuleNotFoundError` once the node actually started).
- **Launching the combined file hit one more real, structural blocker beyond missing packages**:
  `moveit.launch.py` (included via `bringup_main.launch.py`, after a delay) reads `/etc/clearpath/robot.srdf`
  directly and failed with a plain "No such file or directory" -- this file had simply never been generated by
  anything in this project. Root cause: `clearpath_generator_common`'s own `generate_semantic_description`
  console_script (the tool that would normally write it) was already known, from a much earlier session, to
  "crash with an Eigen assertion" and was never run -- re-investigated rather than left as a permanent
  limitation, since the user's new ask now genuinely needed it. **Reproduced live, twice** (once via the
  stand-alone `generate_semantic_description` script, once via its own inner `moveit_collision_updater` binary
  directly): the crash is a real, reliable `stack smashing`/segfault inside `moveit_collision_updater`'s
  exhaustive `--trials 100000` random-sampling self-collision search, not an occasional flake, and not
  model-specific -- reproduced on both an armed robot (`j100_0921`) and an armless one (`a300_00036`).
  **Found a real, working fix by reading the tool's own `--help`**: `--default --always` (disable exactly the
  two colliding-pair search categories that crash) plus `--trials 1` avoids the crash entirely and still
  produces a complete, valid SRDF (confirmed by inspection: real planning groups, group states, hundreds of
  real `disable_collisions` entries from the cheap adjacent-link check, which doesn't need the crashing random
  search). Implemented as a new script, `robot/bin/generate_srdf` (run at boot, right after `generate_params`),
  reimplementing `generate_semantic_description`'s own two-step logic with the safe flags instead of calling the
  crashing stock script. Two more real things this new script had to handle, both caught by testing an actual
  fresh container boot rather than trusting a manual mid-session run: (1) `/etc/clearpath/robot.urdf.xacro`
  doesn't exist yet at the point `entrypoint.sh` would run this -- it's normally written by `robot_state`'s own
  background loop, which starts *after* this in the boot sequence -- fixed by having `generate_srdf` run
  `generate_description` itself first, redundant with `robot_state`'s own later run but harmless and
  order-independent; (2) `j100_0922` (the armless twin, whose xacro has the already-known dangling
  `camera_1_joint`→`arm_0_end_effector_link` reference `scripts/flatten_urdf.py`'s `prune_dangling_joints`
  already fixes for the *sim's* flattened URDF) hit the exact same dangling-joint problem again here, since this
  is a completely different, freshly-xacro'd URDF that pass never touches -- `moveit_collision_updater`'s own
  strict URDF loader aborts on it (unlike plain `xacro`, which never validates). Fixed by re-implementing the
  same `prune_dangling_joints` logic inline in `generate_srdf` (not imported -- `flatten_urdf.py` is a host-side
  build script, not installed in this image) before handing the cleaned URDF to `moveit_collision_updater`.
  Verified live: all 4 currently-configured real robots (`j100_0921`, `a300_00036`, `a200_0333`, `j100_0922`)
  now get a real, non-empty `/etc/clearpath/robot.srdf` at every boot, with zero tracebacks/aborts in any of
  their logs.
- **Full end-to-end result, verified over an actual 20+ second run** (not just a launch-and-immediately-check):
  every node in `sim_robot_upstart.launch.py` (now including `bringup_main.launch.py`) starts and stays up --
  `ekf_node`, `scan_to_scan_filter_chain`, `depthimage_to_laserscan_node`, `apriltag_node`, `tf2_pose_node`,
  `filter_crop_box_node`, `grid_cutter_action_server`, `pruner_server`, `cut_stem_gamepad_node`, `move_group`,
  `servo_node` -- `move_group`/`servo_node` both genuinely load the real robot model and initialize
  successfully (`servo_node`'s own twist-mode activation service call returns `success=True`). `drive_test.py`
  still passes afterward, confirming none of this touched the sim's own working drive pipeline. **Remaining
  warnings, deliberately left as-is, not chased**: `pruner_server` logs a clean (non-crashing) failure to open
  `/dev/ttyOpenCR` -- real pruner hardware this sim has no equivalent of, same category as every other
  real-hardware gap this session already accepted as out of scope; the `arm_0_gripper` planning group logs
  `'is not a chain'` and gets no KDL kinematics solver -- expected, a parallel-jaw gripper group doesn't need
  IK; `move_group` logs "No 3D sensor plugin(s) defined for octomap updates" -- expected, no 3D perception
  source is configured; `tf2_pose_node`'s `jackal_charger_april` TF lookup failures are expected too -- that
  frame only exists once an AprilTag dock target is actually visible in-camera, which nothing in the current
  scene provides.
- Files touched: `colcon_ws/src/mtu32_husky/mtu32_bringup/launch/sim_robot_upstart.launch.py` (the
  `bringup_main.launch.py` include, the `nav2_common`/`name="ekf_node"` fixes -- the user's own file, edited
  directly during live debugging), `robot/bin/generate_srdf` (new), `robot/entrypoint.sh` (call site, right
  after `generate_params`), `robot/Dockerfile` (`nav2_common`/`pcl_ros`/`moveit_ros_move_group`/`moveit_servo`/
  `apriltag`/`apriltag-msgs`/`camera-ros`/`python3-serial` added, `chmod +x` for `generate_srdf`), 10 packages
  copied into `colcon_ws/src/` from `robot_data/j100_0921/colcon_ws/src/` (see above), `CLAUDE.md` (new
  `generate_srdf` and `sim_robot_upstart.launch.py` paragraphs).

## Addendum: the arm accepted cut_stem goals but never actually moved -- a new moveit_sim_bridge package closes the real gap

User: "for j100_0921, the arm start to move with '...send_goal /j100_0921/cut_stem...' command. but it does
not [move]." First reproduced properly rather than trusted at face value: my own prior verification run had
left several overlapping test launches half-cleaned-up (two `grid_cutter_action_server` zombies, orphaned
`move_group`/`servo_node` still alive from a killed parent, a stale `/j100_0921/cut_stem` DDS advertisement with
no live server behind it) -- a `docker restart j100_0921` for a genuinely clean slate, then one single fresh
launch, was needed before the real behavior could even be observed.

- **Root cause, confirmed live via `ros2 action info`**: `move_group`'s own `moveit_simple_controller_manager`
  plugin is a real, permanent *client* of `/j100_0921/manipulators/arm_0_joint_trajectory_controller/
  follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) -- 0 action servers existed for it.
  This project's arm has no real `ros2_control` hardware interface at all (Isaac's own OmniGraph drives it
  directly via `arm_0/joint_command`, position-mode, per `configure_arm_drives`/the "Arm + gripper" section in
  `sim/scripts/setup_scene.py`) -- so every `MoveGroupInterface::execute()` call failed instantly (~150ms, far
  too fast to be real motion) with error code -4 (`CONTROL_FAILED`), confirmed by reading `move_group`'s own log
  line by line rather than assuming from the action's outcome alone.
- **Checked with the user before building new infrastructure**: this is exactly the "real sim hardware-interface
  bridge" the earlier EKF-launch plan-mode session explicitly scoped *out* as "a separate, larger, unscoped
  follow-up." Since the user's actual current goal genuinely needs *some* execution path to exist, asked
  directly rather than silently reversing that decision or silently doing nothing -- confirmed: build a small,
  scoped bridge (not a full ros2_control `SystemInterface` plugin).
- **New package: `colcon_ws/src/moveit_sim_bridge/`** (ament_python, project-owned -- not MTU's own code, since
  this is purely a sim-side compensation, unlike everything else copied from `robot_data` this session), one
  node with three pieces, two of which were only discovered necessary by testing the actual end-to-end goal
  rather than stopping at the first fix:
  1. A `FollowJointTrajectory` action server at the exact name above, publishing each waypoint as a
     `sensor_msgs/JointState` position command to `arm_0/joint_command`, paced by the waypoint's own
     `time_from_start` (a disclosed simplification: time-based, not feedback-convergence-based -- it trusts
     `IsaacArticulationController`'s own already-tuned position servo to actually get there, the same way the
     real hardware's own controller would, rather than re-implementing tolerance/settling logic itself).
  2. A `GripperCommand` action server at `.../arm_0_gripper_controller/gripper_cmd`, commanding all 4 of the
     Kinova 2F Lite's real finger joints (from `robot.srdf`'s own `arm_0_gripper` group) to one shared position
     -- no mimic-joint solver, overridable via a `gripper_joint_names` parameter.
  3. **Found only after (1) alone still left something broken**: with just the action server built and verified
     (arm genuinely reached its first `move_group`-planned pose -- `platform/joint_states` for `arm_0_joint_1`/
     `_2` moved from ~0 to -0.51/-1.37 rad, confirmed by reading actual joint state before/after, not just
     trusting a log line), `grid_cutter_action_server`'s own later *servo*-based fine-approach phase still timed
     out ("servo_to_pose timed out after 15.0s"). Read `mtu32_bringup`'s own `servo_config.yaml` rather than
     guessing: `moveit_servo`'s `command_out_type: trajectory_msgs/JointTrajectory` means it streams single-point
     trajectories directly onto a **plain topic** (`command_out_topic`, 100Hz), never through the action at all
     -- a completely separate output path nothing was listening on. Fixed with a third piece: a plain
     subscriber on that same topic, forwarding each single-point message straight to `arm_0/joint_command` the
     same way.
- **Wired into `sim_robot_upstart.launch.py`** (the user's own main launch file) as another `Node()` alongside
  the already-present `ekf_node`, in the same `load_nodes` `GroupAction`.
- **Verified live, end to end, after both fixes**: sent the exact same goal from the user's own report
  (`ros2 action send_goal /j100_0921/cut_stem plant_cutter_msgs/action/CutStem "{start_cutting: true}"`) against
  a freshly-restarted container running one single clean launch; `grid_cutter_action_server` now logs genuine
  `Execute request success!` / `Reached 'pose ...'` for its planned poses, confirmed independently via the raw
  `platform/joint_states` topic actually changing. `drive_test.py` still passes afterward.
- **The action's own overall "patch" still comes back failed, for a completely different and already-accepted
  reason, not a regression or a remaining bug in this new bridge**: each patch's own next step calls
  `pruner_server` (`Received goal to send integer: 42`), which fails with `Cannot send data. Serial port is not
  open` -- the exact same real pruner-hardware gap already documented in the previous addendum (no
  `/dev/ttyOpenCR` device in this container), unrelated to arm motion, not chased further this pass.
- Files touched: `colcon_ws/src/moveit_sim_bridge/` (new package: `package.xml`, `setup.py`, `setup.cfg`,
  `moveit_sim_bridge/bridge_node.py`), `colcon_ws/src/mtu32_husky/mtu32_bringup/launch/
  sim_robot_upstart.launch.py` (new `Node()` entry), `CLAUDE.md` (new `moveit_sim_bridge` paragraph, updated the
  prior paragraph's "remaining warnings" list now that the arm-motion gap it once listed is fixed).

## Addendum: the arm moved, but the gripper still didn't -- a persistent-joint-state bug in the new bridge, plus real physical fallout from testing the old one

User: "regardless 'pruner_server' status, the gripper should open and close. but the gripper does not move." --
correctly separating the already-accepted pruner-hardware gap from a second, genuinely separate defect,
immediately after the arm-motion fix above.

- **Isolated the claim rather than trusting it**: a raw, direct `GripperCommand` goal (nothing else running)
  genuinely closed the gripper -- confirmed via `platform/joint_states`, not just the action's own reported
  `reached_goal: true` -- and it *stayed* closed indefinitely while polled. So the bridge's gripper handling
  worked in isolation; the bug had to be in how it interacted with everything else running during a real
  `cut_stem` sequence.
- **Reproduced against the real sequence, polling joint states every second rather than just reading the
  action's own result**: the gripper opened right on cue (`try_prune_once`'s own "open the gripper before
  approaching" step), but the instant the *next* arm-only command arrived (a `moveit_servo` stream point, or
  another trajectory goal), the gripper silently reopened -- not eventually, immediately, every time.
- **Root cause**: `arm_0/joint_command`'s own subscriber (`ROS2SubscribeJointState`/`IsaacArticulationController`,
  the same mechanism `setup_scene.py`'s own comment already documents as "holds its last-received message's
  values between messages... re-applies them every tick") only ever remembers the *last full message's* own
  name/position arrays -- it does not merge across messages. The bridge (from the previous addendum) was
  publishing separate arm-only messages (trajectory waypoints, servo stream points) and gripper-only messages
  (open/close), so any arm-only message wholesale replaced the entire commanded set and dropped the gripper's
  just-closed target the instant it arrived. Not a rare race: `try_prune_once` always calls another arm servo
  move immediately after closing the gripper, so this was guaranteed to happen on every single patch attempt.
- **Fixed by keeping the bridge's own persistent last-known position for every joint it has ever been told
  about, and publishing all of them together on every update** (a small `dict` merge, lock-protected since the
  action servers and the servo-topic subscriber can all run concurrently under the `MultiThreadedExecutor`) --
  a gripper command can no longer be silently overwritten by an unrelated arm command, or vice versa.
- **A second, unrelated thing found while re-verifying**: repeated testing of the *pre-fix* bridge had left the
  arm/gripper in a genuinely corrupted physical state -- gripper joints reading wildly out of range (e.g. 2.5
  rad, well outside any sane open/close value) and an arm joint stuck ~2.3 rad away from a commanded 0 and not
  converging even when correctly re-commanded, confirmed live rather than assumed. Root cause: a joint dropped
  from the commanded array (the exact bug just fixed) leaves `IsaacArticulationController` no longer actively
  driving it that tick, and depending on the position servo's own state at that instant this manifested as a
  genuinely wild, physically-real displacement, not a display/echo artifact -- confirmed by directly commanding
  the same joints back to 0 and observing them fail to visibly converge within several seconds afterward. Also
  confirmed `docker restart j100_0921` (the ROS-side container) does **not** reset this -- the robot's physical
  joint state is owned by Isaac Sim, a separate, still-running container -- only `docker restart a300-isaac-sim`
  (re-running `setup_scene.py`'s own spawn logic) actually gave the arm a clean starting pose again.
- **Verified live, on a freshly-reset arm, after both fixes**: sent the user's own exact goal again and polled
  `platform/joint_states` continuously (not just the action's own result) -- the gripper genuinely closes and
  *holds* its position through subsequent arm motion this time, matching `grid_cutter_action_server`'s own real
  open→approach→close→(pruner attempt, fails)→reopen-and-retry cycle exactly. `drive_test.py` still passes
  afterward.
- Files touched: `colcon_ws/src/moveit_sim_bridge/moveit_sim_bridge/bridge_node.py` (persistent merged joint
  state, lock-protected), `CLAUDE.md` (extended the `moveit_sim_bridge` paragraph with this fix and the
  physical-corruption/sim-restart finding).

## Addendum: the gripper held correctly now, but moved the wrong direction -- a real <mimic> joint the bridge was ignoring

User, immediately after the previous fix: "the gripper motion is reversed. it closes with openning motion
command, vice versa."

- **Read the real URDF rather than guessing at a sign flip**: `/etc/clearpath/robot.urdf`'s own gripper joints
  showed only `arm_0_gripper_right_finger_bottom_joint` is independently actuated (range -0.1 to 0.96); the
  other 3 are `<mimic>` joints of it with their own real multipliers -- `left_finger_bottom_joint` at `1.0`, but
  both finger *tip* joints at `-0.676` (their own axis is physically opposite the bottom joints', which is what
  the negative multiplier compensates for). The bridge (previous addendum) was sending `GripperCommand`'s single
  `position` value identically to all 4 joint names, never reading any of this -- so the 2 tip joints were
  always commanded with the wrong sign relative to the 1 real driven joint, which is exactly what a user watching
  the whole gripper would describe as "reversed," not a single uniform sign error that a naive `-position` fix
  would have addressed.
- **Fixed** by treating `position` as `arm_0_gripper_right_finger_bottom_joint`'s own raw value and deriving the
  other 3 from their real multipliers (`gripper_joint_multipliers`, a new parallel parameter to
  `gripper_joint_names`) instead of copying it verbatim to every joint.
- **Verified live, joint-by-joint, not just via the action's own reported result**: `position: 0.9` (close) now
  moves both bottom joints positive (~0.75, short of the full 0.9 target -- plausibly contact/settle time, not
  re-chased) and both tip joints negative (-0.5, landing exactly on that joint's own URDF lower limit of -0.50)
  -- a single, physically consistent closing motion. `position: 0.0` (open) returns all 4 to ~0.
- **Needed a fully clean physical baseline to test against, not just a fresh launch**: residual corruption from
  testing the *pre-multiplier-fix* bridge (gripper joints commanded with the wrong sign, plus the
  already-known pre-merge-fix corruption from the addendum before this one) meant `docker restart j100_0921`
  alone wasn't enough -- had to restart `a300-isaac-sim` itself again to get a trustworthy zero-ish starting
  pose before the joint-by-joint verification above could mean anything.
- Re-ran the full `cut_stem` sequence and `drive_test.py` afterward: both still work, no regression from this
  change.
- Files touched: `colcon_ws/src/moveit_sim_bridge/moveit_sim_bridge/bridge_node.py` (mimic-aware gripper
  multipliers, replacing the flat "same position to all 4 joints" logic), `CLAUDE.md` (extended the
  `moveit_sim_bridge` paragraph with this finding).

## Addendum: a fake serial device for pruner_action_server -- and the first fully successful cut_stem patch this session

User: "can you make a fake device on /dev/ttyOpenCR for pruner_server to work properly?" -- the one remaining
gap from the whole arm/gripper investigation above, now asked for directly.

- **Read `pruner_action_server`'s own `pruner_server.py` before building anything**, rather than assuming "open
  the port" would be enough: it writes `"<target_integer>\n"`, then reads lines for up to 30s waiting for a
  literal `"STATUS:DONE"`/`"STATUS:FAIL"` line before reporting the goal's own success/failure. A bare PTY with
  nothing on the other end would open fine but then time out every single call -- the stub has to actually
  *respond*, not just exist.
- **New script, `robot/bin/pruner_stub`**: pure Python stdlib, no `rclpy`/ROS dependency at all (it doesn't need
  ROS -- it only ever talks over the fake serial link). Uses `os.openpty()` to create a real PTY and symlinks
  `/dev/ttyOpenCR` to its slave side -- confirmed this needs no new system package (`socat` and similar were the
  first idea, but pyserial's own `Serial()` call opens a PTY slave exactly like a real tty, no special-casing
  needed). Reads the incoming integer command, sleeps 2s to simulate a real pruning cycle, writes back
  `"STATUS:DONE\n"` -- deliberately always succeeds, since this sim has no failure mode of its own to report.
- **Wired in as another restart-looped background service in `entrypoint.sh`**, alongside `robot_state`/
  `foxglove` (same "restarted if it dies" pattern) -- harmless on every model that never launches
  `pruner_action_server` at all, so no per-model guarding needed.
- **Verified in stages, not just "it opens now"**: first a direct `SendInteger` goal, confirming the full
  write→wait→`STATUS:DONE`→result round trip (`Command sent...` → `Goal succeeded. Pruning confirmed.`). Then
  the real `cut_stem` goal -- which needed a fully clean physical arm state to mean anything (the same
  corrupted-resting-pose lesson from the addenda above recurred here too: `servo_node` was still alive from an
  earlier aborted run and kept fighting a manual reset attempt, so `a300-isaac-sim` had to be restarted again for
  a trustworthy zero pose before this test). Result: **the first fully successful `cut_stem` patch this entire
  session** -- `Reached 'pose ...'` (arm) → pruner `Goal succeeded. Pruning confirmed.` (the new stub) →
  `Planning`/`Reached 'named pose drop'` (the real post-prune retract-and-drop sequence). `drive_test.py` still
  passes afterward.
- Files touched: `robot/bin/pruner_stub` (new), `robot/entrypoint.sh` (new background-service line),
  `robot/Dockerfile` (`chmod +x` for the new script), `CLAUDE.md` (new `pruner_stub` paragraph).

## Addendum: the arm moved too fast and overshot -- the bridge was jumping straight to each waypoint instead of interpolating

User, comparing directly against real-robot behavior they'd tested themselves: "the arm moves too fast compare
to the real robot I have tested. And it is overshooting at the goal position."

- **Root cause, reasoned from first principles rather than guessed at blindly**: `_execute_trajectory` published
  exactly one `JointState` per trajectory waypoint, at that waypoint's own `time_from_start`. A real
  `JointTrajectoryController` continuously *interpolates* between a planned trajectory's own (often sparse)
  waypoints, so the real robot's commanded position ramps smoothly between them. This bridge instead handed
  `IsaacArticulationController`'s PD position servo (`configure_arm_drives`' own `STIFFNESS`/`DAMPING` in
  `sim/scripts/setup_scene.py` -- that code's own comment already flagged these as "not the Kinova's own real
  servo gains... adjust if the arm moves too slowly/oscillates in practice") a sequence of large instantaneous
  target jumps instead of a ramp, and it tried to close each one as fast as its own gains allowed -- fast, and
  prone to overshoot on a step input, independent of how slowly the trajectory itself was actually paced.
- **Fixed by linearly interpolating between consecutive waypoints** at a fixed 50Hz (`INTERP_PERIOD_S`, roughly
  a real JTC's own control-loop cadence) instead of jumping straight to each one -- removes the step inputs
  without needing to retune the PD gains themselves. The first waypoint is still jumped to directly (matches a
  MoveIt-generated trajectory's own first point normally being at `time_from_start == 0`, i.e. the planning
  start state -- no prior point exists to interpolate from).
- **Verified with a controlled, timestamp-correlated test, not just eyeballing it**: sent a raw 2-point
  `FollowJointTrajectory` goal (`arm_0_joint_1`: 0.0 → 1.0 rad over a planned 4s) directly, bracketing the goal
  send and every poll with real host timestamps to avoid the exact trap an earlier, uncorrelated poll fell into
  (it looked like the arm reached the target in ~1.5s, which would have meant the fix didn't work at all --
  re-tested with proper timestamps and found that was just `docker exec`/action-client discovery latency before
  polling even started, not a real result). With real correlation: 0.09 rad at goal+0.5s, ~1.0 rad by goal+5s
  (matching the planned ~4s duration), settling at `1.0001` -- smooth, no measurable overshoot.
- **Re-verified against the real usage pattern**, not just the isolated single-joint test: ran the actual
  `cut_stem` sequence again afterward and it still completes its own real motion/gripper/pruner cycle correctly.
  `drive_test.py` still passes.
- Files touched: `colcon_ws/src/moveit_sim_bridge/moveit_sim_bridge/bridge_node.py` (waypoint interpolation,
  replacing the flat "publish once per waypoint" logic), `CLAUDE.md` (extended the `moveit_sim_bridge` paragraph
  with this finding).

## Addendum: a restart_ros script for a fast in-container ROS reset -- which then surfaced a real, previously-unnoticed robot_state bug on j100_0922

User: "create a script to stop and restart all the ros nodes in the robot containder [container]." Motivated
directly by this whole session's own repeated pain: getting a genuinely clean ROS-graph state for testing kept
requiring a full `docker restart` (or, worse, restarting `a300-isaac-sim` itself) -- slow, and a bigger hammer
than needed for "just get rid of stale/orphaned nodes before the next test."

- **Inspected the real process tree live before designing anything** (`ps aux` inside a busy `j100_0921` running
  the full `sim_robot_upstart.launch.py` stack), rather than assuming a kill pattern would work: found
  `robot_state`/`foxglove`/`pruner_stub` are each supervised by their own `while true; do X || true; sleep 2;
  done` background loop in `entrypoint.sh`, and -- important, easy to get wrong -- those three loop *processes*
  are completely indistinguishable from each other by cmdline (`ps` shows all three as the literal inherited
  `/bin/bash /entrypoint.sh sleep infinity`, since bash subshells don't get a distinct argv0). That ruled out
  trying to individually target or preserve specific loops by pattern; instead, the loops don't need touching
  at all -- killing only their *current worker process* is enough, since the still-alive loop notices the exit
  and respawns a fresh one within ~2s on its own.
- **New script, `robot/bin/restart_ros`**: `pkill -9` against a pattern broad enough to catch everything
  actually running in this project (`ros2 (run|launch)`, `/opt/ros/.../lib/`, `colcon_ws/install/.../lib/`, plus
  `pruner_stub` explicitly since it's a bare `python3 <path>` invocation matching none of the others) --
  verified against the real `ps aux` output line by line before trusting it, not just written from memory of
  what "should" be running. Confirmed this pattern does *not* match PID 1 (`sleep infinity`), the three loop
  supervisors, or the `ros2cli` discovery daemon (a long-lived CLI helper, not a "node," deliberately left
  alone) -- so nothing gets killed by accident. After killing, waits, then reports back which of the three
  self-healing services came back and which didn't -- and is explicit that anything else (a manually
  `ros2 launch`'d stack) is simply stopped, not relaunched, since the script has no way to know what command you
  wanted reissued.
- **Verified live**: launched the full stack, confirmed a big, busy process tree; ran `restart_ros`; confirmed
  via a fresh `ps aux` that PID 1 and the three loop supervisors survived untouched, everything else was gone
  (zombied, harmlessly -- their reaping parent, the killed `ros2 launch`, is gone too, a known cosmetic
  leftover, not a resource leak), and `robot_state`/`foxglove`/`pruner_stub` were back up within the script's
  own wait margin. Also confirmed via `ros2 node list` that the *ROS graph* itself was clean afterward (no
  stale entries for the killed nodes), not just that their OS processes were gone, and that a fresh relaunch of
  `sim_robot_upstart.launch.py` afterward worked normally. `drive_test.py` still passes.
- **Testing this on `j100_0922` (not just `j100_0921`, where all the arm/gripper work above happened) surfaced
  a real, separate, previously-unnoticed bug**: `robot_state_publisher` reported "NOT back up yet" after the
  restart, repeatably, not just a one-off timing miss. Its own log showed the exact same dangling
  `camera_1_joint`→`arm_0_end_effector_link` reference `generate_srdf` (two addenda ago) and
  `scripts/flatten_urdf.py`'s own `prune_dangling_joints` (much earlier session) already fix elsewhere --
  except `robot/bin/robot_state`'s own in-container xacro processing had never gotten that fix at all. Root
  cause: `robot_state`'s own `while true; do robot_state || true; sleep 2; done` loop's `|| true` had been
  silently swallowing this exact abort at *every single normal boot* of `j100_0922` all session -- nobody had
  actually looked closely at `/tmp/robot_state.log` before now, so `robot_state_publisher` for this one robot
  had likely never actually been running successfully at all, this whole time. Fixed by re-implementing
  `prune_dangling_joints` inline in `robot_state` too (same pattern as `generate_srdf`'s own reimplementation,
  for the same reason -- `flatten_urdf.py` is a host-side build script, not installed in this image), applied
  to `robot_state`'s own freshly-xacro'd URDF before handing it to `robot_state_publisher`'s strict loader.
  Verified live: `robot_state.log` now shows the same two joints dropped, then a real "Robot initialized" line,
  `robot_state_publisher` stays up, and `/j100_0922/tf_static` genuinely publishes real transform data
  afterward (confirmed by reading actual message content, not just that the process exists).
- **One remaining thing found while verifying this, explicitly left alone as out of scope**: `j100_0922`'s own
  `/j100_0922/platform/odom`/`platform/joint_states` (published by the *sim*, not anything in the robot
  container) aren't publishing at all right now, while `j100_0921`'s own equivalent topics work fine --
  confirmed this is unrelated to `restart_ros` (which never touches the sim container) and is a separate,
  pre-existing sim-side state issue, not caused by anything this addendum touched. Not chased further this
  pass.
- Files touched: `robot/bin/restart_ros` (new), `robot/bin/robot_state` (dangling-joint pruning, matching
  `generate_srdf`'s own fix), `robot/Dockerfile` (`chmod +x` for `restart_ros`), `CLAUDE.md` (new `restart_ros`
  paragraph, corrected "two restarting background services" → three, documented the `robot_state` fix, added
  `restart_ros` to the Commands list).

## Addendum: pushed back on -- correctly -- and a real joint_2-specific gain fix found, but it only solved half the problem

User, after the interpolation fix above: "the arm moves too fast compare to the real robot I have tested. And
it is overshooting at the goal position." Then, after I initially misdiagnosed this as the same singularity
issue from the addendum before it: "the stiffness increase did not resolve the issue. Even with the previous
setting, the arm could hold the zero state. Problem was when it starts moving... compare the arm command input
and the joint states, and see if it follows tightly." -- correctly pointing out I hadn't actually measured
anything yet, just assumed.

- **Built a proper commanded-vs-observed comparison** (a throwaway rclpy node subscribing to both
  `arm_0/joint_command` and `platform/joint_states`) and ran it during an actual `zero`→`cut_init`
  `FollowJointTrajectory` move. Confirmed the user right: `arm_0_joint_2` specifically overshoots by over 0.6 rad
  during the move (swinging *through* zero tracking error, not just lagging), while joints 1 and 3 show only
  small, bounded following error the whole time -- a real, joint-2-specific underdamped oscillation, not the
  uniform "arm can't hold gravity" story the earlier `1.0e7`/`1.0e5` uniform-gain test seemed to fix (that test
  only checked *static* holding at `cut_init`, never the actual moving transient -- a real gap in my own
  verification, caught by the user re-reading my own claim skeptically).
- **Two uniform-gain attempts both failed for the same underlying reason**: scaling stiffness+damping together
  preserves the same damping *ratio*, so joint_2 -- the "shoulder"/highest-effort-limit joint (14 N·m vs. 7-10
  for the others), carrying by far the most gravitational load of the 6 -- stayed just as relatively underdamped
  regardless of the absolute scale. Damping alone (stiffness left at the original value) didn't fix joint_2
  either, and *added* much worse lag to joints 1/3, which had been fine before. One set of uniform gains
  genuinely cannot serve joints this differently loaded.
- **Fixed with a joint_2-only, much stronger gain** (`ARM_JOINT_2_STIFFNESS`/`ARM_JOINT_2_DAMPING` = `1.0e7`/
  `1.0e6` in `configure_arm_drives`, vs. `1.0e5`/`1.0e4` for every other arm/gripper joint) -- a targeted,
  minimal-blast-radius change rather than perturbing all 6 joints again. Verified live: the same `zero`→
  `cut_init` move now settles to `err≈0.002` almost immediately after the commanded ramp finishes, versus still
  not fully settled 44 seconds later under the old uniform gain. Resting/spawn behavior (all joints holding ~0
  with no commands) confirmed unaffected for the other 5 joints.
- **This genuinely fixed what the user reported -- but the real `cut_stem` sequence's own patch-2 `servo_to_pose`
  stall (from three addenda ago) turned out to be a separate, still-unresolved problem sharing only
  `arm_0_joint_2` as a symptom, not the same root cause.** Confirmed by testing the full sequence with the new
  gain: patches 0 and 1 succeed (with normal RRTConnect retry variance, not a regression), patch 2 still times
  out at the exact same ~0.19m short, unchanged from every earlier test.
- **Redid the commanded-vs-observed comparison specifically during a live patch-2 stall, with the new gain
  active**: `cmd` (what `moveit_servo` is telling the bridge) sits essentially frozen at one value the entire
  time -- consistent with `moveit_servo`'s own logged "Moving closer to a singularity, decelerating" from
  several addenda back, meaning it has genuinely stopped commanding much joint_2 velocity -- while `obs` (the
  real simulated position) independently *oscillates* by over a full radian around a completely different
  value. This is conclusive on two points: (1) it's not a `moveit_servo`-side commanded-signal instability
  (the command is stable), and (2) since this persists even at 100x the original gain, it's not a simple
  insufficient-torque problem either.
- **Retested the user's own "maybe it's just speed" theory directly, a second time, now with the stronger gain
  -- and disproved it again**: a genuinely slow, hand-stepped approach (0.02 rad/step, ~0.3s settle each,
  `moveit_servo`/MoveIt not involved at all) to the same target still gets stuck, in the same ~-1.7 to -2.4 rad
  range, every time -- reachable cleanly in one direction (more negative: -2.5 reached exactly, repeatedly) and
  consistently blocked in the other (less negative, toward 0), regardless of approach speed or gain strength.
  This directional, position-dependent pattern is the same one found several addenda ago, before the user's
  gravity hypothesis -- re-confirmed, not just assumed to still hold.
- **Dumped TF positions for every arm link** (`base_link`→`arm_0_shoulder_link`→...→
  `arm_0_gripper_gripper_base_link`, plus `top_mount_link`/`top_shelf_link`) at the stuck configuration and
  computed pairwise distances looking for a self-collision candidate, rather than continuing to guess blindly.
  Nothing came back suspiciously close by link-*origin* distance alone (closest non-adjacent pairs:
  `top_mount_link`↔`arm_0_base_link` at 0.19m, `arm_0_upper_wrist_link`↔`arm_0_gripper_gripper_base_link` at
  0.11m) -- but this is an acknowledged imperfect proxy, since it ignores each link's actual mesh/collision-shape
  extent, so it doesn't rule out a real self-collision, just fails to obviously confirm one.
- **Left open, not resolved this pass**: the joint_2 gain fix is kept (real, verified improvement for the
  transient-overshoot case the user originally reported). Patch 2's own servo stall needs either visual
  inspection of the arm at the stuck configuration (Foxglove/RViz -- not available in this session) or a PhysX
  contact-report query (a new sim-side script, more invasive than anything tried so far) to actually identify
  what's physically stopping it, rather than more indirect joint-state comparisons. `drive_test.py` still passes
  throughout; nothing in this addendum touched the wheel-drive pipeline.
- Files touched: `sim/scripts/setup_scene.py` (`ARM_JOINT_2_STIFFNESS`/`ARM_JOINT_2_DAMPING`, per-joint gain
  logic in `configure_arm_drives`), `CLAUDE.md` (two new paragraphs documenting the fix and the still-open
  patch-2 investigation).

## Addendum: a local web UI (`tools/sim_ui/`) for driving the investigation

User: "to investigate this problem, i want you to create a web ui ... start, stop, and reset the simulation
scene, and spawn robots ... start the sim_robot_upstart.launch.py, and move the arm to one of the predefined
state, like 'cut_init' or 'stow'."

- **Local server, not a hosted page**: it has to run docker commands on this host. `tools/sim_ui/server.py`
  (Python stdlib `ThreadingHTTPServer`, no new dependencies) + `index.html` (vanilla JS, no CDN). Every button maps
  onto an existing command: start = `scripts/fleet.sh`, stop = `scripts/stop_sim.sh`, reset scene =
  `docker restart a300-isaac-sim` + wait for healthy, spawn = write `NUM_ROBOTS`/`ROBOT_MODEL_<i>` into `.env`
  then `scripts/fleet.sh N`, launch start = the same `docker exec -d ... ros2 launch mtu32_bringup
  sim_robot_upstart.launch.py` used by hand all session (output to `/tmp/sim_robot_upstart.log` in the robot),
  launch stop = `restart_ros`. Long actions run as in-memory background jobs; the page polls their logs.
- **Two new in-container helpers** in `robot/bin/` (so they also work from the CLI): `arm_goto <state>` reads
  group_states from `/etc/clearpath/robot.srdf` and either plans+executes via `move_group`'s `move_action`
  (MoveGroup goal with joint constraints, like RViz's "plan & execute") or, with `--direct`, sends a 2-point
  `FollowJointTrajectory` straight to `moveit_sim_bridge` (no planning, duration from the joint delta and the
  1.6 rad/s URDF limit x velocity scale); gripper states go through `GripperCommand`. `arm_joints [--record S]`
  prints commanded (`arm_0/joint_command`) vs observed (`platform/joint_states`) as JSON; the UI's "Record"
  option runs it alongside a move and plots both per joint.
- **Safety**: binds 127.0.0.1 by default; POST requires `Content-Type: application/json` (forces a CORS
  preflight the server never answers, so other sites can't trigger docker commands); robot names are checked
  against running containers, state/group names against `^[A-Za-z0-9_]+$`, models against `sim/assets`.
- **Verified via the API** (no browser available in this session -- page JS syntax-checked with `node --check`,
  but not clicked through): status, launch start (MoveIt came up), `cut_init` via MoveIt plan (`SUCCESS`, 281
  recorded samples), `stow` via direct mode, gripper close/open, joints snapshot, CSRF and name-validation
  rejections. **Useful finding from the first recording**: the direct `stow` move showed `joint_2` lagging its
  command by up to 0.93 rad mid-move (settling to 0.0001 after) even with the dedicated joint_2 gain -- so the
  transient tracking problem is reduced but not gone, and the plot now makes it visible without ad-hoc scripts.
- Files: `tools/sim_ui/{server.py,index.html}` (new), `robot/bin/{arm_goto,arm_joints}` (new),
  `robot/Dockerfile` (chmod), `CLAUDE.md` (commands + a `tools/sim_ui/` note).

## Addendum: joint_2 root cause = drive torque limit (14 N*m), fixed with ARM_JOINT_2_MAX_FORCE = 40

User, looking at the UI's joint plots: "As we observed, it drops drastically." Reading the recorded samples
(`/api/job?id=...` result) showed joint_2 tracking within ~0.05 rad until ~-0.65 rad, then falling to -2.1 while
commanded -0.85, and, on cut_init->zero, drifting *down* to -2.35 while commanded 0 -- only ever moving with gravity.

- `platform/joint_states` effort for joint_2 read 13.9995 N*m near cut_init: saturated. The USD drive's
  `maxForce = 14` comes from the URDF effort limit; `configure_arm_drives` set stiffness/damping only. That is why
  the 100x gain increase earlier "didn't help": any error already requested the full 14 N*m.
- Test (user-approved): `ARM_JOINT_2_MAX_FORCE = 40.0`. zero<->cut_init both track (max err ~0.06 rad, final
  0.002/0.0014). Gains 1e5/1e4 and 1e4/1e3 tested too: same tracking, same torque (peak ~26, hold ~11-13 N*m),
  so torque demand is gain-independent.
- Full `cut_stem` retest: 9+ patches, each reached -> "Pruning confirmed" -> drop, no servo stall (the old
  patch-2 stall was this same saturation). joint_2 effort median 11.2, p95 17.5, peak 24.3 N*m, >14 for 27%.
- User: "let's settle with this fix". Kept 40 N*m with original joint_2 gains (1e7/1e6).
- **Open**: measured torque ~1.5x (hold) to ~2x (moving, roughly +10-13 N*m at 0.14 rad/s) the static
  gravity torque computed by FK from the URDF masses (masses/COM/inertia match the imported USD; FK matches
  measurement at `zero`). Ruled out: joint friction/damping (none authored), drive gain, self-collision
  (disabled on the articulation root as a test: no change, reverted). Not checked: chassis pitch/rocking
  during arm moves, environment contact. The real arm works within 14 N*m, so 40 is a disclosed workaround.

## Addendum: lavender-farm scene (2026-09-29)

User asked to make the scene resemble their real lavender-farm photos. Done in `sim/scripts/setup_scene.py` (see CLAUDE.md "Sim" item 5 and README *Scene dressing*): grass field (`Ground_cover`), soil-coloured ground box, two lavender hedge rows, Omniverse-library trees/shrubs/boulders on the horizon, cloud-HDR dome (400) + sun (10000, rot -60/33/-30); target boxes/wall/pillars removed.
- Findings: `ground_cover.usd` says cm but is really metres (blades 0.09-0.11 units); 400 tiles = 14.4M instances -> "Unable to create ... instances", nothing rendered; instanceable references of it didn't render either. Tree/shrub/rock assets need their sibling `materials/`/`textures/` dirs (else red foliage); asset roots carry xform ops (reference under a child prim); `Cedar_Shrub` bbox invalid; camera far clip is 30 m so vegetation sits at 19-29 m.
- Verified by capturing j100_0921's `sensors/camera_0/color/image` (rotated 180) after each change; camera ~21 Hz with everything in. Not committed. Not done: buildings/mowed aisles from the photos, a textured soil, fps measurement with `FLEET_DEBUG=1`.

## Addendum: a300_00036 robot.yaml updated (arm, D435, Hokuyo, GPS, Microstrain IMU)

User: "a300_00036 robot.yaml file is updated." -> regenerate and adapt (details in CLAUDE.md, "`a300_00036`, updated robot.yaml").
- `gen_urdf.sh` OK; new URDF 102 links. `MODEL_PARAMS["a300_00036"]`: has_arm, lidar2d_link, gps_0/1, imu_link=top_plate_link,
  imu_index=0, chassis_link=chassis_link, no more has_camera=False.
- First live try (chassis_link=base_link): Odom graph error "base_link is not a valid rigid body or articulation root".
  physics.usda showed 9 ArticulationRootAPIs and 7 fixed joints with body0 = the robot root prim (world). Root cause: empty
  base_link + several collision children. Fix = `weld_empty_root_children` in flatten_urdf.py (see CLAUDE.md). After it: 1 root.
- Verified: drive test, lidar/IMU/GPS/camera topics, arm joint command. Not verified: sim_robot_upstart on this robot, MoveIt/cut_stem.
- `fleet.sh` doesn't restart a running sim -> `docker restart a300-isaac-sim` after regenerating URDFs (cost me one confusing
  re-check that was still the old URDF). One unexplained segfault (139) on the first start; the next start was stable (90 s+).
- .env: ROBOT_MODEL_0 was switched from j100_0921 to a300_00036 for testing (NUM_ROBOTS=1); revert or keep as wanted.

## Addendum: "Cut stem" / "Stop cutting" buttons in the web UI

User: "add a button to run the cut stem action to the webui". `tools/sim_ui/server.py`: `POST /api/cutstem/start` (job running
`ros2 action send_goal --feedback /$ROBOT_NAMESPACE/cut_stem ... "{start_cutting: true}"`, exec'd with no wrapper shell so exactly one
process matches `CUT_MATCH`; refuses if one is already running or the action server is missing) and `/api/cutstem/stop` (SIGINT to
that client -> ros2cli cancels the goal). `robots()` gained a `cutting` flag; `index.html` has the two buttons + a "cutting" pill.
Verified on a300_00036 via the API: start -> "Goal accepted", feedback "Patch 1/20 ..." streamed into the job log, cutting=true;
stop -> "Canceling goal...", grid_cutter_action_server logged "Received request to cancel goal", flag back to false; stop with
nothing running, bad robot name (400) and non-JSON POST (415) all handled. Page JS only syntax-checked (`node --check`), not clicked
in a browser. Side observation: `ros2 action list` inside a300_00036 also shows a stale `/j100_0921/cut_stem` (old graph entry).

## Addendum: cut_stem on a300_00036 -- works after making stow_arm_cpp's names relative

User: "check the cut stem action on a300_00036". Reset sim, restarted sim_robot_upstart (all nodes up; only the known gripper-not-a-chain /
no-octomap-sensor errors), ran cut_stem from the new UI button.
- Run 1: arm reached every pose but every patch failed "Gripper action server not available" (3 attempts, then skipped).
  Root cause: grid_cutter_action_server.cpp default `gripper_action` (and `servo_twist_topic`) and grid_cutter_params.yaml `pruner_action`
  were absolute `/j100_0921/...`; the a300_00036 server is at `/a300_00036/manipulators/arm_0_gripper_controller/gripper_cmd` (the
  `/j100_0921/...` entry in `ros2 action list` was a stale graph leftover). Fix: relative names (resolve under the node namespace).
- `scripts/colcon_build.sh` refused to run ("no running robot containers"): its regex needed exactly 4 digits; now `{4,}`.
- Rebuild (10 s) + relaunch + run: 20/20 patches, 0 failures, "Grid complete. Moving to stow.", Goal finished SUCCEEDED (~8 min; ~25 s/patch).
  joint_2 torque-limit workaround (40 N*m) was in effect. Not compared: cut quality/positions relative to the real robot.
- Uncommitted: stow_arm_cpp (2 files; plain-file package in this repo), scripts/colcon_build.sh, tools/sim_ui, flatten_urdf.py,
  setup_scene.py (a300_00036 params), .env (ROBOT_MODEL_0=a300_00036 test setting), docs.

## Addendum: a200_0284 "does not spawn correctly" -> MODEL_ASSETS KeyError + ROS domain mismatch

User: "a200_0284 does not spawn correctly. check it." (after fixing the yaml typo and regenerating).
- Sim log: `KeyError: 'a200_0284'` in import_urdf_if_needed -> MODEL_ASSETS was a hardcoded name tuple. Now derived from sim/assets/*/*.urdf.
  Added MODEL_PARAMS["a200_0284"] (see CLAUDE.md). URDF validated (no dup names, sane inertias, all meshes present).
- After that the sim ran ("simulation running with 1 robots") but the robot container saw only its own topics: robot.yaml has
  domain_id 1, sim is on 0. entrypoint.sh now forces the fleet ROS_DOMAIN_ID onto the copied real yaml. Image rebuilt, container recreated.
- Verified: drive 1.28 m @ 0.42 m/s, imu_0/gps_0/gps_1/camera publish, 7 arm joints, joint_1 and joint_4 follow commands and return to 0.
- One Kit startup hang (6 min silent after 35 s) on the first try; second docker restart was fine. Cause unknown.
- Uncommitted; .env currently ROBOT_MODEL_0=a200_0284 (test setting, like a300_00036 before).

## Addendum: cut_stem on a200_0284 with the shared colcon_ws (result: 14/14; a300_00036 re-verified 20/20)

User: "test the cut stem action on a200_0284 ... use the same source code (colcon_ws/src) for all different model."
Full write-up is in CLAUDE.md ("`cut_stem` on `a200_0284` ..."). Order in which things broke, each fixed generically or per-robot:
1. basket joint with a non-existent parent -> move_group/servo/cutter died (robot_state now rewrites the pruned xacro).
2. cutter: 6-value IK seed vs 7 joints -> per-robot override file (config/robots/<ns>.yaml) + launch support.
3. zone too close for the 7-DOF (contorted pose, servo whipped joint 1 into the robot body) -> zone x 0.50-0.62, seed scored via compute_ik.
4. joint_2 torque saturated at the fixed 40 N*m -> ARM_EFFORT_SCALE x URDF effort for every arm joint.
5. Robotiq: continuous mimic followers w/o limits (sim crash) -> limits from mimic relation; wrong-coefficient NewtonMimicAPI constraints ->
   removed for this model (drop_mimic_constraints). This was the long one: identical numbers under every gain/force/self-collision setting.
6. bridge reported trajectory success on playback time only while the sim wrist lagged ~1.2 rad -> settle wait + slower planning (0.4).
7. continuous joints wound past +-pi -> MoveIt START_STATE_INVALID -> +-3.12 limits (flatten + robot_state).
8. no `drop` pose in its robot.yaml -> joint-space stand-in over the `basket` (ASSUMPTION), drop_lower_distance 0.
Open / to tell the user: the drop pose is a guess; the servo refused to move from it (kept as 0 lowering); left/right wheel-radius calibration and the
+6% speed; sim start flakiness (hang or segfault, second restart fixes); one transient MoveIt code -4 per ~20 patches (cutter retry recovers).
.env: ROBOT_MODEL_0=a300_00036 (test setting; committed value is j100_0921).
Uncommitted: nearly everything above (scripts/, robot/, sim/, colcon_ws/src/stow_arm_cpp + moveit_sim_bridge, tools/sim_ui, docs).

## Addendum: RViz gripper execution failure + j100_0921 cut re-test
- User: "Start from simple gripper action ... Open and close planning is successful, but it fails the execution." Found TIMED_OUT (0.5 s bound)
  in the launch log from the user's RViz attempt; bridge slept 1.0 s. Fixed with target/stall detection. Verified via move_group (same as RViz
  Plan & Execute) incl. 1-point plans and velocity scaling 1.0. Not clicked in RViz by me.
- j100_0921 cut: first run 20/20 but "open" left the Lite's driven joint at -0.204 (mimic constraint vs. bridge offset). Constraints now removed
  for all arms. Second run aborted at patch 9 after a sim stall caused by a streaming-client connect (13:13:39Z). Third run: 20/20, 0 retries,
  60/60 gripper moves reached. a300_00036 and a200_0284 not re-run after the all-arms constraint change.
- .env ROBOT_MODEL_0=j100_0921 (= committed value). Nothing committed.

## Addendum: velocity calibration of all models (branch worktree-velocity-calibration)
User: "calibrate the robots' linear and angular velocities ... 0.0 to 1.0 m/s / 0.0 to 1.0 rad/s ... within 10% ... low acceleration";
then "put the arm to stow before it starts" and "search for contact property of isaac sim and its physics engine".
- Method: `scripts/calibrate_velocity.py` (docker exec in a robot container; `--ns` drives another robot): stows the arm (SRDF `stow`), ramps cmd_vel at
  0.25 m/s^2 / 0.5 rad/s^2, holds 8 s, measures from the pose in platform/odom over SIM time (odom stamps are wall clock; sim time = messages x 1/30 s).
- Baseline (open loop): linear +0-5% except j100_0921 reverse 0.6-0.9x; angular dead zone (0.2 rad/s -> 0, 0.5 -> 0.2-0.6x), Ridgeback erratic.
- What did not help the angular dead zone: wheel friction 0.1-0.4 (min combine), wheel drive damping x10/x1000, doubling wheel speed. PhysX has no anisotropic
  friction (forum); PGS + CCD + patch friction + 360 Hz helped (forum advice), but only PGS+CCD at 60 Hz is free (360 Hz halves the frame rate).
- Fix = `VelCtl` feedback + PGS/CCD + pose-based measurement + pose-based odom twist + Ridgeback lateral/yaw channels + j100_0922 mass fix. See CLAUDE.md.
- Surprises: (1) a frame advances physics by floor(60/22)/60 = 1/30 s, not 1/22: physical rtf = fps/30 (debug rtf is timeline based and too high). (2) PhysX's reported
  angular velocity is 0.02-0.03 rad/s above the real yaw change. (3) j100_0922 weighed 75 kg in the sim vs 18.4 kg URDF (massless links get collider volume x 1000 kg/m^3):
  tipped over about every other reverse drive, feedback on or off. (4) `docker exec -e ROBOT_NAMESPACE=x` is overridden by /etc/robot_ns_env.sh (use --ns).
  (5) a200_0333 has no robot_data, so no robot container: drive it from another container with --ns a200_0333.
- Final results (pose ratio = achieved/commanded, all within 10%): a300/a200/j100 0.965-1.013; r100 0.988-1.015 (lin/lat/ang); a300_00036 0.96-1.04; a200_0333 0.95-1.09;
  j100_0921 0.98-1.05; a200_0284 0.98-1.04; j100_0922 0.98-1.00 (tilt <= 1.4 deg). j100_0936 not tested (no robot_data). Sim start segfault happened once (exit 139), a second start was fine.
- Not done / for the user: j100_0921 (and a200_0284/a300_00036?) also carry heavy massless links (not fixed, cut_stem was tuned with them); .env SIM_RATE_HZ=22 gives a
  physical rtf of only ~0.5-0.6 (set it so 60/SIM_RATE_HZ is whole, e.g. 15 or 20); the odom twist is now pose-derived (filtered, ~1.5 frames lag).
- Testing used the worktree's sim/ mounted over /sim via a compose override and temporary .env model swaps (.env restored afterwards). Nothing pushed.
