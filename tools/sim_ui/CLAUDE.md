# tools/sim_ui

Local web UI: `server.py` (stdlib only; settings through `scripts/fleetcfg.py`, SQLite) + `index.html` + `config.html` (`/config`), default `127.0.0.1:8090`. It runs the repo's scripts and `docker`/`ssh` commands; long actions are background jobs whose log the page polls. Routes: the `POST`/`GET` tables at the end of `server.py`.

## Security and setup

- Binds 127.0.0.1 by default; `--host 0.0.0.0` exposes docker control to the LAN.
- POSTs must send `Content-Type: application/json` (else 415): other websites can't send it without a CORS preflight, which is never answered.
- A body with `password` is refused unless from 127.0.0.1/::1 (plain HTTP over the LAN otherwise).
- `FLEET_ROOT=<checkout>` drives another checkout's fleet (`.env`, scripts, `sim/`, its `fleetcfg.py`), e.g. from a worktree. The settings database is that checkout's `fleet_config.sqlite` too (`FLEET_CONFIG_DB=<file>` for another).

## Buttons: enabled only when they can work

- A button whose action can't work now carries `data-gate="<rule>"` (plus `data-robot` for a robot's); `applyGates()` (`GATES` in `index.html` / `config.html`) disables it with the reason leading its tooltip ("Not now: ..."). It runs after every status / jobs / base station poll and on the inputs a rule reads. Rebuilt cards (robots, real robots) keep their `data-gate`, so their buttons follow state without a rebuild.
- `/api/jobs` gives each job's `lock`: a button whose action a running job's lock would refuse is off while it runs (`busy()`). Stop motion, Stop sim, launch stop, logs and RViz have no lock.
- New button that depends on state: give it a rule, don't set `disabled` inline. `node --check` the page's script, and every `data-gate` needs its `GATES` entry.

## Modes and targets

- Header mode `sim`/`real`/`both`, per browser in `localStorage` (`--mode` = default); `.only-sim`/`.only-real` hide cards, `/api/status?mode=` picks the robots.
- Per-robot APIs take `{robot, kind}` (`sim` default, or `real`); `resolve()` gives `SimTarget` (`docker exec <c> bash -c`) or `RealTarget` (ssh, sourcing `/etc/clearpath/setup.bash` and exporting `ROBOT_NAMESPACE`, which a robot's shell doesn't set).
- ssh: `BatchMode`, ControlMaster socket in `/tmp/fleet-ui-ssh-<uid>/` (polls reuse one connection), stdin `/dev/null` (else ssh reads the server's terminal).
- Commands run in `bash -c`, so `pgrep`/`pkill` patterns bracket their first letter (`[r]os2`) or they match the wrapping shell.
- Name clash: a running sim robot and a real robot with the same name that is online, or linked (its `tcp/<host or last-resolved IP>:7447` in `BASESTATION_ZENOH_CONNECT`: the base station's router joins the graphs, also once it powers up), share every ROS name, so `resolve()` refuses actions on either (stop/read-only calls skip the check). `fleetcfg.problems()` refuses the setup itself (spawn, link, `NUM_ROBOTS`), and removing a linked real robot from the list.

## Simulation and Spawn cards

- `/api/sim/start` saves the settings `SIM_MODE` (`stream`/`headed`), `ROBOT_LOOKS` (`full`/`basic`/`off`) and `SIM_SCENE`, then runs `scripts/fleet.sh scene` (waits for the scene). Changing any of them recreates the sim. The page's mode/looks selects follow the settings at every status poll until the user picks one (`startTouched`), else Start would undo a change made on `/config` meanwhile.
- Headed: uses the server's `$DISPLAY` or the first `/tmp/.X11-unix` socket; runs `scripts/x11_auth.sh` (writes `.x11/xauth`) first.
- `SIM_SCENE`: `""` (ground plane + lights), `lavender` (only via the Configuration page / `fleetcfg.py`), or a file under `sim/scene/`.
- `/api/scene/upload` (base64 JSON, <= 512 MB): a browser only gives the page a file's contents, not its path, and saved scenes reference assets relative to themselves (`../assets/...`), so the scene must live in `sim/scene/`. Same SHA-256 there → reused; else copied in, never over a different file (`<stem>_<hash8>.usd`).
- `/api/sim/stop` = `scripts/stop_sim.sh`. `/api/sim/reset` = `fleet_ctl.reset()` (module reloaded each call): stops robot containers, stops/plays the sim timeline, starts them again; robots return to spawn state.
- `/api/spawn` `{robots: [{model, x, y, yaw°}]}` (<= 8): saves slots 0..N-1 (model and pose) and `NUM_ROBOTS`, runs `scripts/fleet.sh spawn --poses <json>`. Refused until the sim's `state.json` says the scene is ready. Models = the sim's imported `models`. The page's Spawn is off while the form asks for exactly the robots in the scene (`formMatchesScene`: models, poses within 5 cm / 1°), and reads "Replace the N robot(s)" when it differs (that restarts the sim).

## Per-robot actions

- **Launch** (sim): `/api/launch/start` waits for `/etc/clearpath/robot.srdf` (written during boot; the launch dies without it), runs `sim_robot_upstart.launch.py` detached (log `/tmp/sim_robot_upstart.log`); if it exits at once, `restart_ros` removes the nodes it left. Stop = `restart_ros`. Real robot log = `journalctl -u clearpath-platform-extras`.
- **Arm**: `arm_goto`/`arm_joints` from `moveit_sim_bridge` in `~/colcon_ws` (fallback: an older image's `/usr/local/bin`). Need `sim_robot_upstart` (plan mode: `move_group`; both modes: `moveit_sim_bridge`). The SRDF's duplicate `zero` state: first wins.
- **Record** runs `arm_joints --record` alongside; samples go in the job `result`, plotted commanded vs. observed.
- Real robots: planned moves only (`direct` is sim only), velocity <= 0.3, and a browser confirm before anything moves.
- **Cut stem**: checks the `cut_stem` action exists, then runs `ros2 action send_goal --feedback /<ns>/cut_stem ...` as a job; stop = SIGINT to that client (cancels the goal). "cutting" pill = `pgrep` on it.
- **Stop motion** / **Stop all motion** (every robot of the mode): SIGINT to `cut_stem`/`arm_goto` clients, then `CancelGoal` with a zero goal id (= all) on `cut_stem`, `move_action`, `execute_trajectory`, the arm trajectory and gripper controllers, so goals stop even without their client. ~2 s on a real robot: not an e-stop.
  - The job fails unless the stop is known to have happened (`stop_robot`: `ok` / `unreachable` = no answer, ssh failed, container stopped / `unconfirmed` = a cancel sent, its reply missing). Stop all goes to every real robot in the list at once, not only those last probed online; one offline at the last check and unreachable now doesn't fail it.
- **Job locks** (`start_job(..., lock=)`, `check_free` before an action writes settings): one running job per lock, else 400 with the running job's name. `FLEET_LOCK`: sim start/reset, spawn, Configuration apply (sim, robots); `stack:<kind>:<name>`: a robot's launch start, service restart, deploy; `motion:<kind>:<name>`: arm moves and cut_stem; `BASESTATION_LOCK`: base station actions and links. Unlocked on purpose: Stop motion, Stop sim, launch stop, RViz, read-only jobs. Only finished jobs are pruned from the 50 kept.
- **RViz** (`view` = `navigation`/`moveit`/`robot`): refreshes X auth, runs `clearpath_viz view_<view>.launch.py` detached (log `/tmp/rviz_<view>.log`). Sim: in the robot container. Real: in the base station container, `use_sim_time:=false`, as a zenoh client of the robot's own router (see Communication). `clearpath_viz` must be built.

## Real robots card

- The settings database's `real_robot` table (`fleetcfg.real_robots()`, cached 5 s; if the database can't be opened, the copy it last wrote to `real_robots.json`, untracked): `{"<id>": {"host", "user", "cutter"}}`. Default host `cpr-<id with ->.local` doesn't always resolve: give an IP. `cutter` gates Cut stem because `bringup_main` advertises `cut_stem` on every robot.
- Offline robots are retried in the background every 15 s (don't stall polls); `ros2 action list` only every 30 s (a ros2 CLI call costs a robot seconds of CPU). Sim robots: every 10 s while the launch runs and move_group or cut_stem is missing, then every 30 s (each new session pauses the whole fleet's data through the shared router: ~1-2 s for one, ~10 s for eight); the last result is kept in between. Every ros2 CLI call is `timeout -k 2 N` (a hung one ignores SIGTERM).
- Up to date = `git diff <~/colcon_ws/DEPLOYED commit> HEAD -- colcon_ws/src` is empty (a commit compare gives false "out of date").
- Linked = the robot's `platform/joint_states` appears in the base station's `ros2 topic list -v`; a configured endpoint can route nothing. Link adds `tcp/<ip>:7447` to `BASESTATION_ZENOH_CONNECT` and runs `compose up -d`.
- `/api/deploy`: `scripts/deploy_robot.sh` dry-run / deploy (`--yes`, after a browser confirm: its own prompt needs a tty) / pull.
- **Restart services**: `sudo systemctl restart clearpath-robot`. If `sudo -n true` fails the reply is `need_password`; the page asks in a masked `<dialog>` and reposts. The password goes only to `sudo -k -S -p ''` on ssh's stdin: never a command line, job log or disk. A NOPASSWD sudoers entry skips it.

## Communication card (`/api/link`)

- Four layers, since each has looked fine while another was broken: ping from this machine, robot WiFi (`iw`, `/proc/net/wireless`), its zenoh router (state, 7447 accept queue, ERRORs), clock offset (bare `ssh date`; applied only if known within 25 ms). Thresholds: `LINK_LIMITS`.
- ROS data from `scripts/link_monitor.py` in the base station: odom, GPS fixes, TF chains (map->odom, odom->base_link, arm), plus a `get_parameter_types` round trip (topics only show robot->here; ping skips zenoh).
- One monitor per robot, started once it answers ping, a zenoh client of that robot's own router (`ZENOH_CONFIG_OVERRIDE`): via the base station's router, two robots' router-to-router links deadlocked and held data for minutes.
- `joint_states` deliberately not subscribed (1 kHz on a300; costly everywhere); the arm TF chain stands in.
- Not counted as link faults: map->odom absent while a fresh `ref_localizer/status` says `publishing: false`; one silent topic while the robot's other streams arrive ("sensor fault"). TF chains always count.
- Runs only while polled (page visible, mode real/both); stops 30 s later or on server exit, which `pkill`s `link_monitor.py` in the container (killing `docker exec` leaves it running).

## Base station card

- Poll: `docker inspect` plus env drift vs. the settings (`.env`) (`RMW_IMPLEMENTATION` from `BASESTATION_RMW`/`FLEET_RMW`, `ROS_DOMAIN_ID`, `USE_SIM_TIME`, `BASESTATION_ZENOH_CONNECT`) and image rebuilt since; Recreate applies either.
- Background every 10 s: `docker stats` first (else it measures the probe), then `scripts/basestation_probe.py` in the container.
- Endpoints get robot names only from already-known IPs: an unresolvable mDNS name takes 5 s and stalls the poll.
- Actions: start/stop/restart/recreate/rebuild, `router` (`pkill -f [r]mw_zenohd`; `entrypoint.sh` restarts it), link/unlink an endpoint (edits `BASESTATION_ZENOH_CONNECT`, then `up -d`).

## Fleet view card

- `/api/fleetview` (GET, polled with the base station every 5 s): one `docker exec` for `pgrep` of the relay and the bridge plus the relay's status file (`/tmp/fleet_viz_status.json`). POST `{action: start|stop}` runs `scripts/fleet_viz.sh` as a job, lock `fleetview`.
- `/api/fleetview/layout[?cameras=1]`: `fleet_viz_layout.layout()` on that status; the page saves it as `fleet.json` (Blob download), for Foxglove's *Import from file*.

## Motion capture card

- `/api/mocap?server=<ip>` (default 192.168.50.80): `MocapMonitor` listens while polled (stops 30 s after), registers for unicast and joins the default multicast group (either Motive transmission type works). Parser: `mocap_fake_localizer/scripts/natnet.py`, imported by path.
- `/api/mocap/assign` writes `mtu32_bringup/config/ref_localization/assignments.yaml` (`/<ns>/natnet_ref_pose: rigid_body`) as JSON under a comment header (valid YAML, no YAML library). `bringup_main.launch.py` loads it between `ref_localization.yaml` and `<ns>.yaml`. One robot per rigid body; no entry = the body named after the robot. Real robots get it on deploy.
- ufw drops Motive's frames (from port 1511, not a reply) while command replies pass, so names show but no frames: `sudo ufw allow from 192.168.50.80 to any port 1511 proto udp`.

## Localization

- `/api/localization`: `ref_localizer/status` via `ros2 topic echo --once` (a few seconds).
- `/api/localization/set`: `ros2 param set` `source`/`anchor`, or `save_anchor`/`reset_anchor`; `ros2 service call` exits 0 regardless, so the job greps `success=True`.

## Configuration page (`/config`, `config.html`)

- Everything goes through `scripts/fleetcfg.py` (the settings database `fleet_config.sqlite`; `.env` is rendered from its active profile): `read_env()` renders first (a change made with `fleetcfg.py` or SQL shows at the next poll), `write_env()` = `fleetcfg.set_settings`. Changes are made before a job starts, so a refusal (database unusable and the value differs, invalid value, the same real robot in two started slots) is the request's 400, not a job error.
- Database unusable (can't be opened): `.env` as last written, the page read-only with a red banner, the main page's *settings* pill red. Writes that change nothing still pass (Start with the same mode/scene).
- `/api/config` = `fleetcfg.view()` + models, scenes, real robots + **drift**: `docker compose config --format json` (from `.env`) vs. `docker inspect` of the running sim, robot slots (container name, hostname, env) and base station, cached 4 s. Not compared: `DISPLAY`/`XAUTHORITY` (the server's shell, not the settings) and anything named like a secret (never sent to the page). `/api/config/apply` = `fleet.sh scene` (sim; compose recreates it when its env changed, its robots respawn from `applied_request.json`), `fleet.sh spawn` (robots), `compose -f basestation.compose.yml up -d`.
- Change log actor is `web UI`; the CLI's is `user@host`. Profile create/copy/delete log one row (`DB.bulk`), so their settings can't be reverted one by one.
