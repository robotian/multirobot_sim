# status_server

`status_server` is a ROS 2 `ament_python` package that synchronizes robot status into a PostgreSQL farm database and publishes task assignments back to the fleet.

## Overview

The package has two nodes, both started by the launch file:

- **`status_server`** (`robot_status_sync.py`). Subscribes to `<ns>/status/robot` (`status_interfaces/RobotStatus`) for every namespace in `namespace_config`, and writes each update into `robot_live`. A liveness watchdog marks a robot `OFFLINE` once it has been silent for `liveness.timeout`. Robots that never report at all are also marked offline.
- **`task_manager`** (`job_publisher.py`). Every 5 s it scans `robot_info` / `robot_live` for robots that are online. For each new robot it creates:
  - a publisher on `/<serial>/status/task` (`status_interfaces/Task`), ticked at 1 Hz
  - a `BEST_EFFORT` subscription to `/<serial>/platform/bms/state` (`sensor_msgs/BatteryState`), used for [battery feasibility](#battery-feasibility)

  Tasks (harvesting, unloading, charging) are built by `task_generator.py`. It routes over the farm graph, using shortest paths with scipy, and adds dock/undock goals from `dock_station`. On start-up, `task_manager` releases any dock reservations left behind by a run that ended while docked.

Database tables used: `robot_info`, `robot_live`, `dock_station`, `farm_harvesting_job`, and the routing graph. The graph is `graph_node` / `graph_edge` / `object_data`, or the legacy `farm_node` / `farm_edge` / `farm_asset_on_map`, depending on `farm.graph_source`.

## Package Contents

- `launch/status_server.launch.py` – launches both nodes together
- `config/config.yaml` – namespaces, battery, liveness, farm, clearance, dock and PostgreSQL settings
- `status_server/robot_status_sync.py` – `status_server` node entry point
- `status_server/job_publisher.py` – `task_manager` node entry point
- `status_server/task_generator.py` – graph routing and task/waypoint construction
- `status_server/postgres_manager.py` – all database access (psycopg 3)
- `status_server/configuration.py`, `dataclass.py`, `data_utils.py`, `enum.py` – config loading, data types, status/task enums
- `tools/` – offline graph checkers that need no ROS environment (see [Tools](#tools))
- `data/cost_matrix.csv` – reference data only. It is not installed and not read by the nodes
- `test/` – ament flake8 / pep257 / copyright tests

## Dependencies

- ROS 2 (Jazzy): `rclpy`, `launch`, `launch_ros`, `ament_index_python`
- `status_interfaces` (in this workspace)
- `geometry_msgs`, `sensor_msgs`, `tf_transformations`
- Python: `python3-yaml`, `python3-numpy`, `python3-scipy`
- **psycopg 3** (`sudo apt install python3-psycopg`). It has no rosdep key, so `rosdep` won't install it. Don't substitute `python3-psycopg2`, which has a different API.
- A PostgreSQL database holding the farm schema. In this repo that is the base station (`basestation.compose.yml`, port 5433).

## Build and Install

From the workspace root:

```bash
colcon build --packages-select status_server
source install/setup.bash
```

In this repo, use `scripts/colcon_build.sh --packages-select status_server` instead.

## Run

```bash
ros2 launch status_server status_server.launch.py
ros2 launch status_server status_server.launch.py log_level:=debug
```

- `log_level` is applied to `task_manager` only.
- The `namespace` argument is declared but not applied to the nodes. Topics are absolute (`/<serial>/...`), and the robots to watch come from `config.yaml`.
- The launch file does not set `use_sim_time`. Both nodes run on wall time, which is consistent within each node: liveness compares message arrival times against the node clock.

The database password is not in `config.yaml`. Export `PGPASSWORD` or use `~/.pgpass` before launching (see [`database`](#database--postgresql-connection)).

## Configuration

All runtime settings live in `config/config.yaml`, parsed into the `Config`
dataclass by `status_server/configuration.py`.

### Loading behaviour

- **Every key is mandatory.** `DataUtils.from_dict` has no defaults; a missing
  key raises `KeyError` inside the parser at node start-up.
- **Config is cached at first load** (`Configuration._config`). A running node
  will not pick up edits — **restart the node** to apply changes.
- With `--symlink-install` the installed `config.yaml` points at the source
  file, so no rebuild is needed; only a restart.
- Values are used as-is, no type coercion beyond what YAML infers. Write floats
  as `20.0`, not `20`.

### `namespace_config` — which robots to track

Consumer: `robot_status_sync.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `mode` | str | `multi` | `single` = subscribe to `namespace` only. `multi` = subscribe to every entry in `namespaces`. |
| `namespace` | str | `/a300_00036` | Namespace used when `mode` is `single`. Ignored in `multi`. |
| `namespaces` | list[str] | `[/j100_0921, /a300_00036]` | Namespaces subscribed when `mode` is `multi`. Ignored in `single`. |

### `battery` — task feasibility

Consumer: `job_publisher.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `reserve_threshold` | float | `20.0` | Minimum battery level in percent (0–100) to accept a new harvesting or unloading task. At or below this the robot is sent to a charging dock instead. |
| `ewma_alpha` | float | `0.1` | Smoothing factor for the running power and speed estimates, `0 < alpha <= 1`. Higher reacts faster and is noisier. At a 1 Hz BMS feed, `0.1` means the last ~10 samples dominate. |
| `safety_factor` | float | `1.5` | Margin applied to predicted energy before comparing against the charge on board. `1.5` = the robot must carry 50% more than the estimate says it needs. |
| `min_move_distance` | float | `0.05` | Metres a robot must have moved between ticks before the sample counts towards the speed estimate. Stops a parked robot dragging the speed estimate to zero, which would make the return-leg prediction diverge. |
| `fixed_task_seconds` | dict[str, float] | `{harvesting: 480.0, unloading: 300.0}` | Whole-task durations in seconds, including each task's own navigation. The charger return trip is **not** listed — it is derived from distance. `charging` has no entry: it is the fallback task and is never vetoed. |

See [Battery feasibility](#battery-feasibility) below for the prediction model.

### `liveness` — offline watchdog

Consumer: `robot_status_sync.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `timeout` | float | `30.0` | Seconds without a status message before a robot is marked `OFFLINE`. Must exceed the worst-case gap between messages (including DDS jitter) or a healthy robot flaps offline. |
| `check_period` | float | `1.0` | How often the watchdog scans for stale robots, in seconds. |

### `farm` — farm-wide settings not held in the database

Consumers: `postgres_manager.py` (`graph_source`), `task_generator.py` (`crop_type`)

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `crop_type` | str | `lavender` | Crop grown on this farm. Written onto generated harvest tasks; the `farm_asset_on_map` / `object_data` tables carry no crop info of their own. |
| `graph_source` | str | `graph` | Which table family supplies the routing graph. `graph` → `graph_node` / `graph_edge` / `object_data` (current). `farm` → `farm_node` / `farm_edge` / `farm_asset_on_map` (legacy). The old tables are left in place, so switching back to `farm` restores the previous routing with no code change. |

### `clearance` — waypoint shift off the crop

Consumer: `task_generator.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `enabled` | bool | `false` | Apply the shift at all. `false` publishes raw node positions — correct when the graph has already been moved so lanes clear the mapped crop. Set `true` if waypoints start landing on the crop again. |
| `robot_half_width` | float | `0.349` | Half the robot footprint width. This is the inscribed radius Nav2's inflation layer treats as definitely-in-collision around an obstacle; it sets how far sideways a bush must be before the robot may sit beside it. |
| `robot_half_length` | float | `0.495` | Half the robot footprint length. Sets how far ahead or behind an object still counts as level with the robot. A long robot's front corner reaches a bush its centre is nowhere near. |
| `obstacle_radius` | float | `0.225` | How far the mapped obstacle extends from the object centre held in `object_data`. Crop rows measure 0.45 m thick in the map, so 0.225 m. |
| `safety_margin` | float | `0.10` | Extra metres on top of the two radii. Measured, not guessed: crop rows in `zone_end_1.pgm` have a ragged edge with stray occupied cells up to a costmap cell beyond the nominal half-thickness. At `0.05` the footprint still clipped 413 of 760 harvest waypoints; at `0.10` it clips none. |
| `max_shift` | float | `0.30` | Largest shift allowed for a single waypoint. A waypoint needing more is published as close as the cap allows and the shortfall is logged. Past this the geometry is wrong and the fix belongs in the map or the graph. |
| `collinear_tolerance` | float | `0.05` | Drop a waypoint lying within this distance of the straight line between its neighbours, so a straight run publishes only its two ends. A row of Pickup nodes is one straight drive; measured deviation along a 7.1 m pickup row is 0.0007 m. |

`ClearanceConfig.standoff` (derived, not configured) = `obstacle_radius + robot_half_width + safety_margin`.

### `docks` — dock IDs per robot

Consumers: `job_publisher.py`, `task_generator.py`

Structure: `graph_source -> namespace -> purpose -> list of dock IDs`, where
`graph_source` is `graph` or `farm` and `purpose` is `charging` or `unloading`.

```yaml
docks:
  graph:
    /a300_00036:
      charging: [wiferion_charger]
      unloading: [unloading_station]
  farm:
    /a300_00036:
      charging: [husky_charger]
      unloading: [unloading_station]
```

- The block is keyed by graph source first, because node IDs are **not** shared
  between the two schemas. Node 12 in `farm_node` is a different place from
  node 12 in `graph_node`, so switching `farm.graph_source` switches the docks
  with it. The live source's block is used; if it is missing, the robot gets no
  docks and an error is logged.
- A flat block (namespaces at the top level, no source keys) is still accepted
  and used for every source. Keys starting with `/` tell the two layouts apart.
- Only **dock IDs** live here. The node the robot drives to, the Nav2 plugin
  type and the reservation state all stay in the `dock_station` table, so
  giving a robot a second dock is a config edit, not a schema change.
- A dock ID is **global**: it is both the `dock_station` key **and** the name
  registered with that robot's Nav2 docking server, so the two must agree.
- Listing the same ID under two robots is allowed. They share the dock, and the
  `dock_station` reservation decides who gets it.
- An empty `unloading` list makes unloading tasks fall back to a charging dock.
- Config keys are written as ROS namespaces (`/j100_0921`); lookup also accepts
  the bare serial number (`j100_0921`).

### `database` — PostgreSQL connection

Consumer: `postgres_manager.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `host` | str | `host.docker.internal` | Hostname or IP of the PostgreSQL server. `host.docker.internal` reaches the base station from the sim's robot containers and the base station itself. A real robot uses the base station's LAN IP. |
| `port` | int | `5433` | Server port. The base station uses 5433 so it doesn't collide with a PostgreSQL installed on the host on 5432. |
| `connect_timeout` | int | `5` | Seconds to wait for a connection before giving up. Without it psycopg falls back to the OS TCP timeout, which can block a node for minutes if the host is unreachable. |
| `dbname` | str | `test_lavender_farming` | Database name. |
| `user` | str | `admin` | Login user. |
| `password` | str | `''` | Leave empty: the password is not kept in git. With it empty, libpq reads `PGPASSWORD` from the environment or `~/.pgpass` (`host:port:dbname:user:password`, mode 0600). On a robot, set either for the user that runs the node; in the sim, put `PGPASSWORD=...` in the untracked `db.env` at the repo root (loaded into every robot container). |

## Clearance geometry

The graph puts a Pickup node where the manipulator wants it, which is closer to
the bush than the robot's body may legally sit. Nav2 paints a band of the
robot's own inscribed radius around every mapped obstacle as untraversable and
rejects any pose whose footprint touches it, so those nodes are goals no planner
will accept. Shifting the waypoint away from the bush buys that clearance back,
and costs the same distance in arm reach.

Measured against `zone_end_1.pgm`: the crop rows are drawn 0.45 m thick, the
lanes sit 0.45 m off the bush centres, and the robot is 0.698 m wide — so the
footprint overlaps the mapped crop by about 0.12 m and needs roughly 0.17 m of
shift. If the arm cannot reach that far, the fix belongs in the map or the
graph, not in `clearance`.

With `enabled: false`, the graph itself was moved so the lanes clear the mapped
crop, which is the cheaper fix — moving a lane costs nothing, whereas the shift
here buys clearance by spending manipulator reach. Re-enable only if waypoints
start landing on the crop again; verify with:

```bash
python3 tools/graph_audit.py --checks clearance
```

## Battery feasibility

`job_publisher.py` subscribes to `/{namespace}/platform/bms/state`
(`BEST_EFFORT`) and keeps a per-robot EWMA of power draw and travel speed. A
task is feasible when:

```
predicted_energy = avg_power_w * (fixed_task_seconds[task] + return_leg_m / avg_speed_mps)
predicted_energy * safety_factor <= energy_on_board_wh
```

- `return_leg_m` is the distance from the task end to the assigned charging
  dock, taken from the routing graph.
- Speed samples only count when the robot moved at least `min_move_distance`
  between ticks; a speed of zero is treated as "not yet measured" so the
  division cannot blow up.
- `charging` is never vetoed — it is the fallback when every other task fails
  the check.

## Tools

Standalone scripts in `tools/`. They read `config/config.yaml` for the
database connection and need psycopg, pyyaml, numpy and scipy, but no ROS
environment and no running nodes. Both accept `--config` and `--dbname`.

- **`path_check.py`**: routing checker. It mirrors `TaskGenerator`'s routing
  and `RobotStatusSync`'s node snapping without importing them. If its output
  and the running node ever disagree, one of them has drifted. `--source
  graph|farm` overrides `farm.graph_source`.

  ```bash
  python3 tools/path_check.py path 926 1 --waypoints   # route + published waypoints
  python3 tools/path_check.py sweep --limit 80         # harvest order, as tasks will be generated
  python3 tools/path_check.py task --at 970            # simulate the next harvest task
  python3 tools/path_check.py snap -5.114 -0.895 1.600 # pose -> nearest node
  python3 tools/path_check.py edges 926                 # edges into and out of a node
  python3 tools/path_check.py audit --samples 20000    # graph health and path validity
  ```

- **`graph_audit.py`**: checks whether a planner can actually drive the graph,
  using geometry only. The checks are `sideways`, `zerolen`, `approach`,
  `turns`, `clearance` and `dubins`. Robot dimensions default to the A300;
  override them with `--robot-width`, `--robot-length`, `--turning-radius`,
  `--inflation` and `--cost-scaling`. `--sql` prints the SQL that adds the missing approach nodes, then exits. The exit code
  is the number of failed checks.

  ```bash
  python3 tools/graph_audit.py
  python3 tools/graph_audit.py --checks sideways,approach --limit 0
  ```

## Entry Points

- `status_server`: starts the status synchronization node
- `task_manager`: starts the task publishing node

## License

Apache-2.0
