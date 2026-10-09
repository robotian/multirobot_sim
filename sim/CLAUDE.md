# sim/ — Isaac Sim scene (`sim/scripts/setup_scene.py`)

`setup_scene.py` runs via `--exec` in the Isaac streaming app, `./sim` mounted at `/sim`. Real-robot params, chassis_link/IMU findings, flatten_urdf fixes → skill `add-real-robot`; URDF pipeline → `scripts/CLAUDE.md`.

## Start, spawn, reset

- **Import** (`import_urdf_if_needed`, `MODEL_ASSETS`): once per model, cached in `sim/generated/<model>/`, invalidated by `.import_stamp` (import settings + URDF mtime/size) or `FORCE_REIMPORT=1`. Each model's `usd_dir` is deleted first, or the importer writes `<model>_1/`, `<model>_2/`, ….
- `main()` imports every model before building the scene (the importer opens its output as the stage, so it can't run later); a model whose URDF changed after start is refused at spawn. Then: world built, timeline left **stopped**, `sim/generated/fleet/state.json` written (`scene: ready`, `models`, `default_poses`, `lavender_rows`, `scene_source`), `spawn_loop` runs.
- **Spawn:** `spawn_loop` polls `spawn_request.json` (`{id, robots: [{model, x, y, yaw_deg}]}` from `scripts/fleet_ctl.py`; slot = list index, namespace `robot_namespace(slot, model)`). `parse_request`: known model, |x|,|y| <= `SPAWN_LIMIT` 38 m, >= 1 m apart, unique namespaces; a rejection leaves robots alone (`last_error`). `spawn_fleet` stops the timeline, spawns, re-aims the viewport, plays. Successes go to `applied_request.json`; answered ids are never replayed.
- **Replacing robots restarts the sim** (~27 s): with robots present, `fleet_ctl.spawn` writes the request with `at_start: true` (ignored by the running loop) and `docker restart`s; at start the pending request is spawned (fallback `applied_request.json` if rejected), then `boot: done` (what `wait_scene` waits for). Not in-place: `DeletePrims` on `/World/<ns>` + `/Graphs/<ns>` made the headed RTX renderer abort (SIGABRT in `libnrend.so`). Spawning into an empty scene stays in place.
- **Deleting a graph doesn't run its ScriptNodes' `cleanup()`**: script nodes that create rclpy nodes register them in `fleet_nodes.py`; `spawn_fleet` destroys them. Do the same for any new one.
- **Reset** (no restart): `control.json` `{id, action: "reset"}` → `reset_timeline` (stop, 5 frames, play, ~3.5 s) → `state.json` `reset: {id, state: done}`. `fleet_ctl.reset` stops the robot containers around it so no robot process acts on the restarted scene.
- `default_poses(n)`: `SCENE_LANES` (3) robots 1.6 m apart at x=0, further ranks 2.5 m behind. `platform/odom` and `ground_truth` TF start at zero at the spawn pose (no spawn yaw); GPS has it (datum = world origin, X/Y = East/North).
- **Save As in a running sim** reopens the file as a new, *stopped* stage: `current_stage` takes the open stage on every request, and `spawn_loop` replays the timeline if robots exist. Never cache the stage from `main()` (spawns then fail with `Stage.DefinePrim ... did not match C++ signature`).
- `apply_kit_settings` sets `/app/omni.graph.scriptnode/opt_in` true; otherwise a dialog appears whose **No** disables every robot script node.

## Per-robot graph

One OmniGraph per robot (`isaacsim.ros2.bridge`, topics relative to the namespace) plus a USD camera per camera.

- **cmd_vel is `TwistStamped`** (as on the real Jazzy platform). `ROS2SubscribeTwist` only takes `Twist`, so `CMD_VEL_SCRIPT` (ScriptNode with an rclpy subscriber) replaces it. Publishers must send stamped.
- Chain: cmd_vel → `VelCtl` → `Diff` (`DifferentialController`) → `DriveFront`/`DriveRear`; `Odom`. Their `chassisPrim` is `chassis_prim()`, the one prim with `ArticulationRootAPI`, read at spawn (a `chassis_link` override is honoured). A wrong one only fails at runtime ("Articulation controller failed"); more than one root is refused at spawn.

### Velocity control (`VEL_CTL_SCRIPT`)

- Feed-forward + PI on achieved forward speed / yaw rate (`VELCTL_DEFAULT`, per-model `vel_ctl` in `model_params.yaml`, `VELCTL=0` = open loop). Target within ~10% over 0-1 m/s, 0-1 rad/s; check with `scripts/calibrate_velocity.py` (stows the arm first).
- Speed is measured **from the pose** over `PHYSICS_FRAME_DT`, not PhysX's velocity (angular reads 0.02-0.03 rad/s high); the `platform/odom` twist too (`VelCtl.outputs:odom_lin/odom_ang`).
- `Diff` max speeds = `VELCTL_HEADROOM` (4) × limit; the command is clamped to the real limit in `VelCtl`.
- PGS + CCD on wheels (`enable_wheel_ccd`): TGS barely turns skid-steer robots. Keep `wheel_radius` etc. at the real calibrated values.
- Didn't help the skid-steer rotation dead zone (don't retry): wheel friction, drive damping, 360 Hz physics.

### Resting stillness

- `WHEEL_BRAKE_SCRIPT`: with zero command and wheels < 0.05 rad/s (or after 1 s), latches wheel angles as position targets at `WHEEL_BRAKE_STIFFNESS` 1e4; any command resets stiffness to 0. Needed because wheel drives are pure velocity dampers, so idle robots rocked.
- `ARTIC_VEL_ITERS` (8) on every articulation root stops contact jitter; extra position iterations only cost fps.

### Ridgeback (r100, `drive: omni`)

- Same `Diff` graph for forward/rotation; a `BodyDrive` ScriptNode (`Articulation.get/set_velocities` on the root) replaces the body-frame lateral velocity with the commanded `vy` (fed back, `VELCTL_LATERAL`) and sets the yaw rate. Wheels don't spin while strafing.
- Not `HolonomicController`/mecanum wheel speeds: wheel collisions are plain cylinders (rollers are visual-only) and this PhysX has no anisotropic friction, so no sideways thrust; driving the wheels made rotation wrong.
- Limitation: a small pure in-place rotation from standstill (~0.5 rad/s) is mostly absorbed by static friction.

### Lidars (`LIDAR2D_READ_SCRIPT`, `LIDAR3D_READ_SCRIPT`)

- Rays cast per frame with `get_physx_scene_query_interface().raycast_closest` (directions via numpy); no sensor prims. Rays start at `range_min`; misses +Inf (2D) / dropped (3D).
- 2D: 541 rays over ±2.356 rad, 0.1-10 m, `LaserScan`. 3D (VLP16): 16 × 360 rays, ±15°, 0.4-30 m, `PointCloud2` on `sensors/lidar3d_0/points`, pose = chassis × the lidar's offset from it, spread over `LIDAR3D_TICKS_PER_SCAN` (4) frames.
- Not `isaacsim.sensors.experimental.physics` `Raycast`/`RaycastSensor`: its per-step update segfaulted the sim (exit 139) and its `depths` are wrong. The IMU still uses that plugin (`IMUSensor`), never seen crashing; suspect it first on a step-time segfault.
- Never author prims under an instanceable subtree (e.g. a200's `sensor_arch`): instance-proxy error, and retrying per tick crashed the sim. Parent under a non-instanced prim with an explicit offset.

### Cameras

- `CAM_FAR_CLIP` 120 m. Depth limited to the real sensor range (`DEPTH_RANGE_M`: D435i 0.28-10, D405 0.07-1, ZED 2i 0.3-20 m), out-of-range = 0 like the real drivers: `install_depth_range_limit` wraps `rep.writers.get` and swaps in a `distance_to_image_plane` annotator with a Warp augmentation, **registered by name** (the helper deep-copies the writer; an `Annotator` object doesn't copy). Failures only log.
- **On demand** (`CAMERA_ON_DEMAND`, `camera_on_demand_loop`): `HydraTexture.set_updates_enabled` on only while a topic has a subscriber (polled 0.5 s, off `CAM_IDLE_OFF_S` 2 s later); textures from `ViewportManager()._hydra_textures`. Not `IsaacCreateRenderProduct.inputs:enabled` (keeps rendering). No round-robin/per-frame toggling: it's slower than always on.
- Every camera renders `CAM_WARMUP_S` (5 s) after play/spawn/reset, so its publisher exists; otherwise `ros2 topic hz/echo`/Foxglove never subscribe.
- j100_0921 / a200_0333 images are upside-down (real mount).

## Time

- `/Graphs/fleet_clock` (`build_clock_graph`) publishes `/clock` from `IsaacReadSimulationTime`; robot graphs' `Stamp` node stamps everything (`IsaacReadSystemTime` if `USE_SIM_TIME=false`), script nodes via a `stamp` input.
- `resetOnStop=False`, and camera helpers `useSystemTime=False`, `resetSimulationTimeOnStop=False`: otherwise time jumps to 0 on spawn/reset.
- `SIM_RATE_HZ` (compose default 20, tracked `.env` 22 — retune) ≈ the `render_fps` of `FLEET_DEBUG=1`; `PHYSICS_HZ` 60. Kit runs `floor(PHYSICS_HZ / SIM_RATE_HZ)` steps per frame, so a non-divisor makes `FLEET_DEBUG`'s rtf overstate; `/clock` counts physics time and stays right. `PHYSICS_FRAME_DT` = true step per frame.
- FPS levers (README *Faster streaming*): async rendering via `FLEET_SETTINGS`, fewer robots, `CAMERA_STREAMS=none`, cameras on demand; resolution/viewport size don't matter. `FLEET_MERGE_FIXED=1` breaks the build (merges away `camera_0_link`).

## Model parameters

Derived at start (`load_model_params`/`derive_model_params`, logged `[fleet] params <m>:`): drive from `platforms[<platform>]` in `model_params.yaml`, replaced by robot.yaml's `platform_velocity_controller` calibration; sensors/arm/IMU body from robot.yaml + flattened URDF + `merged_links.json`; then `robots[<m>]` hand overrides. Arm drives: `configure_arm_drives` (`ARM_DRIVE_*`, `ARM_EFFORT_SCALE`).

## Robot looks (`ROBOT_LOOKS` full|basic|0, `robot_looks.py`)

- `apply_robot_looks(models)` at spawn, once per model: OmniPBR looks from `/sim/generated/looks/looks_<mode>.usda` via `RULES`/`MODEL_RULES` (hue judged in sRGB); procedural textures cached by stamp, cubic projection (no UVs) scaled per mesh unit to `TILE_M`. Errors only log.
- Visual meshes are instance prototypes, so bindings are retargeted **in memory** in each model's `payloads/instances.usda` / `base.usda` (`Sdf.Layer.FindOrOpen`, never saved, kept in `_held_layers`). Only all-purpose `material:binding`. One prim per tint colour.
- Snapshots: `FLEET_SNAPSHOT=<dir>`, or `fleet_ctl.py snapshot [--poses '{"<ns>": [x, y, yaw]}'] [--views front,rear,close,wide] [--tag T]` → `sim/generated/snapshots/`. Pass `ref_pose` as `--poses` after driving (the USD transform can lag).

## Scenes (`SIM_SCENE`, `build_world`)

- Empty: `build_default_world` (80 m ground box, top z=0, dome + sun; RTX ignores `displayColor`, so bind a material). `lavender`: built-in farm. Otherwise a file in `sim/scene/` (`build_file_world`; `.env` has `lavender_farm_chargers.usda`).
- A file is a **sublayer** of a new stage (relative `../assets/...` resolve, the file is never written); `setup_physics` is authored over its physics scene; default ground/lights only if it has none; saved robots and `/Graphs` are removed in memory. Ground at z=0 within ±`SPAWN_LIMIT`. Spawn-map rows: top layer's `customLayerData["lavender_rows"]` (`[x_min, x_max, y, width]`).
- The sim user (uid 1234) saves only into writable folders: `chmod 777` a new `sim/scene/` subfolder.
- Vegetation (NVIDIA `Assets/Vegetation/...` in `sim/assets/trees|shrubs|rocks/`) needs its `materials/`/`textures/` and the `sim/assets/Trees -> trees` symlink, or renders red; Z-up/cm, referenced under a child prim. `Cedar_Shrub` is unusable (empty bbox). Ground cover: one unscaled patch only (tiling exceeds the instance limit; geometry is metres despite `metersPerUnit=0.01`).

### Soft lavender (`soften_lavender`, `LAVENDER_SOFT`, default 1)

- On `/World/lavender` in any scene: collision group `soft_plants` filtered against `robots` (`spawn_robot` adds each robot), so robots pass through foliage while lidar raycasts still hit it. Guide-purpose cylinder cores (r 0.1 m, h 0.3 m) at each plant's bbox centre (`/World/lavender_cores/`) stop robots at the crown; rows can't be crossed.
- Inverted groups (`invertFilteredGroups=True`) are ignored in Isaac 6.0; use the explicit pair.

### Real field: `sim/scene/lavender_farm.usd`

- Rebuild: `scripts/make_farm_scene.py` (farm DB `public.object_data` via status_server's `config.yaml`) runs `build_farm_scene.py` with plain `pxr` in the Isaac image (no Kit; running sim untouched). Without the DB: `build_farm_scene.py` on `sim/generated/farm/plants.json`. Map frame = sim world frame.
- Self-contained apart from `../assets/...`: colliders live on abstract class prototypes `/World/Prototypes/*` (traverse with `Usd.PrimAllPrimsPredicate`).
- Plants at DB x/y, scaled uniformly to `PLANT_DIAMETER` 0.99 m (per-axis looks squeezed), lanes ~0.86 m clear; `LAVENDER_SUBSURFACE_OPACITY` 0.7 on the prototype. `FIELD_BOX_MARGIN` (not plant size) sets the border's field box, so resizing plants keeps the border.
- Border (`BORDER`, `scatter_points`): seeded random trees/shrubs/rocks over the 100 m ground beyond per-group distances (±`EDGE_JITTER_M`), no overlaps; clearing (spawn poses, charger) stays open. Lights: `SKY_INTENSITY`, `SUN_INTENSITY`, `SUN_ROTATE_XYZ`.
- Weed barrier (`add_weed_barrier`, visual only): strip per row, `BARRIER_WIDTH` 0.9 m, over per-plant soil mounds (`MOUND_*`) burying stem bases; `hide_grass_under` hides grass on strips. Ground: `/World/ground` is guide purpose (collider), `/World/ground_surface` a textured quad. Textures from `make_farm_textures.py` (host python, numpy + Pillow).
- Materials (`textured_material`): OmniPBR with low `specular_level` + `UsdPreviewSurface` fallback; `UsdPreviewSurface` alone mirrors the sky white at grazing camera angles.
- Arch (`add_arch`): `wood_arch.usd` at `/World/arch` across lane `ARCH_LANE` (0 = northernmost), `ARCH_SETBACK_M` before the shorter row's west end.

### Chargers: `sim/scene/lavender_farm_chargers.usda`

- `scripts/make_charger_scene.py` → `add_chargers.py`: wrapper with the farm as sublayer plus `/World/charging_stations/charger_<id>` per `public.charging_stations` row (yaw = direction the tag faces; `FRAMES` only `map`). `CHARGER_MODELS` (TR-302 → `wibotic_tr302_edge`, tag on -Y: rotateZ = yaw + 90), variant `AprilTag_ID` = `id_<%03d>`, non-80 mm tags scaled. Border items within `CLEAR_RADIUS` 3 m deactivated.
- Only the top layer's `customLayerData` is read: after a farm rebuild, rerun it (or copy `lavender_rows`).

### Farm workers (`FARM_WORKERS=1`, `farm_workers.py`)

- Scenes with `lavender_rows`: two static seated NVIDIA digital humans on crates by row `SPOT_ROW`, kinematic capsule colliders. Assets built in `~/ws/gen_3d_model/scripts/farm_workers/` (see its CLAUDE.md); DH characters come from NVIDIA's S3 server (first load ~1.4 GB, cached in `isaac-cache`). Use helmet-free DH characters (helmet ones have cut hair). Animated workers were rejected (look, RAM, fps).

## Crashes

Minidumps: `isaac-ov-data` volume, `Kit/Isaac-Sim Full/6.0/*.dmp.zip` (latest only). Read the log line just before a crash before blaming load.
