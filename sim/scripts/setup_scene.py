"""Isaac Sim fleet scene: N Clearpath robots, each with a RealSense D435i, bridged to ROS 2.

Runs inside the streaming Kit app (isaac-sim.streaming.sh --exec /sim/scripts/setup_scene.py).
Configuration comes from environment variables (see docker-compose.yml):
  NUM_ROBOTS         how many robots to spawn
  ROBOT_MODELS       comma-separated model per slot (a300/a200/j100/r100, one of MODEL_PARAMS below); only the
                     first NUM_ROBOTS entries are used. Robot i is namespaced "<its model>_%04d" % i, e.g. a
                     Jackal (j100) in slot 1 is j100_0001 -- matches docker-compose.yml's container naming.
  CAMERA_WIDTH/HEIGHT, CAMERA_FRAME_SKIP, FORCE_REIMPORT
"""
import asyncio
import math
import os
import traceback

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

_num_robots = int(os.environ.get("NUM_ROBOTS", "3"))
_models = [m.strip() for m in os.environ.get("ROBOT_MODELS", "a300").split(",") if m.strip()][:_num_robots]
# ROBOTS: one (namespace, model) pair per robot, in slot order -- must match docker-compose.yml's container naming.
ROBOTS = [(f"{model}_{i:04d}", model) for i, model in enumerate(_models)]
CAM_W = int(os.environ.get("CAMERA_WIDTH", "640"))
CAM_H = int(os.environ.get("CAMERA_HEIGHT", "360"))
CAM_FRAME_SKIP = int(os.environ.get("CAMERA_FRAME_SKIP", "0"))  # 0 = publish every simulation frame
# Each camera-equipped robot costs ~12 ms of frame time, so 3 robots cannot render at the app's default 60 Hz.
# The stage runs at SIM_RATE_HZ instead and PhysX substeps at PHYSICS_HZ inside each frame, which keeps
# simulated time in step with wall-clock time.
SIM_RATE_HZ = float(os.environ.get("SIM_RATE_HZ", "20"))
PHYSICS_HZ = int(os.environ.get("PHYSICS_HZ", "60"))
FORCE_REIMPORT = os.environ.get("FORCE_REIMPORT", "0") == "1"
# Which D435i streams to publish. Each one costs main-thread time in the sim, so trim if the frame rate suffers.
# "none" turns the cameras off (no render products at all), which is what gives the streamed viewport its full frame rate.
CAM_STREAMS = set(filter(None, os.environ.get("CAMERA_STREAMS", "color,depth").split(","))) - {"none", "off"}

# Per-model URDF/USD paths, from scripts/gen_urdf.sh's output (sim/assets/<model>/) and the importer's cache
# (sim/generated/<model>/, see import_urdf_if_needed).
MODEL_ASSETS = {
    m: {
        "urdf": f"/sim/assets/{m}/{m}.urdf",
        "usd_dir": f"/sim/generated/{m}",
        "usd_path": f"/sim/generated/{m}/{m}/{m}.usda",
    }
    for m in ("a300", "a200", "j100", "r100")
}

# Decorative lavender plants (SM_Lavender_Nanite_01.usd, default prim /Root). The asset's own layer is
# centimetres (metersPerUnit 0.01) but this stage is metres, and USD does not rescale geometry across that
# boundary by itself, so references to it need an explicit 0.01 scale. LAVENDER_BASE_Z lifts each plant so its
# lowest point (bbox min z, measured once in the source asset) sits on the ground instead of poking through it.
LAVENDER_USD = "/sim/assets/lavender/SM_Lavender_Nanite_01.usd"
LAVENDER_SCALE = 0.006
LAVENDER_BASE_Z = 0.05

# Per-model drive parameters, from each model's real clearpath_control/config/<model>/control/diff_4wd.yaml.
# `chassis_link` is the URDF link the drive/odometry OmniGraph targets -- it must be a prim the URDF importer
# actually gave UsdPhysics.ArticulationRootAPI, checked per model in the imported USD after generating it, not
# assumed from the URDF structure alone (see below).
# a300/r100 each have exactly one direct fixed-jointed child of base_link ("chassis_link") that becomes the
# articulation root. a200's base_link has several direct children (top_chassis_link, inertial_link, the bumper
# mounts, ...) with no single obvious "chassis", so the importer roots the articulation at base_link itself
# instead. j100 *used to* pattern-match a300/r100 (one child, "chassis_link"), but scripts/flatten_urdf.py's
# merge_visual_only_links() now folds its fenders (visual-only, no collision/inertial -- see that function for
# why they needed folding in at all) directly into base_link to fix them visually detaching from the chassis
# when driven; that alone was enough to make base_link "look like a body" to the importer too, moving the
# articulation root there exactly like a200. Moral: re-check ArticulationRootAPI after *any* URDF-shape change,
# not just when adding a new model -- targeting the wrong link fails at runtime ("Articulation controller
# failed") with no error at import time, so nothing catches a wrong guess until the robot won't drive.
# Ridgeback (r100) is Clearpath's holonomic mecanum platform. It drives omnidirectionally: real diff_4wd.yaml-
# style wheel driving (same as the other three models, wheel_separation/separation_multiplier below) handles
# forward/back and rotation via genuine wheel-ground rolling, and BodyDrive (see its comment) separately injects
# *just* the sideways (Vy) component the wheels structurally cannot produce, so it can strafe and combine
# translation with rotation, not just drive forward/back and turn like the others. wheel_positions/wheel_axis/
# mecanum_angles below are Ridgeback's real mecanum geometry, NOT currently used to drive anything (a genuine
# per-wheel mecanum solve was tried first -- see git history / last_session.md -- but a spinning cylinder can't
# produce the sideways thrust it computes, and layering it under BodyDrive's override fought rotation instead of
# helping) -- kept as verified reference in case a future fix finds a use for it.
#   wheel_positions/wheel_axis are measured from sim/assets/r100/r100.urdf (chassis_link -> {front,rear}_rocker
#   -> *_wheel_joint, composing both joints' origins; rocker/wheel joints all have rpy="0 0 0", so a wheel's
#   position relative to chassis_link is just its rocker's xyz plus its own xyz, and its axis is chassis-aligned).
#     front_rocker  xyz=( 0.319, 0,      0.05), front_{left,right}_wheel_joint xyz=(0, +-0.2755, 0), axis=(0,1,0)
#     rear_rocker   xyz=(-0.319, 0,      0.05), rear_{left,right}_wheel_joint  xyz=(0, +-0.2755, 0), axis=(0,1,0)
#   0.319+0.2755 = 0.5945, matching (to rounding) omni_4wd.yaml's own kinematics.sum_of_robot_center_projection_
#   on_X_Y_axis: 0.59 -- confirms these numbers against Clearpath's real control config, not just the mesh.
#   mecanum_angles (degrees, per wheel, same order as wheel_positions) are NOT measured from the URDF --
#   Clearpath's ROS description carries no roller-angle metadata, only the mesh -- so they're derived from the
#   standard mecanum kinematics equations instead, for isaacsim.robot.wheeled_robots.HolonomicController's
#   convention (isaacsim.robot.experimental.wheeled_robots.controllers.HolonomicController._build_base rotates
#   wheelAxis by mecanum_angle about upAxis to get each wheel's effective ground-push direction; its OGN schema
#   *says* mecanumAngles is in radians but the actual implementation applies it with degrees=True -- confirmed
#   by reading that source, not the schema doc, and the values here are already in degrees accordingly).
#   front_left/rear_right share one diagonal roller angle, front_right/rear_left the other, per the standard "X"
#   mecanum wheel arrangement.
MODEL_PARAMS = {
    "a300": dict(chassis_link="chassis_link", drive="diff", wheel_radius=0.1625, wheel_separation=0.562, separation_multiplier=1.75, max_linear=2.0, max_angular=2.0),
    "a200": dict(chassis_link="base_link", drive="diff", wheel_radius=0.1651, wheel_separation=0.555, separation_multiplier=1.875, max_linear=1.0, max_angular=1.0),
    "j100": dict(chassis_link="base_link", drive="diff", wheel_radius=0.098, wheel_separation=0.37559, separation_multiplier=1.5, max_linear=2.0, max_angular=4.0),
    "r100": dict(
        chassis_link="chassis_link", drive="omni", wheel_radius=0.0759, wheel_separation=0.551, separation_multiplier=1.0,
        wheel_positions=[(0.319, 0.2755, 0.05), (0.319, -0.2755, 0.05), (-0.319, 0.2755, 0.05), (-0.319, -0.2755, 0.05)],
        wheel_axis=[0.0, 1.0, 0.0], mecanum_angles=[-135.0, -45.0, -45.0, -135.0],
        max_linear=1.3, max_angular=4.0,
    ),
}

# Ridgeback's real sideways motion. Forward/back and rotation are left entirely to the same real
# DifferentialController + IsaacArticulationController wheel driving every model uses (genuine wheel-ground
# rolling -- reliable even from a standstill, exactly like the other three models). Only linear.y is patched in
# here, since real wheel rolling structurally cannot produce it (see MODEL_PARAMS' r100 comment). Every tick,
# this reads the chassis' CURRENT actual world velocity, decomposes it into the chassis' own body frame,
# replaces just the lateral component with the commanded vy (leaving the forward component -- whatever the real
# diff-drive wheels produced -- untouched), and recomposes back to world frame; angular velocity is left alone
# entirely (not passed to set_velocities at all), so rotation is 100% real wheel physics.
#
# An earlier version set the FULL (vx, vy, wz) velocity directly every tick, bypassing wheel physics for
# everything, not just Vy, and hit the same limitation described there: a *pure*, small in-place rotation
# command (0.5 rad/s alone, verified live) produced ~0 measured rotation, while a larger one (2.0 rad/s) or one
# combined with any translation came through mostly intact. Switching rotation to the real diff-drive wheels
# (this version) did NOT fix that -- it's the same underlying effect either way: from a standstill, turning in
# place needs each wheel's contact patch to break static friction and scrub sideways (skid-steer's normal
# mechanism), and PhysX's contact solver resists that breakaway far more than it resists continuing an already-
# sliding motion, for both a direct velocity override *and* real wheel torque. Real wheel rolling was kept
# anyway since it's at least as effective, and keeps rotation on the same, already-proven code path as the other
# 3 models rather than adding a second special case to BodyDrive.
#
# Articulation.set_velocities()/.get_velocities() (isaacsim.core.experimental.prims) act on a floating-base
# articulation's *root* velocity, which is what a URDF import with fix_base=False gives every robot here --
# confirmed chassis == that root via ArticulationRootAPI, same check MODEL_PARAMS' chassis_link already relies
# on. Constructing Articulation() needs the physics tensor view, which only exists once the timeline is playing,
# so it's created lazily on first compute rather than in setup(), and re-tried on failure instead of latching a
# permanent error.
BODY_DRIVE_SCRIPT = """
import math

import omni.usd
from pxr import UsdGeom, Gf


def setup(db):
    db.per_instance_state.articulation = None


def compute(db):
    state = db.per_instance_state
    if state.articulation is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            state.articulation = Articulation(str(db.inputs.chassisPath))
        except Exception as e:
            db.log_warning(f"BodyDrive: Articulation not ready yet ({e}), retrying next tick")
            return

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(str(db.inputs.chassisPath))
    rot = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractRotation()
    fwd = rot.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))  # chassis' local +X axis, in world frame
    yaw = math.atan2(fwd[1], fwd[0])

    lin, _ang = state.articulation.get_velocities()
    cur_vx_w, cur_vy_w, cur_vz_w = lin.numpy()[0]
    cur_fwd = cur_vx_w * math.cos(yaw) + cur_vy_w * math.sin(yaw)  # decompose actual velocity into body frame

    vy = db.inputs.vy  # commanded lateral speed, body frame -- the only component this overrides
    vx_w = cur_fwd * math.cos(yaw) - vy * math.sin(yaw)
    vy_w = cur_fwd * math.sin(yaw) + vy * math.cos(yaw)
    state.articulation.set_velocities(linear_velocities=[[vx_w, vy_w, cur_vz_w]])
"""

ROBOT_SPACING = 1.6  # m between robots along Y
SPAWN_Z = 0.15  # base_link height: wheel bottoms end up ~1.4 cm above the ground, then it settles


# D435i RGB sensor: 69.4 deg horizontal FOV
HFOV_DEG = 69.4


def find_prim(stage, root, name):
    """First prim called `name` under `root` (the importer's nesting depends on its merge settings)."""
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if prim.GetName() == name:
            return str(prim.GetPath())
    raise RuntimeError(f"no prim named {name!r} under {root}")


def log(msg):
    print(f"[fleet] {msg}", flush=True)


def apply_kit_settings():
    """FLEET_SETTINGS="/path/a=1;/path/b=text" -> carb settings, applied before the scene is built."""
    import carb.settings

    st = carb.settings.get_settings()
    for item in filter(None, os.environ.get("FLEET_SETTINGS", "").split(";")):
        key, _, raw = item.partition("=")
        val = {"true": True, "false": False}.get(raw.lower())
        if val is None:
            for cast in (int, float, str):
                try:
                    val = cast(raw)
                    break
                except ValueError:
                    continue
        st.set(key.strip(), val)
        log(f"setting {key.strip()} = {val!r}")
    # Harmless but per-frame: IsaacReadSystemTime falls back to "now" when it has no sim-time history.
    for channel in ("isaacsim.core.simulation_manager.plugin",):
        st.set(f"/log/channels/{channel}", "error")
    log(f"rtx rendermode = {st.get('/rtx/rendermode')!r}, dlss = {st.get('/rtx/post/dlss/execMode')!r}, "
        f"aa = {st.get('/rtx/post/aa/op')!r}, timeCodesPerSecond-fixed = {st.get('/app/player/useFixedTimeStepping')!r}")


def enable_extensions(names):
    em = omni.kit.app.get_app().get_extension_manager()
    for n in names:
        em.set_extension_enabled_immediate(n, True)


IMPORT_SETTINGS = {
    "merge_fixed_joints": os.environ.get("FLEET_MERGE_FIXED", "0") == "1",
    "merge_mesh": False,
    "collision_from_visuals": False,
    "joint_target_type": "velocity",
    "joint_drive_type": "force",
    "override_joint_stiffness": 0.0,
    "override_joint_damping": 1000.0,
    # The URDF root (base_link) has no inertia, so the generated chassis joint would pin the chassis to the
    # world. Floating base = the robot is free to drive.
    "fix_base": False,
}


def import_urdf_if_needed(model):
    import json

    assets = MODEL_ASSETS[model]
    urdf_path, usd_dir, usd_path = assets["urdf"], assets["usd_dir"], assets["usd_path"]
    st = os.stat(urdf_path)
    stamp = json.dumps({"settings": IMPORT_SETTINGS, "urdf": [st.st_mtime_ns, st.st_size]}, sort_keys=True)
    stamp_path = f"{usd_dir}/.import_stamp"
    try:
        cached = open(stamp_path).read()
    except OSError:
        cached = None
    if os.path.exists(usd_path) and cached == stamp and not FORCE_REIMPORT:
        log(f"using cached USD {usd_path}")
        return
    log(f"converting {model} URDF -> USD (cached afterwards)")
    import shutil

    shutil.rmtree(usd_dir, ignore_errors=True)  # the importer would otherwise write to <model>_1/, <model>_2/, ...
    os.makedirs(usd_dir, exist_ok=True)
    enable_extensions(["omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"])
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    cfg = URDFImporterConfig()
    cfg.urdf_path = urdf_path
    cfg.usd_path = usd_dir
    for key, value in IMPORT_SETTINGS.items():
        setattr(cfg, key, value)
    out = URDFImporter(cfg).import_urdf()
    if out != usd_path:
        log(f"WARNING: importer wrote {out}, expected {usd_path}")
    with open(stamp_path, "w") as f:
        f.write(stamp)


def add_lavender(stage, path, pos, rot_z=0.0):
    """One lavender clump (~2 x 2 x 1.3 m) from LAVENDER_USD. instanceable=True shares the (heavy) mesh data
    and BVH between the copies instead of duplicating it per prim."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(LAVENDER_USD)
    prim.SetInstanceable(True)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2] + LAVENDER_BASE_Z))
    xf.AddRotateZOp().Set(rot_z)
    xf.AddScaleOp().Set(Gf.Vec3d(LAVENDER_SCALE, LAVENDER_SCALE, LAVENDER_SCALE))
    return prim


def add_box(stage, path, size, pos, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def build_world(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    from pxr import PhysxSchema

    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    stage.SetTimeCodesPerSecond(SIM_RATE_HZ)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(10_000_000)
    omni.timeline.get_timeline_interface().set_time_codes_per_second(SIM_RATE_HZ)

    # Grippy ground so the skid-steer wheels get traction
    mat = UsdShade.Material.Define(stage, "/World/Materials/ground_physics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(0.9)
    pm.CreateRestitutionAttr(0.0)
    ground = add_box(stage, "/World/ground", (80, 80, 1), (0, 0, -0.5), (0.35, 0.37, 0.35))
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(350)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(1500)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-50, 20, 0))

    # Something for the cameras to look at: a coloured box in front of each robot, plus a wall and pillars.
    palette = [(0.9, 0.15, 0.15), (0.15, 0.75, 0.2), (0.2, 0.3, 0.95), (0.95, 0.8, 0.1), (0.8, 0.2, 0.8)]
    n = len(ROBOTS)
    for i in range(n):
        y = (i - (n - 1) / 2) * ROBOT_SPACING
        add_box(stage, f"/World/targets/box_{i}", (0.6, 0.6, 0.6), (3.0 + 0.7 * i, y, 0.3), palette[i % len(palette)])
    span = ROBOT_SPACING * n + 2
    add_box(stage, "/World/targets/wall", (0.3, span * 2, 2.0), (9.0, 0, 1.0), (0.75, 0.75, 0.8))
    for j, (px, py) in enumerate([(5.5, -span), (6.5, span), (7.5, 0.0)]):
        add_box(stage, f"/World/targets/pillar_{j}", (0.4, 0.4, 1.5), (px, py, 0.75), palette[(j + 3) % len(palette)])

    # A small lavender row alongside the robots, clear of their driving lane and of the targets/wall/pillars.
    lavender_y = ((n - 1) / 2) * ROBOT_SPACING + 2.0
    for i, x in enumerate([4.0, 6.5, 9.0]):
        add_lavender(stage, f"/World/lavender/plant_{i}", (x, lavender_y, 0.0), rot_z=i * 47.0)


def spawn_robot(stage, ns, model, index, count):
    root = f"/World/{ns}"
    prim = stage.DefinePrim(root, "Xform")
    prim.GetReferences().AddReference(MODEL_ASSETS[model]["usd_path"])
    y = (index - (count - 1) / 2) * ROBOT_SPACING
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0.0, y, SPAWN_Z))
    return root


def add_camera(stage, robot_root):
    """RealSense D435i colour camera. Frame chain: camera_0_link -> optical frame (z fwd, y down) -> USD camera."""
    link = find_prim(stage, robot_root, "camera_0_link")
    optical = f"{link}/camera_0_color_optical_frame"
    xf = UsdGeom.Xform.Define(stage, optical)
    # rows = optical x/y/z axes expressed in the ROS link frame (x fwd, y left, z up)
    m = Gf.Matrix4d(1.0)
    m.SetRow3(0, Gf.Vec3d(0, -1, 0))
    m.SetRow3(1, Gf.Vec3d(0, 0, -1))
    m.SetRow3(2, Gf.Vec3d(1, 0, 0))
    m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in os.environ.get("CAMERA_OFFSET", "0.0,0.0,0.0").split(",")]))
    xf.AddTransformOp().Set(m)

    cam_path = f"{optical}/camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    flip = Gf.Matrix4d(1.0)  # USD camera looks down -Z with +Y up; optical frame looks down +Z with +Y down
    flip.SetRow3(1, Gf.Vec3d(0, -1, 0))
    flip.SetRow3(2, Gf.Vec3d(0, 0, -1))
    UsdGeom.Xformable(cam).AddTransformOp().Set(flip)
    h_aperture = 20.955
    cam.CreateHorizontalApertureAttr(h_aperture)
    cam.CreateVerticalApertureAttr(h_aperture * CAM_H / CAM_W)
    cam.CreateFocalLengthAttr(h_aperture / 2 / math.tan(math.radians(HFOV_DEG) / 2))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30.0))
    return cam_path


def build_ros_graph(og, usdrt_sdf, chassis, ns, cam_path, params):
    keys = og.Controller.Keys
    articulation_controller = "isaacsim.core.nodes.IsaacArticulationController"

    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("SysTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
        # --- drive: cmd_vel -> wheel velocities (diff drive for every model; BodyDrive adds Ridgeback's sideways motion below)
        ("CmdVel", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLin", "omni.graph.nodes.BreakVector3"),
        ("BreakAng", "omni.graph.nodes.BreakVector3"),
        # --- state: odometry, tf, joint states
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTfOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    values = [
        ("CmdVel.inputs:nodeNamespace", ns),
        ("CmdVel.inputs:topicName", "cmd_vel"),
        ("Odom.inputs:chassisPrim", [usdrt_sdf.Path(chassis)]),
        ("PubOdom.inputs:nodeNamespace", ns),
        ("PubOdom.inputs:topicName", "platform/odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubTfOdom.inputs:nodeNamespace", ns),
        ("PubTfOdom.inputs:parentFrameId", "odom"),
        ("PubTfOdom.inputs:childFrameId", "base_link"),
        ("PubJoints.inputs:nodeNamespace", ns),
        ("PubJoints.inputs:topicName", "platform/joint_states"),
        ("PubJoints.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
    ]
    connections = [
        ("Tick.outputs:tick", "CmdVel.inputs:execIn"),
        ("CmdVel.outputs:linearVelocity", "BreakLin.inputs:tuple"),
        ("CmdVel.outputs:angularVelocity", "BreakAng.inputs:tuple"),
    ]
    create_attributes = []

    # Every model, including Ridgeback, drives forward/back and rotation via real diff_4wd.yaml-style wheel
    # rolling -- see MODEL_PARAMS' r100 comment for why Ridgeback's sideways motion needs a different mechanism
    # (BodyDrive, added below) instead of extending this same approach to linear.y.
    front = ["front_left_wheel_joint", "front_right_wheel_joint"]
    rear = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    nodes += [
        ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("DriveFront", articulation_controller),
        ("DriveRear", articulation_controller),
    ]
    values += [
        ("Diff.inputs:wheelRadius", params["wheel_radius"]),
        ("Diff.inputs:wheelDistance", params["wheel_separation"] * params["separation_multiplier"]),
        ("Diff.inputs:maxLinearSpeed", params["max_linear"]),
        ("Diff.inputs:maxAngularSpeed", params["max_angular"]),
        ("DriveFront.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveFront.inputs:jointNames", front),
        ("DriveRear.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveRear.inputs:jointNames", rear),
    ]
    connections += [
        # Diff recomputes wheel speeds when a Twist arrives, the drives apply them every tick
        ("CmdVel.outputs:execOut", "Diff.inputs:execIn"),
        ("Tick.outputs:tick", "DriveFront.inputs:execIn"),
        ("Tick.outputs:tick", "DriveRear.inputs:execIn"),
        ("BreakLin.outputs:x", "Diff.inputs:linearVelocity"),
        ("BreakAng.outputs:z", "Diff.inputs:angularVelocity"),
        ("Diff.outputs:velocityCommand", "DriveFront.inputs:velocityCommand"),
        ("Diff.outputs:velocityCommand", "DriveRear.inputs:velocityCommand"),
    ]

    if params["drive"] == "omni":
        # Ridgeback: BodyDrive patches in the one motion component real wheel rolling structurally cannot
        # produce (sideways/linear.y) -- see its comment (BODY_DRIVE_SCRIPT) for the full reasoning and the
        # comment above MODEL_PARAMS' r100 entry for the underlying wheel-collision-geometry finding.
        nodes += [("BodyDrive", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("BodyDrive.inputs:vy", "double"),
            ("BodyDrive.inputs:chassisPath", "token"),
        ]
        values += [
            ("BodyDrive.inputs:chassisPath", chassis),
            ("BodyDrive.inputs:script", BODY_DRIVE_SCRIPT),
        ]
        connections += [
            ("Tick.outputs:tick", "BodyDrive.inputs:execIn"),
            ("BreakLin.outputs:y", "BodyDrive.inputs:vy"),
        ]

    connections += [
        # state
        ("Tick.outputs:tick", "Odom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubTfOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubJoints.inputs:execIn"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ("Odom.outputs:position", "PubTfOdom.inputs:translation"),
        ("Odom.outputs:orientation", "PubTfOdom.inputs:rotation"),
        ("SysTime.outputs:systemTime", "PubOdom.inputs:timeStamp"),
        ("SysTime.outputs:systemTime", "PubTfOdom.inputs:timeStamp"),
        ("SysTime.outputs:systemTime", "PubJoints.inputs:timeStamp"),
    ]

    # --- camera: one render product feeds every enabled stream
    if cam_path and CAM_STREAMS:
        optical = "camera_0_color_optical_frame"
        nodes += [
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("PubTfCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ]
        values += [
            ("RenderProduct.inputs:cameraPrim", [usdrt_sdf.Path(cam_path)]),
            ("RenderProduct.inputs:width", CAM_W),
            ("RenderProduct.inputs:height", CAM_H),
            # camera_0_link -> optical frame (fixed): quaternion (x, y, z, w)
            ("PubTfCamera.inputs:nodeNamespace", ns),
            ("PubTfCamera.inputs:parentFrameId", "camera_0_link"),
            ("PubTfCamera.inputs:childFrameId", optical),
            ("PubTfCamera.inputs:rotation", [-0.5, 0.5, -0.5, 0.5]),
        ]
        connections += [
            ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
            ("Odom.outputs:execOut", "PubTfCamera.inputs:execIn"),
            ("SysTime.outputs:systemTime", "PubTfCamera.inputs:timeStamp"),
        ]
        for stream, kind in (("color", "rgb"), ("depth", "depth")):
            if stream not in CAM_STREAMS:
                continue
            pub, info = f"Pub{stream.title()}", f"Pub{stream.title()}Info"
            nodes += [(pub, "isaacsim.ros2.bridge.ROS2CameraHelper"), (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
            for name, topic, extra in (
                (pub, f"sensors/camera_0/{stream}/image", [(f"{pub}.inputs:type", kind)]),
                (info, f"sensors/camera_0/{stream}/camera_info", []),
            ):
                values += [
                    (f"{name}.inputs:nodeNamespace", ns),
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", optical),
                    (f"{name}.inputs:frameSkipCount", CAM_FRAME_SKIP),
                    (f"{name}.inputs:useSystemTime", True),
                ] + extra
                connections += [
                    ("RenderProduct.outputs:execOut", f"{name}.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", f"{name}.inputs:renderProductPath"),
                ]

    og.Controller.edit(
        {"graph_path": f"/Graphs/{ns}", "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CREATE_ATTRIBUTES: create_attributes, keys.SET_VALUES: values, keys.CONNECT: connections},
    )


def aim_viewport():
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        n = len(ROBOTS)
        state = ViewportCameraState("/OmniverseKit_Persp")
        state.set_position_world(Gf.Vec3d(-5.0, -ROBOT_SPACING * n * 0.9, 3.2), True)
        state.set_target_world(Gf.Vec3d(3.0, 0.0, 0.3), True)
    except Exception as e:  # cosmetic only
        log(f"could not aim viewport camera: {e}")


def set_viewport_resolution():
    """FLEET_VIEWPORT_RES="1280x720": render the main viewport (the one that is streamed) at a fixed size.

    By default the streamed framebuffer follows the size of the WebRTC client window, and the viewport is path
    traced at that size, which is expensive on a large window.
    """
    res = os.environ.get("FLEET_VIEWPORT_RES", "").lower()
    if not res:
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        w, h = (int(v) for v in res.split("x"))
        vp = get_active_viewport()
        vp.fill_frame = False
        vp.resolution = (w, h)
        log(f"viewport resolution set to {vp.resolution}, fill_frame={vp.fill_frame}")
    except Exception as e:
        log(f"could not set viewport resolution: {e}")


async def debug_loop(og):
    """FLEET_DEBUG=1: log the command chain and chassis pose of the first robot every 2 s."""
    from pxr import UsdGeom as _G
    ns, model = ROBOTS[0]
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(find_prim(stage, f"/World/{ns}", MODEL_PARAMS[model]["chassis_link"]))
    app = omni.kit.app.get_app()
    from pxr import Usd as _U
    cache = _U.__dict__  # noqa: F841
    bbox = _G.BBoxCache(0, [_G.Tokens.default_, _G.Tokens.guide, _G.Tokens.proxy, _G.Tokens.render])
    lows = []
    for prim in _U.PrimRange(stage.GetPrimAtPath(f"/World/{ns}")):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            r = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            if not r.IsEmpty():
                lows.append((round(r.GetMin()[2], 3), round(r.GetMax()[2], 3), prim.GetTypeName(), str(prim.GetPath()).split(ns + "/")[-1][-90:]))
    for row in sorted(lows)[:12]:
        log(f"debug lowest collider z: {row}")
    import time as _t

    tl = omni.timeline.get_timeline_interface()
    w0, s0 = _t.time(), tl.get_current_time()
    while True:
        n_frames = 120
        for _ in range(n_frames):
            await app.next_update_async()
        w1, s1 = _t.time(), tl.get_current_time()
        log(f"debug rtf={(s1 - s0) / (w1 - w0):.2f} render_fps={n_frames / (w1 - w0):.1f}")
        w0, s0 = w1, s1
        g = f"/Graphs/{ns}"
        try:
            vals = {k: og.Controller.attribute(f"{g}/{k}").get() for k in (
                "CmdVel.outputs:linearVelocity", "CmdVel.outputs:angularVelocity", "Diff.outputs:velocityCommand")}
        except Exception as e:
            vals = f"attr read failed: {e}"
        pos = _G.Xformable(chassis).ComputeLocalToWorldTransform(0).ExtractTranslation()
        log(f"debug {ns}: chassis_pos={tuple(round(v, 3) for v in pos)} {vals}")


async def main():
    try:
        app = omni.kit.app.get_app()
        for _ in range(5):
            await app.next_update_async()
        apply_kit_settings()
        enable_extensions(["isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots.nodes", "omni.graph.action", "omni.graph.nodes", "omni.graph.scriptnode"])
        for model in dict.fromkeys(model for _, model in ROBOTS):  # each distinct model once, first-seen order
            import_urdf_if_needed(model)

        import omni.graph.core as og
        import usdrt.Sdf as usdrt_sdf

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        build_world(stage)
        for i, (ns, model) in enumerate(ROBOTS):
            root = spawn_robot(stage, ns, model, i, len(ROBOTS))
            cam_path = add_camera(stage, root)

            log(f"spawned {ns} ({model}) at {root}")
            for _ in range(3):
                await app.next_update_async()
            chassis = find_prim(stage, root, MODEL_PARAMS[model]["chassis_link"])
            build_ros_graph(og, usdrt_sdf, chassis, ns, cam_path, MODEL_PARAMS[model])
        aim_viewport()
        set_viewport_resolution()
        for _ in range(10):
            await app.next_update_async()
        omni.timeline.get_timeline_interface().play()
        log(f"simulation running with {len(ROBOTS)} robots: {', '.join(ns for ns, _ in ROBOTS)}")
        if os.environ.get("FLEET_DEBUG") == "1":
            asyncio.ensure_future(debug_loop(og))
    except Exception:
        log("FATAL error while building the scene:\n" + traceback.format_exc())


asyncio.ensure_future(main())
