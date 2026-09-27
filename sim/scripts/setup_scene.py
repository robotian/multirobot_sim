"""Isaac Sim fleet scene: N identical Clearpath A300s, each with a RealSense D435i, bridged to ROS 2.

Runs inside the streaming Kit app (isaac-sim.streaming.sh --exec /sim/scripts/setup_scene.py).
Configuration comes from environment variables (see docker-compose.yml):
  NUM_ROBOTS         how many robots to spawn; they are called a300_0000, a300_0001, ...
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

NAMESPACES = [f"a300_{i:04d}" for i in range(int(os.environ.get("NUM_ROBOTS", "3")))]  # must match the robot services in docker-compose.yml
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

URDF_PATH = "/sim/assets/a300/a300.urdf"
USD_DIR = "/sim/generated/a300"
USD_PATH = f"{USD_DIR}/a300/a300.usda"

# Clearpath A300 drive parameters (clearpath_control/config/a300/control/diff_4wd.yaml)
WHEEL_RADIUS = 0.1625
WHEEL_SEPARATION = 0.562
SEPARATION_MULTIPLIER = 1.75  # compensates for skid-steer slip
MAX_LINEAR = 2.0
MAX_ANGULAR = 2.0

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


def import_urdf_if_needed():
    import json

    st = os.stat(URDF_PATH)
    stamp = json.dumps({"settings": IMPORT_SETTINGS, "urdf": [st.st_mtime_ns, st.st_size]}, sort_keys=True)
    stamp_path = f"{USD_DIR}/.import_stamp"
    try:
        cached = open(stamp_path).read()
    except OSError:
        cached = None
    if os.path.exists(USD_PATH) and cached == stamp and not FORCE_REIMPORT:
        log(f"using cached USD {USD_PATH}")
        return
    log("converting A300 URDF -> USD (cached afterwards)")
    import shutil

    shutil.rmtree(f"{USD_DIR}", ignore_errors=True)  # the importer would otherwise write to a300_1/, a300_2/, ...
    os.makedirs(USD_DIR, exist_ok=True)
    enable_extensions(["omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"])
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    cfg = URDFImporterConfig()
    cfg.urdf_path = URDF_PATH
    cfg.usd_path = USD_DIR
    for key, value in IMPORT_SETTINGS.items():
        setattr(cfg, key, value)
    out = URDFImporter(cfg).import_urdf()
    if out != USD_PATH:
        log(f"WARNING: importer wrote {out}, expected {USD_PATH}")
    with open(stamp_path, "w") as f:
        f.write(stamp)


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
    n = len(NAMESPACES)
    for i in range(n):
        y = (i - (n - 1) / 2) * ROBOT_SPACING
        add_box(stage, f"/World/targets/box_{i}", (0.6, 0.6, 0.6), (3.0 + 0.7 * i, y, 0.3), palette[i % len(palette)])
    span = ROBOT_SPACING * n + 2
    add_box(stage, "/World/targets/wall", (0.3, span * 2, 2.0), (9.0, 0, 1.0), (0.75, 0.75, 0.8))
    for j, (px, py) in enumerate([(5.5, -span), (6.5, span), (7.5, 0.0)]):
        add_box(stage, f"/World/targets/pillar_{j}", (0.4, 0.4, 1.5), (px, py, 0.75), palette[(j + 3) % len(palette)])


def spawn_robot(stage, ns, index, count):
    root = f"/World/{ns}"
    prim = stage.DefinePrim(root, "Xform")
    prim.GetReferences().AddReference(USD_PATH)
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


def build_ros_graph(og, usdrt_sdf, chassis, ns, cam_path):
    keys = og.Controller.Keys
    front = ["front_left_wheel_joint", "front_right_wheel_joint"]
    rear = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    articulation_controller = "isaacsim.core.nodes.IsaacArticulationController"

    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("SysTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
        # --- drive: cmd_vel -> wheel velocities (4WD skid steer: both wheels of a side share a command)
        ("CmdVel", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLin", "omni.graph.nodes.BreakVector3"),
        ("BreakAng", "omni.graph.nodes.BreakVector3"),
        ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("DriveFront", articulation_controller),
        ("DriveRear", articulation_controller),
        # --- state: odometry, tf, joint states
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTfOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    values = [
        ("CmdVel.inputs:nodeNamespace", ns),
        ("CmdVel.inputs:topicName", "cmd_vel"),
        ("Diff.inputs:wheelRadius", WHEEL_RADIUS),
        ("Diff.inputs:wheelDistance", WHEEL_SEPARATION * SEPARATION_MULTIPLIER),
        ("Diff.inputs:maxLinearSpeed", MAX_LINEAR),
        ("Diff.inputs:maxAngularSpeed", MAX_ANGULAR),
        ("DriveFront.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveFront.inputs:jointNames", front),
        ("DriveRear.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveRear.inputs:jointNames", rear),
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
        # Diff recomputes wheel speeds when a Twist arrives, the drives apply them every tick
        ("Tick.outputs:tick", "CmdVel.inputs:execIn"),
        ("CmdVel.outputs:execOut", "Diff.inputs:execIn"),
        ("Tick.outputs:tick", "DriveFront.inputs:execIn"),
        ("Tick.outputs:tick", "DriveRear.inputs:execIn"),
        ("CmdVel.outputs:linearVelocity", "BreakLin.inputs:tuple"),
        ("CmdVel.outputs:angularVelocity", "BreakAng.inputs:tuple"),
        ("BreakLin.outputs:x", "Diff.inputs:linearVelocity"),
        ("BreakAng.outputs:z", "Diff.inputs:angularVelocity"),
        ("Diff.outputs:velocityCommand", "DriveFront.inputs:velocityCommand"),
        ("Diff.outputs:velocityCommand", "DriveRear.inputs:velocityCommand"),
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
        {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connections},
    )


def aim_viewport():
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        n = len(NAMESPACES)
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
    ns = NAMESPACES[0]
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(find_prim(stage, f"/World/{ns}", "chassis_link"))
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
        enable_extensions(["isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots.nodes", "omni.graph.action", "omni.graph.nodes"])
        import_urdf_if_needed()

        import omni.graph.core as og
        import usdrt.Sdf as usdrt_sdf

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        build_world(stage)
        for i, ns in enumerate(NAMESPACES):
            root = spawn_robot(stage, ns, i, len(NAMESPACES))
            cam_path = add_camera(stage, root)

            log(f"spawned {ns} at {root}")
            for _ in range(3):
                await app.next_update_async()
            build_ros_graph(og, usdrt_sdf, find_prim(stage, root, "chassis_link"), ns, cam_path)
        aim_viewport()
        set_viewport_resolution()
        for _ in range(10):
            await app.next_update_async()
        omni.timeline.get_timeline_interface().play()
        log(f"simulation running with {len(NAMESPACES)} robots: {', '.join(NAMESPACES)}")
        if os.environ.get("FLEET_DEBUG") == "1":
            asyncio.ensure_future(debug_loop(og))
    except Exception:
        log("FATAL error while building the scene:\n" + traceback.format_exc())


asyncio.ensure_future(main())
