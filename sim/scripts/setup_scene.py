"""Isaac Sim fleet scene: N Clearpath robots, each with a RealSense D435i, bridged to ROS 2.

Runs inside the streaming Kit app (isaac-sim.streaming.sh --exec /sim/scripts/setup_scene.py).
Configuration comes from environment variables (see docker-compose.yml):
  NUM_ROBOTS         how many robots to spawn
  ROBOT_MODELS       comma-separated model per slot (a300/a200/j100/r100/a real robot id like j100_0921, one of
                     MODEL_PARAMS below); only the first NUM_ROBOTS entries are used. Robot i is namespaced
                     "<its model>_%04d" % i, e.g. a Jackal (j100) in slot 1 is j100_0001 -- matches docker-
                     compose.yml's container naming -- except a real robot id, used directly as its own
                     namespace with no slot suffix (j100_0921, not j100_0921_0000): it's one specific physical
                     robot, not a generic model needing a slot index to stay unique.
  CAMERA_WIDTH/HEIGHT, CAMERA_FRAME_SKIP, FORCE_REIMPORT
"""
import asyncio
import math
import os
import random
import re
import traceback

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

_num_robots = int(os.environ.get("NUM_ROBOTS", "3"))
_models = [m.strip() for m in os.environ.get("ROBOT_MODELS", "a300").split(",") if m.strip()][:_num_robots]
# ROBOTS: one (namespace, model) pair per robot, in slot order -- must match docker-compose.yml's container/
# ROBOT_NAMESPACE naming (and its own real-robot special case, see robot/entrypoint.sh's own copy of this same
# logic): a real robot's own id (contains "_", e.g. j100_0921) already *is* its correct namespace -- it's one
# specific physical robot with one fixed real identity, not a generic model that needs a slot index to stay
# unique -- so it's used directly; only generic catalog models (a300, ...) get the slot-indexed "<model>_%04d".
ROBOTS = [(model if "_" in model else f"{model}_{i:04d}", model) for i, model in enumerate(_models)]
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
# Every model scripts/gen_urdf.sh has produced (sim/assets/<m>/<m>.urdf) is available -- gen_urdf.sh itself takes
# the real robots from robot_data/<id>/, so a new robot needs no list edited here (only a MODEL_PARAMS entry).
# The fixed names are kept as a floor so a listing problem can never make a known model disappear.
_ASSET_ROOT = "/sim/assets"
_GENERATED = [d for d in (os.listdir(_ASSET_ROOT) if os.path.isdir(_ASSET_ROOT) else [])
              if os.path.isfile(f"{_ASSET_ROOT}/{d}/{d}.urdf")]
MODEL_ASSETS = {
    m: {
        "urdf": f"/sim/assets/{m}/{m}.urdf",
        "usd_dir": f"/sim/generated/{m}",
        "usd_path": f"/sim/generated/{m}/{m}/{m}.usda",
    }
    for m in sorted({"a300", "a200", "j100", "r100", "j100_0921", "j100_0936", "a200_0333", "a300_00036", "j100_0922"}
                    | set(_GENERATED))
}

# Decorative lavender plants (SM_Lavender_Nanite_01.usd, default prim /Root). The asset's own layer is
# centimetres (metersPerUnit 0.01) but this stage is metres, and USD does not rescale geometry across that
# boundary by itself, so references to it need an explicit 0.01 scale. LAVENDER_BASE_Z lifts each plant so its
# lowest point (bbox min z, measured once in the source asset) sits on the ground instead of poking through it.
SKY_HDR = "/sim/assets/sky/farm_field_puresky_2k.hdr"
SKY_INTENSITY = 400
TREE_USDS = [f"/sim/assets/trees/{n}.usd" for n in ("Douglas_Fir", "Black_Oak", "Douglas_Fir")]
TREE_COUNT = 36
TREE_DIST = 22.0  # the cameras clip at 30 m, so the tree line has to sit inside that
TREE_HEIGHT = (7.0, 11.0)  # m
SHRUB_USDS = [f"/sim/assets/shrubs/{n}.usd" for n in ("Rhododendron", "Lilac", "Goldflame_Spirea", "Barberry")]
SHRUB_COUNT = 70
SHRUB_DIST = 21.0
SHRUB_HEIGHT = (1.0, 2.2)  # m
ROCK_USDS = [f"/sim/assets/rocks/rock_small_{i:02d}.usda" for i in range(1, 7)]
ROCK_COUNT = 30
ROCK_DIST = 19.0
ROCK_HEIGHT = (0.4, 1.0)  # m
GROUND_SOIL_COLOR = (0.16, 0.10, 0.06)  # dark brown soil under the grass
GROUND_COVER_USD = "/sim/assets/Ground_cover/ground_cover.usd"
GROUND_COVER_SCALE_XY = 1.0
GROUND_COVER_SCALE_Z = 0.6
LAVENDER_USD = "/sim/assets/lavender/SM_Lavender_Nanite_01.usd"
LAVENDER_SCALE = 0.006
LAVENDER_BASE_Z = 0.05
LAVENDER_ROW_OFFSET = 1.6   # row centre distance beyond the outermost robot lane, m
LAVENDER_ROW_X0 = 2.0
LAVENDER_PLANT_PITCH = 1.0  # plants are ~1.2 m wide, so they overlap into a hedge
LAVENDER_ROW_PLANTS = 10

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
    # Real MTU robots. wheel_radius/separation_multiplier/max_linear/max_angular are this specific robot's own
    # real calibrated values (platform.extras.ros_parameters.platform_velocity_controller in the real
    # robot.yaml): wheel_radius = generic j100's 0.098 * the real left/right_wheel_radius_multiplier (0.95, both
    # sides equal); wheel_separation is the same physical constant as generic j100 (a hardware geometry fact,
    # not something the real robot's software recalibrates); separation_multiplier 1.17 *replaces* generic
    # j100's 1.5 (the real robot.yaml's value is the actual calibrated one, not an additional factor on top);
    # max_linear/max_angular 1.0/1.0 replace generic j100's 2.0/4.0 the same way. Both real robots have
    # identical values here (confirmed: diffing the two real robot.yaml files shows no difference in this
    # section). camera_optical_link/imu_link/gps_links/has_arm/lidar2d_link (used by build_ros_graph/add_camera,
    # not by the 4 Clearpath-catalog models above) describe this robot's real, richer sensor/arm loadout: a
    # Stereolabs ZED2i (already gives a correctly-oriented ROS optical frame from its own xacro, unlike the
    # D435i models above which build one by hand in add_camera -- see camera_optical_link), a Microstrain IMU,
    # dual SwiftNav Duro GPS, and a Kinova Gen3 Lite arm + 2F Lite gripper; j100_0936 additionally has the real
    # SICK LMS1xx 2D lidar the real 0921 doesn't carry (lidar2d_link is None there) -- though add_lidar2d
    # currently does nothing with it regardless of model, since the only 2D-lidar pipeline this Isaac Sim
    # version has (RTX Lidar) is broken in this specific install; see add_lidar2d's own docstring.
    #
    # j100_0921 now generates from its own real robot_data/j100_0921/robot.yaml directly (see robot/entrypoint.sh
    # and scripts/gen_urdf.sh), with platform.extras (mtu32_description's own custom xacro) genuinely built and
    # included -- no more stripped-template workaround, and no more fake top_mount->default_mount alias link
    # (mtu32_description's xacro genuinely defines top_mount_link, real mesh + collision, parented on
    # default_mount). That changes imu_link from the previous chassis_link: imu_1_link/imu_1_base_link (the
    # sensor's own mount, still visual-only) now merge only as far up as the real top_mount_link (which has real
    # collision so merge_visual_only_links stops there, unlike before when the whole chain up to chassis_link was
    # visual-only and got merged away entirely) -- confirmed in the actual flattened URDF (grep for "imu_1": no
    # remaining reference, i.e. fully merged; top_shelf_link's own joint parent is top_mount_link directly, not
    # chassis_link), not assumed. chassis_link is still base_link (fenders are unchanged; re-confirmed live via a
    # successful drive test with no "Articulation controller failed" error). camera_optical_link/gps_links/
    # has_arm are unaffected -- they come from clearpath_sensors_description's own sensor macros, unrelated to
    # mtu32_description.
    "j100_0921": dict(
        chassis_link="base_link", drive="diff", wheel_radius=0.098 * 0.95, wheel_separation=0.37559, separation_multiplier=1.17,
        max_linear=1.0, max_angular=1.0,
        camera_optical_link="camera_0_left_camera_frame_optical",
        imu_link="top_mount_link", imu_index=1, gps_links=["gps_1_link", "gps_2_link"], has_arm=True,
        # RealSense D405 on arm_0_end_effector_link (mtu32_description's camera_1): its own URDF link, mounted
        # with the ROS link convention (x fwd), so the default hand-built optical frame path applies.
        wrist_camera=True,
    ),
    # j100_0936 still uses the old stripped robot.j100_0936.yaml.tmpl (platform.extras dropped, fake
    # top_mount->default_mount alias) -- its own robot_data folder isn't available to migrate it the same way
    # j100_0921 was; out of scope until it reappears or this is asked for specifically. imu_link is chassis_link
    # here for that reason (see the merge-chain explanation this comment used to carry for both robots): imu_1_
    # link and every link up to the chassis (imu_1_base_link, the *fake* top_mount_link, default_mount) are all
    # visual-only, so merge_visual_only_links folds the whole chain into chassis_link.
    "j100_0936": dict(
        chassis_link="base_link", drive="diff", wheel_radius=0.098 * 0.95, wheel_separation=0.37559, separation_multiplier=1.17,
        max_linear=1.0, max_angular=1.0,
        camera_optical_link="camera_0_left_camera_frame_optical",
        imu_link="chassis_link", imu_index=1, gps_links=["gps_1_link", "gps_2_link"], has_arm=True,
        lidar2d_link="lidar2d_0_laser",
    ),
    # Two more real MTU robots, same "use the real robot_data/<serial>/robot.yaml directly" pipeline as
    # j100_0921 -- unlike the Jackals, neither references any private package (a200_0333's platform.extras.urdf
    # is an empty {}; a300_00036 has no extras key at all), so there was no missing-package workaround to retire
    # and no fake link to alias; generate_description succeeded first try. Drivetrain (wheel_radius/separation/
    # separation_multiplier/max_linear/max_angular) is identical to the generic a200/a300 entries above -- neither
    # real robot.yaml has a platform_velocity_controller override the way the real Jackals do, so there's no
    # recalibrated value to carry.
    #
    # a200_0333: camera is a plain "d435" (not "d435i" like every other model here) via sensors.camera + a
    # mounts.fath_pivot adapter (a Clearpath mount type not seen elsewhere in this project) -- still produces the
    # same camera_0_link name add_camera's default (hand-built optical frame) path already expects, confirmed in
    # the flattened URDF, so no camera_optical_link override needed, same code path as the 4 generic models.
    # lidar2d (hokuyo_ust) and lidar3d (velodyne VLP16, a new sensor category -- see add_lidar3d) are both
    # present in the URDF but neither is simulated: both are RTX Lidar in this Isaac Sim version, and that whole
    # extension is broken in this specific install (see add_lidar2d's own docstring) -- not specific to 2D lidar.
    "a200_0333": dict(
        chassis_link="base_link", drive="diff", wheel_radius=0.1651, wheel_separation=0.555, separation_multiplier=1.875,
        max_linear=1.0, max_angular=1.0,
        lidar2d_link="lidar2d_0_laser", lidar3d_link="lidar3d_0_laser",
    ),
    # a300_00036 (real robot.yaml updated after this entry's first version, which had only a Phidgets IMU and no
    # camera): now a full MTU field robot -- D435 (sensors.camera, via mounts.fath_pivot on top_plate_mount_c1,
    # same camera_0_link name/hand-built-optical-frame path as a200_0333, so no camera_optical_link override), a
    # Hokuyo UST 2D lidar (lidar2d_0_laser, on wireless_charger_link), dual SwiftNav Duro GPS (gps_0_link/
    # gps_1_link -- a300 numbers its GPS from 0, unlike the Jackals' gps_1/gps_2), a Microstrain IMU, and a Kinova
    # Gen3 Lite arm + 2F Lite gripper on top_plate_mount_e9 (has_arm), plus platform.extras -> mtu32_description's
    # generic urdf/robot_description.urdf.xacro. Drivetrain unchanged (no platform_velocity_controller override).
    # imu_link is top_plate_link: the Microstrain (imu_0_link, imu_index 0 -- a300 has no platform-default IMU
    # occupying slot 0) and its empty mount frames are visual-only, so merge_visual_only_links folds them into
    # the nearest link with real collision, top_plate_link -- confirmed in the flattened URDF (no "imu" string
    # survives; top_plate_link's visuals are top_plate.dae plus one extra box), not assumed.
    # chassis_link is "chassis_link" again (it was base_link while the Phidgets IMU box gave base_link geometry):
    # base_link is now a pure frame, and flatten_urdf.py's weld_empty_root_children re-parents its extra collision
    # children (arch/estop/button/eth, both GPS, wireless charger + lidar) onto chassis_link. Without that pass
    # the importer rooted 8 separate articulations, pinned to the world (physics:body0 = the robot root prim),
    # and the drive graph failed on base_link ("not a valid rigid body or articulation root"). Re-verified live
    # per this project's rule for chassis_link after any URDF shape change.
    "a300_00036": dict(
        chassis_link="chassis_link", drive="diff", wheel_radius=0.1625, wheel_separation=0.562, separation_multiplier=1.75,
        max_linear=2.0, max_angular=2.0,
        imu_link="top_plate_link", imu_index=0, gps_links=["gps_0_link", "gps_1_link"], has_arm=True,
        lidar2d_link="lidar2d_0_laser",
    ),
    # a200_0284: an A200 with the MTU field-robot loadout -- Microstrain IMU (imu_0), D435 (via sensors.camera on
    # front_camera_mount_link, so the usual camera_0_link + hand-built optical frame), dual Duro GPS (gps_0/gps_1),
    # SICK LMS1xx 2D lidar (lidar2d_0_laser, on top_plate_base_link) and a Kinova Gen3 *7-DOF* arm (arm_0_joint_1
    # .. 7, no gripper: that section of the yaml is commented out) on arm_mount_plate_link. The arm graph and
    # configure_arm_drives match by name/articulation, so 7 joints need nothing special.
    # Drivetrain: the real robot.yaml overrides platform_velocity_controller with wheel_radius 0.157 and
    # left/right radius multipliers 1.01 / 0.96 (wheel_separation_multiplier 1.875, max 1.0 m/s and 1.0 rad/s).
    # This sim's DifferentialController takes a single radius, so wheel_radius = 0.157 * mean(1.01, 0.96) = 0.1546;
    # the left/right asymmetry itself can't be represented (same "real calibrated value" approach as j100_0921).
    # imu_link is base_link: the Microstrain and its mount frames are visual-only, so merge_visual_only_links folds
    # them into base_link (confirmed: base_link gains exactly one extra "box" visual vs. the plain a200 URDF).
    # chassis_link="base_link" like a200 (base_link has real visual+collision and several direct children, so the
    # importer roots the articulation there) -- to be re-verified live with a drive test.
    "a200_0284": dict(
        chassis_link="base_link", drive="diff", wheel_radius=0.157 * (1.01 + 0.96) / 2, wheel_separation=0.555,
        separation_multiplier=1.875, max_linear=1.0, max_angular=1.0,
        imu_link="base_link", imu_index=0, gps_links=["gps_0_link", "gps_1_link"], has_arm=True,
        lidar2d_link="lidar2d_0_laser",
    ),
    # j100_0922: same real robot.yaml lineage as j100_0921 (identical camera/IMU/GPS/links sections, same
    # platform_velocity_controller values) but with its entire manipulators.arms section commented out -- no
    # arm/gripper at all, so has_arm is omitted (falsy) and configure_arm_drives is never called for it.
    # chassis_link/imu_link are the same as j100_0921 for the same reasons (identical URDF structure otherwise,
    # re-verified live via drive test since chassis_link has flipped on unrelated-looking changes before).
    # Real upstream bug found generating this one, fixed in scripts/flatten_urdf.py (prune_dangling_joints, not
    # specific to this robot): mtu32_description's own xacro unconditionally mounts a second camera (camera_1,
    # a RealSense D405) on arm_0_end_effector_link, assuming every Jackal running it has the Kinova arm -- with
    # no arm here, that link is never defined anywhere, leaving camera_1's mount joint (and its own child joint)
    # dangling references that Isaac's importer would have choked on; both are now dropped during flattening.
    "j100_0922": dict(
        chassis_link="base_link", drive="diff", wheel_radius=0.098 * 0.95, wheel_separation=0.37559, separation_multiplier=1.17,
        max_linear=1.0, max_angular=1.0,
        camera_optical_link="camera_0_left_camera_frame_optical",
        imu_link="top_mount_link", imu_index=1, gps_links=["gps_1_link", "gps_2_link"],
        # Its mtu32 top frame, GPS spheres and ~13 frame links have no <inertial>, so PhysX weighs the robot 75 kg
        # (URDF: 18.4 kg; top_mount_link 29.8 kg from its mesh at 1000 kg/m^3) with all of it high up: it tips
        # backwards at 0.2 m/s. See fix_massless_bodies.
        massless_density=100.0, frame_mass=0.02,
    ),
}

# Velocity calibration. The open-loop wheel kinematics above (MODEL_PARAMS) cannot hold the commanded speed on a
# skid-steer robot: a 0.2 rad/s in-place turn did not move at all (static-friction breakaway), 0.5 turned 0.2-0.6x,
# a200's forward speed was +5%, j100_0921's reverse 0.6-0.9x. VEL_CTL_SCRIPT therefore closes a small PI loop per
# robot around the DifferentialController (feed-forward = the commanded value, so the wheel params above still
# matter), on the speed the chassis actually achieves; scripts/calibrate_velocity.py measures the result
# (every model within ~5-8% over 0.1-1.0 m/s and 0.1-1.0 rad/s at 0.25 m/s^2 / 0.5 rad/s^2).
# (kp_v, ki_v, int_limit_v [m/s], kp_w, ki_w, int_limit_w [rad/s]); per-model override via MODEL_PARAMS["vel_ctl"].
# ki_w 10 is what removed the low-rate dead zone (5 left 0.1 rad/s at 0.75-0.85); the linear gains are kept low
# because the combination kp 0.3 + ki 3 is about the most a light Jackal tolerates before its pitch oscillates.
VELCTL_DEFAULT = (0.3, 3.0, 1.0, 0.5, 10.0, 3.0)
VELCTL_LATERAL = (0.3, 6.0, 1.5)  # kp, ki, integrator limit of the sideways channel (Ridgeback's BodyDrive)
VELCTL_ENABLED = os.environ.get("VELCTL", "1") == "1"  # VELCTL=0: open loop, for A/B comparisons
# The feedback may command more than the robot's own limit (max_linear/max_angular clamp the *command* only).
VELCTL_HEADROOM = 4.0
# Wheel parking brake (WHEEL_BRAKE_SCRIPT). The wheel drives are pure velocity dampers (stiffness 0, damping 1000),
# which resist speed but never hold a position, so at rest the robots rocked forward/back by ~2 cm (a300_00036: 69
# direction reversals and 0.4 deg of yaw in 60 s with no cmd_vel at all; Diff's wheel targets were exactly 0): the
# solver's small alternating contact torques integrate into wheel rotation. Real motor controllers hold position at
# zero speed (their integral term acts as a spring). With no command, once every wheel is slower than
# WHEEL_BRAKE_LATCH_SPEED (or after WHEEL_BRAKE_LATCH_DELAY s), the brake latches the wheels' current angles as
# position targets with WHEEL_BRAKE_STIFFNESS; any non-zero command releases it (stiffness back to 0), so driving
# and the velocity calibration are unchanged.
WHEEL_BRAKE = True
WHEEL_BRAKE_STIFFNESS = 1.0e4  # N*m/rad per wheel: a 5 N*m disturbance holds within 0.5 mrad
WHEEL_BRAKE_LATCH_SPEED = 0.05  # rad/s
WHEEL_BRAKE_LATCH_DELAY = 1.0  # s
# Physics time advanced per rendered frame: Kit runs floor(PHYSICS_HZ / SIM_RATE_HZ) fixed PhysX steps per frame
# (22 Hz frames, 60 Hz physics -> 2 steps = 1/30 s, not the 1/22 s the timeline counts). Measured: the position
# change per published odom message is 1/30 s of commanded velocity at SIM_RATE_HZ=22, for every model.
# Consequence: real-time factor = render fps x this, e.g. 14 fps -> 0.47; SIM_RATE_HZ near 60/k (20, 15) would
# make frames whole multiples of the step.
PHYSICS_FRAME_DT = max(1, int(PHYSICS_HZ // SIM_RATE_HZ)) / PHYSICS_HZ

# Ridgeback's real sideways motion (and, see below, its yaw rate). Forward/back is left entirely to the same real
# DifferentialController + IsaacArticulationController wheel driving every model uses (genuine wheel-ground
# rolling -- reliable even from a standstill, exactly like the other three models). Only linear.y is patched in
# here, since real wheel rolling structurally cannot produce it (see MODEL_PARAMS' r100 comment). Every tick,
# this reads the chassis' CURRENT actual world velocity, decomposes it into the chassis' own body frame,
# replaces just the lateral component with the commanded vy (leaving the forward component -- whatever the real
# diff-drive wheels produced -- untouched), and recomposes back to world frame. Since the velocity-calibration
# work it also sets the yaw rate (VelCtl's feedback-corrected command): rolling the cylinder wheels alone turned
# the Ridgeback erratically (stick-slip), 0.74-1.0 of the command on repeat runs, now within ~1%.
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

    lin, ang = state.articulation.get_velocities()
    cur_wx, cur_wy, _cur_wz = ang.numpy()[0]
    cur_vx_w, cur_vy_w, cur_vz_w = lin.numpy()[0]
    cur_fwd = cur_vx_w * math.cos(yaw) + cur_vy_w * math.sin(yaw)  # decompose actual velocity into body frame

    vy = db.inputs.vy  # commanded lateral speed, body frame -- the only component this overrides
    vx_w = cur_fwd * math.cos(yaw) - vy * math.sin(yaw)
    vy_w = cur_fwd * math.sin(yaw) + vy * math.cos(yaw)
    # yaw rate too (VelCtl's feedback-corrected value): rolling the cylinder wheels alone turns the Ridgeback
    # erratically (stick-slip), measured 0.74-1.0 of the command on repeat runs
    state.articulation.set_velocities(
        linear_velocities=[[vx_w, vy_w, cur_vz_w]], angular_velocities=[[cur_wx, cur_wy, float(db.inputs.wz)]]
    )
"""

WHEEL_BRAKE_SCRIPT = """
import time

import numpy as np


def setup(db):
    st = db.per_instance_state
    st.art = None
    st.latched = False
    st.idle_since = None


def compute(db):
    st = db.per_instance_state
    if st.art is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            st.art = Articulation(str(db.inputs.chassisPath))
            names = list(st.art.dof_names)
            st.dofs = [names.index(n) for n in str(db.inputs.wheelNames).split(",") if n in names]
        except Exception as e:
            db.log_warning(f"WheelBrake: articulation not ready yet ({e}), retrying next tick")
            st.art = None
            return
    commanded = max(abs(float(db.inputs.cmd_v)), abs(float(db.inputs.cmd_w)), abs(float(db.inputs.cmd_y))) > 1e-4
    if commanded:
        st.idle_since = None
        if st.latched:
            st.art.set_dof_gains(stiffnesses=np.zeros(len(st.dofs)), dof_indices=st.dofs)
            st.latched = False
        return
    if st.latched:
        return
    now = time.monotonic()
    if st.idle_since is None:
        st.idle_since = now
    speed = np.abs(st.art.get_dof_velocities(dof_indices=st.dofs).numpy()[0]).max()
    if speed < float(db.inputs.latchSpeed) or now - st.idle_since > float(db.inputs.latchDelay):
        pos = st.art.get_dof_positions(dof_indices=st.dofs).numpy()
        st.art.set_dof_position_targets(pos, dof_indices=st.dofs)
        st.art.set_dof_gains(stiffnesses=np.full(len(st.dofs), float(db.inputs.stiffness)), dof_indices=st.dofs)
        st.latched = True
"""

# Velocity feedback between cmd_vel and the DifferentialController (see VELCTL_* below): PI on the chassis'
# measured body-frame forward speed and yaw rate, plus feed-forward (the commanded value itself).
VEL_CTL_SCRIPT = """
import math

import omni.usd
from pxr import UsdGeom, Gf


def setup(db):
    st = db.per_instance_state
    st.prev = None
    st.int_v = 0.0
    st.int_w = 0.0
    st.int_y = 0.0
    st.mv = 0.0
    st.mw = 0.0
    st.my = 0.0


def _chan(cmd, meas, integ, kp, ki, dt, lim):
    if abs(cmd) < 1e-4:
        return 0.0, 0.0
    e = cmd - meas
    integ = max(-lim, min(lim, integ + ki * e * dt))
    return cmd + kp * e + integ, integ


def compute(db):
    st = db.per_instance_state
    cmd_v = max(-db.inputs.max_v, min(db.inputs.max_v, float(db.inputs.cmd_v)))
    cmd_w = max(-db.inputs.max_w, min(db.inputs.max_w, float(db.inputs.cmd_w)))
    cmd_y = max(-db.inputs.max_v, min(db.inputs.max_v, float(db.inputs.cmd_y)))
    # Measured from the chassis' pose change per physics frame, NOT from PhysX's reported velocity: the reported
    # angular velocity read ~0.02-0.03 rad/s above the actual yaw change (a 0.1 rad/s command "measured" 0.1 while
    # the robot turned 0.078), so a loop closed on it settled 20% low at low rates.
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(str(db.inputs.chassisPath))).ComputeLocalToWorldTransform(0)
    pos = xf.ExtractTranslation()
    fwd = xf.ExtractRotation().TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    yaw = math.atan2(fwd[1], fwd[0])
    dt = float(db.inputs.dt)
    if st.prev is not None:
        px, py, pyaw = st.prev
        a = float(db.inputs.alpha)
        v = ((pos[0] - px) * math.cos(yaw) + (pos[1] - py) * math.sin(yaw)) / dt
        w = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)) / dt
        y = (-(pos[0] - px) * math.sin(yaw) + (pos[1] - py) * math.cos(yaw)) / dt
        st.mv += a * (v - st.mv)
        st.mw += a * (w - st.mw)
        st.my += a * (y - st.my)
    st.prev = (pos[0], pos[1], yaw)
    db.outputs.odom_lin = (st.mv, st.my, 0.0)
    db.outputs.odom_ang = (0.0, 0.0, st.mw)
    if not db.inputs.enabled:
        db.outputs.out_v = cmd_v
        db.outputs.out_w = cmd_w
        db.outputs.out_y = cmd_y
        return
    db.outputs.out_y, st.int_y = _chan(cmd_y, st.my, st.int_y, db.inputs.kp_y, db.inputs.ki_y, dt, db.inputs.lim_y)
    db.outputs.out_v, st.int_v = _chan(cmd_v, st.mv, st.int_v, db.inputs.kp_v, db.inputs.ki_v, dt, db.inputs.lim_v)
    db.outputs.out_w, st.int_w = _chan(cmd_w, st.mw, st.int_w, db.inputs.kp_w, db.inputs.ki_w, dt, db.inputs.lim_w)
"""

# Real MTU robots' Microstrain IMU (see MODEL_PARAMS' imu_link comment for why it's physically attached to
# chassis_link, not the real imu_1_link name). isaacsim.sensors.experimental.physics has no OGN "read" node, so
# this authors the actual IsaacImuSensor prim on first tick (lazily, same reasoning as BodyDrive's Articulation:
# needs the physics tensor view, which only exists once playing) and reads it every tick after.
# IMUSensor.get_data()'s orientation is [w, x, y, z]; ROS2PublishImu's quatd[4] input wants IJKR (x, y, z, w).
IMU_READ_SCRIPT = """
def setup(db):
    db.per_instance_state.sensor = None


def compute(db):
    state = db.per_instance_state
    if state.sensor is None:
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        path = str(db.inputs.imuPath)
        try:
            IMU.create(path)
            state.sensor = IMUSensor(path)
        except Exception as e:
            db.log_warning(f"ImuRead: sensor not ready yet ({e}), retrying next tick")
            return

    frame = state.sensor.get_data()
    o = frame["orientation"]
    db.outputs.orientation = [float(o[1]), float(o[2]), float(o[3]), float(o[0])]
    db.outputs.linearAcceleration = [float(v) for v in frame["linear_acceleration"]]
    db.outputs.angularVelocity = [float(v) for v in frame["angular_velocity"]]
"""

# Real MTU robots' dual SwiftNav Duro GPS: no real satellite geometry (this is a simulation, not an RF model --
# same simplification Gazebo's own GPS plugins make), just a flat-earth/equirectangular projection of the
# chassis' actual simulated world XY around a fixed reference origin, computed fresh every tick from each GPS
# antenna's own real link position (not just the chassis) so the two antennas' real ~0.56m separation still
# shows up as a small, physically meaningful difference between the two NavSatFix readings -- SwiftNav Duro
# pairs are commonly used for exactly that, deriving heading from dual-antenna GPS. Origin: Michigan Tech's
# Houghton, MI campus (~47.1211, -88.5455, ~326m elevation) -- ties the fake origin to the real institution
# these robots belong to (mtu32_description, *.sabu.mtu.edu hostnames in the real robot.yaml) rather than an
# arbitrary placeholder. Sim world X/Y map to East/North (ENU), matching the real robot.yaml's own
# microstrain_imu use_enu_frame: true.
#
# Publishes sensor_msgs/NavSatFix via a plain rclpy publisher created directly in this script, not via
# isaacsim.ros2.bridge.ROS2Publisher (the generic any-message OGN node, tried first). That node's own literal
# SET_VALUES on its dynamically-created attributes work fine (confirmed live: a literal test altitude came
# through correctly), but a *connection* from another node's dynamically-created output into one of its
# dynamically-created inputs silently never propagates (confirmed live: latitude/longitude stayed exactly 0.0,
# not merely wrong -- ruling out a units/sign bug -- while status/covariance, set as literals, worked) -- since
# GPS position is inherently a per-tick computed value, not a literal, a real connection is unavoidable here,
# so this bypasses that node entirely. Isaac Sim's ROS2 bridge already loads an internal rclpy into this same
# process (confirmed live in the sim's own boot log) and every container in this fleet shares one
# RMW_IMPLEMENTATION/ROS_DOMAIN_ID, so a plain rclpy.init()/Node() here joins the same ROS graph the OmniGraph
# ROS2 bridge nodes do, no special context wiring needed. As a bonus this also gives a real, populated
# header.frame_id/stamp, which the generic publisher route left empty/zero with no way found to set them.
GPS_READ_SCRIPT = """
import math

import omni.usd
import rclpy
from pxr import UsdGeom
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

_ORIGIN_LAT = 47.1211
_ORIGIN_LON = -88.5455
_ORIGIN_ALT = 326.0
_M_PER_DEG_LAT = 111320.0


def setup(db):
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"gps_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(NavSatFix, str(db.inputs.topicName), 10)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(str(db.inputs.gpsPath))
    pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(_ORIGIN_LAT))

    msg = NavSatFix()
    msg.header.stamp = state.node.get_clock().now().to_msg()
    msg.header.frame_id = str(db.inputs.frameId)
    msg.status.status = 0  # STATUS_FIX
    msg.status.service = 1  # SERVICE_GPS
    msg.latitude = _ORIGIN_LAT + pos[1] / _M_PER_DEG_LAT
    msg.longitude = _ORIGIN_LON + pos[0] / m_per_deg_lon
    msg.altitude = _ORIGIN_ALT + pos[2]
    msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

# cmd_vel as geometry_msgs/TwistStamped, like the real Clearpath (Jazzy) platform: isaacsim.ros2.bridge's
# ROS2SubscribeTwist only takes plain Twist and has no stamped option, so this ScriptNode subscribes with an
# in-process rclpy node (same mechanism and reasoning as GPS_READ_SCRIPT) and exposes the same outputs
# (linearVelocity/angularVelocity, holding the last message like the bridge node did), so the drive graph is
# unchanged. Pending messages are drained every tick without blocking.
CMD_VEL_SCRIPT = """
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def setup(db):
    db.per_instance_state.node = None


def compute(db):
    state = db.per_instance_state
    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        ns = str(db.inputs.namespace)
        state.node = Node(f"cmd_vel_sub_{ns}", namespace=ns)
        state.last = None

        def on_cmd(msg):
            state.last = msg

        state.sub = state.node.create_subscription(TwistStamped, str(db.inputs.topicName), on_cmd, 10)
        state.executor = SingleThreadedExecutor()
        state.executor.add_node(state.node)
    for _ in range(20):  # drain everything queued since the last tick
        before = state.last
        state.executor.spin_once(timeout_sec=0.0)
        if state.last is before:
            break
    if state.last is not None:
        t = state.last.twist
        db.outputs.linearVelocity = [t.linear.x, t.linear.y, t.linear.z]
        db.outputs.angularVelocity = [t.angular.x, t.angular.y, t.angular.z]


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.executor.shutdown()
        state.node.destroy_node()
        state.node = None
"""

# Real MTU robots' 2D lidar (a200_0333's Hokuyo UST, j100_0936's SICK LMS1xx): both declare (or, for the SICK,
# really have -- clearpath_config's own lms1xx schema just doesn't expose it as a robot.yaml field the way
# urg_node's does) the same ~270deg FOV -- hokuyo's is the real robot.yaml's own urg_node.angle_min/max
# (-2.356/2.356 rad); SICK's isn't in its robot.yaml at all, so this uses the same value, matching the real
# LMS1xx hardware's own published FOV. NUM_RAYS/RANGE_* are approximate (0.5deg resolution, not either sensor's
# exact real spec) -- shared module-level constants rather than a MODEL_PARAMS field per robot, since both real
# robots that need this happen to agree; revisit if a future robot's 2D lidar genuinely differs.
LIDAR2D_ANGLE_MIN = -2.356
LIDAR2D_ANGLE_MAX = 2.356
LIDAR2D_NUM_RAYS = 541  # ~0.5deg resolution over the 270deg FOV
LIDAR2D_RANGE_MIN = 0.1
LIDAR2D_RANGE_MAX = 10.0

# isaacsim.sensors.experimental.physics.Raycast/RaycastSensor: a real per-physics-step PhysX raycast sensor
# (its own C++ IRaycastSensor interface, acquired the same way IMU_READ_SCRIPT's IMU/IMUSensor is) -- entirely
# separate from isaacsim.sensors.rtx, the broken extension add_lidar2d's own docstring (see its history) had
# concluded blocked 2D lidar outright. Confirmed via this install's own benchmark_physx_lidar.py standalone
# example and the extension's test suite, not assumed: ray_origins/ray_directions are per-ray vectors in the
# sensor prim's own local frame (so nesting the sensor as a plain child of the real lidar link, no extra
# translation/orientation, makes it inherit that link's own mount pose automatically, same as add_camera's
# child-Xform pattern).
#
# Real bug found live in this "experimental"-namespace API, worked around here: get_data()['depths'] does NOT
# report a genuine per-ray distance -- tested a 3-ray sensor (down/forward/up) and depths came back [0.1, 0.1,
# 0.1] (exactly min_range) for all three regardless of what each ray actually hit, while get_data()
# ['hit_positions'] for the SAME reading was correct per-ray (down: [0,0,-0.1], a real 0.1m hit; forward:
# [3.77,0,0], a real ~3.77m hit on scene geometry; up: [0,0,0], genuinely no hit) -- confirmed depths is broken
# specifically, not the sensor itself, since hit_positions independently gives the right per-ray answer.
# Workaround: compute each ray's range as the Euclidean norm of its own hit_positions entry instead of trusting
# depths at all. output_frame="SENSOR" (Raycast's own default) keeps hit_positions in the sensor's own local
# frame, same frame ray_origins/ray_directions are already in, so this norm is directly the range in metres --
# no extra transform needed. A hit_positions entry of exactly [0,0,0] means no hit (confirmed: the "up" ray
# above, a genuine miss, reported exactly that), remapped to +Inf per REP-117 ("no return") rather than 0.0.
# Publishes via a plain rclpy publisher, same reasoning as GPS_READ_SCRIPT: this is a per-tick computed reading,
# not a literal, and no RTX-specific OGN LaserScan publisher node applies to non-RTX raycast data anyway.
LIDAR2D_READ_SCRIPT = """
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def setup(db):
    db.per_instance_state.sensor = None
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    angle_min = float(db.inputs.angleMin)
    angle_max = float(db.inputs.angleMax)
    num_rays = int(db.inputs.numRays)
    range_min = float(db.inputs.rangeMin)
    range_max = float(db.inputs.rangeMax)

    if state.sensor is None:
        from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
        path = str(db.inputs.sensorPath)
        angles = [angle_min + (angle_max - angle_min) * i / max(num_rays - 1, 1) for i in range(num_rays)]
        ray_dirs = [[math.cos(a), math.sin(a), 0.0] for a in angles]
        ray_origins = [[0.0, 0.0, 0.0] for _ in angles]
        try:
            Raycast.create(path, ray_origins=ray_origins, ray_directions=ray_dirs,
                            min_range=range_min, max_range=range_max)
            state.sensor = RaycastSensor(path)
        except Exception as e:
            db.log_warning(f"Lidar2dRead: sensor not ready yet ({e}), retrying next tick")
            return

    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"lidar2d_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(LaserScan, str(db.inputs.topicName), 10)

    frame = state.sensor.get_data()
    hits = frame["hit_positions"]
    if len(hits) == 0:
        return

    n = len(hits)
    ranges = []
    for x, y, z in hits:
        d = math.sqrt(float(x) * float(x) + float(y) * float(y) + float(z) * float(z))
        ranges.append(float("inf") if d < 1e-6 else d)

    msg = LaserScan()
    msg.header.stamp = state.node.get_clock().now().to_msg()
    msg.header.frame_id = str(db.inputs.frameId)
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = (angle_max - angle_min) / max(n - 1, 1)
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = ranges
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

# a200_0333's real Velodyne VLP16: 16 channels over a real ±15deg vertical FOV (VLP16's actual, evenly-2deg-
# spaced channel angles -- real hardware fires them in an interleaved, non-sequential order for timing reasons,
# irrelevant here since this is a per-tick snapshot, not a simulated scan sweep), full 360deg horizontal.
# H_COUNT (1deg horizontal resolution, 5760 rays total, matching VLP16's real ~10Hz/0.2deg ballpark closely
# enough) genuinely crashed the whole sim (segfault, container exit 139) on the very first attempt -- root
# cause turned out to be unrelated to ray count at all (see the instance-proxy explanation on the
# Lidar3dRead wiring in build_ros_graph): once that real bug was fixed, 5760 rays was retested and is safe,
# confirmed live (no crash, real point data, fps cost measured below) -- ray count itself was never the
# problem, so this is the real target value, not a cautious reduction.
# RANGE_MIN/MAX are real-hardware-representative (VLP16's real minimum is close to this; its real 100m max is
# cut down to something sane for this scene's own scale). Z_OFFSET nudges every ray's own origin up by 4cm in
# the sensor's local frame before casting -- found live that the flattened URDF's own lidar3d_0_link collision
# (a cylinder representing the VLP16's base housing) has its top surface only ~3.4cm above lidar3d_0_laser's
# own frame origin, so an unmoved horizontal ray would immediately self-intersect that housing; this offset
# clears it (re-verified live: horizontal rays now correctly reach real scene geometry, not an immediate ~5cm
# self-hit). Measured cost (FLEET_DEBUG=1, 4-robot fleet, only a200_0333 has this sensor): render_fps 12.1 -> 10.4
# (2D lidar alone -> +3D lidar at full resolution), a real but modest ~14% additional cost on top of the
# already camera-rendering-bound baseline.
LIDAR3D_V_ANGLE_MIN = -0.2618  # -15deg
LIDAR3D_V_ANGLE_MAX = 0.2618  # +15deg
LIDAR3D_V_COUNT = 16
LIDAR3D_H_COUNT = 360  # 1deg horizontal resolution, 5760 rays total -- see comment above
LIDAR3D_RANGE_MIN = 0.4
LIDAR3D_RANGE_MAX = 30.0
LIDAR3D_Z_OFFSET = 0.04

# Same isaacsim.sensors.experimental.physics.Raycast/RaycastSensor mechanism as LIDAR2D_READ_SCRIPT, including
# its own workaround for the same real depths-field bug (see that script's own comment for the full
# explanation and how it was isolated) -- range/position both come from hit_positions, never from depths.
# Publishes sensor_msgs/PointCloud2 (unorganized: height=1, width=point count) instead of LaserScan, since this
# is a 3D point set, not a single-plane range array; misses (hit_positions exactly [0,0,0], confirmed live to be
# this API's own "no hit" sentinel) are dropped from the cloud entirely rather than encoded as a sentinel point,
# matching how a real point cloud publisher only emits actual returns. output_frame="SENSOR" (Raycast's own
# default) reports each ray's hit as ray_origins[i] + depth*ray_directions[i] in the sensor prim's own frame, so
# a per-ray ray_origins offset (Z_OFFSET) is already baked into hit_positions with no extra math needed when
# packing points -- re-verify this live rather than assuming, same as everywhere else in this project.
LIDAR3D_READ_SCRIPT = """
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def setup(db):
    db.per_instance_state.sensor = None
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    v_min = float(db.inputs.vAngleMin)
    v_max = float(db.inputs.vAngleMax)
    v_count = int(db.inputs.vCount)
    h_count = int(db.inputs.hCount)
    range_min = float(db.inputs.rangeMin)
    range_max = float(db.inputs.rangeMax)
    z_offset = float(db.inputs.zOffset)

    if state.sensor is None:
        from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
        path = str(db.inputs.sensorPath)
        local_pos = [float(v) for v in db.inputs.localPos]
        local_quat = [float(v) for v in db.inputs.localQuat]
        ray_dirs = []
        for vi in range(v_count):
            v_angle = v_min + (v_max - v_min) * vi / max(v_count - 1, 1)
            cv, sv = math.cos(v_angle), math.sin(v_angle)
            for hi in range(h_count):
                h_angle = 2.0 * math.pi * hi / h_count
                ray_dirs.append([cv * math.cos(h_angle), cv * math.sin(h_angle), sv])
        ray_origins = [[0.0, 0.0, z_offset] for _ in ray_dirs]
        try:
            Raycast.create(path, translations=[local_pos], orientations=[local_quat],
                            ray_origins=ray_origins, ray_directions=ray_dirs,
                            min_range=range_min, max_range=range_max)
            state.sensor = RaycastSensor(path)
        except Exception as e:
            db.log_warning(f"Lidar3dRead: sensor not ready yet ({e}), retrying next tick")
            return

    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"lidar3d_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(PointCloud2, str(db.inputs.topicName), 10)

    frame = state.sensor.get_data()
    hits = frame["hit_positions"]
    if len(hits) == 0:
        return

    buf = bytearray()
    n = 0
    for x, y, z in hits:
        x, y, z = float(x), float(y), float(z)
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        buf += struct.pack("<fff", x, y, z)
        n += 1

    msg = PointCloud2()
    msg.header.stamp = state.node.get_clock().now().to_msg()
    msg.header.frame_id = str(db.inputs.frameId)
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.data = bytes(buf)
    msg.is_dense = True
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

ROBOT_SPACING = 1.6  # m between robots along Y
SPAWN_Z = 0.15  # base_link height: wheel bottoms end up ~1.4 cm above the ground, then it settles


# D435i RGB sensor: 69.4 deg horizontal FOV
HFOV_DEG = 69.4
# ZED2i (real MTU robots, HD720 general.grab_resolution): Stereolabs' published horizontal FOV at 16:9.
ZED_HFOV_DEG = 87.0
# RealSense D405 (j100_0921's wrist camera_1): published depth/colour horizontal FOV ~87 deg.
D405_HFOV_DEG = 87.0


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


PHYSICS_SOLVER = os.environ.get("PHYSICS_SOLVER", "PGS")


def fix_massless_bodies(stage, root, density, frame_mass):
    """Give every rigid body that has no mass authored by the URDF importer (a URDF link without <inertial>) a
    realistic one. PhysX otherwise computes it from the collider volume at 1000 kg/m^3 (solid water) and gives a
    collider-less frame link a default 1 kg: j100_0922 weighs 18.4 kg in its URDF and 75 kg in the sim (its
    top_mount_link mesh 29.8 kg, each GPS sphere 5.9 kg, 13 frame links 1 kg each), all of it high above a 26 cm
    wheelbase, and it tips over backwards at 0.2 m/s. Returns (bodies with density, frame bodies)."""
    n_dens = n_frame = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mapi = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else None
        if mapi is not None and (mapi.GetMassAttr().Get() or 0) > 0:
            continue
        has_collider = any(
            q.HasAPI(UsdPhysics.CollisionAPI)
            for q in Usd.PrimRange(prim)
            if q == prim or not q.HasAPI(UsdPhysics.RigidBodyAPI)
        )
        mapi = UsdPhysics.MassAPI.Apply(prim)
        if has_collider:
            mapi.CreateDensityAttr(float(density))
            n_dens += 1
        else:
            mapi.CreateMassAttr(float(frame_mass))
            n_frame += 1
    return n_dens, n_frame


def enable_wheel_ccd(stage, root):
    """CCD on every wheel rigid body (see the solver comment in build_world)."""
    from pxr import PhysxSchema

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "wheel" in prim.GetName().lower() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)


IMPORT_SETTINGS = {
    "merge_fixed_joints": os.environ.get("FLEET_MERGE_FIXED", "0") == "1",
    "merge_mesh": False,
    "collision_from_visuals": False,
    "joint_target_type": "velocity",
    "joint_drive_type": "force",
    "override_joint_stiffness": 0.0,
    "override_joint_damping": 1000.0,
    # All models creep forward very slowly at rest (Diff.outputs:velocityCommand genuinely [0, 0], confirmed live
    # via FLEET_DEBUG=1) -- worse on j100_0921/j100_0936 (heavier, full arm + sensor loadout) than plain j100
    # (~0.012m vs ~0.005m over 8s), but present even on the light, arm-less model, so it scales with mass/
    # complexity rather than being specific to those two. Tried raising this damping (a force-drive, velocity-
    # target, zero-stiffness joint's holding torque against a disturbance is damping * (targetVelocity -
    # currentVelocity), which is also what should resist creep at targetVelocity=0) to 10x and 100x -- zero
    # measurable effect at either, ruling out "insufficient holding torque" as the mechanism. Left at the
    # original value; the creep looks like a small, universal contact/substep convergence characteristic of this
    # sim rather than something this parameter controls -- not chased further without a next concrete lever to
    # try (a PHYSICS_HZ/substep change, or PhysX solver iteration counts, are the more likely next places to
    # look if this needs solving).

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


# Vegetation colliders. The lidars are PhysX raycasts (Raycast/RaycastSensor), which only hit prims with a collider,
# and the plant/tree/rock assets come without any, so the lidars saw straight through them. collider_asset() writes a
# small wrapper layer per asset into /sim/generated/colliders/ that references the asset and gives each of its meshes
# a static, exact triangle-mesh collider (approximation "none"; static colliders don't need convex shapes, and a hull
# around e.g. the oak's limb mesh would be a 20 m wall). The scene references the wrapper instead of the asset, so the
# visuals and instancing (lavender) are unchanged and PhysX cooks each distinct mesh once. Meshes that are
# PointInstancer prototypes (the trees' and shrubs' leaves/twigs) can't be colliders and stay invisible to the lidar.
# They are solid for the robots too: driving into a hedge is a collision, as it would be in the field.
VEGETATION_COLLIDERS = True
_COLLIDER_DIR = "/sim/generated/colliders"


def collider_asset(asset):
    if not VEGETATION_COLLIDERS:
        return asset
    name = os.path.splitext(os.path.basename(asset))[0]
    out = f"{_COLLIDER_DIR}/{name}.usda"
    stamp = f"{asset} {os.path.getmtime(asset)} v1"
    try:
        if Sdf.Layer.FindOrOpen(out).customLayerData.get("collider_stamp") == stamp:
            return out
    except Exception:
        pass
    os.makedirs(_COLLIDER_DIR, exist_ok=True)
    src = Usd.Stage.Open(asset)
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(st, UsdGeom.GetStageMetersPerUnit(src))
    root = st.DefinePrim("/Root", "Xform")
    root.GetReferences().AddReference(asset)
    st.SetDefaultPrim(root)
    # Opinions below an instanceable prim are ignored, so un-instance any inside the asset (the rocks have one).
    while True:
        inst = [q for q in st.Traverse() if q.IsInstanceable()]
        if not inst:
            break
        for q in inst:
            q.SetInstanceable(False)
    n = 0
    it = iter(Usd.PrimRange(root))
    for q in it:
        if q.IsA(UsdGeom.PointInstancer):
            it.PruneChildren()
        elif q.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(UsdPhysics.Tokens.none)
            n += 1
    st.GetRootLayer().customLayerData = {"collider_stamp": stamp}
    st.GetRootLayer().Export(out)
    log(f"collider wrapper {out}: {n} mesh colliders")
    return out


def add_lavender(stage, path, pos, rot_z=0.0):
    """One lavender clump (~1.2 m wide, ~0.75 m tall at LAVENDER_SCALE) from LAVENDER_USD, with colliders (see
    collider_asset). instanceable=True shares the (heavy) mesh data, BVH and cooked collider between the copies."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(collider_asset(LAVENDER_USD))
    prim.SetInstanceable(True)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2] + LAVENDER_BASE_Z))
    xf.AddRotateZOp().Set(rot_z)
    xf.AddScaleOp().Set(Gf.Vec3d(LAVENDER_SCALE, LAVENDER_SCALE, LAVENDER_SCALE))
    return prim


def add_ground_cover(stage, path="/World/GroundCover"):
    """One GROUND_COVER_USD patch (a 100 x 100 m grass field, ~73k PointInstancer blades of ~9-11 cm) over the
    80 x 80 m ground plane. The layer says metersPerUnit=0.01, but its geometry is really authored in metres
    (blade meshes measure 0.09-0.11 units), so it is referenced at GROUND_COVER_SCALE_XY/_Z (1.0 = unscaled).
    Visual only: no collider, so wheel traction still comes from /World/ground."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(GROUND_COVER_USD)
    UsdGeom.Xformable(prim).AddScaleOp().Set(
        Gf.Vec3d(GROUND_COVER_SCALE_XY, GROUND_COVER_SCALE_XY, GROUND_COVER_SCALE_Z)
    )
    return prim


def add_scatter(stage, root, usds, count, dist, height, seed, arc_deg=80.0):
    """Scatter `count` copies of the given USDs on an arc ahead of the robots at a random radius in `dist` and a
    random target height in `height` (m). Assets are Z-up and authored in cm, but referenced unscaled and sized
    from their own measured bbox, so the scale factor is just target_height / bbox_height."""
    rng = random.Random(seed)
    UsdGeom.Xform.Define(stage, root)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    sizes = {}
    for u in set(usds):
        st = Usd.Stage.Open(u)
        r = cache.ComputeWorldBound(st.GetDefaultPrim() or st.GetPseudoRoot()).ComputeAlignedRange()
        sizes[u] = r.GetSize()[2]
        log(f"scatter asset {u}: size={r.GetSize()} up={UsdGeom.GetStageUpAxis(st)}")
    for t in range(count):
        ang = math.radians(-arc_deg + 2 * arc_deg * t / (count - 1)) + rng.uniform(-0.03, 0.03)
        d = rng.uniform(*dist)
        u = usds[rng.randrange(len(usds))]
        k = rng.uniform(*height) / max(sizes[u], 1e-6)
        # The asset's own root may carry xform ops, so it gets its own child prim under the placement Xform.
        prim = stage.DefinePrim(f"{root}/item_{t}", "Xform")
        stage.DefinePrim(f"{root}/item_{t}/asset", "Xform").GetReferences().AddReference(collider_asset(u))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(d * math.cos(ang), d * math.sin(ang), 0.0))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rng.uniform(0, 360)))
        xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))


def add_horizon_vegetation(stage):
    """Tree line (NVIDIA Omniverse Assets/Vegetation/Trees) with shrubs and boulders (Shrub, Rocks) in front of
    and between the trunks, so the horizon has no bare gap. All within the cameras' 30 m clipping range."""
    add_scatter(stage, "/World/trees", TREE_USDS, TREE_COUNT, (TREE_DIST, TREE_DIST + 5), TREE_HEIGHT, 7, 80.0)
    add_scatter(stage, "/World/shrubs", SHRUB_USDS, SHRUB_COUNT, (SHRUB_DIST, SHRUB_DIST + 8), SHRUB_HEIGHT, 11, 85.0)
    add_scatter(stage, "/World/rocks", ROCK_USDS, ROCK_COUNT, (ROCK_DIST, ROCK_DIST + 8), ROCK_HEIGHT, 13, 85.0)


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

    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    scene_api.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    # Skid-steer wheels need the PGS solver (TGS, the default, barely turns them in place: angular velocity far
    # below the wheel speeds; NVIDIA forum "Skid-Steered behavior for robots") plus CCD on the wheels (below).
    # Measured here: PGS at the same 60 Hz costs no frame rate, 360 Hz halves it.
    scene_api.CreateSolverTypeAttr(PHYSICS_SOLVER)
    scene_api.CreateEnableCCDAttr(True)
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

    # Dirt-coloured surface, so what shows through the grass is soil, not a bright grey box.
    soil = UsdShade.Material.Define(stage, "/World/Materials/ground_soil")
    shader = UsdShade.Shader.Define(stage, "/World/Materials/ground_soil/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GROUND_SOIL_COLOR))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    soil.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(soil)
    ground.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_SOIL_COLOR)])

    add_ground_cover(stage)
    add_horizon_vegetation(stage)

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(SKY_INTENSITY)
    dome.CreateTextureFileAttr(SKY_HDR)  # cloudy sky panorama, also what the cameras see as background
    dome.CreateTextureFormatAttr("latlong")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(10000)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-60, 33, -30))

    n = len(ROBOTS)

    # Lavender farm: a hedge row of overlapping plants on each side of the fleet's driving lanes (robots drive
    # along +X), like the real field's rows. Each plant is ~1.26M triangles, so the row length is what costs fps.
    half = ((n - 1) / 2) * ROBOT_SPACING
    k = 0
    for side in (-1, 1):
        y = side * (half + LAVENDER_ROW_OFFSET)
        for x in [LAVENDER_ROW_X0 + LAVENDER_PLANT_PITCH * j for j in range(LAVENDER_ROW_PLANTS)]:
            add_lavender(stage, f"/World/lavender/plant_{k}", (x, y, 0.0), rot_z=(k * 47.0) % 360)
            k += 1


def spawn_robot(stage, ns, model, index, count):
    root = f"/World/{ns}"
    prim = stage.DefinePrim(root, "Xform")
    prim.GetReferences().AddReference(MODEL_ASSETS[model]["usd_path"])
    y = (index - (count - 1) / 2) * ROBOT_SPACING
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0.0, y, SPAWN_Z))
    return root


def add_camera(stage, robot_root, optical_link=None, hfov_deg=HFOV_DEG, index=0):
    """Colour camera. Frame chain: camera_0_link -> optical frame (z fwd, y down) -> USD camera.

    optical_link (MODEL_PARAMS' camera_optical_link): most models' D435i xacro doesn't produce a ROS-optical-
    convention frame on its own, so the default path builds one by hand under camera_0_link. The real MTU
    robots' ZED2i xacro already emits one (verified: its joint rpy is exactly the standard link->optical
    rotation), so for them this is instead the existing link name to mount the camera under directly, with no
    extra rotation needed beyond the USD-camera/ROS-optical "flip" every model needs.
    """
    if optical_link:
        optical = find_prim(stage, robot_root, optical_link)
    else:
        link = find_prim(stage, robot_root, f"camera_{index}_link")
        optical = f"{link}/camera_{index}_color_optical_frame"
        xf = UsdGeom.Xform.Define(stage, optical)
        # rows = optical x/y/z axes expressed in the ROS link frame (x fwd, y left, z up)
        m = Gf.Matrix4d(1.0)
        m.SetRow3(0, Gf.Vec3d(0, -1, 0))
        m.SetRow3(1, Gf.Vec3d(0, 0, -1))
        m.SetRow3(2, Gf.Vec3d(1, 0, 0))
        if index == 0:
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
    cam.CreateFocalLengthAttr(h_aperture / 2 / math.tan(math.radians(hfov_deg) / 2))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30.0))
    return cam_path


# Real position-servo gains for the real MTU robots' Kinova arm+gripper joints, overriding IMPORT_SETTINGS'
# global stiffness=0/damping=1000 there (see configure_arm_drives). Round, conventional Isaac Sim
# position-control values (in the same ballpark commonly used for imported robot-arm URDFs, e.g. Franka Panda
# samples), not the Kinova's own real servo gains -- this is a simulation of the joints' *response*, not a
# torque-accurate model, and disclosed as such; adjust if the arm moves too slowly/oscillates in practice.
ARM_DRIVE_STIFFNESS = 1.0e5
ARM_DRIVE_DAMPING = 1.0e4
# arm_0_joint_2 (the "shoulder" joint -- carries by far the most gravitational load/inertia of the 6, matching
# its own URDF effort limit of 14 N*m vs 7-10 for the others) is a real outlier under the uniform gains above:
# confirmed live, comparing commanded vs. observed joint_states during an actual zero->cut_init trajectory
# (control_msgs/FollowJointTrajectory, not a raw instantaneous step) -- joints 1/3 show smooth, bounded
# following error throughout, but joint_2 swings *through* zero error and overshoots by over 0.6 rad before
# slowly settling, a genuine underdamped oscillation, not simple lag. Tried uniformly scaling stiffness+damping
# together (preserves the same relative damping ratio, so joint_2 stayed just as underdamped, only smaller in
# absolute terms) and damping alone (didn't fix joint_2's overshoot and added much worse lag to joints 1/3,
# which were fine before) -- neither worked, because one uniform gain pair can't properly serve joints with
# this different a load. Only arm_0_joint_2 gets its own much stronger, non-uniform gain; every other arm/
# gripper joint keeps the values above.
ARM_JOINT_2_STIFFNESS = 1.0e7
ARM_JOINT_2_DAMPING = 1.0e6
# The actual root cause of joint_2 "falling" (the gain above only helped with overshoot): the importer copies the
# URDF effort limit (14 N*m) into the drive's maxForce, and in this sim joint_2 needs more than that -- measured
# via platform/joint_states effort: ~11-12 N*m just holding cut_init, 19-26 N*m while moving at ~0.14 rad/s.
# Once the drive saturates, gain is irrelevant and gravity wins: joint_2 dropped from -0.65 to -2.1 rad mid
# zero->cut_init, couldn't climb back from -2.35 on cut_init->zero, and this was also grid_cutter's patch-2
# servo stall (blocked toward 0, free toward -2.5). With 40 N*m every move tracks within ~0.06 rad and a full
# cut_stem runs patch after patch (peak 24 N*m, >14 N*m for 27% of the run). Gains from 1e4/1e3 to 1e7/1e6 all
# need the same torque, so the extra demand isn't the drive fighting itself. OPEN: static gravity from the URDF
# masses (which match the imported USD) predicts only ~7.5 N*m at cut_init and ~9-10 N*m peak; the real arm
# works within 14 N*m. Self-collision and joint friction ruled out; chassis pitch/rocking not yet checked. This
# is therefore a disclosed workaround, not a realistic torque model.
#
# Made generic (was a fixed 40 N*m on arm_0_joint_2, i.e. 14 x ~3) when a200_0284's Kinova Gen3 7-DOF (joint effort
# limits 39/39/39/39/9/9/9) went through the same failure at a bigger size: joint_2 sat at exactly the 40 N*m cap
# for two seconds during the cutter's first plan move, then the arm collapsed and the physics went chaotic (joint 5
# swung 7 rad in a second, joint 6 wound up to 14 rad, efforts of hundreds of N*m from the resulting collisions).
# So every arm_0_joint_N drive now gets ARM_EFFORT_SCALE x its own URDF effort limit: 14 -> 42 for the Gen3
# Lite's joint_2 (~ the old 40), 39 -> 117 for the 7-DOF's. The gripper joints keep their URDF limits.
ARM_EFFORT_SCALE = 3.0


def configure_arm_drives(stage, root, drop_mimic_constraints=True):
    """Real MTU robots' Kinova arm+gripper: give every arm_0_*/gripper joint a real position-servo drive.

    IMPORT_SETTINGS' global override_joint_stiffness=0.0 is correct for the wheels (a pure velocity drive with
    no position-holding spring, so they can spin continuously) but leaves EVERY joint, arm included, unable to
    hold a commanded position at all -- confirmed live: a JointState position command barely moved the arm
    (a few hundredths of a radian over 2s, not the ~0.5 rad commanded). USD's DriveAPI needs nonzero stiffness
    to act as a position servo at all (with stiffness=0 it's pure velocity damping, so a targetPosition write
    has no effect regardless of what IsaacArticulationController sends) -- this reconfigures just the arm/
    gripper joints' DriveAPI directly on the imported USD, leaving every other joint (wheels, rockers, ...)
    on the global velocity-drive settings untouched. Matched by "arm_0" appearing anywhere in the joint prim's
    own name, covering both arm_0_joint_N and arm_0_gripper_*_joint uniformly without hardcoding either list
    (except arm_0_joint_2's own stronger gain, see above).
    No-op for every model without an arm (nothing named "arm_0" exists in their USD).
    """
    # Mimic constraints (drop_mimic_constraints, default on for every arm): the URDF importer turns each gripper
    # <mimic> joint into a physics constraint (NewtonMimicAPI) on top of the position drive every arm_0 joint gets
    # here, and moveit_sim_bridge also commands every gripper joint (from the URDF's own mimic multiplier/offset).
    # Constraint and drives then fight: on the Kinova 2F Lite, commanding only the driven joint barely moved it (0.04
    # rad for any target) and "open" left it at -0.204, outside its -0.1 limit (bridge pulls the tip to its +0.149
    # offset, constraint + drive settle at bottom = -0.149/0.676); on a200_0284's Robotiq 2F-85 the constraints had
    # wrong coefficients and closing threw joints >1 rad past their limits and spun the wrist. Removed (the whole API
    # schema; clearing only the target relationship did nothing), each gripper joint follows its own command.
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "arm_0" not in prim.GetName():
            continue
        stiffness = ARM_JOINT_2_STIFFNESS if prim.GetName() == "arm_0_joint_2" else ARM_DRIVE_STIFFNESS
        damping = ARM_JOINT_2_DAMPING if prim.GetName() == "arm_0_joint_2" else ARM_DRIVE_DAMPING
        mimic = prim.GetRelationship("newton:mimicJoint")  # set by the URDF importer for <mimic> follower joints
        if drop_mimic_constraints and mimic and mimic.GetTargets():
            # the joint keeps its position drive; only the constraint goes. Clearing the target alone wasn't enough
            # (the constraint kept coupling the joints), so the API schema itself is removed as well.
            mimic.ClearTargets(True)
            prim.RemoveAppliedSchema("NewtonMimicAPI")
            log(f"mimic follower {prim.GetName()}: mimic constraint removed (driven directly)")
        for dof in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                if re.fullmatch(r"arm_0_joint_\d+", prim.GetName()):
                    max_force = drive.GetMaxForceAttr()
                    effort = max_force.Get()  # the importer's copy of the URDF <limit effort>
                    if effort and effort > 0:
                        max_force.Set(float(effort) * ARM_EFFORT_SCALE)


def build_ros_graph(og, usdrt_sdf, stage, root, chassis, ns, cam_path, params, cam1_path=None):
    keys = og.Controller.Keys
    articulation_controller = "isaacsim.core.nodes.IsaacArticulationController"

    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("SysTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
        # --- drive: cmd_vel -> wheel velocities (diff drive for every model; BodyDrive adds Ridgeback's sideways motion below)
        ("CmdVel", "omni.graph.scriptnode.ScriptNode"),  # TwistStamped, see CMD_VEL_SCRIPT
        ("BreakLin", "omni.graph.nodes.BreakVector3"),
        ("BreakAng", "omni.graph.nodes.BreakVector3"),
        # --- state: odometry, tf, joint states
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTfOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    values = [
        ("CmdVel.inputs:namespace", ns),
        ("CmdVel.inputs:topicName", "cmd_vel"),
        ("CmdVel.inputs:script", CMD_VEL_SCRIPT),
        ("Odom.inputs:chassisPrim", [usdrt_sdf.Path(chassis)]),
        ("PubOdom.inputs:nodeNamespace", ns),
        ("PubOdom.inputs:topicName", "platform/odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubTfOdom.inputs:nodeNamespace", ns),
        # Exact pose as its own TF branch, ground_truth -> base_link_ground_truth: odom -> base_link belongs to the
        # robot's platform EKF (robot/bin/ekf), and a frame can have only one parent. platform/odom (above) stays in
        # the odom frame: it is the EKF's wheel-odometry input, like the real platform/odom.
        ("PubTfOdom.inputs:parentFrameId", "ground_truth"),
        ("PubTfOdom.inputs:childFrameId", "base_link_ground_truth"),
        ("PubJoints.inputs:nodeNamespace", ns),
        ("PubJoints.inputs:topicName", "platform/joint_states"),
        ("PubJoints.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
    ]
    connections = [
        ("Tick.outputs:tick", "CmdVel.inputs:execIn"),
        ("CmdVel.outputs:linearVelocity", "BreakLin.inputs:tuple"),
        ("CmdVel.outputs:angularVelocity", "BreakAng.inputs:tuple"),
    ]
    create_attributes = [
        ("CmdVel.inputs:namespace", "token"), ("CmdVel.inputs:topicName", "token"),
        ("CmdVel.outputs:linearVelocity", "double[3]"), ("CmdVel.outputs:angularVelocity", "double[3]"),
    ]

    # Every model, including Ridgeback, drives forward/back and rotation via real diff_4wd.yaml-style wheel
    # rolling -- see MODEL_PARAMS' r100 comment for why Ridgeback's sideways motion needs a different mechanism
    # (BodyDrive, added below) instead of extending this same approach to linear.y.
    front = ["front_left_wheel_joint", "front_right_wheel_joint"]
    rear = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    nodes += [
        ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("VelCtl", "omni.graph.scriptnode.ScriptNode"),
        ("DriveFront", articulation_controller),
        ("DriveRear", articulation_controller),
    ]
    create_attributes += [
        ("VelCtl.inputs:cmd_v", "double"), ("VelCtl.inputs:cmd_w", "double"),
        ("VelCtl.inputs:chassisPath", "token"), ("VelCtl.inputs:enabled", "bool"),
        ("VelCtl.inputs:kp_v", "double"), ("VelCtl.inputs:ki_v", "double"), ("VelCtl.inputs:lim_v", "double"),
        ("VelCtl.inputs:kp_w", "double"), ("VelCtl.inputs:ki_w", "double"), ("VelCtl.inputs:lim_w", "double"),
        ("VelCtl.inputs:alpha", "double"), ("VelCtl.inputs:dt", "double"),
        ("VelCtl.inputs:max_v", "double"), ("VelCtl.inputs:max_w", "double"),
        ("VelCtl.inputs:cmd_y", "double"), ("VelCtl.outputs:out_y", "double"),
        ("VelCtl.outputs:odom_lin", "double[3]"), ("VelCtl.outputs:odom_ang", "double[3]"),
        ("VelCtl.inputs:kp_y", "double"), ("VelCtl.inputs:ki_y", "double"), ("VelCtl.inputs:lim_y", "double"),
        ("VelCtl.outputs:out_v", "double"), ("VelCtl.outputs:out_w", "double"),
    ]
    kp_v, ki_v, lim_v, kp_w, ki_w, lim_w = params.get("vel_ctl", VELCTL_DEFAULT)
    values += [
        ("VelCtl.inputs:script", VEL_CTL_SCRIPT),
        ("VelCtl.inputs:chassisPath", chassis),
        ("VelCtl.inputs:enabled", VELCTL_ENABLED),
        ("VelCtl.inputs:kp_v", kp_v), ("VelCtl.inputs:ki_v", ki_v), ("VelCtl.inputs:lim_v", lim_v),
        ("VelCtl.inputs:kp_w", kp_w), ("VelCtl.inputs:ki_w", ki_w), ("VelCtl.inputs:lim_w", lim_w),
        ("VelCtl.inputs:alpha", 0.4), ("VelCtl.inputs:dt", PHYSICS_FRAME_DT),
        ("VelCtl.inputs:max_v", params["max_linear"]), ("VelCtl.inputs:max_w", params["max_angular"]),
        ("VelCtl.inputs:kp_y", VELCTL_LATERAL[0]), ("VelCtl.inputs:ki_y", VELCTL_LATERAL[1]),
        ("VelCtl.inputs:lim_y", VELCTL_LATERAL[2]),
        ("Diff.inputs:wheelRadius", params["wheel_radius"]),
        ("Diff.inputs:wheelDistance", params["wheel_separation"] * params["separation_multiplier"]),
        # headroom: VelCtl clamps the *command* to the robot's real limits, its feedback may exceed them
        ("Diff.inputs:maxLinearSpeed", params["max_linear"] * VELCTL_HEADROOM),
        ("Diff.inputs:maxAngularSpeed", params["max_angular"] * VELCTL_HEADROOM),
        ("DriveFront.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveFront.inputs:jointNames", front),
        ("DriveRear.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveRear.inputs:jointNames", rear),
    ]
    connections += [
        # VelCtl (feedback on the commanded twist) feeds Diff every tick, the drives apply its wheel speeds
        ("Tick.outputs:tick", "VelCtl.inputs:execIn"),
        ("VelCtl.outputs:execOut", "Diff.inputs:execIn"),
        ("Tick.outputs:tick", "DriveFront.inputs:execIn"),
        ("Tick.outputs:tick", "DriveRear.inputs:execIn"),
        ("BreakLin.outputs:x", "VelCtl.inputs:cmd_v"),
        ("BreakAng.outputs:z", "VelCtl.inputs:cmd_w"),
        ("BreakLin.outputs:y", "VelCtl.inputs:cmd_y"),
        ("VelCtl.outputs:out_v", "Diff.inputs:linearVelocity"),
        ("VelCtl.outputs:out_w", "Diff.inputs:angularVelocity"),
        ("Diff.outputs:velocityCommand", "DriveFront.inputs:velocityCommand"),
        ("Diff.outputs:velocityCommand", "DriveRear.inputs:velocityCommand"),
    ]
    if WHEEL_BRAKE:  # see WHEEL_BRAKE_SCRIPT
        nodes += [("WheelBrake", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("WheelBrake.inputs:chassisPath", "token"), ("WheelBrake.inputs:wheelNames", "token"),
            ("WheelBrake.inputs:cmd_v", "double"), ("WheelBrake.inputs:cmd_w", "double"),
            ("WheelBrake.inputs:cmd_y", "double"), ("WheelBrake.inputs:stiffness", "double"),
            ("WheelBrake.inputs:latchSpeed", "double"), ("WheelBrake.inputs:latchDelay", "double"),
        ]
        values += [
            ("WheelBrake.inputs:script", WHEEL_BRAKE_SCRIPT),
            ("WheelBrake.inputs:chassisPath", chassis),
            ("WheelBrake.inputs:wheelNames", ",".join(front + rear)),
            ("WheelBrake.inputs:stiffness", WHEEL_BRAKE_STIFFNESS),
            ("WheelBrake.inputs:latchSpeed", WHEEL_BRAKE_LATCH_SPEED),
            ("WheelBrake.inputs:latchDelay", WHEEL_BRAKE_LATCH_DELAY),
        ]
        connections += [
            ("Tick.outputs:tick", "WheelBrake.inputs:execIn"),
            ("BreakLin.outputs:x", "WheelBrake.inputs:cmd_v"),
            ("BreakAng.outputs:z", "WheelBrake.inputs:cmd_w"),
            ("BreakLin.outputs:y", "WheelBrake.inputs:cmd_y"),
        ]

    if params["drive"] == "omni":
        # Ridgeback: BodyDrive patches in the one motion component real wheel rolling structurally cannot
        # produce (sideways/linear.y) -- see its comment (BODY_DRIVE_SCRIPT) for the full reasoning and the
        # comment above MODEL_PARAMS' r100 entry for the underlying wheel-collision-geometry finding.
        nodes += [("BodyDrive", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("BodyDrive.inputs:vy", "double"),
            ("BodyDrive.inputs:wz", "double"),
            ("BodyDrive.inputs:chassisPath", "token"),
        ]
        values += [
            ("BodyDrive.inputs:chassisPath", chassis),
            ("BodyDrive.inputs:script", BODY_DRIVE_SCRIPT),
        ]
        connections += [
            ("Tick.outputs:tick", "BodyDrive.inputs:execIn"),
            ("VelCtl.outputs:out_y", "BodyDrive.inputs:vy"),
            ("VelCtl.outputs:out_w", "BodyDrive.inputs:wz"),
        ]

    connections += [
        # state
        ("Tick.outputs:tick", "Odom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubTfOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubJoints.inputs:execIn"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        # published twist = VelCtl's pose-derived body velocity (PhysX's reported angular velocity read up to 25%
        # above the real yaw change at low rates, and does not include BodyDrive's overrides)
        ("VelCtl.outputs:odom_lin", "PubOdom.inputs:linearVelocity"),
        ("VelCtl.outputs:odom_ang", "PubOdom.inputs:angularVelocity"),
        ("Odom.outputs:position", "PubTfOdom.inputs:translation"),
        ("Odom.outputs:orientation", "PubTfOdom.inputs:rotation"),
        ("SysTime.outputs:systemTime", "PubOdom.inputs:timeStamp"),
        ("SysTime.outputs:systemTime", "PubTfOdom.inputs:timeStamp"),
        ("SysTime.outputs:systemTime", "PubJoints.inputs:timeStamp"),
    ]

    # --- camera: one render product feeds every enabled stream
    if cam_path and CAM_STREAMS:
        optical_link = params.get("camera_optical_link")
        nodes += [("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
        values += [
            ("RenderProduct.inputs:cameraPrim", [usdrt_sdf.Path(cam_path)]),
            ("RenderProduct.inputs:width", CAM_W),
            ("RenderProduct.inputs:height", CAM_H),
        ]
        connections += [("Tick.outputs:tick", "RenderProduct.inputs:execIn")]
        if optical_link:
            # A real URDF link (the ZED2i's own xacro already emits a correctly-oriented optical frame, see
            # add_camera/MODEL_PARAMS) -- its fixed joint to its parent is part of the real URDF, so the
            # robot's own robot_state_publisher (robot/bin/robot_state, running in that robot's container)
            # already publishes this TF relationship. No manual PubTfCamera needed, unlike the D435i case below.
            optical = optical_link
        else:
            optical = "camera_0_color_optical_frame"
            nodes += [("PubTfCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")]
            values += [
                # camera_0_link -> optical frame (fixed): quaternion (x, y, z, w)
                ("PubTfCamera.inputs:nodeNamespace", ns),
                ("PubTfCamera.inputs:parentFrameId", "camera_0_link"),
                ("PubTfCamera.inputs:childFrameId", optical),
                ("PubTfCamera.inputs:rotation", [-0.5, 0.5, -0.5, 0.5]),
            ]
            connections += [
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

    # --- wrist camera (camera_1, D405 on the arm's end effector): its own render product, same helpers as
    # camera_0. frameId is camera_1_depth_optical_frame, which the robot's own robot_state_publisher already
    # publishes from the real URDF (same standard link->optical rotation the D435i path builds by hand).
    if cam1_path and CAM_STREAMS:
        nodes += [("RenderProduct1", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
        values += [
            ("RenderProduct1.inputs:cameraPrim", [usdrt_sdf.Path(cam1_path)]),
            ("RenderProduct1.inputs:width", CAM_W),
            ("RenderProduct1.inputs:height", CAM_H),
        ]
        connections += [("Tick.outputs:tick", "RenderProduct1.inputs:execIn")]
        for stream, kind in (("color", "rgb"), ("depth", "depth")):
            if stream not in CAM_STREAMS:
                continue
            pub, info = f"Pub1{stream.title()}", f"Pub1{stream.title()}Info"
            nodes += [(pub, "isaacsim.ros2.bridge.ROS2CameraHelper"), (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
            for name, topic, extra in (
                (pub, f"sensors/camera_1/{stream}/image", [(f"{pub}.inputs:type", kind)]),
                (info, f"sensors/camera_1/{stream}/camera_info", []),
            ):
                values += [
                    (f"{name}.inputs:nodeNamespace", ns),
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", "camera_1_depth_optical_frame"),
                    (f"{name}.inputs:frameSkipCount", CAM_FRAME_SKIP),
                    (f"{name}.inputs:useSystemTime", True),
                ] + extra
                connections += [
                    ("RenderProduct1.outputs:execOut", f"{name}.inputs:execIn"),
                    ("RenderProduct1.outputs:renderProductPath", f"{name}.inputs:renderProductPath"),
                ]

    # --- IMU (real MTU robots only): a ScriptNode reads isaacsim.sensors.experimental.physics' IMU/IMUSensor
    # (no ready-made OGN node exists to read an IMU, only isaacsim.ros2.bridge.ROS2PublishImu to publish one
    # already-read) and feeds it into ROS2PublishImu. frameId/topicName use imu_index (Clearpath's own real
    # per-robot numbering, confirmed from each robot's raw, un-flattened generate_description+xacro output, not
    # guessed): Jackal (j100) always has a separate platform-default IMU occupying slot 0, so the explicit
    # sensor these robots configure is imu_1; A300 has no such default, so its own explicit sensor is imu_0 --
    # publishing it as "imu_1" (this code's old hardcoded literal) would be wrong for a300_00036 specifically.
    # frameId is the real robot's own imu_<n>_link name (what that robot's robot_state_publisher, running from
    # its own un-flattened URDF regeneration in robot_state, actually publishes in its TF tree), not
    # params["imu_link"] -- that field names where the sensor is physically attached in *this sim's* flattened/
    # merged USD (see MODEL_PARAMS' comment on why those differ), which has no bearing on the real TF frame name
    # ROS clients expect.
    if params.get("imu_link"):
        imu_idx = params.get("imu_index", 0)
        imu_frame = f"imu_{imu_idx}_link"
        imu_body = find_prim(stage, root, params["imu_link"])
        imu_sensor_path = f"{imu_body}/{imu_frame}_sensor"
        nodes += [("ImuRead", "omni.graph.scriptnode.ScriptNode"), ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu")]
        create_attributes += [
            ("ImuRead.inputs:imuPath", "token"),
            ("ImuRead.outputs:orientation", "quatd[4]"),
            ("ImuRead.outputs:linearAcceleration", "vectord[3]"),
            ("ImuRead.outputs:angularVelocity", "vectord[3]"),
        ]
        values += [
            ("ImuRead.inputs:imuPath", imu_sensor_path),
            ("ImuRead.inputs:script", IMU_READ_SCRIPT),
            ("PubImu.inputs:nodeNamespace", ns),
            ("PubImu.inputs:topicName", f"sensors/imu_{imu_idx}/data"),
            ("PubImu.inputs:frameId", imu_frame),
        ]
        connections += [
            ("Tick.outputs:tick", "ImuRead.inputs:execIn"),
            ("ImuRead.outputs:execOut", "PubImu.inputs:execIn"),
            ("ImuRead.outputs:orientation", "PubImu.inputs:orientation"),
            ("ImuRead.outputs:linearAcceleration", "PubImu.inputs:linearAcceleration"),
            ("ImuRead.outputs:angularVelocity", "PubImu.inputs:angularVelocity"),
            ("SysTime.outputs:systemTime", "PubImu.inputs:timeStamp"),
        ]

    # --- 2D lidar (real robots with lidar2d_link only): see LIDAR2D_READ_SCRIPT's own comment for the sensor
    # API and why this publishes directly via rclpy. lidar2d_link (e.g. "lidar2d_0_laser") survives flattening
    # as its own real link (unlike imu_link, it's never merged away -- confirmed in the flattened URDF), so it
    # doubles correctly as both the mount-search name and the real TF frame name, no divergence to handle.
    if params.get("lidar2d_link"):
        lidar_link = params["lidar2d_link"]
        lidar_body = find_prim(stage, root, lidar_link)
        lidar_sensor_path = f"{lidar_body}/{lidar_link}_raycast"
        # "lidar2d_0" from the link's own "lidar2d_0_laser" name -- Clearpath's own sensor-index convention,
        # same reasoning as GPS's gps_<n> derivation.
        lidar_topic_stem = "_".join(lidar_link.split("_")[:2])
        nodes += [("Lidar2dRead", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("Lidar2dRead.inputs:sensorPath", "token"),
            ("Lidar2dRead.inputs:topicName", "token"),
            ("Lidar2dRead.inputs:frameId", "token"),
            ("Lidar2dRead.inputs:namespace", "token"),
            ("Lidar2dRead.inputs:angleMin", "double"),
            ("Lidar2dRead.inputs:angleMax", "double"),
            ("Lidar2dRead.inputs:numRays", "int"),
            ("Lidar2dRead.inputs:rangeMin", "double"),
            ("Lidar2dRead.inputs:rangeMax", "double"),
        ]
        values += [
            ("Lidar2dRead.inputs:sensorPath", lidar_sensor_path),
            ("Lidar2dRead.inputs:topicName", f"/{ns}/sensors/{lidar_topic_stem}/scan"),
            ("Lidar2dRead.inputs:frameId", lidar_link),
            ("Lidar2dRead.inputs:namespace", ns),
            ("Lidar2dRead.inputs:angleMin", LIDAR2D_ANGLE_MIN),
            ("Lidar2dRead.inputs:angleMax", LIDAR2D_ANGLE_MAX),
            ("Lidar2dRead.inputs:numRays", LIDAR2D_NUM_RAYS),
            ("Lidar2dRead.inputs:rangeMin", LIDAR2D_RANGE_MIN),
            ("Lidar2dRead.inputs:rangeMax", LIDAR2D_RANGE_MAX),
            ("Lidar2dRead.inputs:script", LIDAR2D_READ_SCRIPT),
        ]
        connections += [("Tick.outputs:tick", "Lidar2dRead.inputs:execIn")]

    # --- 3D lidar (real robots with lidar3d_link only): see LIDAR3D_READ_SCRIPT's own comment for the sensor
    # API, its shared depths-field workaround with 2D lidar, and the Z_OFFSET self-collision fix. lidar3d_link
    # (e.g. "lidar3d_0_laser") survives flattening as its own real link, same as lidar2d_link, so its real TF
    # frame name is just that string directly -- BUT unlike lidar2d_link, it can't be used as the raycast
    # sensor's own *parent* prim: a200_0333's sensor_arch subtree (lidar3d_0_laser's own ancestor chain) is
    # USD-instanceable, and authoring a new child prim under an instance proxy is rejected outright ("authoring
    # to an instance proxy is not allowed", confirmed live -- the sensor silently never got created, retrying
    # every tick). Worked around by parenting the raycast sensor under `chassis` instead (never instanced --
    # it's the articulation root every drive/odometry node already targets) and computing lidar3d_link's real
    # pose *relative to chassis* once here (both are static, real rigid links -- this offset never changes at
    # runtime regardless of where the robot drives), passed to Raycast.create() as an explicit local
    # translation/orientation instead of relying on parent-child nesting for the pose.
    if params.get("lidar3d_link"):
        lidar3d_link = params["lidar3d_link"]
        laser_prim = stage.GetPrimAtPath(find_prim(stage, root, lidar3d_link))
        chassis_prim = stage.GetPrimAtPath(chassis)
        laser_world = UsdGeom.Xformable(laser_prim).ComputeLocalToWorldTransform(0)
        chassis_world = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(0)
        rel = laser_world * chassis_world.GetInverse()
        rel_t = rel.ExtractTranslation()
        rel_q = rel.ExtractRotationQuat()
        rel_im = rel_q.GetImaginary()
        lidar3d_sensor_path = f"{chassis}/{lidar3d_link}_raycast"
        lidar3d_topic_stem = "_".join(lidar3d_link.split("_")[:2])
        nodes += [("Lidar3dRead", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("Lidar3dRead.inputs:sensorPath", "token"),
            ("Lidar3dRead.inputs:topicName", "token"),
            ("Lidar3dRead.inputs:frameId", "token"),
            ("Lidar3dRead.inputs:namespace", "token"),
            ("Lidar3dRead.inputs:localPos", "double[3]"),
            ("Lidar3dRead.inputs:localQuat", "double[4]"),
            ("Lidar3dRead.inputs:vAngleMin", "double"),
            ("Lidar3dRead.inputs:vAngleMax", "double"),
            ("Lidar3dRead.inputs:vCount", "int"),
            ("Lidar3dRead.inputs:hCount", "int"),
            ("Lidar3dRead.inputs:rangeMin", "double"),
            ("Lidar3dRead.inputs:rangeMax", "double"),
            ("Lidar3dRead.inputs:zOffset", "double"),
        ]
        values += [
            ("Lidar3dRead.inputs:sensorPath", lidar3d_sensor_path),
            ("Lidar3dRead.inputs:topicName", f"/{ns}/sensors/{lidar3d_topic_stem}/points"),
            ("Lidar3dRead.inputs:frameId", lidar3d_link),
            ("Lidar3dRead.inputs:namespace", ns),
            ("Lidar3dRead.inputs:localPos", [rel_t[0], rel_t[1], rel_t[2]]),
            ("Lidar3dRead.inputs:localQuat", [rel_q.GetReal(), rel_im[0], rel_im[1], rel_im[2]]),
            ("Lidar3dRead.inputs:vAngleMin", LIDAR3D_V_ANGLE_MIN),
            ("Lidar3dRead.inputs:vAngleMax", LIDAR3D_V_ANGLE_MAX),
            ("Lidar3dRead.inputs:vCount", LIDAR3D_V_COUNT),
            ("Lidar3dRead.inputs:hCount", LIDAR3D_H_COUNT),
            ("Lidar3dRead.inputs:rangeMin", LIDAR3D_RANGE_MIN),
            ("Lidar3dRead.inputs:rangeMax", LIDAR3D_RANGE_MAX),
            ("Lidar3dRead.inputs:zOffset", LIDAR3D_Z_OFFSET),
            ("Lidar3dRead.inputs:script", LIDAR3D_READ_SCRIPT),
        ]
        connections += [("Tick.outputs:tick", "Lidar3dRead.inputs:execIn")]

    # --- GPS x2 (real MTU robots only): see GPS_READ_SCRIPT's own comment for why this publishes directly via
    # a plain rclpy publisher inside the script, not isaacsim.ros2.bridge.ROS2Publisher (the generic any-
    # message OGN node, tried first -- its literal SET_VALUES work but a connection into one of its
    # dynamically-created inputs silently never propagates a value, confirmed live).
    for gps_link in params.get("gps_links", []):
        # Index from the link's own name (Clearpath's real numbering, e.g. gps_1_link/gps_2_link), not list
        # position -- currently the same thing since gps_links is always listed in that order, but deriving it
        # from the actual name is what's actually correct and doesn't depend on staying that way.
        i = int(re.search(r"gps_(\d+)_link", gps_link).group(1))
        node = f"Gps{i}"
        gps_path = find_prim(stage, root, gps_link)
        nodes += [(node, "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            (f"{node}.inputs:gpsPath", "token"),
            (f"{node}.inputs:topicName", "token"),
            (f"{node}.inputs:frameId", "token"),
            (f"{node}.inputs:namespace", "token"),
        ]
        values += [
            (f"{node}.inputs:gpsPath", gps_path),
            (f"{node}.inputs:topicName", f"/{ns}/sensors/gps_{i}/fix"),
            (f"{node}.inputs:frameId", gps_link),
            # topicName above is already an absolute path, so the rclpy Node's own namespace doesn't affect
            # which topic it publishes to -- this is purely so the *node itself* (ros2 node list) shows up
            # under the robot's namespace instead of at the top level (confirmed live: without this, GPS nodes
            # appeared as bare /gps_read_j100_0921_sensors_gps_1_fix instead of /j100_0921/gps_read_...).
            (f"{node}.inputs:namespace", ns),
            (f"{node}.inputs:script", GPS_READ_SCRIPT),
        ]
        connections += [("Tick.outputs:tick", f"{node}.inputs:execIn")]

    # --- Arm + gripper (real MTU robots only): a second IsaacArticulationController, position-mode, targeting
    # the same chassis articulation root as the wheel drive above. jointNames/positionCommand are wired
    # straight through from ROS2SubscribeJointState's own outputs rather than a hardcoded joint list, so
    # whatever names/order a published JointState message actually uses is exactly what gets commanded --
    # hardcoding a fixed jointNames list here and trusting positionCommand's array to line up with it
    # positionally (this project's existing DriveFront/DriveRear pattern) would silently command the wrong
    # joints if a client ever published a different subset/order than guessed. Every joint IMPORT_SETTINGS
    # configured for velocity-mode (this whole project's global import setting) still accepts a positionCommand
    # write here -- IsaacArticulationController takes position *or* velocity *or* effort per call regardless of
    # the prim's own configured drive type -- so no special per-joint drive reconfiguration was needed.
    if params.get("has_arm"):
        nodes += [
            ("ArmCmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("DriveArm", articulation_controller),
        ]
        values += [
            ("ArmCmd.inputs:nodeNamespace", ns),
            ("ArmCmd.inputs:topicName", "arm_0/joint_command"),
            ("DriveArm.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ]
        connections += [
            # ArmCmd's outputs hold their last-received message's values between messages (same as CmdVel's
            # linearVelocity/angularVelocity do for the wheel drive above), so DriveArm re-applies them every
            # tick rather than only on ArmCmd's own execOut -- matches DriveFront/DriveRear/Drive's pattern.
            ("Tick.outputs:tick", "ArmCmd.inputs:execIn"),
            ("Tick.outputs:tick", "DriveArm.inputs:execIn"),
            ("ArmCmd.outputs:jointNames", "DriveArm.inputs:jointNames"),
            ("ArmCmd.outputs:positionCommand", "DriveArm.inputs:positionCommand"),
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
        enable_extensions([
            "isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots.nodes", "omni.graph.action", "omni.graph.nodes", "omni.graph.scriptnode",
            "isaacsim.sensors.experimental.physics",  # real MTU robots' IMU (ImuRead script)
            # isaacsim.sensors.experimental.rtx (j100_0936's 2D lidar) deliberately NOT enabled -- broken in
            # this Isaac Sim 6.0 install, see add_lidar2d's docstring.
        ])
        for model in dict.fromkeys(model for _, model in ROBOTS):  # each distinct model once, first-seen order
            import_urdf_if_needed(model)

        import omni.graph.core as og
        import usdrt.Sdf as usdrt_sdf

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        build_world(stage)
        for i, (ns, model) in enumerate(ROBOTS):
            root = spawn_robot(stage, ns, model, i, len(ROBOTS))
            cam_path = None
            if MODEL_PARAMS[model].get("has_camera", True):
                optical_link = MODEL_PARAMS[model].get("camera_optical_link")
                hfov = ZED_HFOV_DEG if optical_link else HFOV_DEG
                cam_path = add_camera(stage, root, optical_link=optical_link, hfov_deg=hfov)

            log(f"spawned {ns} ({model}) at {root}")
            for _ in range(3):
                await app.next_update_async()
            chassis = find_prim(stage, root, MODEL_PARAMS[model]["chassis_link"])
            enable_wheel_ccd(stage, root)
            if MODEL_PARAMS[model].get("massless_density"):
                nd, nf = fix_massless_bodies(stage, root, MODEL_PARAMS[model]["massless_density"],
                                             MODEL_PARAMS[model].get("frame_mass", 0.02))
                log(f"{ns}: massless bodies: {nd} at {MODEL_PARAMS[model]['massless_density']} kg/m^3, {nf} frames")
            cam1_path = None
            if MODEL_PARAMS[model].get("wrist_camera"):
                cam1_path = add_camera(stage, root, hfov_deg=D405_HFOV_DEG, index=1)
            build_ros_graph(og, usdrt_sdf, stage, root, chassis, ns, cam_path, MODEL_PARAMS[model], cam1_path)
            if MODEL_PARAMS[model].get("has_arm"):
                configure_arm_drives(stage, root, MODEL_PARAMS[model].get("drop_mimic_constraints", True))
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
