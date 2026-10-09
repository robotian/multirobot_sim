"""What each fleet setting is: the catalog behind scripts/fleetcfg.py and the web UI's Configuration page.

The values live in the base station's fleet_config database (one set per profile); .env is generated from them
(scripts/fleetcfg.py render) and read by docker compose, which passes them into the containers. This file says
what a value may be and what it reaches, in git next to the code that reads it, so a new setting needs no database
migration: add it here, and to docker-compose.yml (the isaac-sim service's environment: block for the sim).

`default` must equal docker-compose.yml's `${KEY:-default}` (`scripts/fleetcfg.py check` compares them): a setting
the profile doesn't set is left out of .env, and compose's default applies.
`applies`: the containers that have to be recreated for a change to take effect (they read it at start).
Robot slots (models, poses) are not settings: they are the robot_slot table, rendered as ROBOT_MODEL_<i> etc.
"""

SIM, ROBOTS, BASESTATION = "sim", "robots", "basestation"


def S(key, section, type, default, help, applies, choices=None, min=None, max=None, advanced=False):
    return {"key": key, "section": section, "type": type, "default": default, "help": help,
            "applies": list(applies), "choices": choices, "min": min, "max": max, "advanced": advanced}


BOOL01 = ["0", "1"]
TRUEFALSE = ["true", "false"]

SETTINGS = [
    # ---------------------------------------------------------------- robots
    S("NUM_ROBOTS", "Robots", "int", "1", "Robots spawned and robot containers started, in slots 0..N-1 (the slot "
      "table holds their models and poses).", [ROBOTS], min=0, max=8),
    S("HOST_UID", "Robots", "text", "", "uid of the robot containers' `robot` user (owner of colcon_ws); empty = "
      "the owner of this checkout.", [ROBOTS], advanced=True),
    S("HOST_GID", "Robots", "text", "", "gid of the robot containers' `robot` user; empty = the checkout's.",
      [ROBOTS], advanced=True),

    # ---------------------------------------------------------------- sim
    S("SIM_MODE", "Simulation", "enum", "stream", "stream: headless, viewed with the WebRTC streaming client. "
      "headed: Isaac Sim's own window on this machine's display (scripts/x11_auth.sh first).", [SIM],
      choices=["stream", "headed"]),
    S("ISAACSIM_HOST", "Simulation", "text", "127.0.0.1", "Address the WebRTC client uses to reach the sim: this "
      "machine's LAN IP for a remote client, 127.0.0.1 for local only.", [SIM]),
    S("SIM_SCENE", "Simulation", "text", "", "The world: empty = ground plane + lights, lavender = the built-in "
      "lavender farm, else a USD file under sim/scene/ (path relative to it).", [SIM]),
    S("SCENE_LANES", "Simulation", "int", "3", "Robot lanes between the two lavender rows; the default spawn "
      "poses fill them.", [SIM], min=1, max=8),
    S("ROBOT_LOOKS", "Simulation", "enum", "full", "Robot materials. full: textured (dust, scratches); basic: "
      "plain colours; off: the URDF importer's flat materials.", [SIM], choices=["full", "basic", "off"]),
    S("FARM_WORKERS", "Simulation", "enum", "0", "1: two farm workers at a lavender row (NVIDIA digital humans; "
      "cost frame rate and memory).", [SIM], choices=BOOL01),
    S("LAVENDER_SOFT", "Simulation", "enum", "1", "1: robots pass through lavender foliage, a small rigid core "
      "per plant stops them (lidars still see it); 0: the whole plant is solid.", [SIM], choices=BOOL01),
    S("SIM_RATE_HZ", "Simulation", "float", "20", "Frames per second of simulated time. Set it to about the "
      "render fps FLEET_DEBUG=1 reports, or the sim runs slower than real time (2 robots ~23 fps, 3 on zenoh "
      "~15).", [SIM], min=1, max=120),
    S("PHYSICS_HZ", "Simulation", "float", "60", "PhysX steps per second of simulated time "
      "(PHYSICS_HZ / SIM_RATE_HZ substeps per frame).", [SIM], min=10, max=1000),
    S("USE_SIM_TIME", "Simulation", "enum", "true", "true: the sim publishes /clock and every ROS node (robots, "
      "base station) runs on it, so robot logic keeps step with a sim slower than real time. false: wall-clock "
      "stamps, no /clock (also the base station's setting with real robots).", [SIM, ROBOTS, BASESTATION],
      choices=TRUEFALSE),
    S("SIM_REF_POSE", "Simulation", "enum", "1", "1: publish each robot's exact pose in the world (like the lab's "
      "motion capture) for mocap_fake_localizer.", [SIM], choices=BOOL01, advanced=True),
    S("VELCTL", "Simulation", "enum", "1", "1: closed velocity loop on each robot (commanded speed reached within "
      "~5%); 0: open loop, for A/B comparisons.", [SIM], choices=BOOL01, advanced=True),
    S("PHYSICS_SOLVER", "Simulation", "enum", "PGS", "PhysX solver.", [SIM], choices=["PGS", "TGS"],
      advanced=True),
    S("FLEET_MERGE_FIXED", "Simulation", "enum", "0", "1: the URDF importer merges fixed joints.", [SIM],
      choices=BOOL01, advanced=True),
    S("FORCE_REIMPORT", "Simulation", "enum", "0", "1: import every robot model from its URDF again at start, "
      "even when the cached import is current.", [SIM], choices=BOOL01, advanced=True),
    S("FLEET_DEBUG", "Simulation", "enum", "0", "1: the sim logs its frame rate and real-time factor.", [SIM],
      choices=BOOL01, advanced=True),
    S("FLEET_SETTINGS", "Simulation", "text", "", "Extra Kit settings, \"/path/a=1;/path/b=text\" (e.g. async "
      "rendering: about +15-25% frame rate, camera images a frame late).", [SIM], advanced=True),
    S("FLEET_VIEWPORT_RES", "Simulation", "text", "", "Render the streamed viewport at a fixed size, e.g. "
      "1280x720; empty = the window's.", [SIM], advanced=True),
    S("FLEET_SNAPSHOT", "Simulation", "text", "", "A directory (in the sim container) for viewport PNGs of each "
      "robot after every spawn; empty = off.", [SIM], advanced=True),

    # ---------------------------------------------------------------- cameras
    S("CAMERA_WIDTH", "Cameras", "int", "640", "Robot camera image width.", [SIM], min=16, max=3840),
    S("CAMERA_HEIGHT", "Cameras", "int", "360", "Robot camera image height.", [SIM], min=16, max=2160),
    S("CAMERA_FRAME_SKIP", "Cameras", "int", "0", "Cameras publish every sim frame divided by (skip + 1).",
      [SIM], min=0, max=60),
    S("CAMERA_STREAMS", "Cameras", "text", "color,depth", "Image streams each camera renders: color, depth, or "
      "none.", [SIM]),
    S("CAMERA_ON_DEMAND", "Cameras", "enum", "1", "1: a camera renders only while one of its topics has a "
      "subscriber.", [SIM], choices=BOOL01),

    # ---------------------------------------------------------------- middleware
    S("FLEET_RMW", "Middleware", "enum", "rmw_zenoh_cpp", "ROS 2 middleware of the sim and the robots (and the "
      "base station, unless BASESTATION_RMW says otherwise). rmw_zenoh_cpp: like the real robots, through the "
      "zenoh-router service.", [SIM, ROBOTS, BASESTATION], choices=["rmw_zenoh_cpp", "rmw_fastrtps_cpp"]),
    S("ROS_DOMAIN_ID", "Middleware", "int", "0", "ROS domain.", [SIM, ROBOTS, BASESTATION], min=0, max=232),
    S("ZENOH_ROUTER", "Middleware", "text", "tcp/zenoh-router:7447", "Zenoh router the sim's and robots' "
      "sessions connect to; e.g. tcp/<robot-ip>:7447 for a real robot's.", [SIM, ROBOTS], advanced=True),

    # ---------------------------------------------------------------- base station
    S("BASESTATION_ZENOH_CONNECT", "Base station", "text", "tcp/127.0.0.1:7448", "Routers the base station's "
      "zenoh router dials, space-separated: the sim's zenoh-router (tcp/127.0.0.1:7448) and real robots' "
      "(tcp/<ip>:7447).", [BASESTATION]),
    S("BASESTATION_RMW", "Base station", "text", "", "The base station's own middleware when it differs from "
      "FLEET_RMW (e.g. rmw_cyclonedds_cpp for a real robot on Cyclone DDS); empty = FLEET_RMW.", [BASESTATION],
      advanced=True),
    S("BASESTATION_PG_PORT", "Base station", "int", "5433", "PostgreSQL port of the base station (also where "
      "scripts/fleetcfg.py finds this database).", [BASESTATION], min=1, max=65535, advanced=True),
    S("BASESTATION_PG_USER", "Base station", "text", "admin", "PostgreSQL superuser, set when the database "
      "cluster is first created.", [BASESTATION], advanced=True),
    S("BASESTATION_PG_DB", "Base station", "text", "test_lavender_farming", "The farm database, set when the "
      "cluster is first created.", [BASESTATION], advanced=True),
]

BY_KEY = {s["key"]: s for s in SETTINGS}
NOT_IN_COMPOSE = {"NUM_ROBOTS"}  # read by scripts/fleet.sh and fleet_ctl.py from .env, never interpolated by compose
SECTIONS = list(dict.fromkeys(s["section"] for s in SETTINGS))

MAX_SLOTS = 8
GENERIC_MODELS = ["a300", "a200", "j100", "r100"]
# Written by scripts/fleetcfg.py render from the slots, never stored or set by hand.
DERIVED_PREFIXES = ("ROBOT_MODEL_", "ROBOT_POSE_", "ROBOT_SUFFIX_", "ROBOT_HOSTNAME_")
DERIVED_KEYS = ("COMPOSE_PROFILES",)


def is_derived(key):
    return key in DERIVED_KEYS or key.startswith(DERIVED_PREFIXES)


def check_value(key, value):
    """None if `value` is valid for `key`, else why not. Keys outside the catalog take any plain text."""
    if any(c in value for c in "\n\r\"'\\$`"):
        return "no quotes, backslashes, $ or line breaks"
    s = BY_KEY.get(key)
    if s is None:
        return None
    if s["type"] == "enum" and value not in s["choices"]:
        return f"one of {', '.join(s['choices'])}"
    if s["type"] in ("int", "float"):
        try:
            n = int(value) if s["type"] == "int" else float(value)
        except ValueError:
            return f"{'a whole number' if s['type'] == 'int' else 'a number'}"
        if (s["min"] is not None and n < s["min"]) or (s["max"] is not None and n > s["max"]):
            return f"between {s['min']} and {s['max']}"
    return None
