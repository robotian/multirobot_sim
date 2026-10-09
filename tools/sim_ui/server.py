#!/usr/bin/env python3
"""Local web UI for the fleet: the simulated robots, the real ones, or both (the page's mode).

Sim: start/stop/reset the sim (the scene only), spawn robots at chosen poses into the running scene (and start
their containers), start/stop mtu32_bringup's sim_robot_upstart.launch.py per robot.
Real (the real_robot table of the settings database, reached over SSH like scripts/deploy_robot.sh): Clearpath's services and
restarting them, deploying colcon_ws/src, linking the robot to the base station's zenoh router.
Either: move the arm to named SRDF states (optionally recording commanded vs. observed joint positions while it
moves), cut_stem, stop motion, RViz, each robot's ref_localizer (map -> odom source / anchor), OptiTrack Motive's
rigid bodies and which robot follows each.
Configuration page (/config): the settings, robot slots and profiles in the base station's fleet_config database
(scripts/fleetcfg.py), which .env is generated from; their change log and runs; what the running containers differ in.

  python3 tools/sim_ui/server.py                 # http://127.0.0.1:8090
  python3 tools/sim_ui/server.py --mode real     # the page's mode until the browser picks its own
  python3 tools/sim_ui/server.py --host 0.0.0.0  # reachable from the LAN -- it runs docker commands, so only on a trusted network
  FLEET_ROOT=<checkout> python3 server.py        # drive another checkout's fleet (e.g. this UI from a worktree)

Stdlib plus psycopg (or psycopg2) for the settings database; without it, or without the database, the page runs on
.env as last written and refuses setting changes. Long operations run as background jobs whose output the page polls.
"""
import argparse
import base64
import collections
import hashlib
import importlib
import json
import math
import os
import re
import select
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(os.environ.get("FLEET_ROOT") or Path(__file__).resolve().parents[2]).resolve()
sys.path.insert(0, str(ROOT / "scripts"))
import fleet_ctl  # noqa: E402  (the sim's spawn protocol: request/state files)
import fleetcfg  # noqa: E402  (the settings: the base station's fleet_config database, .env generated from it)
sys.path.insert(0, str(ROOT / "colcon_ws/src/mocap_fake_localizer/scripts"))
import natnet  # noqa: E402  (OptiTrack Motive's NatNet protocol, shared with the robots' natnet_ref_pose.py)

PAGE = Path(__file__).with_name("index.html")
PROJECT = "clearpath-fleet"
SIM = "a300-isaac-sim"
BASESTATION = "basestation"  # basestation.compose.yml's container (host network); real robots' RViz runs there
MAX_SLOTS = 8
MODES = ("sim", "real", "both")
DEFAULT_MODE = "sim"  # --mode
NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
# Commands run as `bash -c '<command>'` (docker exec, or ssh's remote shell), whose own command line then holds every
# pgrep/pkill pattern in it: each pattern's first letter is bracketed ("[r]os2") so it can't match that shell.
# The cut_stem client (`ros2 action send_goal ... /<ns>/cut_stem`) is exec'd directly (no wrapper shell), so this
# matches exactly one process per run; SIGINT to it makes ros2cli cancel the goal.
CUT_MATCH = "[r]os2 action send_goal.*cut_stem"
CUT_CMD = ('exec ros2 action send_goal --feedback /$ROBOT_NAMESPACE/cut_stem '
           'plant_cutter_msgs/action/CutStem "{start_cutting: true}"')
LAUNCH_LOG = "/tmp/sim_robot_upstart.log"
LAUNCH_MATCH = "[s]im_robot_upstart.launch.py"
SRDF = "/etc/clearpath/robot.srdf"  # written at container boot; sim_robot_upstart needs it
SRDF_WAIT_S = 120
LAUNCH_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
              f"exec ros2 launch mtu32_bringup sim_robot_upstart.launch.py > {LAUNCH_LOG} 2>&1")
# RViz from clearpath_viz (colcon_ws/src/clearpath_desktop), opened on the host display through the container's X11
# mount; one window per view and robot. The launch exits when its rviz2 window is closed. A sim robot's runs in its
# own container; a real robot's in the base station container (host network, the same colcon_ws), on wall time.
RVIZ_VIEWS = ("navigation", "moveit", "robot")
RVIZ_MATCH = r"[c]learpath_viz view_(navigation|moveit|robot)\.launch\.py"
RVIZ_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
            "exec ros2 launch clearpath_viz view_{view}.launch.py namespace:={ns} "
            "use_sim_time:={sim_time} > /tmp/rviz_{log}.log 2>&1")
# A real robot: Clearpath's systemd services (bringup_main runs from clearpath-platform-extras).
REAL_SERVICES = ("clearpath-robot", "clearpath-platform-extras", "clearpath-manipulators")
SUDOERS_HINT = ("once, on the robot: echo 'robot ALL=(root) NOPASSWD: /usr/bin/systemctl restart clearpath-robot' "
                "| sudo tee /etc/sudoers.d/fleet-ui && sudo chmod 440 /etc/sudoers.d/fleet-ui")
REAL_MAX_VELOCITY = 0.3  # arm velocity/acceleration scale cap on a real robot
# Every action goal a robot may be running from here: cut_stem, arm_goto (move_action / the trajectory and gripper
# controllers), RViz's MoveIt panel (execute_trajectory). A CancelGoal with a zero goal id and stamp cancels all.
STOP_ACTIONS = ("cut_stem", "move_action", "execute_trajectory",
                "manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory",
                "manipulators/arm_0_gripper_controller/gripper_cmd")
# ros2 service call prints "making request" once it found the server and sent it. On a300_00036 (2026-10-06) the
# move_action / trajectory controller cancels arrived (PREEMPTED 1.9 s after the button) but their replies never
# came back within the timeout: that is "sent, no reply", not "no action server".
STOP_CMD = (f"pkill -INT -f '{CUT_MATCH}' && echo 'cut_stem client: Ctrl+C'; "
            "pkill -INT -f '[/ ]arm_goto( |$)' && echo 'arm_goto: Ctrl+C'; "
            "for a in " + " ".join(STOP_ACTIONS) + "; do ( "
            "out=$(timeout 10 ros2 service call /$ROBOT_NAMESPACE/$a/_action/cancel_goal action_msgs/srv/CancelGoal "
            "'{}' 2>&1); r=$(grep -o 'return_code=[0-9]*, goals_canceling=\\[[^]]*\\]' <<< \"$out\"); "
            "if [ -z \"$r\" ]; then grep -q 'making request' <<< \"$out\" && r='cancel sent, no reply in 10 s' "
            "|| r='no action server'; fi; echo \"$a: $r\" ) & done; wait")


def sh(argv, timeout=30, input=None):
    # stdin: `input` (e.g. a sudo password, never put on a command line), else /dev/null: ssh would otherwise read
    # the server's terminal
    stdin = {"input": input} if input is not None else {"stdin": subprocess.DEVNULL}
    p = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=timeout, **stdin)
    return p.returncode, p.stdout + p.stderr


def arm_tool(tool, args=""):
    """moveit_sim_bridge's arm_goto / arm_joints from ~/colcon_ws (the sim's shared workspace, a real robot's
    deployed one); a sim robot image from before they moved there has its own copy in /usr/local/bin."""
    exe = f"$HOME/colcon_ws/install/moveit_sim_bridge/lib/moveit_sim_bridge/{tool}"
    return (f"[ -f ~/colcon_ws/install/setup.bash ] && source ~/colcon_ws/install/setup.bash >/dev/null 2>&1; "
            f"if [ -x {exe} ]; then exec ros2 run moveit_sim_bridge {tool} {args}; fi; "
            f"command -v {tool} >/dev/null || {{ echo 'no {tool} here: build/deploy colcon_ws (moveit_sim_bridge)'; "
            f"exit 127; }}; exec {tool} {args}")


# ---------------------------------------------------------------- robots: sim containers and real robots

class SimTarget:
    """A sim robot: its container, `docker exec` (bash -c, so BASH_ENV sources ROS and ROBOT_NAMESPACE)."""
    kind = "sim"
    cutter = True  # every sim robot runs the cutter stack (pruner_stub); cut_stem is gated on its action server

    def __init__(self, name):
        self.name = name

    def argv(self, command):
        return ["docker", "exec", self.name, "bash", "-c", command]

    def run(self, command, timeout=30):
        return sh(self.argv(command), timeout=timeout)


SSH_DIR = Path(f"/tmp/fleet-ui-ssh-{os.getuid()}")  # ControlMaster sockets: one connection per robot, reused
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=3", "-o", "ControlMaster=auto", "-o", f"ControlPath={SSH_DIR}/%C",
            "-o", "ControlPersist=120"]


class RealTarget:
    """A real robot over SSH (key login, as scripts/deploy_robot.sh): its login shell runs `bash -c` with Clearpath's
    setup.bash (ROS + its workspaces from robot.yaml) and ROBOT_NAMESPACE, which a robot's own shell doesn't set."""
    kind = "real"

    def __init__(self, name, host, user, cutter):
        self.name, self.host, self.user, self.cutter = name, host, user, cutter

    def argv(self, command, ros_env=True):
        SSH_DIR.mkdir(mode=0o700, exist_ok=True)
        # ros_env=False: the bare command (sourcing setup.bash takes ~0.5 s, too slow for timing a round trip)
        script = (f"source /etc/clearpath/setup.bash >/dev/null 2>&1; export ROBOT_NAMESPACE={self.name}; {command}"
                  if ros_env else command)
        return ["ssh", *SSH_OPTS, f"{self.user}@{self.host}", "bash -c " + shlex.quote(script)]

    def run(self, command, timeout=30, input=None, ros_env=True):
        return sh(self.argv(command, ros_env), timeout=timeout, input=input)


# Which real robots the UI shows: {"<id>": {"host": ..., "user": ..., "cutter": true}}, the settings database's
# real_robot table (fleetcfg.real_robots(); while it can't be reached, the copy it last wrote to real_robots.json).
# Host defaults to the robot's mDNS name (cpr-<id with - for _>.local, as deploy_robot.sh), user to robot, cutter
# (the stem cutter is fitted: cut_stem is offered; bringup_main advertises the action on every robot) to false.
ROBOT_ID_RE = re.compile(r"^[a-z0-9]+_[0-9]+$")  # deploy_robot.sh's
HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9.-]*$")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]*$")
REAL_LOCK = threading.Lock()
REAL_CACHE_S = 5  # read by every status poll and the link monitors: one database round trip per 5 s at most
_real_cache = {"t": 0.0, "data": {}}
ACTOR = "web UI"  # who the settings database's change log names for changes made here


def default_host(robot_id):
    return f"cpr-{robot_id.replace('_', '-')}.local"


def real_config():
    with REAL_LOCK:
        if time.time() - _real_cache["t"] > REAL_CACHE_S:
            data = fleetcfg.real_robots(ACTOR)
            _real_cache.update(t=time.time(), data={k: v for k, v in data.items() if ROBOT_ID_RE.match(k)})
        return dict(_real_cache["data"])


def real_target(name, config=None):
    c = (real_config() if config is None else config).get(name)
    if c is None:
        raise ValueError(f"{name} is not in the real robots list")
    return RealTarget(name, c.get("host") or default_host(name), c.get("user") or "robot", bool(c.get("cutter")))


def known_real_ids():
    """MTU's real robots this checkout knows (robot_data/<id>/robot.yaml), offered when adding one."""
    d = ROOT / "robot_data"
    return sorted(p.parent.name for p in d.glob("*/robot.yaml") if ROBOT_ID_RE.match(p.parent.name)) if d.is_dir() else []


def act_real_save(body):
    robot = body.get("robot")
    if not (isinstance(robot, str) and ROBOT_ID_RE.match(robot)):
        raise ValueError("robot id like j100_0921")
    host = (body.get("host") or "").strip()
    user = (body.get("user") or "").strip()
    if host and not HOST_RE.match(host):
        raise ValueError("bad host")
    if user and not USER_RE.match(user):
        raise ValueError("bad user")
    fleetcfg.real_save(robot, host or default_host(robot), user or "robot", bool(body.get("cutter")), ACTOR)
    _real_cache["t"] = 0.0
    PROBES.pop(("real", robot), None)
    return {"real_robots": real_config()}


def act_real_remove(body):
    fleetcfg.real_remove(body.get("robot"), ACTOR)
    _real_cache["t"] = 0.0
    return {"real_robots": real_config()}


# One command per robot per status poll (the old per-field docker execs were 5 per robot, one after another; over
# WiFi with SSH that adds up): `key=value` lines. ros2 action list doubles as the move_group check (move_action).
ACTIONS_PROBE = ('timeout 6 ros2 action list 2>/dev/null | '
                 'sed -n "s#^/$ROBOT_NAMESPACE/\\(move_action\\|cut_stem\\)\\$#action=\\1#p"')
SIM_PROBE = (f"if pgrep -f '{LAUNCH_MATCH}' >/dev/null; then echo launch=1; {ACTIONS_PROBE}; fi; "
             f"pgrep -f '{CUT_MATCH}' >/dev/null && echo cutting=1; "
             f"pgrep -af '{RVIZ_MATCH}' | grep -o 'view_[a-z]*' | sed 's/^view_/rviz=/'; true")
REAL_PROBE = ("for s in " + " ".join(REAL_SERVICES) + "; do echo \"service=$s:$(systemctl is-active $s)\"; done; "
              f"pgrep -f '{CUT_MATCH}' >/dev/null && echo cutting=1; echo \"rmw=$RMW_IMPLEMENTATION\"; "
              "sed -n '1s/^multirobot_sim \\([0-9a-f]*\\).*/deployed=\\1/p' ~/colcon_ws/DEPLOYED 2>/dev/null; true")
# A ros2 CLI call costs a robot's CPU 1-5 s (5 s on a300_00036 while its zenoh router hung): on a real robot the
# actions (move_group, cut_stem) are checked this often, systemd and processes every poll.
REAL_ACTIONS_EVERY_S = 30
PROBES = {}  # (kind, name) -> {"t": time, "actions_t": time, "data": {...}}
OFFLINE_RETRY_S = 15
RETRYING = set()  # offline robots being probed again in the background


def probe(target, max_age=0.0):
    key = (target.kind, target.name)
    last = PROBES.get(key)
    if last and time.time() - last["t"] < max_age:
        return last["data"]
    if last and not last["data"]["online"]:
        # An unreachable robot costs ssh's ConnectTimeout (or a few seconds of mDNS) per try: retry it now and then,
        # in the background, so it doesn't hold up every status poll.
        if time.time() - last["t"] > OFFLINE_RETRY_S and key not in RETRYING:
            RETRYING.add(key)
            threading.Thread(target=lambda: (_probe(target, key), RETRYING.discard(key)), daemon=True).start()
        return last["data"]
    return _probe(target, key)


def _probe(target, key):
    last = PROBES.get(key) or {}
    command, actions_t = SIM_PROBE, time.time()
    if target.kind == "real":
        command = REAL_PROBE
        if time.time() - last.get("actions_t", 0) > REAL_ACTIONS_EVERY_S or not last["data"]["online"]:
            command += f"; {ACTIONS_PROBE}; true"
        else:
            actions_t = last["actions_t"]
    try:
        code, out = target.run(command, timeout=20)
    except subprocess.TimeoutExpired:
        code, out = -1, "no answer in 20 s"
    data = {"online": code == 0, "launch": False, "cutting": False, "move_group": False, "cut_stem_action": False,
            "rviz": [], "services": {}, "rmw": "", "deployed": "", "error": ""}
    if code != 0:
        data["error"] = (out.strip().splitlines() or [f"exit {code}"])[-1]
    for line in out.splitlines() if code == 0 else []:
        k, _, v = line.partition("=")
        if k in ("launch", "cutting"):
            data[k] = True
        elif k == "action":
            data["move_group" if v == "move_action" else "cut_stem_action"] = True
        elif k == "rviz":
            data["rviz"].append(v[len("view_"):] if v.startswith("view_") else v)
        elif k == "service":
            name, _, state = v.partition(":")
            data["services"][name] = state
        elif k in ("rmw", "deployed"):
            data[k] = v
    if target.kind == "real":
        data["launch"] = data["services"].get("clearpath-platform-extras") == "active"
        if code == 0 and actions_t == last.get("actions_t"):  # not checked this time: the last check's
            data["move_group"], data["cut_stem_action"] = last["data"]["move_group"], last["data"]["cut_stem_action"]
    PROBES[key] = {"t": time.time(), "actions_t": actions_t, "data": data}
    return data


def running_sims(rows=None):
    return {c["name"] for c in (containers() if rows is None else rows)
            if c["state"] == "running" and re.fullmatch(r"robot\d+", c["service"])}


CONFLICT = ("{name} is both a running sim robot and an online real robot: they share every ROS name and the base "
            "station can bridge their networks, so a command could reach the other one. Spawn the sim with another "
            "model, or take the real robot off the list")


def resolve(body, kinds=("sim", "real"), check_conflict=True):
    """The robot an action is for: body {"robot": <name>, "kind": "sim" (default) | "real"}."""
    kind, name = body.get("kind") or "sim", body.get("robot")
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ValueError("bad robot name")
    if kind not in kinds:
        raise ValueError(f"not for a {kind} robot")
    if kind == "sim":
        if name not in running_sims():
            raise ValueError(f"robot {name} is not running")
        target = SimTarget(name)
        if check_conflict and name in real_config() and probe(real_target(name), max_age=30)["online"]:
            raise ValueError(CONFLICT.format(name=name))
        return target
    target = real_target(name)
    if check_conflict and name in running_sims():
        raise ValueError(CONFLICT.format(name=name))
    return target


# ---------------------------------------------------------------- jobs

class Job:
    def __init__(self, name):
        self.id = uuid.uuid4().hex[:8]
        self.name = name
        self.status = "running"
        self.lines = []
        self.result = None
        self.started = time.time()

    def log(self, line):
        self.lines.append(line.rstrip("\n"))
        del self.lines[:-2000]

    def run(self, argv, **kw):
        # an ssh argv ends in the whole remote script: log what it runs, not the ssh options
        self.log("$ " + (f"ssh {argv[-2]} {argv[-1]}" if argv[0] == "ssh" else " ".join(argv)))
        p = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, **kw)
        for line in p.stdout:
            self.log(line)
        return p.wait()

    def to_json(self, full=False):
        d = {"id": self.id, "name": self.name, "status": self.status, "started": self.started}
        if full:
            d["log"] = self.lines
            d["result"] = self.result
        return d


JOBS = {}
JOBS_LOCK = threading.Lock()


def start_job(name, fn):
    job = Job(name)
    with JOBS_LOCK:
        JOBS[job.id] = job
        for old in sorted(JOBS.values(), key=lambda j: j.started)[:-50]:
            JOBS.pop(old.id)

    def target():
        try:
            ok = fn(job)
            # not `ok in (None, True, 0)`: False == 0, so every failed `return rc == 0` read as done
            job.status = "done" if ok is None or ok is True or (type(ok) is int and ok == 0) else "failed"
        except Exception as e:  # surfaced in the job log rather than killing the server
            job.log(f"error: {e!r}")
            job.status = "failed"

    threading.Thread(target=target, daemon=True).start()
    return job


def label(t):
    return t.name if t.kind == "sim" else f"{t.name} (real)"


# ---------------------------------------------------------------- state

def read_env():
    """The variables compose sees: .env, rendered again from the settings database first when it is reachable (so a
    change made elsewhere, e.g. scripts/fleetcfg.py set, shows here)."""
    return fleetcfg.env(ACTOR)


def write_env(updates):
    """Settings into the database's active profile, then .env rendered from it. Refused (ValueError) while the
    database can't be reached, unless the values are already in effect."""
    try:
        return fleetcfg.set_settings(updates, ACTOR)
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't change settings: {e}") from None


def available_models():
    # what the running sim imported and can spawn; without a sim, every generated URDF (scripts/gen_urdf.sh)
    state = fleet_ctl.read_state()
    if state and state.get("models"):
        return sorted(state["models"])
    return sorted(d.name for d in (ROOT / "sim/assets").iterdir() if (d / f"{d.name}.urdf").exists())


SCENE_DIR = ROOT / "sim/scene"  # mounted in the sim as /sim/scene
SCENE_EXTS = (".usd", ".usda", ".usdc", ".usdz")
SCENE_MAX_BYTES = 512 * 1024 * 1024
# "lavender" is only reachable through .env (SIM_SCENE=lavender); the page offers a file picker + Default
BUILTIN_SCENES = {"": "Default (ground plane + lights)", "lavender": "Lavender farm (built-in)"}


def scene_files():
    """USD files under sim/scene/ (paths relative to it), e.g. scenes saved from Isaac Sim with File > Save As."""
    if not SCENE_DIR.is_dir():
        return []
    return sorted(str(p.relative_to(SCENE_DIR)) for p in SCENE_DIR.rglob("*")
                  if p.is_file() and p.suffix.lower() in SCENE_EXTS)


def scene_label(scene):
    return BUILTIN_SCENES.get(scene, f"sim/scene/{scene}")


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def act_scene_upload(body):
    """The page's file picker can only hand over the file's contents, not its path. A scene's assets are usually
    referenced relative to it (Isaac's Save As writes ../assets/...), so it has to be loaded from where it lives:
    a file under sim/scene/ with the same contents is used in place; anything else is copied into sim/scene/
    (never over a different file of the same name) and its relative references then resolve from there."""
    name = Path(str(body.get("name") or "")).name
    if not name.lower().endswith(SCENE_EXTS):
        raise ValueError(f"pick a USD file ({', '.join(SCENE_EXTS)})")
    try:
        data = base64.b64decode(body.get("data") or "", validate=True)
    except ValueError:
        raise ValueError("bad file data")
    if not data or len(data) > SCENE_MAX_BYTES:
        raise ValueError(f"the file must be 1 byte to {SCENE_MAX_BYTES >> 20} MB")
    digest = hashlib.sha256(data).hexdigest()
    same_size = [f for f in scene_files() if (SCENE_DIR / f).stat().st_size == len(data)]
    # prefer a file with the picked name (the usual case: picking a scene saved into sim/scene/)
    for f in sorted(same_size, key=lambda f: Path(f).name != name):
        if _sha256(SCENE_DIR / f) == digest:
            return {"scene": f, "label": scene_label(f), "copied": False}
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    try:
        SCENE_DIR.chmod(0o777)  # the sim (uid 1234) saves scenes here too
    except OSError:
        pass
    dest = SCENE_DIR / name
    if dest.exists():
        dest = SCENE_DIR / f"{Path(name).stem}_{digest[:8]}{Path(name).suffix}"
    dest.write_bytes(data)
    f = str(dest.relative_to(SCENE_DIR))
    return {"scene": f, "label": scene_label(f), "copied": True}


def containers():
    _, out = sh(["docker", "ps", "-a", "--filter", f"label=com.docker.compose.project={PROJECT}",
                 "--format", '{{.Names}}\t{{.Label "com.docker.compose.service"}}\t{{.State}}\t{{.Status}}'])
    rows = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) == 4:
            rows.append(dict(zip(("name", "service", "state", "status"), parts)))
    return rows


def container_state(name):
    code, out = sh(["docker", "inspect", "-f", "{{.State.Status}}", name], timeout=10)
    return out.strip() if code == 0 else None


def zenoh_endpoints(env):
    return env.get("BASESTATION_ZENOH_CONNECT", "tcp/127.0.0.1:7448").strip("\"'").split()


HOST_IPS = {}  # host -> (time, ip or None); an mDNS name that doesn't resolve can take seconds


def host_ip(host):
    if re.fullmatch(r"\d{1,3}(\.\d{1,3}){3}", host):
        return host
    last = HOST_IPS.get(host)
    if last and time.time() - last[0] < 60:
        return last[1]
    try:
        ip = socket.gethostbyname(host)
    except OSError:
        ip = None
    HOST_IPS[host] = (time.time(), ip)
    return ip


def known_ip(host):
    """host_ip() without a lookup: a literal IP, or the last one resolved (None if never)."""
    if re.fullmatch(r"\d{1,3}(\.\d{1,3}){3}", host):
        return host
    return (HOST_IPS.get(host) or (0, None))[1]


DEPLOY_CURRENT = {}  # (deployed commit, HEAD) -> bool or None


def deploy_current(deployed, head):
    """Whether a robot's deployed commit has this checkout's colcon_ws/src (submodule pointers included): a newer
    commit that changed nothing there needs no deploy. None: that commit isn't in this repository."""
    if not deployed or not head:
        return None
    if (deployed, head) not in DEPLOY_CURRENT:
        code, _ = sh(["git", "-C", str(ROOT), "diff", "--quiet", deployed, head, "--", "colcon_ws/src"], timeout=10)
        DEPLOY_CURRENT[(deployed, head)] = {0: True, 1: False}.get(code)
    return DEPLOY_CURRENT[(deployed, head)]


# What the base station has of each real robot: its RViz windows, and whether the robot's graph reaches it (a robot
# publishing platform/joint_states). Configured isn't enough: a300_00036's router once took the base station's link
# but logged "Could not find corresponding link in routers network" and routed nothing.
BASESTATION_PROBE = (f"pgrep -af '{RVIZ_MATCH}'; timeout 6 ros2 topic list -v 2>/dev/null | "
                     "sed -n 's#^ \\* /\\([A-Za-z0-9_]*\\)/platform/joint_states .* publishers\\?$#seen=\\1#p'")


def basestation_view():
    """({robot: [RViz views open]}, {robots whose topics the base station sees}); ({}, None) without it."""
    try:
        code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", BASESTATION_PROBE], timeout=15)
    except subprocess.TimeoutExpired:
        return {}, None
    if code != 0:
        return {}, None
    views = collections.defaultdict(list)
    for view, ns in re.findall(r"view_(\w+)\.launch\.py namespace:=(\w+)", out):
        views[ns].append(view)
    return views, set(re.findall(r"^seen=(\w+)$", out, re.M))


def robots(mode, rows, env, head):
    sims = [c for c in rows if re.fullmatch(r"robot\d+", c["service"])] if mode != "real" else []
    config = real_config() if mode != "sim" else {}
    reals = [real_target(name, config) for name in sorted(config)]
    targets = [SimTarget(c["name"]) for c in sims if c["state"] == "running"] + reals
    with ThreadPoolExecutor(max_workers=len(targets) + 1) as pool:
        bs = pool.submit(basestation_view) if reals else None
        probed = dict(zip(((t.kind, t.name) for t in targets), pool.map(probe, targets)))
        views, seen = bs.result() if bs else ({}, None)
    running = running_sims(rows)
    out = []
    for c in sims:
        r = dict(c, kind="sim", slot=int(c["service"][5:]), cutter=True, **probed.get(("sim", c["name"]), {}))
        real = PROBES.get(("real", c["name"]))
        r["conflict"] = c["name"] in running and c["name"] in real_config() and bool(real and real["data"]["online"])
        out.append(r)
    endpoints, zenoh = zenoh_endpoints(env), env.get("FLEET_RMW", "rmw_zenoh_cpp") == "rmw_zenoh_cpp"
    for t in reals:
        p = probed[("real", t.name)]
        ip = host_ip(t.host)
        out.append(dict(p, kind="real", name=t.name, host=t.host, user=t.user, cutter=t.cutter, ip=ip,
                        state="online" if p["online"] else "offline", rviz=views.get(t.name, []),
                        conflict=t.name in running and p["online"],
                        deploy_current=deploy_current(p["deployed"], head),
                        # the base station's router dials the robot's (basestation.compose.yml); None: not zenoh
                        zenoh_linked=(ip is not None and f"tcp/{ip}:7447" in endpoints) if zenoh else None,
                        # and the robot's topics actually reach it; None: no base station, or can't tell (offline,
                        # or a sim robot of the same name publishes the same topics)
                        basestation_sees=None if seen is None or not p["online"] or t.name in running
                        else t.name in seen))
    return sorted(out, key=lambda r: (r["kind"] != "sim", r.get("slot", 0), r["name"]))


def status(q):
    mode = q.get("mode") if q.get("mode") in MODES else DEFAULT_MODE
    env = read_env()
    rows = containers()
    sim = next((c for c in rows if c["service"] == "isaac-sim"), None)
    n = int(env.get("NUM_ROBOTS", "0") or 0)
    code, head = sh(["git", "-C", str(ROOT), "rev-parse", "HEAD"], timeout=10)
    head = head.strip() if code == 0 else ""
    return {
        "mode": mode,
        "default_mode": DEFAULT_MODE,
        "sim": sim,
        # the running sim's own report (scene loading/ready, robots in it, last spawn); None if not running
        "scene": fleet_ctl.read_state() if sim and sim["state"] == "running" else None,
        # last spawn request that worked, else the last one written (prefills the spawn form)
        "request": fleet_ctl.read_request(applied=True) or fleet_ctl.read_request(),
        "num_robots": n,
        "slots": [env.get(f"ROBOT_MODEL_{i}", "a300") for i in range(MAX_SLOTS)],
        "models": available_models(),
        "robots": robots(mode, rows, env, head),
        "max_slots": MAX_SLOTS,
        "sim_mode": env.get("SIM_MODE", "stream"),
        "robot_looks": env.get("ROBOT_LOOKS", "full"),
        "sim_scene": env.get("SIM_SCENE", ""),
        "sim_scene_label": scene_label(env.get("SIM_SCENE", "")),
        "fleet_rmw": env.get("FLEET_RMW", "rmw_zenoh_cpp"),
        "use_sim_time": env.get("USE_SIM_TIME", "true"),
        "basestation": container_state(BASESTATION) if mode != "sim" else None,
        "known_real_ids": known_real_ids() if mode != "sim" else [],
        "head": head,
        # where .env came from: the settings database (its active profile), or "offline" (.env as last written)
        "settings": {k: fleetcfg.LAST_RENDER.get(k) for k in ("source", "profile", "why", "errors")},
    }


# ---------------------------------------------------------------- actions

SIM_MODES = ("stream", "headed")
ROBOT_LOOKS = ("full", "basic", "off")  # sim/scripts/robot_looks.py MODE


def host_display():
    """The X display for SIM_MODE=headed: this server's $DISPLAY, else the first local X socket (:N)."""
    if os.environ.get("DISPLAY"):
        return os.environ["DISPLAY"]
    sockets = sorted(Path("/tmp/.X11-unix").glob("X*"))
    return f":{sockets[0].name[1:]}" if sockets else None


def act_sim_start(body):
    # mode: "stream" (headless, WebRTC client) or "headed" (Isaac Sim's own window on this machine's display).
    # Saved as the setting SIM_MODE; fleet.sh's `docker compose up -d` recreates the sim when it changes.
    current = read_env()
    mode = (body or {}).get("mode") or current.get("SIM_MODE", "stream")
    if mode not in SIM_MODES:
        raise ValueError(f"mode must be one of {SIM_MODES}")
    # looks: robot materials (ROBOT_LOOKS), "full" (textured), "basic" (plain colours) or "off" (importer's own);
    # saved like SIM_MODE, so a change recreates the sim too
    looks = (body or {}).get("looks") or current.get("ROBOT_LOOKS", "full")
    if looks not in ROBOT_LOOKS:
        raise ValueError(f"looks must be one of {ROBOT_LOOKS}")
    # scene: SIM_SCENE, "" (ground plane + lights), "lavender" or a file in sim/scene/; saved like SIM_MODE, so a
    # change recreates the sim
    scene = (body or {}).get("scene")
    scene = current.get("SIM_SCENE", "") if scene is None else scene
    if scene not in BUILTIN_SCENES and scene not in scene_files():
        raise ValueError(f"no scene {scene!r}: pick a USD file again")
    # before the job, so a refusal (database down and a value changed) shows on the page
    write_env({"SIM_MODE": mode, "ROBOT_LOOKS": looks, "SIM_SCENE": scene})

    def fn(j):
        env = dict(os.environ)
        if mode == "headed":
            display = host_display()
            if not display:
                j.log("headed mode needs an X display: no $DISPLAY and no /tmp/.X11-unix socket")
                return False
            env["DISPLAY"] = display
            j.log(f"headed: DISPLAY={display}")
            if j.run(["scripts/x11_auth.sh"], env=env) != 0:
                return False
        j.log(f"settings: SIM_MODE={mode} ROBOT_LOOKS={looks} SIM_SCENE={scene}")
        # scene only: the robots are spawned afterwards (act_spawn); waits until the scene is ready
        return j.run(["scripts/fleet.sh", "scene"], env=env) == 0

    return start_job(f"start sim ({mode}, looks {looks}, scene {scene_label(scene)})", fn)


def act_sim_stop(_):
    return start_job("stop sim", lambda j: j.run(["scripts/stop_sim.sh"]) == 0)


def act_sim_reset(_):
    # The running sim stops and plays its timeline (Isaac's Stop and Play buttons): every robot back at its spawn
    # state in seconds, instead of the ~1 min of restarting the sim container. Robot containers keep running.
    # reloaded so an edited scripts/fleet_ctl.py takes effect without restarting this server (a stale copy once
    # reset the sim before stopping the robots)
    def fn(j):
        importlib.reload(fleet_ctl)
        return fleet_ctl.reset(log=j.log)

    return start_job("reset scene", fn)


def act_spawn(body):
    # body: {robots: [{model, x, y, yaw}, ...]}, one per slot; x/y in m (world frame), yaw in degrees.
    # The sim checks the rest (ground limits, spacing, duplicate real robots) and the job log shows its answer.
    entries = body.get("robots")
    allowed = set(available_models())
    if not isinstance(entries, list) or len(entries) > MAX_SLOTS:
        raise ValueError(f"robots must be a list of up to {MAX_SLOTS} entries")
    poses = []
    for i, e in enumerate(entries):
        if not isinstance(e, dict) or e.get("model") not in allowed:
            raise ValueError(f"slot {i}: model must be one of {sorted(allowed)}")
        pose = {}
        for k in ("x", "y", "yaw"):
            try:
                pose[k] = float(e.get(k))
            except (TypeError, ValueError):
                raise ValueError(f"slot {i}: {k} must be a number")
            if not math.isfinite(pose[k]):
                raise ValueError(f"slot {i}: {k} must be finite")
        poses.append(pose)
    state = fleet_ctl.read_state()
    if not state or state.get("scene") != "ready":
        raise ValueError("the scene is not ready -- press Start and wait for it")
    if (state.get("spawn") or {}).get("state") == "spawning":
        raise ValueError("a spawn is already in progress")
    models = [e["model"] for e in entries]
    # slots 0..N-1 (model and pose) and NUM_ROBOTS into the settings database, before the job so a refusal shows on
    # the page; the poses are kept, so a later `scripts/fleet.sh spawn` places the robots here again
    try:
        fleetcfg.set_slots([dict(model=m, **p) for m, p in zip(models, poses)], ACTOR, num_robots=len(models))
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't save the robots: {e}") from None

    def fn(j):
        j.log(f"settings: NUM_ROBOTS={len(models)}, slots {', '.join(models) or '(none)'}")
        # the sim replaces its robots (the scene keeps running), then the robot containers are (re)created
        return j.run(["scripts/fleet.sh", "spawn", "--poses", json.dumps(poses)]) == 0

    return start_job(f"spawn {len(models)} robot(s)", fn)


def act_launch_start(body):
    robot = resolve(body, kinds=("sim",)).name

    def fn(j):
        if sh(["docker", "exec", robot, "pgrep", "-f", "sim_robot_upstart.launch.py"])[0] == 0:
            j.log("already running")
            return True
        # The launch reads /etc/clearpath/robot.srdf, which the container's boot (robot/bin/generate_srdf) writes
        # ~5-30 s after the container starts; launched before that it dies at once.
        deadline = time.time() + SRDF_WAIT_S
        if sh(["docker", "exec", robot, "test", "-s", SRDF])[0] != 0:
            j.log(f"waiting for {SRDF} (the robot is still booting)...")
            while sh(["docker", "exec", robot, "test", "-s", SRDF])[0] != 0:
                if time.time() > deadline:
                    j.log(f"no {SRDF} after {SRDF_WAIT_S} s; see `docker logs {robot}` ([generate_srdf] lines)")
                    return False
                time.sleep(2)
        rc = j.run(["docker", "exec", "-d", robot, "bash", "-c", LAUNCH_CMD])
        if rc != 0:
            return False
        time.sleep(5)  # a launch that can't start (missing file, bad package) exits within a second or two
        if sh(["docker", "exec", robot, "pgrep", "-f", "sim_robot_upstart.launch.py"])[0] != 0:
            j.log("the launch exited right away; end of its log:")
            j.log(SimTarget(robot).run(f"tail -n 15 {LAUNCH_LOG}")[1].rstrip())
            # nodes it started before failing (e.g. moveit_sim_bridge) outlive it and would double up with the
            # next launch's; restart_ros (= the Stop button) removes them
            j.run(["docker", "exec", robot, "restart_ros"])
            return False
        j.log(f"started; output in {robot}:{LAUNCH_LOG} (move_group comes up ~20 s later)")
        return True

    return start_job(f"{robot}: start sim_robot_upstart", fn)


def act_launch_stop(body):
    robot = resolve(body, kinds=("sim",)).name
    return start_job(f"{robot}: restart_ros", lambda j: j.run(["docker", "exec", robot, "restart_ros"]) == 0)


class NeedPassword(ValueError):
    """The robot's sudo wants a password: the page asks for it and sends the request again with it."""


def act_real_restart(body):
    # The real robot's counterpart of start/stop: Clearpath's services (clearpath-platform-extras runs bringup_main,
    # clearpath-manipulators the arm driver and move_group). Without a NOPASSWD sudoers entry for this, the page asks
    # for the robot user's sudo password: it goes to `sudo -S` on ssh's stdin, never on a command line or in a log,
    # and isn't kept (Handler.do_POST only takes it from this machine).
    t = resolve(body, kinds=("real",))
    password = body.get("password")
    if password is None:
        code, out = t.run("sudo -n true 2>&1", timeout=30)
        if code != 0:
            if "password" not in out:
                raise ValueError(f"sudo on {t.name}: {out.strip() or f'exit {code}'}")
            raise NeedPassword(f"sudo on {t.name} needs robot's password (or, to skip this: {SUDOERS_HINT})")
    elif not isinstance(password, str) or not password or len(password) > 1024 or "\n" in password:
        raise ValueError("bad password")

    def fn(j):
        if password is None:
            code, out = t.run("sudo -n systemctl restart clearpath-robot 2>&1", timeout=120)
        else:
            # -p '': no prompt in the output; -k: always read the password (a cached sudo timestamp would leave it
            # unread on stdin)
            code, out = t.run("sudo -k -S -p '' systemctl restart clearpath-robot 2>&1", timeout=120,
                              input=password + "\n")
        j.log(out.rstrip() or f"exit {code}")
        if code != 0:
            if password is not None and re.search(r"incorrect password|Sorry, try again|no password was provided", out):
                j.log(f"wrong sudo password for {t.user}@{t.host}")
            return False
        j.log("restarted; waiting for clearpath-platform-extras ...")
        for _ in range(30):
            code, out = t.run("systemctl is-active " + " ".join(REAL_SERVICES), timeout=20)
            if out.split()[1:2] == ["active"]:
                j.log("services: " + " ".join(f"{s}={v}" for s, v in zip(REAL_SERVICES, out.split())))
                return True
            time.sleep(2)
        j.log("clearpath-platform-extras isn't active after 60 s: see the robot's Service log")
        return False

    return start_job(f"{label(t)}: restart clearpath-robot", fn)


def act_rviz_start(body):
    t = resolve(body)
    view = body.get("view")
    if view not in RVIZ_VIEWS:
        raise ValueError(f"view must be one of {RVIZ_VIEWS}")
    exec_env = []
    if t.kind == "sim":
        where, ns, sim_time, log = t.name, "$ROBOT_NAMESPACE", "${USE_SIM_TIME:-false}", view
    else:
        if container_state(BASESTATION) != "running":
            raise ValueError("a real robot's RViz runs in the base station container: "
                             "docker compose -f basestation.compose.yml up -d")
        where, ns, sim_time, log = BASESTATION, t.name, "false", f"{t.name}_{view}"
        # a zenoh client of that robot's own router, like its link monitor: not through the base station's router,
        # where opening / closing RViz once held up another robot's data (router-to-router deadlock)
        exec_env = ["-e", f'ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/{host_ip(t.host) or t.host}:7447"]']
    match = f"[c]learpath_viz view_{view}.launch.py namespace:={t.name if t.kind == 'real' else ''}"

    def fn(j):
        if sh(["docker", "exec", where, "bash", "-c", f"pgrep -f '{match}'"])[0] == 0:
            j.log(f"view_{view} is already open")
            return True
        if sh(["docker", "exec", where, "bash", "-c",
               "source /home/robot/colcon_ws/install/setup.bash && ros2 pkg prefix clearpath_viz"])[0] != 0:
            j.log("clearpath_viz is not built: scripts/colcon_build.sh --packages-select clearpath_viz")
            return False
        env = dict(os.environ)
        display = host_display()
        if display:
            env["DISPLAY"] = display
            j.run(["scripts/x11_auth.sh"], env=env)  # .x11/xauth, mounted into the robot containers
        cmd = RVIZ_CMD.format(view=view, ns=ns, sim_time=sim_time, log=log)
        if j.run(["docker", "exec", "-d", *exec_env, where, "bash", "-c", cmd]) != 0:
            return False
        time.sleep(3)  # no display / bad package: the launch exits within a second or two
        if sh(["docker", "exec", where, "bash", "-c", f"pgrep -f '{match}'"])[0] != 0:
            j.log("the launch exited right away; end of its log:")
            j.log(sh(["docker", "exec", where, "tail", "-n", "15", f"/tmp/rviz_{log}.log"])[1].rstrip())
            return False
        j.log(f"started; output in {where}:/tmp/rviz_{log}.log")
        return True

    return start_job(f"{label(t)}: rviz {view}", fn)


def act_cutstem_start(body):
    t = resolve(body)
    if not t.cutter:
        raise ValueError(f"{t.name} has no stem cutter (tick 'cutter' in Real robots if it has one now)")

    def fn(j):
        if t.run(f"pgrep -f '{CUT_MATCH}'")[0] == 0:
            j.log("a cut_stem goal is already running")
            return False
        code, out = t.run("ros2 action list 2>/dev/null | grep -x /$ROBOT_NAMESPACE/cut_stem", timeout=20)
        if code != 0:
            j.log("no cut_stem action server -- " + ("start sim_robot_upstart first and wait ~20 s for it to come up"
                                                    if t.kind == "sim" else "is clearpath-platform-extras running?"))
            return False
        return j.run(t.argv(CUT_CMD)) == 0

    return start_job(f"{label(t)}: cut_stem", fn)


def act_cutstem_stop(body):
    t = resolve(body, check_conflict=False)  # stopping is always safe

    def fn(j):
        # SIGINT, not SIGKILL: ros2 action send_goal cancels the goal on Ctrl+C, so the server stops the task.
        code, _ = t.run(f"pkill -INT -f '{CUT_MATCH}'")
        j.log("sent Ctrl+C to the cut_stem client (goal is cancelled)" if code == 0 else "no cut_stem goal running")
        return True

    return start_job(f"{label(t)}: stop cut_stem", fn)


def stop_robot(j, t):
    """Ctrl+C to this UI's goal clients, then cancel every goal of every action that moves the robot's arm, so it
    stops even if a client is gone (e.g. this server restarted mid-goal) or another program sent the goal."""
    lines = []
    try:
        code, out = t.run(STOP_CMD, timeout=40)
        lines = out.strip().splitlines() or [f"exit {code}"]
    except subprocess.TimeoutExpired:
        lines = ["no answer in 40 s"]
    for line in lines:
        j.log(f"{label(t)}: {line}")


def act_stop(body):
    t = resolve(body, check_conflict=False)
    return start_job(f"{label(t)}: stop motion", lambda j: stop_robot(j, t))


def act_stop_all(body):
    """Every robot the page shows (its mode): sim robots running, real robots in the list and online."""
    mode = body.get("mode") if body.get("mode") in MODES else DEFAULT_MODE
    targets = [SimTarget(n) for n in sorted(running_sims())] if mode != "real" else []
    if mode != "sim":
        config = real_config()
        targets += [t for t in (real_target(n, config) for n in sorted(config)) if probe(t, max_age=30)["online"]]
    if not targets:
        raise ValueError("no robots to stop")

    def fn(j):
        with ThreadPoolExecutor(max_workers=len(targets)) as pool:
            list(pool.map(lambda t: stop_robot(j, t), targets))

    return start_job("stop all motion: " + ", ".join(label(t) for t in targets), fn)


def act_arm_goto(body):
    t = resolve(body)
    state, group = body.get("state"), body.get("group", "arm_0")
    if not (isinstance(state, str) and NAME_RE.match(state) and isinstance(group, str) and NAME_RE.match(group)):
        raise ValueError("bad state/group")
    vel = min(max(float(body.get("velocity_scale", 0.3)), 0.01), 1.0)
    if t.kind == "real":
        if body.get("direct"):
            raise ValueError("direct (unplanned) moves are sim only: they go to moveit_sim_bridge")
        vel = min(vel, REAL_MAX_VELOCITY)
    args = f"{state} --group {group} --velocity-scale {vel}" + (" --direct" if body.get("direct") else "")
    record = float(body.get("record", 0) or 0)

    def fn(j):
        if record <= 0:
            return j.run(t.argv(arm_tool("arm_goto", args))) == 0
        samples = []
        rec = subprocess.Popen(t.argv(arm_tool("arm_joints", f"--record {min(record, 120)}")),
                               stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        reader = threading.Thread(target=lambda: samples.extend(json.loads(l) for l in rec.stdout if l.startswith("{")))
        reader.start()
        time.sleep(1.5)  # let the recorder's own rclpy startup finish before the arm starts moving
        rc = j.run(t.argv(arm_tool("arm_goto", args)))
        j.log(f"recording joints for {record:.0f}s total ...")
        rec.wait()
        reader.join()
        j.result = {"samples": samples}
        j.log(f"recorded {len(samples)} samples")
        return rc == 0

    return start_job(f"{label(t)}: {group} -> {state}" + (" (direct)" if body.get("direct") else ""), fn)


def last_json(code, out, what):
    # the last JSON object line: zenoh can log after it (a300_00036: "close operation timed out!" at shutdown)
    lines = [l for l in out.strip().splitlines() if l.startswith("{")]
    if code != 0 or not lines:
        raise ValueError(out.strip() or f"{what} failed")
    return json.loads(lines[-1])


def get_arm_states(q):
    t = resolve(q, check_conflict=False)  # read only
    return last_json(*t.run(arm_tool("arm_goto", "--list")), "arm_goto --list")


def get_joints(q):
    t = resolve(q, check_conflict=False)
    return last_json(*t.run(arm_tool("arm_joints")), "arm_joints")


def get_launch_log(q):
    t = resolve(q, check_conflict=False)
    if t.kind == "sim":
        return {"log": t.run(f"tail -n 300 {LAUNCH_LOG}")[1]}
    return {"log": t.run("journalctl -u clearpath-platform-extras -n 300 --no-pager")[1]}


def act_deploy(body):
    """scripts/deploy_robot.sh: dry-run (what would change, and files edited on the robot), deploy (after the page
    showed a dry run; --yes, since the script's own delete prompt needs a terminal) or pull (the robot's edits)."""
    t = resolve(body, kinds=("real",), check_conflict=False)
    what = body.get("action")
    flags = {"dry-run": ["--dry-run"], "deploy": ["--yes"], "pull": ["--pull"]}
    if what not in flags:
        raise ValueError(f"action must be one of {sorted(flags)}")
    argv = ["scripts/deploy_robot.sh", t.name, "--host", t.host, "--user", t.user, *flags[what]]
    return start_job(f"{label(t)}: deploy {what}", lambda j: j.run(argv) == 0)


def act_basestation_link(body):
    """Add a real robot's zenoh router to the base station's (BASESTATION_ZENOH_CONNECT) and recreate the base
    station with it, so the base station, the sim and that robot share one ROS graph."""
    t = resolve(body, kinds=("real",), check_conflict=False)
    ip = host_ip(t.host)
    if not ip:
        raise ValueError(f"cannot resolve {t.host}: give the robot an IP address in Real robots")
    endpoints = zenoh_endpoints(read_env())
    endpoint = f"tcp/{ip}:7447"
    if endpoint in endpoints:
        raise ValueError(f"the base station already dials {endpoint}")
    value = " ".join(endpoints + [endpoint])
    write_env({"BASESTATION_ZENOH_CONNECT": value})

    def fn(j):
        j.log(f"settings: BASESTATION_ZENOH_CONNECT={value}")
        return j.run(["docker", "compose", "-f", "basestation.compose.yml", "up", "-d"]) == 0

    return start_job(f"base station: link {t.name} ({endpoint})", fn)


# ---------------------------------------------------------------- base station card

BASESTATION_COMPOSE = ["docker", "compose", "-f", "basestation.compose.yml"]
BASESTATION_IMAGE = "basestation:jazzy"
BASESTATION_EVERY_S = 10
BASESTATION_IDLE_S = 30
ENDPOINT_RE = re.compile(r"^tcp/[A-Za-z0-9.-]+:\d{1,5}$")


class BasestationWatch:
    """The Base station card's slow half: scripts/basestation_probe.py in the container (zenoh router and its links,
    PostgreSQL, ROS programs, the graph) and `docker stats`, refreshed in the background every 10 s while the card
    polls, so a poll never waits on them (~1 s each)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data, self.t, self.busy = {}, 0.0, False

    def poll(self, running):
        with self.lock:
            if not running:
                self.data, self.t = {}, 0.0
            elif not self.busy and time.time() - self.t > BASESTATION_EVERY_S:
                self.busy = True
                threading.Thread(target=self.refresh, daemon=True).start()
            return dict(self.data)

    def refresh(self):
        data = {}
        try:
            # stats first: sampled right after the probe, they'd measure the probe's own ros2 CLI
            code, out = sh(["docker", "stats", "--no-stream", "--format", "{{json .}}", BASESTATION], timeout=15)
            data["stats"] = json.loads(out) if code == 0 else None
            code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", "python3 /scripts/basestation_probe.py"],
                           timeout=25)
            try:
                data["probe"] = json.loads(out.strip().splitlines()[-1]) if code == 0 else None
            except (ValueError, IndexError):
                data["probe"] = None
            if data["probe"] is None:
                data["probe_error"] = out.strip()[-400:] or f"exit {code}"
        except (subprocess.TimeoutExpired, ValueError) as e:
            data.setdefault("probe_error", repr(e))
        finally:
            with self.lock:
                self.data, self.t, self.busy = dict(data, probed=time.time()), time.time(), False


BASESTATION_WATCH = BasestationWatch()


def unquote(value):
    return (value or "").strip().strip("\"'")


def get_basestation(q):
    """The container (state, health, image, settings it was created with vs. .env's) on every poll; the probe's
    report as of its last background run."""
    env = read_env()
    code, out = sh(["docker", "inspect", BASESTATION], timeout=10)
    info = json.loads(out)[0] if code == 0 else None
    # what basestation.compose.yml would give the container now (compose treats an empty value as unset)
    want = {"RMW_IMPLEMENTATION": unquote(env.get("BASESTATION_RMW")) or unquote(env.get("FLEET_RMW")) or "rmw_zenoh_cpp",
            "ROS_DOMAIN_ID": unquote(env.get("ROS_DOMAIN_ID")) or "0",
            "USE_SIM_TIME": unquote(env.get("USE_SIM_TIME")) or "true",
            "BASESTATION_ZENOH_CONNECT": " ".join(zenoh_endpoints(env))}
    r = {"exists": info is not None, "want": want, "endpoints_env": zenoh_endpoints(env)}
    if info is None:
        return r
    st = info["State"]
    have = dict(e.split("=", 1) for e in info["Config"]["Env"] if "=" in e)
    code, image = sh(["docker", "image", "inspect", "-f", "{{.Id}}\t{{.Created}}", BASESTATION_IMAGE], timeout=10)
    image_id, image_created = (image.strip().split("\t") + [""])[:2] if code == 0 else ("", "")
    r.update(state=st["Status"], health=(st.get("Health") or {}).get("Status"), started_at=st["StartedAt"],
             finished_at=st["FinishedAt"], exit_code=st["ExitCode"], restart_count=info["RestartCount"],
             have={k: have.get(k) for k in want},
             drift=[k for k in want if have.get(k, "") != want[k]],
             # the image was rebuilt since the container was created: Recreate runs the new one
             image_outdated=bool(image_id) and image_id != info["Image"], image_created=image_created)
    r.update(BASESTATION_WATCH.poll(st["Status"] == "running"))
    # name the router's endpoints (the container's own list, not .env's) and whether each has a live link
    # labels only: IPs already known (literal, or resolved earlier by a status poll), never a lookup here -- an
    # mDNS name that doesn't resolve (cpr-j100-0921.local) takes 5 s
    names = {"127.0.0.1": "sim (zenoh-router)"}
    config = real_config()
    for name in config:
        ip = known_ip(real_target(name, config).host)
        if ip:
            names[ip] = name
    probe = r.get("probe") or {}
    links = (probe.get("router") or {}).get("links") or []
    endpoints = []
    for e in (have.get("BASESTATION_ZENOH_CONNECT") or "").split():
        host, _, port = e.removeprefix("tcp/").rpartition(":")
        ip = known_ip(host) or host
        endpoints.append({"endpoint": e, "name": names.get(ip, ""), "in_env": e in r["endpoints_env"],
                          "linked": any(l["dir"] == "out" and l["remote"] == f"{ip}:{port}" for l in links)
                          if probe.get("router") else None})
    r["endpoints"] = endpoints
    # incoming links: robots' routers dialling this one; local ones are this container's own sessions
    r["sessions_local"] = sum(1 for l in links if l["dir"] == "in" and l["remote"].startswith("127."))
    r["incoming"] = [dict(l, name=names.get(l["remote"].rsplit(":", 1)[0], "")) for l in links
                     if l["dir"] == "in" and not l["remote"].startswith("127.")]
    return r


def get_basestation_log(q):
    which = q.get("which", "container")
    if which == "container":
        code, out = sh(["docker", "logs", "--tail", "300", BASESTATION], timeout=15)
    elif which == "router":
        code, out = sh(["docker", "exec", BASESTATION, "tail", "-n", "300", "/tmp/zenoh_router.log"], timeout=15)
    else:
        raise ValueError("which must be container or router")
    return {"log": re.sub(r"\x1b\[[0-9;]*m", "", out)}


def act_basestation(body):
    """Start / stop / restart / recreate (.env applied) / rebuild the base station container, restart its zenoh
    router, or add / remove one of the endpoints its router dials (BASESTATION_ZENOH_CONNECT, then recreate)."""
    action = body.get("action")
    if action in ("link", "unlink"):
        endpoint = (body.get("endpoint") or "").strip()
        if not ENDPOINT_RE.match(endpoint):
            raise ValueError("endpoint must look like tcp/<host or ip>:<port>")
        endpoints = zenoh_endpoints(read_env())
        if (endpoint in endpoints) == (action == "link"):
            raise ValueError(f"{endpoint} is {'already' if action == 'link' else 'not'} in BASESTATION_ZENOH_CONNECT")
        new = endpoints + [endpoint] if action == "link" else [e for e in endpoints if e != endpoint]
        value = " ".join(new)
        write_env({"BASESTATION_ZENOH_CONNECT": value})

        def fn(j):
            j.log(f"settings: BASESTATION_ZENOH_CONNECT={value}")
            return j.run(BASESTATION_COMPOSE + ["up", "-d"]) == 0

        return start_job(f"base station: {action} {endpoint}", fn)
    commands = {
        "start": BASESTATION_COMPOSE + ["up", "-d"],
        "stop": BASESTATION_COMPOSE + ["stop"],
        "restart": ["docker", "restart", BASESTATION],
        "recreate": BASESTATION_COMPOSE + ["up", "-d", "--force-recreate"],
        "rebuild": BASESTATION_COMPOSE + ["up", "-d", "--build"],
        # entrypoint.sh runs the router in a loop: it is back 2 s after it ends; sessions reconnect on their own
        "router": ["docker", "exec", BASESTATION, "pkill", "-f", "[r]mw_zenohd"],
    }
    if action not in commands:
        raise ValueError(f"action must be one of {sorted(commands) + ['link', 'unlink']}")
    if action != "start" and container_state(BASESTATION) is None:
        raise ValueError("there is no base station container: Start creates it")
    with BASESTATION_WATCH.lock:
        BASESTATION_WATCH.t = 0.0  # probe again at the next poll
    return start_job(f"base station: {action}", lambda j: j.run(commands[action]) == 0)


# ---------------------------------------------------------------- motion capture / localization

# Which Motive rigid body each robot follows: a ROS params file keyed by node (/<ns>/natnet_ref_pose), loaded by
# mtu32_bringup's bringup_main.launch.py between config/ref_localization.yaml and the hand-written
# config/ref_localization/<ns>.yaml. Written as JSON (valid YAML: the server stays stdlib-only) under a comment
# header. It is part of colcon_ws/src, so a real robot gets it with the workspace.
ASSIGNMENTS = ROOT / "colcon_ws/src/mtu32_husky/mtu32_bringup/config/ref_localization/assignments.yaml"
ASSIGNMENTS_HEADER = """# Motive rigid body per robot (natnet_ref_pose's rigid_body), written by multirobot_sim's web UI (tools/sim_ui).
# A robot without an entry follows the rigid body named after its namespace. Loaded by bringup_main.launch.py
# after ../ref_localization.yaml and before <namespace>.yaml.
"""
MOCAP_IDLE_S = 30  # the listener stops this long after the page's last poll


def read_assignments():
    try:
        text = "\n".join(l for l in ASSIGNMENTS.read_text().splitlines() if not l.lstrip().startswith("#"))
        data = json.loads(text or "{}")
    except (OSError, json.JSONDecodeError):
        return {}
    return {k.strip("/").split("/")[0]: v.get("ros__parameters", {}).get("rigid_body", "")
            for k, v in data.items() if k.endswith("/natnet_ref_pose")}


def write_assignments(mapping):
    data = {f"/{ns}/natnet_ref_pose": {"ros__parameters": {"rigid_body": rb}} for ns, rb in sorted(mapping.items())}
    tmp = ASSIGNMENTS.with_suffix(".tmp")
    tmp.write_text(ASSIGNMENTS_HEADER + json.dumps(data, indent=2) + "\n")
    tmp.replace(ASSIGNMENTS)


class MocapMonitor:
    """Listens to Motive's NatNet stream on this host while the page polls it: rigid bodies, tracking, rate.

    Registers for unicast (NAT_CONNECT from the data socket, as natnet_ref_pose.py does) and joins the default
    multicast group, so either of Motive's transmission types works."""

    def __init__(self):
        self.lock = threading.Lock()
        self.server = None
        self.thread = None
        self.last_poll = 0.0
        self.reset()

    def reset(self):
        self.version = None
        self.app = None
        self.names = {}
        self.bodies = {}
        self.frames = collections.deque()
        self.last_frame = None
        self.error = None

    def poll(self, server):
        with self.lock:
            self.last_poll = time.time()
            if server != self.server or not (self.thread and self.thread.is_alive()):
                self.server = server
                self.reset()
                self.thread = threading.Thread(target=self.run, args=(server,), daemon=True)
                self.thread.start()
            now = time.time()
            while self.frames and now - self.frames[0] > 2.0:
                self.frames.popleft()
            return {
                "server": server,
                "app": self.app, "natnet": ".".join(map(str, self.version[:2])) if self.version else None,
                "rate": round(len(self.frames) / 2.0, 1),
                # seconds since the last frame (None: none yet); bodies keep the last frame's values
                "age": round(now - self.last_frame, 1) if self.last_frame else None,
                "error": self.error,
                "bodies": [dict(id=i, name=self.names.get(i, ""), **b) for i, b in sorted(self.bodies.items())],
            }

    def run(self, server):
        try:
            local = self.local_ip(server)
            data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            data.bind(("", natnet.DATA_PORT))
            try:
                data.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                socket.inet_aton(natnet.DEFAULT_MULTICAST) + socket.inet_aton(local))
            except OSError:
                pass  # e.g. Motive on this machine (loopback): unicast only
        except OSError as e:
            self.error = f"cannot listen for Motive: {e}"
            return
        target = (server, natnet.COMMAND_PORT)

        def send(message_id):
            try:
                data.sendto(struct.pack("<HH", message_id, 0), target)
            except OSError as e:
                self.error = f"cannot reach {server}: {e}"

        last_connect = last_modeldef = 0.0
        last_number = None
        try:
            while self.server == server and time.time() - self.last_poll < MOCAP_IDLE_S:
                now = time.time()
                if now - last_connect > 2.0 and (not self.frames or now - self.frames[-1] > 1.0):
                    send(natnet.NAT_CONNECT)  # (re)register; also answers with the NatNet version
                    last_connect = now
                if self.version and now - last_modeldef > 5.0:
                    send(natnet.NAT_REQUEST_MODELDEF)  # rigid bodies renamed / added in Motive
                    last_modeldef = now
                if not select.select([data], [], [], 0.2)[0]:
                    continue
                packet, addr = data.recvfrom(65535)
                if addr[0] != server or len(packet) < 4:
                    continue
                message_id = struct.unpack_from("<H", packet)[0]
                try:
                    if message_id == natnet.NAT_SERVERINFO:
                        name, app_version, self.version = natnet.parse_server_info(packet)
                        self.app = f"{name} {'.'.join(map(str, app_version[:2]))}"
                        self.error = None
                    elif self.version is None:
                        continue
                    elif message_id == natnet.NAT_MODELDEF:
                        self.names = natnet.parse_model_def(packet, self.version)
                    elif message_id == natnet.NAT_FRAMEOFDATA:
                        number, bodies = natnet.parse_frame(packet, self.version)
                        if last_number is not None and 0 <= last_number - number < 1000:
                            continue  # the same frame twice (unicast + multicast)
                        last_number = number
                        with self.lock:
                            self.last_frame = time.time()
                            self.frames.append(self.last_frame)
                            self.bodies = {i: self.body(pos, quat, err, tracked)
                                           for i, (pos, quat, err, tracked) in bodies.items()}
                except (struct.error, ValueError, IndexError) as e:
                    self.error = f"cannot parse NatNet message {message_id}: {e}"
        finally:
            data.close()

    @staticmethod
    def local_ip(server):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((server, natnet.COMMAND_PORT))
            return s.getsockname()[0]
        finally:
            s.close()

    @staticmethod
    def body(pos, quat, err, tracked):
        x, y, z, w = quat
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))  # Motive Z-up
        return {"tracked": tracked, "pos": [round(v, 3) for v in pos], "yaw": round(yaw, 1),
                "err_mm": round(err * 1000, 2)}


MOCAP = MocapMonitor()
IP_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")


def get_mocap(q):
    server = q.get("server") or "192.168.50.80"
    if not IP_RE.match(server):
        raise ValueError("server must be an IPv4 address")
    mode = q.get("mode") if q.get("mode") in MODES else DEFAULT_MODE
    snap = MOCAP.poll(server)
    snap["assignments"] = read_assignments()
    # robots to offer, by the page's mode: MTU's real robots (ids with "_" the sim knows, and the real robots list)
    # and the running sim robots
    real = {m for m in available_models() if "_" in m} | set(real_config()) if mode != "sim" else set()
    sim = {r["name"] for r in containers() if r["service"].startswith("robot")} if mode != "real" else set()
    snap["robots"] = sorted(real | sim)
    return snap


def act_mocap_assign(body):
    robot, rigid_body = body.get("robot"), body.get("rigid_body", "")
    if not (isinstance(robot, str) and NAME_RE.match(robot)):
        raise ValueError("bad robot name")
    if not isinstance(rigid_body, str) or (rigid_body and not re.fullmatch(r"[\w.\- ]+", rigid_body)):
        raise ValueError("bad rigid body name")
    mapping = read_assignments()
    for ns in [ns for ns, rb in mapping.items() if rb == rigid_body and ns != robot]:
        del mapping[ns]  # a rigid body follows one robot
    if rigid_body and rigid_body != robot:
        mapping[robot] = rigid_body
    else:
        mapping.pop(robot, None)  # unassigned, or the default (rigid body named after the robot)
    write_assignments(mapping)
    return {"assignments": mapping}


# ---------------------------------------------------------------- communication quality (real robots)

# Four layers, each of which has looked fine while another was broken: radio (the robot's WiFi), network (ping from
# here, the base station's spot on the LAN), middleware (the robot's zenoh router: on 2026-10-06 a300_00036's
# stopped accepting sessions, 85 queued, while ping was perfect) and ROS data (scripts/link_monitor.py in the base
# station: odom, GPS, the TF chains). Everything runs only while the page polls /api/link and stops LINK_IDLE_S
# later. Thresholds: (degraded, bad) per measure.
LINK_IDLE_S = 30
LINK_SSH_EVERY_S = 10
CLOCK_TRUSTED_MS = 25  # a robot's clock offset is used only if measured to within this (half the SSH round trip)
PING_WINDOW_S = 60
LINK_LIMITS = {
    "rtt_ms": (20, 100), "loss_pct": (1, 5), "signal_dbm": (-67, -75),  # signal: below these
    "router_queue": (1, 10), "clock_ms": (50, 500), "ros_rtt_ms": (50, 250), "ros_timeouts_pct": (1, 5),
    "age_s": (0.2, 1.0), "gap_s": (0.5, 2.0), "silent_s": (1.0, 2.0), "rate_ratio": (0.8, 0.5),  # rate: below
}
# One SSH round per robot every LINK_SSH_EVERY_S: the interface toward this machine and, if it is WiFi, its link;
# the zenoh router; load; NTP. ~10 ms of shell on the robot.
LINK_PROBE = (
    "iface=$(ip route get {local} 2>/dev/null | sed -n 's/.* dev \\([^ ]*\\).*/\\1/p'); echo \"iface=$iface\"; "
    "if [ -n \"$iface\" ] && [ -d /sys/class/net/$iface/wireless ]; then "
    "iw dev $iface link 2>/dev/null | sed -n 's/^[[:space:]]*\\(SSID\\|freq\\|signal\\|tx bitrate\\|rx bitrate\\): /\\1=/p'; "
    "awk -v i=\"$iface:\" '$1 == i {{print \"retries=\" $9; print \"missed_beacons=\" $11}}' /proc/net/wireless; fi; "
    "echo \"router=$(systemctl is-active clearpath-zenoh-router)\"; "
    "echo \"router_queue=$(ss -ltnH 'sport = :7447' | awk '{{print $2; exit}}')\"; "
    "echo \"router_errors=$(journalctl -u clearpath-zenoh-router --since=-{every}s -q --no-pager 2>/dev/null | grep -c ERROR)\"; "
    "echo \"load=$(cut -d' ' -f1 /proc/loadavg)\"; echo \"cores=$(nproc)\"; "
    "echo \"ntp=$(timedatectl show -p NTPSynchronized --value 2>/dev/null)\"")


def level_of(value, limits, lower_is_worse=False):
    if value is None:
        return None
    degraded, bad = limits
    if lower_is_worse:
        return "bad" if value < bad else "warn" if value < degraded else "ok"
    return "bad" if value > bad else "warn" if value > degraded else "ok"


class Pinger:
    """`ping -O -i 1` to one robot, from this machine: RTT, jitter and loss over the last PING_WINDOW_S."""

    def __init__(self, host, error=None):
        self.host = host
        self.started = time.time()
        self.samples = collections.deque()  # (time, rtt ms or None for no answer)
        # a restart keeps the last one's error until a reply: else the robot flipped between "unreachable" and
        # "starting" every retry (j100_0921, unresolvable host)
        self.error = error
        self.proc = subprocess.Popen(["ping", "-n", "-O", "-i", "1", "-W", "1", host], stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            m = re.search(r"icmp_seq=\d+ .*time=([\d.]+) ms", line)
            if m:
                self.samples.append((time.time(), float(m.group(1))))
                self.error = None
            elif "no answer yet" in line:
                self.samples.append((time.time(), None))
            elif "PING" not in line and line.strip():
                self.error = line.strip()  # e.g. unknown host
        self.error = self.error or "ping stopped"

    def stop(self):
        self.proc.kill()

    def summary(self):
        now = time.time()
        while self.samples and now - self.samples[0][0] > PING_WINDOW_S:
            self.samples.popleft()
        rtts = [r for _, r in self.samples if r is not None]
        if not self.samples:
            return {"error": self.error} if self.error else {}
        return {"rtt_ms": round(sum(rtts) / len(rtts), 2) if rtts else None, "rtt_max_ms": max(rtts, default=None),
                # mean change between consecutive replies (RFC 3550 style, unsmoothed)
                "jitter_ms": round(sum(abs(a - b) for a, b in zip(rtts, rtts[1:])) / (len(rtts) - 1), 2)
                if len(rtts) > 1 else None,
                "loss_pct": round(100 * (len(self.samples) - len(rtts)) / len(self.samples), 1),
                "last_reply_s": round(now - max((t for t, r in self.samples if r is not None), default=0), 1)
                if rtts else None,
                "window_s": round(now - self.samples[0][0])}


class LinkWatch:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_poll = 0.0
        self.pingers = {}       # robot -> Pinger
        self.ssh = {}           # robot -> {"t": time, ...probe values}
        self.ssh_busy = set()
        # One scripts/link_monitor.py per robot, a zenoh client connected straight to that robot's router -- not
        # through the base station's router, whose router-to-router links to two robots deadlocked (2026-10-06):
        # one robot's trouble then held up the other's data. robot -> {"proc", "started", "endpoint", "error"}
        self.monitors = {}
        self.ros = {}           # robot -> (receive time, its last report)
        threading.Thread(target=self.idle_stop, daemon=True).start()

    # -- lifecycle: everything stops LINK_IDLE_S after the page's last poll
    def idle_stop(self):
        while True:
            time.sleep(5)
            with self.lock:
                if self.last_poll and time.time() - self.last_poll > LINK_IDLE_S:
                    self.stop_all()

    def stop_all(self):
        for p in self.pingers.values():
            p.stop()
        self.pingers = {}
        for name in list(self.monitors):
            self.stop_monitor(name)
        self.last_poll = 0.0

    @staticmethod
    def kill_monitor_process(name):
        # killing `docker exec` leaves its process running in the container: stop that one
        sh(["docker", "exec", BASESTATION, "pkill", "-INT", "-f", f"/scripts/[l]ink_monitor.py {name}$"], timeout=10)

    def stop_monitor(self, name):
        m = self.monitors.pop(name, None)
        if m and m["proc"]:
            self.kill_monitor_process(name)
            m["proc"].kill()
        self.ros.pop(name, None)

    def start_monitor(self, name, endpoint):
        self.stop_monitor(name)
        m = self.monitors[name] = {"proc": None, "started": time.time(), "endpoint": endpoint, "error": None}
        if container_state(BASESTATION) != "running":
            m["error"] = "the base station is not running"
            return
        self.kill_monitor_process(name)
        override = f'mode="client";connect/endpoints=["{endpoint}"]'  # replaces the container's (its own router)
        m["proc"] = subprocess.Popen(
            ["docker", "exec", "-e", f"ZENOH_CONFIG_OVERRIDE={override}", BASESTATION, "bash", "-c",
             f"exec python3 -u /scripts/link_monitor.py {name}"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self.read_monitor, args=(name, m), daemon=True).start()

    def read_monitor(self, name, m):
        tail = collections.deque(maxlen=5)
        for line in m["proc"].stdout:
            if line.startswith("{"):
                try:
                    report = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if self.monitors.get(name) is m:
                    self.ros[name] = (time.time(), report["robots"].get(name))
            else:
                tail.append(line.rstrip())
        if self.monitors.get(name) is m:
            m["error"] = "link_monitor.py stopped: " + (" | ".join(tail) or f"exit {m['proc'].wait()}")

    # -- SSH round per robot (background, so a slow robot never holds up a poll)
    def ssh_probe(self, t):
        try:
            local = MocapMonitor.local_ip(host_ip(t.host) or t.host)
            code, out = t.run(LINK_PROBE.format(local=local, every=LINK_SSH_EVERY_S), timeout=20, ros_env=False)
            data = dict(line.split("=", 1) for line in out.splitlines() if "=" in line) if code == 0 else {}
            # clock: the robot's time against the middle of a bare round trip (error: half of it), best of 3
            best = None
            for _ in range(3 if code == 0 else 0):
                t0 = time.time()
                code2, remote = t.run("date +%s.%N", timeout=10, ros_env=False)
                t1 = time.time()
                if code2 == 0 and (best is None or t1 - t0 < best[1]):
                    best = (float(remote.strip().splitlines()[-1]) - (t0 + t1) / 2, t1 - t0)
            if best:
                data["clock_offset_ms"] = round(best[0] * 1000, 1)
                data["clock_error_ms"] = round(best[1] / 2 * 1000, 1)
            prev = self.ssh.get(t.name, {})
            for key in ("retries", "missed_beacons"):  # counters since boot: per-interval change
                if key in data and key in prev:
                    data[key + "_delta"] = int(data[key]) - int(prev[key])
            data["error"] = None if code == 0 else (out.strip().splitlines() or [f"exit {code}"])[-1]
            data["t"] = time.time()
            self.ssh[t.name] = data
        except (subprocess.TimeoutExpired, OSError, ValueError) as e:
            self.ssh[t.name] = {"t": time.time(), "error": str(e)}
        finally:
            self.ssh_busy.discard(t.name)

    def poll(self, targets):
        with self.lock:
            self.last_poll = time.time()
            names = tuple(sorted(t.name for t in targets))
            for name in set(self.pingers) - set(names):
                self.pingers.pop(name).stop()
            for t in targets:
                p = self.pingers.get(t.name)
                # a new robot, a changed host, or a ping process that ended (killed, or the host didn't resolve):
                # start it (again); a dead one is retried at most every 10 s
                if p is None or p.host != t.host or (p.proc.poll() is not None and time.time() - p.started > 10):
                    if p is not None:
                        p.stop()
                    self.pingers[t.name] = Pinger(t.host, p.error if p is not None and p.host == t.host else None)
                if (time.time() - self.ssh.get(t.name, {}).get("t", 0) > LINK_SSH_EVERY_S
                        and t.name not in self.ssh_busy):
                    self.ssh_busy.add(t.name)
                    threading.Thread(target=self.ssh_probe, args=(t,), daemon=True).start()
            for name in set(self.monitors) - set(names):
                self.stop_monitor(name)
            for t in targets:
                # a robot's monitor starts once it answers ping (an unreachable one would only churn), and is
                # restarted for a new address, or 30 s after it last started if it has died since
                endpoint = f"tcp/{host_ip(t.host) or t.host}:7447"
                m = self.monitors.get(t.name)
                reachable = self.pingers[t.name].summary().get("rtt_ms") is not None
                if (m is None and reachable) or (m and m["endpoint"] != endpoint) or (
                        m and (m["proc"] is None or m["proc"].poll() is not None)
                        and time.time() - m["started"] > 30 and reachable):
                    self.start_monitor(t.name, endpoint)
            return {t.name: self.robot_report(t) for t in targets}

    # -- one robot: values + a level per value + the overall verdict and its reasons
    def robot_report(self, t):
        reasons = []
        sensor_faults = []  # robot-side: shown, not counted against the link

        def judge(label, value, limits, unit="", lower_is_worse=False, fmt="{:g}"):
            lv = level_of(value, limits, lower_is_worse)
            if lv in ("warn", "bad"):
                reasons.append((lv, f"{label} {fmt.format(value)}{unit}"))
            return lv

        ping = self.pingers[t.name].summary() if t.name in self.pingers else {}
        net = dict(ping, levels={})
        if ping.get("error"):
            reasons.append(("bad", f"ping: {ping['error']}"))
        elif ping.get("rtt_ms") is None and ping.get("window_s", 0) >= 3:
            reasons.append(("bad", "no ping replies"))
            net["levels"]["rtt_ms"] = "bad"
        else:
            net["levels"]["rtt_ms"] = judge("RTT", ping.get("rtt_ms"), LINK_LIMITS["rtt_ms"], " ms")
            net["levels"]["loss_pct"] = judge("ping loss", ping.get("loss_pct"), LINK_LIMITS["loss_pct"], " %")

        s = self.ssh.get(t.name, {})
        wifi = {k: s.get(k) for k in ("iface", "SSID", "freq", "signal", "tx bitrate", "rx bitrate",
                                      "retries_delta", "missed_beacons_delta") if s.get(k) not in (None, "")}
        signal = float(s["signal"].split()[0]) if s.get("signal") else None
        wifi["signal_dbm"] = signal
        wifi["levels"] = {"signal": judge("WiFi signal", signal, LINK_LIMITS["signal_dbm"], " dBm", True)}
        queue = int(s["router_queue"]) if s.get("router_queue", "").isdigit() else None
        errors = int(s["router_errors"]) if s.get("router_errors", "").isdigit() else None
        zenoh = {"router": s.get("router"), "queue": queue, "errors": errors, "levels": {}}
        if s.get("router") and s["router"] != "active":
            reasons.append(("bad", f"zenoh router {s['router']}"))
            zenoh["levels"]["router"] = "bad"
        zenoh["levels"]["queue"] = judge("zenoh router queue", queue, LINK_LIMITS["router_queue"])
        if errors:
            reasons.append(("warn", f"{errors} zenoh router errors in {LINK_SSH_EVERY_S} s"))
            zenoh["levels"]["errors"] = "warn"
        offset, err = s.get("clock_offset_ms"), s.get("clock_error_ms")
        # only an offset measured to within CLOCK_TRUSTED_MS is judged and used to correct the ROS ages
        if offset is not None and (err is None or err > CLOCK_TRUSTED_MS):
            offset = None
        clock = {"offset_ms": s.get("clock_offset_ms"), "error_ms": err, "trusted": offset is not None,
                 "ntp": s.get("ntp"), "load": s.get("load"), "cores": s.get("cores"),
                 "levels": {"offset_ms": judge("clock offset", max(0.0, abs(offset) - err) if offset is not None
                                               else None, LINK_LIMITS["clock_ms"], " ms", fmt="{:.0f}")}}
        if s.get("error"):
            reasons.append(("warn", f"SSH: {s['error']}"))

        m = self.monitors.get(t.name)
        ros = {"error": m["error"] if m else None}
        received, report = self.ros.get(t.name, (0.0, None))
        if report and time.time() - received < 5:
            correct = (offset or 0) / 1000  # age measured with this machine's clock against the robot's stamps
            # ref_localizer says it publishes no map -> odom (no Motive rigid body / GPS fix yet): that is the
            # robot's localization, not its link -- shown as information. Only when its status is fresh: a status
            # that stopped arriving is itself a link (or bringup) problem.
            loc = report.get("localizer")
            no_source = None
            if loc and loc.get("age", 99) < 5 and loc.get("publishing") is False:
                ages = ", ".join(f"{k} {'none' if loc.get(f'{k}_age') is None else str(loc[f'{k}_age']) + ' s old'}"
                                 for k in ("ref", "gps"))
                no_source = f"not published: ref_localizer has no localization source (source {loc.get('source')}; {ages})"
            ros["localizer"] = loc
            # Streams of this robot that do arrive: if some do, the link works, and a topic that delivers nothing
            # is the robot's sensor / driver (a200_0284's GPS, 2026-10-06), not the link -- a sensor fault, shown
            # apart. If nothing arrives at all, it stays a link problem.
            alive = {n for g in ("topics", "tf") for n, s in report[g].items()
                     if not s.get("broken") and s.get("silent") is not None
                     and s["silent"] <= LINK_LIMITS["silent_s"][1]}
            for group, items in (("topics", report["topics"]), ("tf", report["tf"])):
                ros[group] = {}
                for name, v in items.items():
                    v = dict(v, levels={})
                    if group == "tf" and name == "map->odom" and no_source and (v.get("broken") or (
                            v.get("silent") is None or v["silent"] > LINK_LIMITS["silent_s"][1])):
                        ros[group][name] = {"info": no_source, "levels": {}}
                        continue
                    if v.get("broken"):
                        reasons.append(("bad", f"TF {name}: {v['broken']}"))
                        v["levels"]["broken"] = "bad"
                    elif not v.get("static"):
                        if v.get("age") is not None:
                            v["age"] = round(v["age"] + correct, 4)
                        silent = v.get("silent")
                        if silent is None or silent > LINK_LIMITS["silent_s"][1]:
                            since = "since the monitor started" if silent is None else f"for {silent:.0f} s"
                            if group == "topics" and alive - {name}:
                                v["sensor_fault"] = (f"no data {since} while the robot's other data arrive: "
                                                     "its sensor or driver on the robot, not the link")
                                v["levels"]["rate"] = "sensor"
                                sensor_faults.append(f"{name}: no data {since}")
                            else:
                                reasons.append(("bad", f"{name}: no data {since}"))
                                v["levels"]["rate"] = "bad"
                        else:
                            ratio = v["rate"] / v["usual_rate"] if v.get("usual_rate") else None
                            v["levels"]["rate"] = judge(f"{name} rate", ratio, LINK_LIMITS["rate_ratio"], "x usual",
                                                        True, "{:.2f}")
                            v["levels"]["max_gap"] = judge(f"{name} gap", v.get("max_gap"), LINK_LIMITS["gap_s"], " s")
                            v["levels"]["age"] = judge(f"{name} age", v.get("age"), LINK_LIMITS["age_s"], " s",
                                                       fmt="{:.3f}")
                    ros[group][name] = v
            # request + reply through zenoh both ways (link_monitor.py: robot_state_publisher's parameter service)
            rt = dict(report.get("round_trip") or {}, levels={})
            if rt.get("error"):
                reasons.append(("warn", rt["error"]))
            else:
                rt["levels"]["rtt_ms"] = judge("ROS round trip", rt.get("rtt_ms"), LINK_LIMITS["ros_rtt_ms"], " ms")
                rt["levels"]["timeouts_pct"] = judge("ROS round trips unanswered", rt.get("timeouts_pct"),
                                                     LINK_LIMITS["ros_timeouts_pct"], " %")
            ros["round_trip"] = rt
        elif not ros["error"]:
            ros["error"] = "waiting for this robot's link monitor (a client of its own router)..."

        if ping.get("error") or (ping.get("rtt_ms") is None and ping.get("window_s", 0) >= 3):
            # unreachable: everything else follows from that
            reasons = [("bad", "unreachable: " + (ping.get("error") or "no ping replies"))]
        worst = "bad" if any(l == "bad" for l, _ in reasons) else "warn" if reasons else "ok"
        if reasons and reasons[0][1].startswith("unreachable"):
            sensor_faults = []  # can't tell while the robot is unreachable
        return {"host": t.host, "level": worst, "reasons": [r for _, r in sorted(reasons, key=lambda r: r[0] != "bad")],
                "sensor_faults": sensor_faults,
                "network": net, "wifi": wifi, "zenoh": zenoh, "clock": clock, "ros": ros}


LINK = LinkWatch()


def get_link(q):
    """Communication quality of the real robots in the list (the page polls this while its card is shown)."""
    config = real_config()
    targets = [real_target(name, config) for name in sorted(config)]
    return {"robots": LINK.poll(targets), "limits": LINK_LIMITS}


LOC_SOURCES = ("auto", "ref", "gps", "external")
LOC_ANCHORS = ("fixed", "start", "external")
LOC_NODE = "/$ROBOT_NAMESPACE/ref_localizer"


def get_localization(q):
    t = resolve(q, check_conflict=False)
    code, out = t.run(f"timeout 6 ros2 topic echo --once --no-daemon --full-length {LOC_NODE}/status "
                      "std_msgs/msg/String", timeout=15)
    m = re.search(r"^data: '(.*)'$", out, re.M)
    if code != 0 or not m:
        return {"running": False}  # no ref_localizer (sim_robot_upstart / clearpath-platform-extras not running)
    return {"running": True, **json.loads(m.group(1).replace("''", "'"))}


def act_localization_set(body):
    t = resolve(body)
    cmds = []
    if body.get("source"):
        if body["source"] not in LOC_SOURCES:
            raise ValueError(f"source must be one of {LOC_SOURCES}")
        cmds.append(f"ros2 param set {LOC_NODE} source {body['source']}")
    if body.get("anchor"):
        if body["anchor"] not in LOC_ANCHORS:
            raise ValueError(f"anchor must be one of {LOC_ANCHORS}")
        cmds.append(f"ros2 param set {LOC_NODE} anchor {body['anchor']}")
    if body.get("service"):
        if body["service"] not in ("save_anchor", "reset_anchor"):
            raise ValueError("service must be save_anchor or reset_anchor")
        # ros2 service call exits 0 whatever the response: fail the job on success=False
        cmds.append(f'out=$(ros2 service call {LOC_NODE}/{body["service"]} std_srvs/srv/Trigger); echo "$out"; '
                    f'grep -q "success=True" <<< "$out"')
    if not cmds:
        raise ValueError("nothing to do")
    what = ", ".join(f"{k} {body[k]}" for k in ("source", "anchor", "service") if body.get(k))
    return start_job(f"{label(t)}: localization {what}", lambda j: all(j.run(t.argv(c)) == 0 for c in cmds))


# ---------------------------------------------------------------- configuration page (the settings database)

CONFIG_PAGE = Path(__file__).with_name("config.html")
# Not compared between running containers and the settings: the shell's own (the server's DISPLAY isn't the one a
# container was made with), and secrets from db.env, never sent to the page.
NOT_COMPARED = {"DISPLAY", "XAUTHORITY"}
SECRET_RE = re.compile(r"PASSWORD|SECRET|TOKEN|KEY", re.I)
DRIFT_EVERY_S = 4
_drift = {"t": 0.0, "data": None, "lock": threading.Lock()}


def jsonable(rows):
    """datetimes (timestamptz) as ISO strings: the handler's json.dumps knows no datetime."""
    def conv(v):
        return v.isoformat() if hasattr(v, "isoformat") else v
    return [{k: conv(v) for k, v in r.items()} for r in rows]


def compose_services(args):
    """What `docker compose config` resolves now (from .env): {service: {...}}, or None."""
    try:
        p = subprocess.run(["docker", "compose", *args, "config", "--format", "json"], cwd=ROOT, capture_output=True,
                           text=True, timeout=30, stdin=subprocess.DEVNULL)
        return json.loads(p.stdout)["services"] if p.returncode == 0 else None
    except (subprocess.TimeoutExpired, ValueError, KeyError):
        return None


def env_diff(want, info):
    have = dict(e.split("=", 1) for e in info["Config"]["Env"] if "=" in e)
    out = []
    for k, v in sorted((want.get("environment") or {}).items()):
        if k in NOT_COMPARED or SECRET_RE.search(k):
            continue
        v = "" if v is None else str(v)
        if have.get(k, "") != v:
            out.append({"key": k, "running": have.get(k), "configured": v})
    if want.get("hostname") and info["Config"].get("Hostname") != want["hostname"]:
        out.append({"key": "hostname", "running": info["Config"].get("Hostname"), "configured": want["hostname"]})
    return out


def inspect_all(names):
    if not names:
        return {}
    code, out = sh(["docker", "inspect", *names], timeout=15)
    try:
        return {i["Name"].lstrip("/"): i for i in json.loads(out)} if code == 0 else {}
    except ValueError:
        return {}


def config_drift():
    """What the running containers differ in from the settings (they read them at creation): per target, the
    variables (running vs. configured) and robot slots to start, rename or stop. Cached for a few seconds."""
    with _drift["lock"]:
        if _drift["data"] is not None and time.time() - _drift["t"] < DRIFT_EVERY_S:
            return _drift["data"]
        want = compose_services([]) or {}
        rows = [c for c in containers() if c["state"] == "running"]
        bs_want = (compose_services(["-f", "basestation.compose.yml"]) or {}).get("basestation")
        infos = inspect_all([c["name"] for c in rows] + ([BASESTATION] if container_state(BASESTATION) == "running" else []))
        by_service = {c["service"]: c for c in rows}
        data = {"sim": None, "robots": [], "basestation": None}
        sim = by_service.get("isaac-sim")
        if sim and "isaac-sim" in want and sim["name"] in infos:
            data["sim"] = {"running": True, "diff": env_diff(want["isaac-sim"], infos[sim["name"]])}
        else:
            data["sim"] = {"running": False, "diff": []}
        for i in range(MAX_SLOTS):
            svc, c = f"robot{i}", by_service.get(f"robot{i}")
            w = want.get(svc)
            if w is None and c is None:
                continue
            row = {"slot": i, "configured": w.get("container_name") if w else None, "running": c["name"] if c else None}
            if w is None:
                row["problem"] = "running, but NUM_ROBOTS doesn't start this slot"
            elif c is None:
                row["problem"] = "not running" if data["sim"]["running"] else "not running (the sim isn't either)"
            elif c["name"] != w.get("container_name"):
                row["problem"] = f"running as {c['name']}, configured as {w.get('container_name')}"
            else:
                row["diff"] = env_diff(w, infos.get(c["name"], {"Config": {"Env": []}}))
            data["robots"].append(row)
        if bs_want and BASESTATION in infos:
            data["basestation"] = {"running": True, "diff": env_diff(bs_want, infos[BASESTATION])}
        else:
            data["basestation"] = {"running": False, "diff": []}
        _drift.update(t=time.time(), data=data)
        return data


def get_config(q):
    v = fleetcfg.view(ACTOR)
    v["models"] = sorted(set(available_models()) | set(fleetcfg.cat.GENERIC_MODELS))
    v["scenes"] = [""] + (["lavender"]) + scene_files()
    v["real_robots"] = [dict(id=k, **c) for k, c in sorted(real_config().items())]
    v["known_real_ids"] = known_real_ids()
    v["profiles"] = jsonable(v["profiles"])
    v["drift"] = config_drift()
    return v


def get_config_history(q):
    return jsonable(fleetcfg.history(int(q.get("n", 100)), ACTOR))


def get_config_runs(q):
    return jsonable(fleetcfg.runs(int(q.get("n", 50)), ACTOR))


def get_config_export(q):
    return fleetcfg.profile_export(q.get("profile") or None, ACTOR)


def config_change(fn, *args, **kw):
    """A fleetcfg change, with the database being down as a plain refusal; the drift cache is stale after it."""
    try:
        r = fn(*args, actor=ACTOR, **kw)
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't change settings: {e}") from None
    _drift["t"] = 0.0
    _real_cache["t"] = 0.0
    return r if isinstance(r, dict) else {"ok": True}


def act_config_set(body):
    # {updates: {KEY: "value" | null (= back to the default)}}
    updates = body.get("updates")
    if not isinstance(updates, dict) or not updates:
        raise ValueError("updates: {KEY: value or null}")
    return config_change(fleetcfg.set_settings, {k: (None if v is None else str(v)) for k, v in updates.items()})


def act_config_slots(body):
    # {slots: [{model, x, y, yaw} | null, ...] for slots 0.., num_robots: N}
    slots = body.get("slots")
    if not isinstance(slots, list):
        raise ValueError("slots: a list")
    return config_change(fleetcfg.set_slots, slots, num_robots=body.get("num_robots"))


def act_config_profile(body):
    op, name = body.get("op"), (body.get("name") or "").strip()
    ops = {
        "use": lambda: config_change(fleetcfg.profile_use, name),
        "new": lambda: config_change(fleetcfg.profile_new, name, body.get("from") or None, body.get("notes") or ""),
        "rename": lambda: config_change(fleetcfg.profile_rename, name, (body.get("new_name") or "").strip()),
        "delete": lambda: config_change(fleetcfg.profile_delete, name),
        "notes": lambda: config_change(fleetcfg.profile_notes, name, body.get("notes") or ""),
        "import": lambda: config_change(fleetcfg.profile_import, body.get("data") or {}, name or None),
    }
    if op not in ops:
        raise ValueError(f"op must be one of {sorted(ops)}")
    if not name and op != "import":
        raise ValueError("name: which profile")
    return ops[op]()


def act_config_revert(body):
    return config_change(fleetcfg.revert, int(body.get("id")))


def act_config_apply(body):
    """Recreate what runs with old settings: the sim (fleet.sh scene: compose recreates it when its settings changed;
    its robots respawn by themselves), the robots (fleet.sh spawn: the slots' models and poses), the base station."""
    what = body.get("what")
    commands = {"sim": ["scripts/fleet.sh", "scene"], "robots": ["scripts/fleet.sh", "spawn"],
                "basestation": BASESTATION_COMPOSE + ["up", "-d"]}
    if what not in commands:
        raise ValueError(f"what must be one of {sorted(commands)}")
    if what == "robots":
        state = fleet_ctl.read_state()
        if not state or state.get("scene") != "ready":
            raise ValueError("the scene is not ready: start the sim first")
    _drift["t"] = 0.0
    return start_job(f"apply settings: {what}", lambda j: j.run(commands[what]) == 0)


POST = {
    "/api/sim/start": act_sim_start,
    "/api/sim/stop": act_sim_stop,
    "/api/sim/reset": act_sim_reset,
    "/api/scene/upload": act_scene_upload,
    "/api/spawn": act_spawn,
    "/api/launch/start": act_launch_start,
    "/api/launch/stop": act_launch_stop,
    "/api/real/restart": act_real_restart,
    "/api/real/save": act_real_save,
    "/api/real/remove": act_real_remove,
    "/api/deploy": act_deploy,
    "/api/basestation/link": act_basestation_link,
    "/api/basestation/action": act_basestation,
    "/api/arm/goto": act_arm_goto,
    "/api/cutstem/start": act_cutstem_start,
    "/api/cutstem/stop": act_cutstem_stop,
    "/api/stop": act_stop,
    "/api/stop_all": act_stop_all,
    "/api/rviz/start": act_rviz_start,
    "/api/mocap/assign": act_mocap_assign,
    "/api/localization/set": act_localization_set,
    "/api/config/set": act_config_set,
    "/api/config/slots": act_config_slots,
    "/api/config/profile": act_config_profile,
    "/api/config/revert": act_config_revert,
    "/api/config/apply": act_config_apply,
}
GET = {
    "/api/status": status,
    "/api/arm/states": get_arm_states,
    "/api/joints": get_joints,
    "/api/launch/log": get_launch_log,
    "/api/mocap": get_mocap,
    "/api/localization": get_localization,
    "/api/link": get_link,
    "/api/basestation": get_basestation,
    "/api/basestation/log": get_basestation_log,
    "/api/jobs": lambda q: [j.to_json() for j in sorted(JOBS.values(), key=lambda j: -j.started)],
    "/api/job": lambda q: JOBS[q["id"]].to_json(full=True),
    "/api/config": get_config,
    "/api/config/history": get_config_history,
    "/api/config/runs": get_config_runs,
    "/api/config/export": get_config_export,
}


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, payload, ctype="application/json"):
        data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass  # the page went away mid-request (reload/close during a ~1.5 s status poll); nothing to answer

    def handle_api(self, table, arg):
        path = urlparse(self.path).path
        if path not in table:
            return self.reply(404, {"error": "not found"})
        try:
            out = table[path](arg)
            self.reply(200, out.to_json() if isinstance(out, Job) else out)
        except NeedPassword as e:
            self.reply(400, {"error": str(e), "need_password": True})
        except (ValueError, KeyError) as e:
            self.reply(400, {"error": str(e)})
        except fleetcfg.Unavailable as e:
            self.reply(503, {"error": str(e)})
        except subprocess.TimeoutExpired:
            self.reply(504, {"error": "command timed out"})

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self.reply(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/config", "/config.html"):
            return self.reply(200, CONFIG_PAGE.read_bytes(), "text/html; charset=utf-8")
        self.handle_api(GET, {k: v[0] for k, v in parse_qs(url.query).items()})

    def do_POST(self):
        # A cross-site page can only send a JSON content type after a CORS preflight, which this server never
        # answers -- so requiring it keeps other websites from driving docker through this port.
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return self.reply(415, {"error": "expected application/json"})
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self.reply(400, {"error": "bad json"})
        # A password (a robot's sudo) only from this machine: with --host 0.0.0.0 it would cross the LAN as plain HTTP.
        if isinstance(body, dict) and "password" in body and self.client_address[0] not in ("127.0.0.1", "::1"):
            return self.reply(403, {"error": "passwords are only accepted from this machine (127.0.0.1)"})
        self.handle_api(POST, body)

    def log_message(self, *args):
        pass


def main():
    global DEFAULT_MODE
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--mode", choices=MODES, default=DEFAULT_MODE,
                    help="what the page shows until the browser picks its own: sim, real robots, or both")
    args = ap.parse_args()
    DEFAULT_MODE = args.mode
    print(f"fleet UI on http://{args.host}:{args.port} (fleet: {ROOT}, mode {args.mode})")
    try:
        ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        LINK.stop_all()  # its pings and the base station's link_monitor.py would outlive the server


if __name__ == "__main__":
    main()
