#!/usr/bin/env python3
"""Local web UI for the simulated fleet: start/stop/reset the sim (the scene only), spawn robots at chosen poses
into the running scene (and start their containers), start/stop
mtu32_bringup's sim_robot_upstart.launch.py per robot, and move the arm to named SRDF states
(optionally recording commanded vs. observed joint positions while it moves), list OptiTrack Motive's rigid bodies
and assign them to robots, and set each robot's ref_localizer (map -> odom source / anchor).

  python3 tools/sim_ui/server.py                 # http://127.0.0.1:8090
  python3 tools/sim_ui/server.py --host 0.0.0.0  # reachable from the LAN -- it runs docker commands, so only on a trusted network

Stdlib only. Long operations run as background jobs whose output the page polls.
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
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import fleet_ctl  # noqa: E402  (the sim's spawn protocol: request/state files)
sys.path.insert(0, str(ROOT / "colcon_ws/src/mocap_fake_localizer/scripts"))
import natnet  # noqa: E402  (OptiTrack Motive's NatNet protocol, shared with the robots' natnet_ref_pose.py)

PAGE = Path(__file__).with_name("index.html")
PROJECT = "clearpath-fleet"
SIM = "a300-isaac-sim"
MAX_SLOTS = 8
NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
# The cut_stem client (`ros2 action send_goal ... /<ns>/cut_stem`) is exec'd directly (no wrapper shell), so this
# matches exactly one process per run; SIGINT to it makes ros2cli cancel the goal.
CUT_MATCH = "ros2 action send_goal.*cut_stem"
CUT_CMD = ('exec ros2 action send_goal --feedback /$ROBOT_NAMESPACE/cut_stem '
           'plant_cutter_msgs/action/CutStem "{start_cutting: true}"')
LAUNCH_LOG = "/tmp/sim_robot_upstart.log"
SRDF = "/etc/clearpath/robot.srdf"  # written at container boot; sim_robot_upstart needs it
SRDF_WAIT_S = 120
LAUNCH_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
              f"exec ros2 launch mtu32_bringup sim_robot_upstart.launch.py > {LAUNCH_LOG} 2>&1")
# RViz from clearpath_viz (colcon_ws/src/clearpath_desktop), opened on the host display through the container's X11
# mount; one window per view and robot. The launch exits when its rviz2 window is closed.
RVIZ_VIEWS = ("navigation", "moveit", "robot")
RVIZ_MATCH = r"clearpath_viz view_(navigation|moveit|robot)\.launch\.py"
RVIZ_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
            "exec ros2 launch clearpath_viz view_{view}.launch.py namespace:=$ROBOT_NAMESPACE "
            "use_sim_time:=${{USE_SIM_TIME:-false}} > /tmp/rviz_{view}.log 2>&1")


def sh(argv, timeout=30):
    p = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout + p.stderr


def in_robot(robot, command, timeout=30):
    # bash -c so BASH_ENV sources the robot's ROS environment and ROBOT_NAMESPACE.
    return sh(["docker", "exec", robot, "bash", "-c", command], timeout=timeout)


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
        self.log("$ " + " ".join(argv))
        p = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kw)
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


# ---------------------------------------------------------------- state

def read_env():
    env = {}
    for line in (ROOT / ".env").read_text().splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            k, v = line.split("=", 1)
            env[k.strip()] = v.strip()
    return env


def write_env(updates):
    path = ROOT / ".env"
    lines = path.read_text().splitlines()
    done = set()
    for i, line in enumerate(lines):
        key = line.split("=", 1)[0].strip()
        if key in updates:
            lines[i] = f"{key}={updates[key]}"
            done.add(key)
    lines += [f"{k}={v}" for k, v in updates.items() if k not in done]
    path.write_text("\n".join(lines) + "\n")


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


def robots():
    rows = [c for c in containers() if re.fullmatch(r"robot\d+", c["service"])]
    for r in rows:
        r["slot"] = int(r["service"][5:])
        r["launch"] = False
        r["move_group"] = False
        r["cut_stem_action"] = False
        r["rviz"] = []
        if r["state"] == "running":
            code, _ = sh(["docker", "exec", r["name"], "pgrep", "-f", "sim_robot_upstart.launch.py"], timeout=10)
            r["launch"] = code == 0
            r["cutting"] = sh(["docker", "exec", r["name"], "pgrep", "-f", CUT_MATCH], timeout=10)[0] == 0
            _, out = sh(["docker", "exec", r["name"], "pgrep", "-af", RVIZ_MATCH], timeout=10)
            r["rviz"] = sorted(set(re.findall(r"view_(navigation|moveit|robot)\.launch\.py", out)))
            # Check move_group availability (only check if launch is running)
            if r["launch"]:
                code, _ = in_robot(r["name"], "ros2 node list 2>/dev/null | grep -q move_group", timeout=5)
                r["move_group"] = code == 0
                # Check cut_stem action server availability
                code, _ = in_robot(r["name"], "ros2 action list 2>/dev/null | grep -xq /$ROBOT_NAMESPACE/cut_stem", timeout=5)
                r["cut_stem_action"] = code == 0
        else:
            r["cutting"] = False
    return sorted(rows, key=lambda r: r["slot"])


def running_robot(name):
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ValueError("bad robot name")
    if name not in {r["name"] for r in containers() if r["state"] == "running" and r["service"].startswith("robot")}:
        raise ValueError(f"robot {name} is not running")
    return name


def status():
    env = read_env()
    sim = next((c for c in containers() if c["service"] == "isaac-sim"), None)
    n = int(env.get("NUM_ROBOTS", "0") or 0)
    return {
        "sim": sim,
        # the running sim's own report (scene loading/ready, robots in it, last spawn); None if not running
        "scene": fleet_ctl.read_state() if sim and sim["state"] == "running" else None,
        # last spawn request that worked, else the last one written (prefills the spawn form)
        "request": fleet_ctl.read_request(applied=True) or fleet_ctl.read_request(),
        "num_robots": n,
        "slots": [env.get(f"ROBOT_MODEL_{i}", "a300") for i in range(MAX_SLOTS)],
        "models": available_models(),
        "robots": robots(),
        "max_slots": MAX_SLOTS,
        "sim_mode": env.get("SIM_MODE", "stream"),
        "robot_looks": env.get("ROBOT_LOOKS", "full"),
        "sim_scene": env.get("SIM_SCENE", ""),
        "sim_scene_label": scene_label(env.get("SIM_SCENE", "")),
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
    # Written to .env as SIM_MODE; fleet.sh's `docker compose up -d` recreates the sim when it changes.
    mode = (body or {}).get("mode") or read_env().get("SIM_MODE", "stream")
    if mode not in SIM_MODES:
        raise ValueError(f"mode must be one of {SIM_MODES}")
    # looks: robot materials (ROBOT_LOOKS), "full" (textured), "basic" (plain colours) or "off" (importer's own);
    # written to .env like SIM_MODE, so a change recreates the sim too
    looks = (body or {}).get("looks") or read_env().get("ROBOT_LOOKS", "full")
    if looks not in ROBOT_LOOKS:
        raise ValueError(f"looks must be one of {ROBOT_LOOKS}")
    # scene: SIM_SCENE, "" (ground plane + lights), "lavender" or a file in sim/scene/; written to .env like
    # SIM_MODE, so a change recreates the sim
    scene = (body or {}).get("scene")
    scene = read_env().get("SIM_SCENE", "") if scene is None else scene
    if scene not in BUILTIN_SCENES and scene not in scene_files():
        raise ValueError(f"no scene {scene!r}: pick a USD file again")

    def fn(j):
        write_env({"SIM_MODE": mode, "ROBOT_LOOKS": looks, "SIM_SCENE": scene})
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
        j.log(f".env: SIM_MODE={mode} ROBOT_LOOKS={looks} SIM_SCENE={scene}")
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
    updates = {"NUM_ROBOTS": str(len(models))}
    updates.update({f"ROBOT_MODEL_{i}": m for i, m in enumerate(models)})

    def fn(j):
        write_env(updates)
        j.log(f".env: {updates}")
        # the sim replaces its robots (the scene keeps running), then the robot containers are (re)created
        return j.run(["scripts/fleet.sh", "spawn", "--poses", json.dumps(poses)]) == 0

    return start_job(f"spawn {len(models)} robot(s)", fn)


def act_launch_start(body):
    robot = running_robot(body.get("robot"))

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
            j.log(in_robot(robot, f"tail -n 15 {LAUNCH_LOG}")[1].rstrip())
            # nodes it started before failing (e.g. moveit_sim_bridge) outlive it and would double up with the
            # next launch's; restart_ros (= the Stop button) removes them
            j.run(["docker", "exec", robot, "restart_ros"])
            return False
        j.log(f"started; output in {robot}:{LAUNCH_LOG} (move_group comes up ~20 s later)")
        return True

    return start_job(f"{robot}: start sim_robot_upstart", fn)


def act_launch_stop(body):
    robot = running_robot(body.get("robot"))
    return start_job(f"{robot}: restart_ros", lambda j: j.run(["docker", "exec", robot, "restart_ros"]) == 0)


def act_rviz_start(body):
    robot = running_robot(body.get("robot"))
    view = body.get("view")
    if view not in RVIZ_VIEWS:
        raise ValueError(f"view must be one of {RVIZ_VIEWS}")

    def fn(j):
        if sh(["docker", "exec", robot, "pgrep", "-f", f"clearpath_viz view_{view}.launch.py"])[0] == 0:
            j.log(f"view_{view} is already open")
            return True
        if in_robot(robot, "source /home/robot/colcon_ws/install/setup.bash && ros2 pkg prefix clearpath_viz")[0] != 0:
            j.log("clearpath_viz is not built: scripts/colcon_build.sh --packages-select clearpath_viz")
            return False
        env = dict(os.environ)
        display = host_display()
        if display:
            env["DISPLAY"] = display
            j.run(["scripts/x11_auth.sh"], env=env)  # /tmp/.docker.xauth, mounted into the robot containers
        if j.run(["docker", "exec", "-d", robot, "bash", "-c", RVIZ_CMD.format(view=view)]) != 0:
            return False
        time.sleep(3)  # no display / bad package: the launch exits within a second or two
        if sh(["docker", "exec", robot, "pgrep", "-f", f"clearpath_viz view_{view}.launch.py"])[0] != 0:
            j.log("the launch exited right away; end of its log:")
            j.log(in_robot(robot, f"tail -n 15 /tmp/rviz_{view}.log")[1].rstrip())
            return False
        j.log(f"started; output in {robot}:/tmp/rviz_{view}.log")
        return True

    return start_job(f"{robot}: rviz {view}", fn)


def act_cutstem_start(body):
    robot = running_robot(body.get("robot"))

    def fn(j):
        if sh(["docker", "exec", robot, "pgrep", "-f", CUT_MATCH])[0] == 0:
            j.log("a cut_stem goal is already running")
            return False
        code, out = in_robot(robot, "ros2 action list 2>/dev/null | grep -x /$ROBOT_NAMESPACE/cut_stem", timeout=20)
        if code != 0:
            j.log("no cut_stem action server -- start sim_robot_upstart first and wait ~20 s for it to come up")
            return False
        return j.run(["docker", "exec", robot, "bash", "-c", CUT_CMD]) == 0

    return start_job(f"{robot}: cut_stem", fn)


def act_cutstem_stop(body):
    robot = running_robot(body.get("robot"))

    def fn(j):
        # SIGINT, not SIGKILL: ros2 action send_goal cancels the goal on Ctrl+C, so the server stops the task.
        code, _ = sh(["docker", "exec", robot, "pkill", "-INT", "-f", CUT_MATCH])
        j.log("sent Ctrl+C to the cut_stem client (goal is cancelled)" if code == 0 else "no cut_stem goal running")
        return True

    return start_job(f"{robot}: stop cut_stem", fn)


def act_arm_goto(body):
    robot = running_robot(body.get("robot"))
    state, group = body.get("state"), body.get("group", "arm_0")
    if not (isinstance(state, str) and NAME_RE.match(state) and isinstance(group, str) and NAME_RE.match(group)):
        raise ValueError("bad state/group")
    vel = min(max(float(body.get("velocity_scale", 0.3)), 0.01), 1.0)
    cmd = f"arm_goto {state} --group {group} --velocity-scale {vel}"
    if body.get("direct"):
        cmd += " --direct"
    record = float(body.get("record", 0) or 0)

    def fn(j):
        if record <= 0:
            return j.run(["docker", "exec", robot, "bash", "-c", cmd]) == 0
        samples = []
        rec = subprocess.Popen(["docker", "exec", robot, "bash", "-c", f"arm_joints --record {min(record, 120)}"],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        reader = threading.Thread(target=lambda: samples.extend(json.loads(l) for l in rec.stdout if l.startswith("{")))
        reader.start()
        time.sleep(1.5)  # let the recorder's own rclpy startup finish before the arm starts moving
        rc = j.run(["docker", "exec", robot, "bash", "-c", cmd])
        j.log(f"recording joints for {record:.0f}s total ...")
        rec.wait()
        reader.join()
        j.result = {"samples": samples}
        j.log(f"recorded {len(samples)} samples")
        return rc == 0

    return start_job(f"{robot}: {group} -> {state}" + (" (direct)" if body.get("direct") else ""), fn)


def get_arm_states(q):
    robot = running_robot(q.get("robot"))
    code, out = in_robot(robot, "arm_goto --list")
    if code != 0:
        raise ValueError(out.strip() or "arm_goto --list failed")
    return json.loads(out.strip().splitlines()[-1])


def get_joints(q):
    robot = running_robot(q.get("robot"))
    code, out = in_robot(robot, "arm_joints")
    if code != 0:
        raise ValueError(out.strip())
    return json.loads(out.strip().splitlines()[-1])


def get_launch_log(q):
    robot = running_robot(q.get("robot"))
    _, out = sh(["docker", "exec", robot, "tail", "-n", "300", LAUNCH_LOG])
    return {"log": out}


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
                            self.frames.append(time.time())
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
    snap = MOCAP.poll(server)
    snap["assignments"] = read_assignments()
    # robots to offer: MTU's real robots (ids with "_") and the running sim robots
    snap["robots"] = sorted({m for m in available_models() if "_" in m} |
                            {r["name"] for r in containers() if r["service"].startswith("robot")})
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


LOC_SOURCES = ("auto", "ref", "gps", "external")
LOC_ANCHORS = ("fixed", "start", "external")
LOC_NODE = "/$ROBOT_NAMESPACE/ref_localizer"


def get_localization(q):
    robot = running_robot(q.get("robot"))
    code, out = in_robot(robot, f"timeout 6 ros2 topic echo --once --no-daemon --full-length {LOC_NODE}/status "
                                "std_msgs/msg/String", timeout=15)
    m = re.search(r"^data: '(.*)'$", out, re.M)
    if code != 0 or not m:
        return {"running": False}  # no ref_localizer (sim_robot_upstart not started)
    return {"running": True, **json.loads(m.group(1).replace("''", "'"))}


def act_localization_set(body):
    robot = running_robot(body.get("robot"))
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
    return start_job(f"{robot}: localization {what}",
                     lambda j: all(j.run(["docker", "exec", robot, "bash", "-c", c]) == 0 for c in cmds))


POST = {
    "/api/sim/start": act_sim_start,
    "/api/sim/stop": act_sim_stop,
    "/api/sim/reset": act_sim_reset,
    "/api/scene/upload": act_scene_upload,
    "/api/spawn": act_spawn,
    "/api/launch/start": act_launch_start,
    "/api/launch/stop": act_launch_stop,
    "/api/arm/goto": act_arm_goto,
    "/api/cutstem/start": act_cutstem_start,
    "/api/rviz/start": act_rviz_start,
    "/api/cutstem/stop": act_cutstem_stop,
    "/api/mocap/assign": act_mocap_assign,
    "/api/localization/set": act_localization_set,
}
GET = {
    "/api/status": lambda q: status(),
    "/api/arm/states": get_arm_states,
    "/api/joints": get_joints,
    "/api/launch/log": get_launch_log,
    "/api/mocap": get_mocap,
    "/api/localization": get_localization,
    "/api/jobs": lambda q: [j.to_json() for j in sorted(JOBS.values(), key=lambda j: -j.started)],
    "/api/job": lambda q: JOBS[q["id"]].to_json(full=True),
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
        except (ValueError, KeyError) as e:
            self.reply(400, {"error": str(e)})
        except subprocess.TimeoutExpired:
            self.reply(504, {"error": "command timed out"})

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self.reply(200, PAGE.read_bytes(), "text/html; charset=utf-8")
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
        self.handle_api(POST, body)

    def log_message(self, *args):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()
    print(f"sim UI on http://{args.host}:{args.port}")
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
