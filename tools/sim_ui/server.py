#!/usr/bin/env python3
"""Local web UI for the simulated fleet: start/stop/reset the sim, spawn robots, start/stop
mtu32_bringup's sim_robot_upstart.launch.py per robot, and move the arm to named SRDF states
(optionally recording commanded vs. observed joint positions while it moves).

  python3 tools/sim_ui/server.py                 # http://127.0.0.1:8090
  python3 tools/sim_ui/server.py --host 0.0.0.0  # reachable from the LAN -- it runs docker commands, so only on a trusted network

Stdlib only. Long operations run as background jobs whose output the page polls.
"""
import argparse
import json
import os
import re
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
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
LAUNCH_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
              f"exec ros2 launch mtu32_bringup sim_robot_upstart.launch.py > {LAUNCH_LOG} 2>&1")


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
            job.status = "done" if ok in (None, True, 0) else "failed"
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
    return sorted(d.name for d in (ROOT / "sim/assets").iterdir() if (d / f"{d.name}.urdf").exists())


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
        if r["state"] == "running":
            code, _ = sh(["docker", "exec", r["name"], "pgrep", "-f", "sim_robot_upstart.launch.py"], timeout=10)
            r["launch"] = code == 0
            r["cutting"] = sh(["docker", "exec", r["name"], "pgrep", "-f", CUT_MATCH], timeout=10)[0] == 0
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
        "num_robots": n,
        "slots": [env.get(f"ROBOT_MODEL_{i}", "a300") for i in range(MAX_SLOTS)],
        "models": available_models(),
        "robots": robots(),
        "max_slots": MAX_SLOTS,
        "sim_mode": env.get("SIM_MODE", "stream"),
    }


# ---------------------------------------------------------------- actions

def wait_sim_healthy(job, timeout=300):
    job.log("waiting for the sim to report healthy ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        _, out = sh(["docker", "inspect", "-f", "{{.State.Health.Status}}", SIM])
        if out.strip() == "healthy":
            job.log(f"sim healthy after {time.time() - t0:.0f}s")
            return True
        time.sleep(3)
    job.log("sim did not become healthy in time")
    return False


SIM_MODES = ("stream", "headed")


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

    def fn(j):
        write_env({"SIM_MODE": mode})
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
        j.log(f".env: SIM_MODE={mode}")
        return j.run(["scripts/fleet.sh"], env=env) == 0 and wait_sim_healthy(j)

    return start_job(f"start sim ({mode})", fn)


def act_sim_stop(_):
    return start_job("stop sim", lambda j: j.run(["scripts/stop_sim.sh"]) == 0)


def act_sim_reset(_):
    # Restarting the sim container re-runs setup_scene.py: fresh scene, every robot back at its spawn pose.
    # Robot containers (and their ROS nodes) keep running.
    return start_job("reset scene", lambda j: j.run(["docker", "restart", SIM]) == 0 and wait_sim_healthy(j))


def act_spawn(body):
    models = body.get("models")
    allowed = set(available_models())
    if not isinstance(models, list) or len(models) > MAX_SLOTS or any(m not in allowed for m in models):
        raise ValueError(f"models must be a list of up to {MAX_SLOTS} of {sorted(allowed)}")
    updates = {"NUM_ROBOTS": str(len(models))}
    updates.update({f"ROBOT_MODEL_{i}": m for i, m in enumerate(models)})

    def fn(j):
        write_env(updates)
        j.log(f".env: {updates}")
        return j.run(["scripts/fleet.sh", str(len(models))]) == 0 and wait_sim_healthy(j)

    return start_job(f"spawn {len(models)} robot(s)", fn)


def act_launch_start(body):
    robot = running_robot(body.get("robot"))

    def fn(j):
        if sh(["docker", "exec", robot, "pgrep", "-f", "sim_robot_upstart.launch.py"])[0] == 0:
            j.log("already running")
            return True
        rc = j.run(["docker", "exec", "-d", robot, "bash", "-c", LAUNCH_CMD])
        j.log(f"started; output in {robot}:{LAUNCH_LOG} (move_group comes up ~20 s later)")
        return rc == 0

    return start_job(f"{robot}: start sim_robot_upstart", fn)


def act_launch_stop(body):
    robot = running_robot(body.get("robot"))
    return start_job(f"{robot}: restart_ros", lambda j: j.run(["docker", "exec", robot, "restart_ros"]) == 0)


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


POST = {
    "/api/sim/start": act_sim_start,
    "/api/sim/stop": act_sim_stop,
    "/api/sim/reset": act_sim_reset,
    "/api/spawn": act_spawn,
    "/api/launch/start": act_launch_start,
    "/api/launch/stop": act_launch_stop,
    "/api/arm/goto": act_arm_goto,
    "/api/cutstem/start": act_cutstem_start,
    "/api/cutstem/stop": act_cutstem_stop,
}
GET = {
    "/api/status": lambda q: status(),
    "/api/arm/states": get_arm_states,
    "/api/joints": get_joints,
    "/api/launch/log": get_launch_log,
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
        self.wfile.write(data)

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
