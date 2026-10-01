#!/usr/bin/env python3
"""Host side of the sim's spawn protocol (sim/scripts/setup_scene.py: FLEET_REQUEST / FLEET_STATE).

The sim builds the scene with no robots and then watches sim/generated/fleet/spawn_request.json; each request with
a new id replaces the robots in the scene. It reports progress in sim/generated/fleet/state.json.

  scripts/fleet_ctl.py wait-scene            wait until the running sim's scene is ready (and a pending spawn done)
  scripts/fleet_ctl.py spawn [--poses JSON]  spawn NUM_ROBOTS robots (models ROBOT_MODEL_<i> from .env), wait
  scripts/fleet_ctl.py clear                 delete the request (the next sim start has no robots)
  scripts/fleet_ctl.py state                 print the sim's state (null if the sim isn't running / stale)

Poses: --poses '[{"x":0,"y":-1.6,"yaw":0}, ...]' (one per slot, null or missing keys = default), else
ROBOT_POSE_<i>="x,y,yaw" in .env, else the sim's default layout (state.json's default_poses). x/y in metres in the
world frame, yaw in degrees. Normally run through scripts/fleet.sh (which also starts the robot containers).
Stdlib only; tools/sim_ui/server.py imports it.
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = ROOT / "sim/generated/fleet"
REQUEST = FLEET_DIR / "spawn_request.json"
STATE = FLEET_DIR / "state.json"
APPLIED = FLEET_DIR / "applied_request.json"  # written by the sim: the last request it spawned successfully
SIM = "a300-isaac-sim"
MAX_SLOTS = 8


def read_env():
    env = {}
    path = ROOT / ".env"
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip()
    return env


def sim_started_at():
    """Start time (epoch s) of the running sim container, or None if it isn't running."""
    p = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}} {{.State.StartedAt}}", SIM],
                       capture_output=True, text=True)
    if p.returncode != 0 or not p.stdout.startswith("true"):
        return None
    ts = p.stdout.split()[1]  # 2026-10-01T15:04:05.123456789Z
    base, _, frac = ts.rstrip("Z").partition(".")
    t = datetime.datetime.fromisoformat(base).replace(tzinfo=datetime.timezone.utc).timestamp()
    return t + float(f"0.{frac or 0}")


def read_state():
    """The sim's state.json if it belongs to the sim container running now, else None."""
    started = sim_started_at()
    if started is None:
        return None
    try:
        state = json.loads(STATE.read_text())
    except (OSError, ValueError):
        return None
    # written by this container's run of setup_scene.py (not a previous one)? Same host clock, 5 s of slack.
    return state if state.get("started", 0) >= started - 5 else None


def read_request(applied=False):
    """The last spawn request written (applied=True: the last one the sim spawned successfully)."""
    try:
        return json.loads((APPLIED if applied else REQUEST).read_text())
    except (OSError, ValueError):
        return None


def write_request(robots):
    """Write a spawn request ({model, x?, y?, yaw?} per slot); returns its id."""
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(FLEET_DIR, 0o777)  # the sim (uid 1234) writes state.json here
    except OSError:
        pass
    req = {"id": uuid.uuid4().hex[:12], "written": time.time(), "robots": robots}
    tmp = FLEET_DIR / f".spawn_request.{os.getpid()}.tmp"
    tmp.write_text(json.dumps(req, indent=1))
    os.chmod(tmp, 0o666)
    os.replace(tmp, REQUEST)  # atomic: the sim never reads half a file
    return req["id"]


def clear_request():
    REQUEST.unlink(missing_ok=True)
    APPLIED.unlink(missing_ok=True)


def slot_robots(poses=None, env=None):
    """The requested robots from .env (NUM_ROBOTS, ROBOT_MODEL_<i>, ROBOT_POSE_<i>) and optional per-slot poses."""
    env = env if env is not None else read_env()
    n = int(env.get("NUM_ROBOTS") or 0)
    if not 0 <= n <= MAX_SLOTS:
        raise ValueError(f"NUM_ROBOTS must be 0-{MAX_SLOTS}")
    poses = poses or []
    robots = []
    for i in range(n):
        r = {"model": env.get(f"ROBOT_MODEL_{i}") or "a300"}
        pose = poses[i] if i < len(poses) else None
        if pose is None and env.get(f"ROBOT_POSE_{i}"):
            vals = [v.strip() for v in env[f"ROBOT_POSE_{i}"].split(",")]
            pose = dict(zip(("x", "y", "yaw"), (float(v) for v in vals if v)))
        for k in ("x", "y", "yaw"):
            if pose and pose.get(k) not in (None, ""):
                r[k] = float(pose[k])
        robots.append(r)
    return robots


def wait(pred, timeout, what, log=print):
    """Poll read_state() until pred(state) returns a non-None result (True = ok, a string = error)."""
    t0, last = time.time(), None
    while time.time() - t0 < timeout:
        state = read_state()
        if state is None and sim_started_at() is None:
            log("the sim container is not running")
            return False
        res = pred(state) if state else None
        if res is True:
            return True
        if isinstance(res, str):
            log(res)
            return False
        msg = f"waiting for {what}: " + ("sim starting" if state is None else f"scene {state.get('scene')}")
        if msg != last:
            log(msg)
            last = msg
        time.sleep(1)
    log(f"timed out after {timeout:.0f} s waiting for {what}")
    return False


def scene_ready(state):
    if state["scene"] == "error":
        return "scene failed to build:\n" + state.get("error", "")
    return True if state["scene"] == "ready" else None


def wait_scene(timeout=900, log=print):
    t0 = time.time()
    if not wait(scene_ready, timeout, "the scene", log):
        return False
    state = read_state()
    log(f"scene ready ({state['lanes']} lanes) after {time.time() - t0:.0f} s; models: {', '.join(state['models'])}")
    req = read_request(applied=True) or read_request()
    if req:  # a restarted sim spawns the last applied request again (or a pending one if none was applied)
        return wait_spawn(req["id"], timeout, log)
    return True


def wait_spawn(req_id, timeout=600, log=print):
    def done(state):
        sp = state.get("spawn") or {}
        if sp.get("id") != req_id or sp.get("state") == "spawning":
            return None
        return True if sp["state"] == "done" else f"spawn failed: {sp.get('message')}"

    if not wait(done, timeout, "the robots to spawn", log):
        return False
    state = read_state()
    log(f"spawned: {state['spawn']['message']}")
    for r in state["robots"]:
        log(f"  slot {r['slot']}: {r['ns']:<12} x={r['x']:g} y={r['y']:g} yaw={r['yaw']:g} deg")
    return True


def spawn(robots, timeout=600, log=print):
    if not wait(scene_ready, 900, "the scene", log):
        return False
    req_id = write_request(robots)
    log(f"spawn request {req_id}: " + (", ".join(r["model"] for r in robots) or "no robots"))
    return wait_spawn(req_id, timeout, log)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("wait-scene").add_argument("--timeout", type=float, default=900)
    sp = sub.add_parser("spawn")
    sp.add_argument("--poses", help="JSON list, one {x, y, yaw} (or null) per slot")
    sp.add_argument("--timeout", type=float, default=600)
    sub.add_parser("clear")
    sub.add_parser("state")
    args = ap.parse_args()
    if args.cmd == "wait-scene":
        return 0 if wait_scene(args.timeout) else 1
    if args.cmd == "spawn":
        poses = json.loads(args.poses) if args.poses else None
        try:
            robots = slot_robots(poses)
        except ValueError as e:
            print(e, file=sys.stderr)
            return 2
        return 0 if spawn(robots, args.timeout) else 1
    if args.cmd == "clear":
        clear_request()
        return 0
    print(json.dumps(read_state(), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
