#!/usr/bin/env python3
"""One look inside the base station container, as JSON on stdout: its zenoh router (pid, uptime, TCP links,
recent ERROR lines), PostgreSQL (ready, version, size, client connections), the ROS programs running there
(RViz windows, the web UI's link monitors, other ros2 launch / run) and the ROS graph it sees (topics per namespace).

The web UI's Base station card runs it every 10 s while the card is shown (tools/sim_ui/server.py, BasestationWatch):
    docker exec basestation bash -c 'python3 /scripts/basestation_probe.py'
bash -c, so BASH_ENV sources ROS for `ros2 topic list`. Stdlib only. The container is on the host network, so the
router's sockets are found through /proc/<pid>/fd in the host's TCP table (the image has no ss).
"""
import collections
import datetime
import json
import os
import re
import socket
import subprocess
import time

ROUTER_LOG = "/tmp/zenoh_router.log"  # basestation/entrypoint.sh
ROUTER_PORT = 7447
ERROR_WINDOW_S = 60
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def run(argv, timeout=8):
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, str(e)


def cmdlines():
    """{pid: argv} of every process in the container (its own pid namespace)."""
    out = {}
    for pid in os.listdir("/proc"):
        if pid.isdigit():
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    argv = f.read().decode(errors="replace").split("\0")[:-1]
            except OSError:
                continue
            if argv:
                out[int(pid)] = argv
    return out


def addr(hex_addr):
    """/proc/net/tcp{,6} "0100007F:1D27" -> ("127.0.0.1", 7463); v4-mapped v6 addresses as plain v4."""
    ip_hex, port_hex = hex_addr.split(":")
    raw = bytes.fromhex(ip_hex)
    if len(raw) == 4:
        ip = socket.inet_ntop(socket.AF_INET, raw[::-1])
    else:  # four 32-bit words, each little-endian
        raw = b"".join(raw[i:i + 4][::-1] for i in range(0, 16, 4))
        ip = socket.inet_ntop(socket.AF_INET6, raw)
        if ip.startswith("::ffff:") and "." in ip:
            ip = ip[7:]
    return ip, int(port_hex, 16)


def router(procs):
    pid = next((p for p, argv in procs.items() if os.path.basename(argv[0]) == "rmw_zenohd"), None)
    r = {"running": pid is not None, "pid": pid, "links": []}
    if pid is None:
        return r
    try:
        with open(f"/proc/{pid}/stat") as f:
            start_ticks = int(f.read().rsplit(")", 1)[1].split()[19])
        with open("/proc/uptime") as f:
            r["uptime_s"] = round(float(f.read().split()[0]) - start_ticks / os.sysconf("SC_CLK_TCK"))
        inodes = set()
        for fd in os.listdir(f"/proc/{pid}/fd"):
            try:
                m = re.fullmatch(r"socket:\[(\d+)\]", os.readlink(f"/proc/{pid}/fd/{fd}"))
            except OSError:
                continue
            if m:
                inodes.add(m.group(1))
        for table in ("tcp", "tcp6"):
            try:
                with open(f"/proc/{pid}/net/{table}") as f:
                    rows = f.read().splitlines()[1:]
            except OSError:
                continue
            for row in rows:
                cols = row.split()
                if cols[3] != "01" or cols[9] not in inodes:  # 01: ESTABLISHED
                    continue
                (lip, lport), (rip, rport) = addr(cols[1]), addr(cols[2])
                # in: a session or another router dialled this one; out: this router dials BASESTATION_ZENOH_CONNECT
                r["links"].append({"dir": "in" if lport == ROUTER_PORT else "out", "local": f"{lip}:{lport}",
                                   "remote": f"{rip}:{rport}"})
    except OSError as e:
        r["error"] = str(e)
    return r


def router_log():
    """ERROR lines of the router's log in the last minute, and the last one."""
    out = {"errors": 0, "last_error": None, "last_error_age_s": None}
    try:
        with open(ROUTER_LOG, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 256 * 1024))
            lines = f.read().decode(errors="replace").splitlines()
    except OSError:
        return out
    now = time.time()
    for line in lines:
        line = ANSI.sub("", line)
        if " ERROR " not in line:
            continue
        try:
            stamp = datetime.datetime.fromisoformat(line.split()[0].replace("Z", "+00:00")).timestamp()
        except (ValueError, IndexError):
            continue
        out["last_error"], out["last_error_age_s"] = line[:400], round(now - stamp)
        if now - stamp <= ERROR_WINDOW_S:
            out["errors"] += 1
    return out


def postgres():
    code, _ = run(["pg_isready", "-q"], timeout=5)
    db = {"ready": code == 0}
    if code != 0:
        return db
    # this probe's own connection is one of the client backends
    code, out = run(["psql", "-AtX", "-F", "\t", "-c",
                     "select current_database(), current_setting('server_version'), current_setting('port'), "
                     "pg_database_size(current_database()), "
                     "(select count(*) - 1 from pg_stat_activity where backend_type = 'client backend')"], timeout=5)
    if code == 0 and out:
        name, version, port, size, clients = out.split("\t")
        db.update(database=name, version=version.split()[0], port=int(port), size_bytes=int(size),
                  clients=int(clients))
    else:
        db["error"] = out[-300:]
    return db


def programs(procs):
    rviz, monitors, other = [], [], []
    for pid, argv in sorted(procs.items()):
        line = " ".join(argv)
        m = re.search(r"view_(\w+)\.launch\.py namespace:=(\w+)", line)
        if m:
            rviz.append({"robot": m.group(2), "view": m.group(1)})
            continue
        m = re.search(r"/scripts/link_monitor\.py (\w+)$", line)
        if m:
            monitors.append(m.group(1))
            continue
        # a ros2 launch / run started here (not its node processes, not the router)
        i = next((i for i, a in enumerate(argv) if os.path.basename(a) == "ros2"), None)
        if i is not None and argv[i + 1:i + 2] and argv[i + 1] in ("launch", "run") and "rmw_zenohd" not in line:
            other.append({"pid": pid, "cmd": " ".join(argv[i:])[:200]})
    return {"rviz": rviz, "link_monitors": sorted(set(monitors)), "other": other[:20]}


def graph():
    code, out = run(["ros2", "topic", "list"], timeout=8)
    if code != 0:
        return {"error": out[-300:] or f"ros2 topic list: exit {code}"}
    namespaces = collections.Counter()
    for topic in out.splitlines():
        parts = topic.strip("/").split("/")
        namespaces[parts[0] if len(parts) > 1 else "/"] += 1
    return {"topics": sum(namespaces.values()), "namespaces": dict(sorted(namespaces.items()))}


def main():
    procs = cmdlines()
    print(json.dumps({
        "time": time.time(),
        "env": {k: os.environ.get(k) for k in ("RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "USE_SIM_TIME",
                                                "BASESTATION_ZENOH_CONNECT")},
        "router": dict(router(procs), **router_log()),
        "postgres": postgres(),
        "programs": programs(procs),
        "graph": graph(),
    }))


if __name__ == "__main__":
    main()
