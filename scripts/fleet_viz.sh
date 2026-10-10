#!/bin/bash
# Fleet view: every robot in one Foxglove 3D panel. Runs two processes in the base station container:
#   scripts/fleet_viz.py   the relay: each robot's TF with its frames renamed <ns>/<frame> under the shared
#                          ref_frame, on /fleet/tf(_static), plus a few display topics as /fleet/<ns>/... (its header)
#   foxglove_bridge        ws://<this machine>:FLEET_VIZ_PORT (default 8764), exposing only /fleet/* and the
#                          robots' camera images; read-only (no publishing, services or parameters from the browser)
# Both are ordinary base station sessions (its own zenoh router): run straight against the sim's router, the
# relay's output reached a second client only some of the time (2026-10-10). Start and stop each pause every
# robot's data ~3 s (the two sessions joining / leaving the router): not in the middle of a run.
#   scripts/fleet_viz.sh start [ROBOT...]   # default: every robot in the graph, found as they come and go
#   scripts/fleet_viz.sh stop | status
#   scripts/fleet_viz.sh layout [--cameras] # foxglove/fleet.json: Foxglove > Layouts > Import from file
# The relay is copied in from this checkout at each start (the base station mounts the main checkout's scripts/).
set -euo pipefail
cd "$(dirname "$0")/.."

BS=basestation
RELAY_LOG=/tmp/fleet_viz.log
BRIDGE_LOG=/tmp/fleet_viz_bridge.log
STATUS=/tmp/fleet_viz_status.json
# bash -c command lines hold these patterns too: the bracketed first letter keeps them from matching that shell
RELAY_MATCH='[p]ython3 -u /tmp/fleet_viz.py'
BRIDGE_MATCH='[_]_node:=fleet_viz_bridge'

port() { python3 scripts/fleetcfg.py get FLEET_VIZ_PORT 2>/dev/null || echo 8764; }

running() { docker exec "$BS" pgrep -f "$1" > /dev/null 2>&1; }

need_bs() {
    [ "$(docker inspect -f '{{.State.Running}}' "$BS" 2>/dev/null)" = true ] ||
        { echo "the base station isn't running: docker compose -f basestation.compose.yml up -d" >&2; exit 1; }
}

stop() {
    docker exec "$BS" bash -c "pkill -INT -f '$RELAY_MATCH'; pkill -INT -f '$BRIDGE_MATCH'; sleep 1; \
        pkill -KILL -f '$RELAY_MATCH'; pkill -KILL -f '$BRIDGE_MATCH'; rm -f $STATUS; true"
}

case "${1:-}" in
start)
    shift
    need_bs
    p=$(port)
    stop
    docker cp scripts/fleet_viz.py "$BS":/tmp/fleet_viz.py
    robots=""
    [ $# -gt 0 ] && robots="--robots $*"
    docker exec -d "$BS" bash -c "exec python3 -u /tmp/fleet_viz.py $robots --status $STATUS > $RELAY_LOG 2>&1"
    # Read-only on purpose: the view must never command a robot (no clientPublish, services, parameters).
    # The bridge's own clock follows the base station's USE_SIM_TIME (true with the sim).
    docker exec -d "$BS" bash -c "exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
        -r __node:=fleet_viz_bridge -r __ns:=/fleet -p port:=$p -p address:=0.0.0.0 \
        -p use_sim_time:=\${USE_SIM_TIME:-false} -p 'capabilities:=[assets]' \
        -p \"topic_whitelist:=['^/fleet/.*', '^/[^/]+/sensors/camera_[0-9]+/color/image\$']\" > $BRIDGE_LOG 2>&1"
    sleep 4
    ok=1
    running "$RELAY_MATCH" || { echo "the relay exited:"; docker exec "$BS" tail -n 15 "$RELAY_LOG"; ok=0; }
    running "$BRIDGE_MATCH" || { echo "the bridge exited:"; docker exec "$BS" tail -n 15 "$BRIDGE_LOG"; ok=0; }
    [ "$ok" = 1 ] || { stop; exit 1; }
    echo "fleet view running: Foxglove -> Open connection -> ws://localhost:$p (or this machine's LAN IP)"
    echo "layout: scripts/fleet_viz.sh layout, once the robots show up (scripts/fleet_viz.sh status)"
    ;;
stop)
    need_bs
    stop
    echo "fleet view stopped"
    ;;
status)
    need_bs
    running "$RELAY_MATCH" && echo "relay: running" || echo "relay: stopped"
    running "$BRIDGE_MATCH" && echo "bridge: running on port $(port)" || echo "bridge: stopped"
    docker exec "$BS" cat "$STATUS" 2>/dev/null | python3 -c '
import json, sys
s = json.load(sys.stdin)
for ns, r in s["robots"].items():
    print("  %s: %d frames, %d topics" % (ns, len(r["frames"]), len(r["topics"])))' || true
    ;;
layout)
    shift
    need_bs
    mkdir -p foxglove
    docker exec "$BS" cat "$STATUS" 2>/dev/null > foxglove/.fleet_status.json ||
        { echo "no relay status: scripts/fleet_viz.sh start first" >&2; exit 1; }
    python3 scripts/fleet_viz_layout.py foxglove/.fleet_status.json -o foxglove/fleet.json "$@"
    rm -f foxglove/.fleet_status.json
    echo "foxglove/fleet.json: Foxglove -> Layouts -> Import from file (connected to ws://<host>:$(port))"
    ;;
*)
    sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
