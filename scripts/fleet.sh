#!/bin/bash
# Start the sim, spawn robots into it, or stop everything. The settings (NUM_ROBOTS, each slot's model: a300/a200/
# j100/r100 or a real robot id like j100_0921, SIM_MODE, ...) are the active profile in this checkout's
# fleet_config.sqlite: this script first writes .env from it (scripts/fleetcfg.py render; if the file can't be
# opened, .env is used as it was last written), and every start and spawn is recorded there (fleetcfg.py runs).
#   scripts/fleet.sh scene                  start the sim with the scene only (no robots); waits until it's ready
#   scripts/fleet.sh spawn [N] [--poses J]  spawn N robots (NUM_ROBOTS if omitted) into the running scene,
#                                           replacing the robots it has, then (re)start their robot containers
#   scripts/fleet.sh [N]                    both: scene, then spawn N
#   scripts/fleet.sh down                   stop and remove everything
# Spawn poses: --poses '[{"x":0,"y":0,"yaw":90}, ...]' (one per slot, metres/degrees, world frame), else the slot's
# pose in the database (ROBOT_POSE_<i> in .env), else the sim's default layout -- see scripts/fleet_ctl.py.
# The sim only builds the scene at start (sim/scripts/setup_scene.py); robots arrive through a spawn request, so
# changing the robots never restarts the sim. A sim that is (re)started while a request exists (e.g. `docker restart
# a300-isaac-sim`, the web UI's "Reset scene") spawns those robots again; `scene` on a stopped sim clears it.
# The robot containers are started with --no-deps so `docker compose` never recreates the sim for them.
# The generated .env also has each slot's ROBOT_SUFFIX_<i>/ROBOT_HOSTNAME_<i>, so a real robot's container name is
# its own id with no slot suffix (j100_0921, not j100_0921_0000) and every container's OS hostname is
# cpr-<model>-<serial> (e.g. cpr-a300-0000, cpr-j100-0921) -- use this script (not `docker compose up -d` directly)
# after changing a slot's model for that to take effect.
set -euo pipefail
cd "$(dirname "$0")/.."
MAX=8
SIM=a300-isaac-sim

usage() { echo "usage: $0 [N] | scene | spawn [N] [--poses JSON] | down   (N = 0-$MAX)" >&2; exit 1; }

# NUM_ROBOTS in the database (refused if it can't be opened, unless it already is $1); writes .env again
set_num_robots() {
    [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -le "$MAX" ] || usage
    python3 scripts/fleetcfg.py robots "$1"
}

# Record this run in the database (fleetcfg.py runs): the variables it starts with, and its exit code at the end.
RUN_ID=""
record_run() {
    RUN_ID=$(python3 scripts/fleetcfg.py run-begin "fleet.sh $*" 2>/dev/null || true)
    if [ -n "$RUN_ID" ]; then
        trap 'python3 scripts/fleetcfg.py run-end "$RUN_ID" $? 2>/dev/null || true' EXIT
    fi
}

# Remove robot slots >= $1 by their stable compose service key (robotN), not by guessing a container name -- a
# slot's container name depends on its assigned model, which may have changed since it was last brought up.
remove_slots_from() {
    for ((i = $1; i < MAX; i++)); do
        docker compose rm -sf "robot$i" >/dev/null 2>&1 || true
    done
}

start_scene() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$SIM" 2>/dev/null)" != "true" ]; then
        # fresh start: an empty scene, so no spawn request and no robot containers left over from before
        python3 scripts/fleet_ctl.py clear
        remove_slots_from 0
    fi
    docker compose up -d isaac-sim
    python3 scripts/fleet_ctl.py wait-scene
}

spawn() {
    local n
    n=$(sed -n 's/^NUM_ROBOTS=//p' .env | tail -1)
    n="${n:-0}"
    python3 scripts/fleet_ctl.py spawn "$@"
    remove_slots_from "$n"
    local services=()
    for ((i = 0; i < n; i++)); do services+=("robot$i"); done
    if [ "$n" -gt 0 ]; then
        # recreated even if unchanged: their odometry/EKF/arm state belonged to the robots that were just replaced
        docker compose up -d --no-deps --force-recreate "${services[@]}"
    fi
    # `docker compose ps` with several service names sorts its output alphabetically, not by argument/slot order, so
    # each slot's real name is queried on its own to keep it correctly paired with that slot's fixed Foxglove port.
    local names=()
    for ((i = 0; i < n; i++)); do names+=("$(docker compose ps --format '{{.Name}}' "robot$i" 2>/dev/null)"); done
    echo "started $n robot container(s): ${names[*]}"
    for ((i = 0; i < n; i++)); do printf '  %-14s Foxglove ws://<host>:%d\n' "${names[$i]}" $((8765 + i)); done
}

# .env from the database's active profile (stops here if the profile is invalid, e.g. a real robot in two slots)
[ "${1:-}" = down ] || python3 scripts/fleetcfg.py render --quiet

case "${1:-}" in
    down)
        python3 scripts/fleet_ctl.py clear
        exec docker compose --profile '*' down
        ;;
    scene)
        record_run "$@"
        start_scene
        ;;
    spawn)
        shift
        if [[ "${1:-}" =~ ^[0-9]+$ ]]; then set_num_robots "$1"; shift; fi
        record_run spawn "$@"
        spawn "$@"
        ;;
    "")
        record_run
        start_scene
        spawn
        ;;
    *)
        set_num_robots "$1"
        record_run "$@"
        start_scene
        spawn
        ;;
esac
