#!/bin/bash
# Start the sim, spawn robots into it, or stop everything. Each slot's model (a300/a200/j100/r100, or a real robot
# id like j100_0921) comes from ROBOT_MODEL_<i> in .env, default a300; see docker-compose.yml.
#   scripts/fleet.sh scene                  start the sim with the scene only (no robots); waits until it's ready
#   scripts/fleet.sh spawn [N] [--poses J]  spawn N robots (NUM_ROBOTS from .env if omitted) into the running scene,
#                                           replacing the robots it has, then (re)start their robot containers
#   scripts/fleet.sh [N]                    both: scene, then spawn N
#   scripts/fleet.sh down                   stop and remove everything
# Spawn poses: --poses '[{"x":0,"y":0,"yaw":90}, ...]' (one per slot, metres/degrees, world frame), else
# ROBOT_POSE_<i>="x,y,yaw" in .env, else the sim's default layout -- see scripts/fleet_ctl.py.
# The sim only builds the scene at start (sim/scripts/setup_scene.py); robots arrive through a spawn request, so
# changing the robots never restarts the sim. A sim that is (re)started while a request exists (e.g. `docker restart
# a300-isaac-sim`, the web UI's "Reset scene") spawns those robots again; `scene` on a stopped sim clears it.
# The robot containers are started with --no-deps so `docker compose` never recreates the sim for them.
# This script also keeps ROBOT_SUFFIX_<i>/ROBOT_HOSTNAME_<i> in .env in sync with ROBOT_MODEL_<i>, so a real robot's
# container name is its own id with no slot suffix (j100_0921, not j100_0921_0000) and every container's OS hostname
# is cpr-<model>-<serial> (e.g. cpr-a300-0000, cpr-j100-0921) -- use it (not `docker compose up -d` directly) after
# changing a slot's model for that to take effect.
set -euo pipefail
cd "$(dirname "$0")/.."
MAX=8
SIM=a300-isaac-sim

usage() { echo "usage: $0 [N] | scene | spawn [N] [--poses JSON] | down   (N = 0-$MAX)" >&2; exit 1; }

set_num_robots() {
    [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -le "$MAX" ] || usage
    if grep -q '^NUM_ROBOTS=' .env; then
        sed -i "s/^NUM_ROBOTS=.*/NUM_ROBOTS=$1/" .env
    else
        printf 'NUM_ROBOTS=%s\nCOMPOSE_PROFILES=n${NUM_ROBOTS}\n' "$1" >> .env
    fi
}

# ROBOT_SUFFIX_<i> gives a real robot id (ROBOT_MODEL_<i> containing "_", e.g. j100_0921) its own id as the
# container name/ROS namespace with no slot suffix, and ROBOT_HOSTNAME_<i> gives every slot's container its real
# OS hostname as cpr-<model>-<serial>, underscores turned to hyphens (e.g. cpr-a300-0000, cpr-j100-0921) --
# docker-compose.yml can't compute either itself (its interpolation can't inspect ROBOT_MODEL_<i>'s content or
# do find/replace, see its own comments) -- kept in sync with ROBOT_MODEL_<i> here instead of requiring the user
# to also hand-manage these directly; ROBOT_SUFFIX_<i> is removed again if that slot's model later changes back
# to a generic one, so a stale empty override never lingers.
sync_slots() {
    for ((i = 0; i < MAX; i++)); do
        model=$(sed -n "s/^ROBOT_MODEL_$i=//p" .env | tail -1)
        model="${model:-a300}"
        if [[ "$model" == *_* ]]; then
            if grep -q "^ROBOT_SUFFIX_$i=" .env; then
                sed -i "s/^ROBOT_SUFFIX_$i=.*/ROBOT_SUFFIX_$i=/" .env
            else
                echo "ROBOT_SUFFIX_$i=" >> .env
            fi
            ns="$model"
        else
            sed -i "/^ROBOT_SUFFIX_$i=/d" .env
            ns=$(printf '%s_%04d' "$model" "$i")
        fi
        hostname="cpr-${ns//_/-}"
        if grep -q "^ROBOT_HOSTNAME_$i=" .env; then
            sed -i "s/^ROBOT_HOSTNAME_$i=.*/ROBOT_HOSTNAME_$i=$hostname/" .env
        else
            echo "ROBOT_HOSTNAME_$i=$hostname" >> .env
        fi
    done
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
    # A ROBOT_MODEL_<i> for a slot NUM_ROBOTS doesn't reach is silently ignored (that slot just never starts) --
    # easy to trip over (e.g. NUM_ROBOTS=2 with ROBOT_MODEL_2 set: slot 2 needs NUM_ROBOTS=3), so flag it here
    # instead of leaving "why did I get a300 instead of my configured model" to be debugged by hand.
    while IFS= read -r i; do
        [ -n "$i" ] && [ "$i" -ge "$n" ] && echo "note: .env sets ROBOT_MODEL_$i but NUM_ROBOTS=$n only uses slots 0..$((n - 1)) -- slot $i is not spawned" >&2
    done < <(grep -oE '^ROBOT_MODEL_[0-9]+' .env | sed 's/ROBOT_MODEL_//')
    sync_slots
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

case "${1:-}" in
    down)
        python3 scripts/fleet_ctl.py clear
        exec docker compose --profile '*' down
        ;;
    scene)
        start_scene
        ;;
    spawn)
        shift
        if [[ "${1:-}" =~ ^[0-9]+$ ]]; then set_num_robots "$1"; shift; fi
        spawn "$@"
        ;;
    "")
        start_scene
        spawn
        ;;
    *)
        set_num_robots "$1"
        start_scene
        spawn
        ;;
esac
