#!/bin/bash
# Start the stack with N robots (0-8), or stop it. Each slot's model (a300/a200/j100/r100) comes from
# ROBOT_MODEL_<i> in .env, default a300; see docker-compose.yml.
#   scripts/fleet.sh 5      set NUM_ROBOTS=5 in .env and (re)start the sim and 5 robots
#   scripts/fleet.sh        (re)start with the NUM_ROBOTS from .env
#   scripts/fleet.sh down   stop and remove everything
# `docker compose up` alone does not stop robots that are no longer wanted after lowering NUM_ROBOTS, and the
# sim has to restart to spawn a different number of robots; this script takes care of both.
set -euo pipefail
cd "$(dirname "$0")/.."
MAX=8

if [ "${1:-}" = "down" ]; then
    exec docker compose --profile '*' down
fi
if [ -n "${1:-}" ]; then
    [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -le "$MAX" ] || { echo "usage: $0 [0-$MAX|down]" >&2; exit 1; }
    if grep -q '^NUM_ROBOTS=' .env; then
        sed -i "s/^NUM_ROBOTS=.*/NUM_ROBOTS=$1/" .env
    else
        printf 'NUM_ROBOTS=%s\nCOMPOSE_PROFILES=n${NUM_ROBOTS}\n' "$1" >> .env
    fi
fi
N=$(docker compose config --format json | python3 -c 'import json,sys; print(json.load(sys.stdin)["services"]["isaac-sim"]["environment"]["NUM_ROBOTS"])')
# A ROBOT_MODEL_<i> for a slot NUM_ROBOTS doesn't reach is silently ignored (that slot just never starts) --
# easy to trip over (e.g. NUM_ROBOTS=2 with ROBOT_MODEL_2 set: slot 2 needs NUM_ROBOTS=3), so flag it here
# instead of leaving "why did I get a300 instead of my configured model" to be debugged by hand.
while IFS= read -r i; do
    [ -n "$i" ] && [ "$i" -ge "$N" ] && echo "warning: .env sets ROBOT_MODEL_$i but NUM_ROBOTS=$N only starts slots 0..$((N - 1)) -- slot $i is not running" >&2
done < <(grep -oE '^ROBOT_MODEL_[0-9]+' .env | sed 's/ROBOT_MODEL_//')
# Remove surplus slots by their stable compose service key (robotN), not by guessing a container name -- a
# slot's container name depends on its assigned model, which may have changed since it was last brought up.
for ((i = N; i < MAX; i++)); do
    docker compose rm -sf "robot$i" >/dev/null 2>&1 || true
done
docker compose up -d
# `docker compose ps` with several service names sorts its output alphabetically, not by argument/slot order, so
# each slot's real name is queried on its own to keep it correctly paired with that slot's fixed Foxglove port.
names=(); for ((i = 0; i < N; i++)); do names+=("$(docker compose ps --format '{{.Name}}' "robot$i" 2>/dev/null)"); done
echo "started $N robot(s): ${names[*]}"
for ((i = 0; i < N; i++)); do printf '  %-14s Foxglove ws://<host>:%d\n' "${names[$i]}" $((8765 + i)); done
