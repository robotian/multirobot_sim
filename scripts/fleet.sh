#!/bin/bash
# Start the stack with N robots (0-8), or stop it.
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
for ((i = N; i < MAX; i++)); do
    docker rm -f "$(printf 'a300_%04d' "$i")" >/dev/null 2>&1 || true
done
docker compose up -d
echo "started $N robot(s): $(seq -f 'a300_%04g' 0 $((N - 1)) 2>/dev/null | tr '\n' ' ')"
for ((i = 0; i < N; i++)); do printf '  a300_%04d  Foxglove ws://<host>:%d\n' "$i" $((8765 + i)); done
