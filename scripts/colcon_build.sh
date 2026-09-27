#!/bin/bash
# colcon build ~/colcon_ws in every running robot container, as the `robot` user.
#
#   scripts/colcon_build.sh                    # colcon build
#   scripts/colcon_build.sh --symlink-install   # extra args are passed straight to colcon build
#
# colcon_ws is the *same* host folder bind-mounted into every robot container (see docker-compose.yml), so one
# build's output is already visible to all of them; this still builds it once per container (cheap, incremental
# after the first) so a container that only just started, or one running a different image, is never stale.
set -uo pipefail
cd "$(dirname "$0")/.."

mapfile -t robots < <(docker ps --format '{{.Names}}' | grep -E '^a300_[0-9]+$' | sort)
if [ ${#robots[@]} -eq 0 ]; then
    echo "no running robot containers (docker compose up -d first)" >&2
    exit 1
fi

status=0
for r in "${robots[@]}"; do
    echo "== $r =="
    if ! docker exec -u robot "$r" bash -c 'cd ~/colcon_ws && colcon build "$@"' bash "$@"; then
        echo "$r: colcon build failed" >&2
        status=1
    fi
done
exit $status
