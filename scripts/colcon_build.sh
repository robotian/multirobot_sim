#!/bin/bash
# colcon build ~/colcon_ws in every running robot container (a300_0000, j100_0001, ...), as the `robot` user.
#
#   scripts/colcon_build.sh                    # colcon build
#   scripts/colcon_build.sh --symlink-install   # extra args are passed straight to colcon build
#
# colcon_ws is the *same* host folder bind-mounted into every robot container (see docker-compose.yml), so one
# build's output is already visible to all of them; this still builds it once per container (cheap, incremental
# after the first) so a container that only just started, or one running a different image, is never stale.
set -uo pipefail
cd "$(dirname "$0")/.."

mapfile -t robots < <(docker ps --format '{{.Names}}' | grep -E '^[a-z0-9]+_[0-9]{4,}$' | sort)
if [ ${#robots[@]} -eq 0 ]; then
    echo "no running robot containers (docker compose up -d first)" >&2
    exit 1
fi

status=0
for r in "${robots[@]}"; do
    echo "== $r =="
    # colcon cannot switch an existing build/ + install/ between the copy layout and --symlink-install ("failed to
    # create symbolic link ... because existing path cannot be removed: Is a directory"), so the layout used is
    # recorded in build/.layout and, if it differs (or is missing while build/install exist from an older copy-style
    # build), build/ install/ log/ are cleared once first -- they are generated and gitignored. The marker is
    # written before building so a failed build doesn't cause another full clean next time.
    if ! docker exec -u robot "$r" bash -c '
        cd ~/colcon_ws
        layout=symlink
        if [ "$(cat build/.layout 2>/dev/null)" != "$layout" ] && { [ -d build ] || [ -d install ]; }; then
            echo "build layout changed to $layout-install: clearing build/ install/ log/ (one-time full rebuild)"
            rm -rf build install log || { echo "could not clear build/ install/ log/ (root-owned files? clear them from the host)" >&2; exit 1; }
        fi
        mkdir -p build && echo "$layout" > build/.layout
        colcon build --symlink-install "$@" && source install/setup.bash
    ' bash "$@"; then
        echo "$r: colcon build failed" >&2
        status=1
    fi
done
exit $status
