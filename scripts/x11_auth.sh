#!/bin/bash
# Build an X authority file that is valid inside containers (any hostname) so rqt_image_view can open
# windows on the host display without loosening `xhost`.
set -euo pipefail
XAUTH=/tmp/.docker.xauth
touch "$XAUTH"
if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null; then
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
fi
chmod 644 "$XAUTH"
