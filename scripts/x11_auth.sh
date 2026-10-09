#!/bin/bash
# Build an X authority file that is valid inside containers (any hostname) so rqt_image_view can open
# windows on the host display without loosening `xhost`. Written to .x11/xauth, a directory the compose files
# mount at /tmp/.x11 (XAUTHORITY=/tmp/.x11/xauth): a directory mount, unlike a single-file one, shows running
# containers the replaced file, and .x11/ is tracked, so Docker never creates it root-owned.
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)/.x11"
mkdir -p "$DIR"
if [ ! -w "$DIR" ]; then
    echo "x11_auth: $DIR is not writable (owner: $(stat -c %U "$DIR")); fix with: sudo chown $(id -un): $DIR" >&2
    exit 1
fi
tmp=$(mktemp "$DIR/xauth.XXXXXX")
trap 'rm -f "$tmp"' EXIT
if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null; then
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$tmp" nmerge - 2>/dev/null || true
fi
chmod 644 "$tmp"
# rename, not rewrite: works even if an old xauth was left by another user (the directory is ours)
mv -f "$tmp" "$DIR/xauth"
