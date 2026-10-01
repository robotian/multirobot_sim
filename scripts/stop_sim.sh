#!/bin/bash
# Stop and remove every container of the stack (sim, robots, zenoh router), and the pending spawn request so the
# next start is an empty scene (see scripts/fleet_ctl.py).
# Plain `docker compose down` would miss robot services outside the currently active NUM_ROBOTS profile
# (see docker-compose.yml), so this passes --profile '*' to catch all of them regardless of profile.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 scripts/fleet_ctl.py clear
exec docker compose --profile '*' down
