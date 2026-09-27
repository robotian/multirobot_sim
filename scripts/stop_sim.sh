#!/bin/bash
# Stop and remove every container of the stack (sim, robots, zenoh router).
# Plain `docker compose down` would miss robot services outside the currently active NUM_ROBOTS profile
# (see docker-compose.yml), so this passes --profile '*' to catch all of them regardless of profile.
set -euo pipefail
cd "$(dirname "$0")/.."
exec docker compose --profile '*' down
