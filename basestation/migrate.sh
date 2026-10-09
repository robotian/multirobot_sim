#!/bin/bash
# Apply the base station's schema migrations: every <dir>/*.sql (default /migrations, mounted from
# basestation/migrations/) not yet listed in public.schema_migrations, in file-name order. Each file runs in one
# transaction together with its row in schema_migrations, so a failing file leaves nothing half-applied and is
# retried at the next run. Run by entrypoint.sh at every start and by scripts/db_sync.sh after a pull/restore:
#   docker exec basestation /basestation-migrate.sh
# Connects like everything else in the container: the local socket as $PGUSER to $PGDATABASE (trusted).
set -euo pipefail
dir=${1:-/migrations}
log() { echo "[migrate] $*"; }
export PGOPTIONS="${PGOPTIONS:-} -c client_min_messages=warning"  # no "already exists, skipping" notices

psql -q -v ON_ERROR_STOP=1 <<'SQL'
CREATE TABLE IF NOT EXISTS public.schema_migrations (
    version text PRIMARY KEY,
    applied_at timestamptz NOT NULL DEFAULT now()
);
SQL

shopt -s nullglob
files=("$dir"/*.sql)
applied=0
for f in "${files[@]}"; do
    v=$(basename "$f" .sql)
    if [ -n "$(psql -tAq -v v="$v" <<<"SELECT 1 FROM public.schema_migrations WHERE version = :'v';")" ]; then
        continue
    fi
    log "applying $v"
    { cat "$f"; printf '\n'; echo "INSERT INTO public.schema_migrations (version) VALUES (:'v');"; } |
        psql -q -1 -v ON_ERROR_STOP=1 -v v="$v" -f -
    applied=$((applied + 1))
done
log "$applied applied, $(psql -tAq <<<'SELECT count(*) FROM public.schema_migrations;') recorded, ${#files[@]} in $dir"
