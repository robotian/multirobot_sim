#!/bin/bash
# Copy the farm database between PCs' base stations (basestation.compose.yml: PostgreSQL in the `basestation`
# container). Each PC's base station has its own database; this makes one PC's a copy of another's.
#   scripts/db_sync.sh dump [FILE]                save this PC's database (default basestation/backups/<db>_<time>.dump)
#   scripts/db_sync.sh pull HOST [--port P] [--yes]   replace this PC's database with HOST's (its base station)
#   scripts/db_sync.sh restore FILE [--yes]       replace this PC's database with a dump file (pg_dump -Fc)
# pull and restore first save this PC's database to basestation/backups/<db>_before-<cmd>_<time>.dump, restore into
# a temporary database and only then swap it in, so a failed restore leaves the current one untouched; then apply
# the schema migrations (basestation/migrations/). Connections to the database (status_server, robots) are dropped
# at the swap and reconnect. pull asks HOST's PostgreSQL on port 5433 (BASESTATION_PG_PORT there) with this PC's
# user/database names and password (db.env); a different password: export DB_SYNC_PASSWORD=... first (it is passed to
# the container by name, never on a command line). HOST's firewall must allow TCP 5433 from this PC.
# Dumps are farm data, not source: basestation/backups/ is gitignored. Between PCs that can't reach each other, dump into
# basestation/shared/ (Git LFS) and commit it; restore it on the others (basestation/shared/README.md).
set -euo pipefail
cd "$(dirname "$0")/.."
C=basestation
BACKUPS=basestation/backups

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0" >&2; exit 2; }
die() { echo "db_sync: $*" >&2; exit 1; }
stamp() { date +%Y%m%d-%H%M%S; }

docker inspect -f '{{.State.Running}}' "$C" 2>/dev/null | grep -q true || die "the $C container is not running (docker compose -f basestation.compose.yml up -d)"
DB=$(docker exec "$C" printenv PGDATABASE)
PORT=$(docker exec "$C" printenv PGPORT)
mkdir -p "$BACKUPS"

confirm() {  # confirm "<what>"; --yes skips; refuses without a terminal
    [ "${YES:-0}" = 1 ] && return 0
    [ -t 0 ] || die "$1: needs confirmation, rerun with --yes"
    read -r -p "$1 -- type yes to continue: " a
    [ "$a" = yes ] || die "cancelled"
}

dump_local() {  # dump_local FILE
    docker exec "$C" pg_dump -Fc "$DB" > "$1" || { rm -f "$1"; die "pg_dump of $DB failed"; }
    echo "saved $DB to $1 ($(du -h "$1" | cut -f1))"
}

replace_with() {  # replace_with DUMPFILE LABEL
    local file=$1 tmp="${DB}_sync_tmp"
    [ -s "$file" ] || die "$file is missing or empty"
    dump_local "$BACKUPS/${DB}_before-$2_$(stamp).dump"
    echo "restoring $file into a temporary database $tmp"
    docker exec "$C" dropdb --if-exists --force --maintenance-db=postgres "$tmp"
    docker exec "$C" createdb --maintenance-db=postgres "$tmp"
    if ! docker exec -i "$C" pg_restore --no-owner --exit-on-error -d "$tmp" < "$file"; then
        docker exec "$C" dropdb --if-exists --force --maintenance-db=postgres "$tmp" || true
        die "restore failed; $DB is unchanged"
    fi
    echo "swapping $tmp in as $DB"
    docker exec -i "$C" psql -q -v ON_ERROR_STOP=1 -d postgres -v db="$DB" -v tmp="$tmp" <<'SQL'
DROP DATABASE :"db" WITH (FORCE);
ALTER DATABASE :"tmp" RENAME TO :"db";
SQL
    docker exec "$C" /basestation-migrate.sh /migrations
    echo "done: $DB is now a copy of $file"
}

cmd=${1:-}; [ -n "$cmd" ] || usage; shift
YES=0 RPORT=5433 ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --yes) YES=1 ;;
        --port) RPORT=${2:?--port needs a value}; shift ;;
        -h|--help) usage ;;
        -*) usage ;;
        *) ARGS+=("$1") ;;
    esac
    shift
done

case "$cmd" in
    dump)
        dump_local "${ARGS[0]:-$BACKUPS/${DB}_$(stamp).dump}" ;;
    restore)
        [ ${#ARGS[@]} = 1 ] || usage
        confirm "replace this PC's database $DB with ${ARGS[0]}"
        replace_with "${ARGS[0]}" restore ;;
    pull)
        [ ${#ARGS[@]} = 1 ] || usage
        host=${ARGS[0]}
        docker exec -e DB_SYNC_PASSWORD "$C" bash -c 'PGPASSWORD=${DB_SYNC_PASSWORD:-$PGPASSWORD} pg_isready -q -h "$0" -p "$1" -t 5' "$host" "$RPORT" \
            || die "no PostgreSQL at $host:$RPORT (is its base station up, and TCP $RPORT open in its firewall?)"
        confirm "replace this PC's database $DB with $host:$RPORT's"
        file="$BACKUPS/${DB}_from-${host}_$(stamp).dump"
        echo "dumping $DB from $host:$RPORT"
        docker exec -e DB_SYNC_PASSWORD "$C" bash -c \
            'PGPASSWORD=${DB_SYNC_PASSWORD:-$PGPASSWORD} exec pg_dump -h "$0" -p "$1" -Fc "$PGDATABASE"' "$host" "$RPORT" \
            > "$file" || { rm -f "$file"; die "pg_dump from $host:$RPORT failed (password? set DB_SYNC_PASSWORD)"; }
        echo "pulled $file ($(du -h "$file" | cut -f1))"
        replace_with "$file" pull ;;
    *) usage ;;
esac
