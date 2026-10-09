# fleet_config schema migrations

Tables of the base station's `fleet_config` database: the fleet's settings, edited with `scripts/fleetcfg.py` or the
web UI's Configuration page (`tools/sim_ui`), from which `.env` is generated. Same rules as `../migrations/`
(numbered files, never edited once pushed, idempotent: `IF NOT EXISTS`), applied by the same
`basestation/migrate.sh` at every start of the base station:

    docker exec -e PGDATABASE=fleet_config basestation /basestation-migrate.sh /config_migrations

`fleet_config` is this PC's own: `scripts/db_sync.sh` only dumps and restores the farm database, so pulling farm
data from another PC never replaces this PC's settings. Share a profile with `scripts/fleetcfg.py profile export`
/ `profile import` instead.

What each setting means (type, default, which containers it reaches) is not in the database but in
`scripts/fleet_settings.py`, next to the code that reads it: adding a setting needs no migration.
