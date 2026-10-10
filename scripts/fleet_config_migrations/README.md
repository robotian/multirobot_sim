# fleet_config schema migrations (SQLite)

Tables of `fleet_config.sqlite`, each checkout's own settings database (gitignored, next to `.env`): profiles,
robot slots, real robots, the change log and runs, edited with `scripts/fleetcfg.py` or the web UI's
Configuration page (`tools/sim_ui`, `/config`), from which `.env` is generated.

- One file per change: `NNN_short_name.sql`, applied in name order by `scripts/fleetcfg.py` the first time it
  opens the database after the file appeared (each in one transaction, recorded in `schema_migrations`). Never
  renumber or edit a file that has been pushed: write a new one.
- SQLite: no regex CHECKs (use `GLOB`), booleans 0/1, times as UTC ISO text, JSON as text. `ALTER TABLE ... ADD
  COLUMN` has no `IF NOT EXISTS`: a migration runs exactly once per database, so it doesn't need it.
- What each setting means (type, default, which containers it reaches) is not in the database but in
  `scripts/fleet_settings.py`, next to the code that reads it: adding a setting needs no migration.

Until 2026-10-10 these tables were the database `fleet_config` on the base station's PostgreSQL
(`basestation/config_migrations/`, removed), which made the base station a requirement for changing a setting on
every PC. The first use of a checkout's SQLite file imports that PostgreSQL database when it can reach it.
