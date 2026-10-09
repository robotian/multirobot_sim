# Base station schema migrations

Changes to the farm database's tables, kept in git so every PC's base station has the same schema even when the
data differs (the data itself is not in git: copy it with `scripts/db_sync.sh`).

- One file per change: `NNN_short_name.sql`, e.g. `003_charging_stations.sql`. They run in file-name order, so
  number them with three digits and never renumber or edit a file that has been pushed: write a new one instead.
- `basestation/migrate.sh` applies each file not yet listed in `public.schema_migrations`, at every start of the
  base station and after `scripts/db_sync.sh pull`/`restore`. A file runs in one transaction with its row in
  `schema_migrations`; if it fails, nothing of it stays and the next start tries again (`docker logs basestation`,
  `[migrate]` lines). To apply new files without a restart: `docker exec basestation /basestation-migrate.sh`.
- Write them so they also work on a database that already has the change, made by hand or restored from a dump
  taken on a PC where the file had already run: `CREATE TABLE IF NOT EXISTS`, `ALTER TABLE ... ADD COLUMN IF NOT
  EXISTS`, `CREATE INDEX IF NOT EXISTS`, `INSERT ... ON CONFLICT DO NOTHING`.
- Schema only (tables, columns, indexes, fixed lookup rows). Farm data (plants, tasks, chargers' positions) belongs
  in the database and moves with `scripts/db_sync.sh`.
