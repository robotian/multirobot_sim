# Shared database dumps

Farm database dumps passed between PCs through git, for PCs that can't reach each other's base station (so
`scripts/db_sync.sh pull` can't be used). Stored through Git LFS (`.gitattributes`), so each dump costs the
repository a small pointer, not its size; `git lfs install` is needed once per machine, as for `sim/assets/`.

On the PC whose data the others should get:

```bash
scripts/db_sync.sh dump basestation/shared/test_lavender_farming_$(date +%Y%m%d-%H%M%S).dump
git add basestation/shared && git commit -m "Farm database from <PC>" && git push
```

On the other PCs, after `git pull`:

```bash
scripts/db_sync.sh restore basestation/shared/<file>.dump   # replaces this PC's whole database (saves it first)
```

Only the dumps put here on purpose. The automatic backups `db_sync.sh` takes before every pull/restore go to
`basestation/backups/`, which stays gitignored. Delete old dumps here once every PC has the newer one (LFS keeps the
history). A dump holds the whole farm database, so this repository's access is the database's access.
